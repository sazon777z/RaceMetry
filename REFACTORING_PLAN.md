# План рефакторинга RaceMetry / DRAGon

Основание: `CODE_REVIEW.md` от 2026-08-29  
Целевая платформа: ESP32-S3, Arduino-ESP32 3.3.6  
Цель: сделать измерительное ядро детерминированным, устранить гонки между задачами, восстановить целостность истории и привести PWA/документацию в соответствие с реально работающими функциями.

## 1. Зафиксированный baseline

Установленные инструменты:

- Arduino CLI 1.5.1;
- Node.js LTS 24.19.0;
- npm 11.17.0;
- Arduino-ESP32 3.3.6 уже был установлен в системе.

Проверочная конфигурация:

```text
FQBN: esp32:esp32:esp32s3
USB CDC On Boot: Enabled
Flash Size: 4 MB
Partition Scheme: Default 4MB with SPIFFS
PSRAM: Disabled
```

Команда baseline-сборки:

```powershell
arduino-cli compile `
  --fqbn "esp32:esp32:esp32s3:CDCOnBoot=cdc,FlashSize=4M,PartitionScheme=default,PSRAM=disabled" `
  --warnings all .
```

Результат baseline:

- сборка успешна;
- flash: 724 179 / 1 310 720 байт, 55%;
- глобальные данные: 31 316 / 327 680 байт, 9%;
- inline JavaScript из `index.html` проходит `node --check`;
- есть предупреждения проекта: `%u` для `uint32_t`, deprecated `neopixelWrite`, deprecated ручной `BLE2902`, неиспользуемый `nowMs`;
- предупреждения из заголовков ESP-IDF/Arduino core учитывать отдельно и не смешивать с предупреждениями проекта.

До начала функциональных изменений следует закрепить Arduino-ESP32 3.3.6 как reproducible baseline. Переход на доступную 3.3.11 выполнять отдельной задачей после исправления основной логики.

## 2. Правила проведения рефакторинга

1. Сначала characterization tests, затем изменение поведения.
2. Один изменяемый ресурс — один task-владелец.
3. BLE callbacks не выполняют бизнес-логику, I2C, NVS, delay или deep sleep.
4. Математика заезда не читает `millis()`/`micros()` внутри себя: время передаётся явно.
5. IMU-выборки и GNSS epochs обрабатываются разными входами и на своих частотах.
6. Нельзя сохранять результат, если источник времени/скорости стал stale или потерял допустимое качество.
7. Формат NVS и BLE-протокол версионируются до изменения структуры данных.
8. Каждый этап должен оставлять проект компилируемым и иметь собственный acceptance gate.
9. Исправления измерительной математики, конкурентности и UI не объединять в один большой коммит.
10. Старые функции удалять только после появления теста или явного решения об отказе от заявленной возможности.

## 3. Целевая модель владения данными

| Ресурс | Единственный владелец | Остальные взаимодействуют через |
|---|---|---|
| `GpsEngine`, UART1 | `TelemetryTask` | immutable `GpsEpoch`/snapshot |
| `ImuEngine`, `Wire` | `TelemetryTask` | immutable `ImuSample`/snapshot |
| `TelemetryEngine` | `TelemetryTask` | `TelemetryCommandQueue` и события |
| `ButtonManager`, `LedController` | `TelemetryTask` | команды индикации |
| `BLECharacteristic`, TX buffer | `CommTask` | `BleTxQueue` |
| `StorageManager`, Preferences/NVS | `CommTask` | `StorageRequestQueue`/run events |
| `DeviceSettings` | immutable versioned snapshot | замена целиком после подтверждённого сохранения |
| Завершённый `RunRecord` | immutable event | `CompletedRunQueue` |

Рекомендуемый поток:

```text
BLE onWrite
    -> parse + validate
    -> CommandQueue
        -> CommTask dispatch
            -> TelemetryCommandQueue -> TelemetryTask
            -> Storage operation     -> CommTask

TelemetryTask
    -> live immutable snapshot -> snapshot mailbox -> CommTask -> BLE
    -> split/run/error event    -> event queue       -> CommTask -> NVS/BLE
```

## 4. Этап 0 — воспроизводимый baseline и каркас проверок

Приоритет: обязательный до изменения логики.  
Связанные пункты ревью: ограничения раздела 2, P3 и предупреждения компилятора.

### Работы

- Добавить документированную команду сборки или скрипт `tools/build.ps1` с фиксированным FQBN.
- Добавить `package.json` с командами проверки web-кода без production-зависимостей.
- Вынести inline JavaScript из HTML в `app.js`, если это можно сделать отдельным mechanical commit без изменения поведения.
- Зафиксировать SHA/version установленного core в документации CI.
- Устранить предупреждения собственного кода:
  - использовать `PRIu32`/явное безопасное приведение в GPS debug output;
  - заменить `neopixelWrite` на поддерживаемый API core либо локализовать compatibility wrapper;
  - убрать ручной `BLE2902` для текущего NimBLE/BLE API после проверки подписки;
  - удалить `nowMs`;
  - явно инициализировать все поля `TelemetryEngine`.
- Не исправлять в этом этапе математику, state machine или протокол.

### Acceptance gate

- Arduino CLI compile завершается с кодом 0.
- В исходниках проекта нет собственных warnings при `--warnings all`.
- `node --check` проходит для всех JS-файлов.
- `manifest.json` валиден, все assets Service Worker существуют.
- Baseline memory report сохранён для сравнения следующих этапов.

## 5. Этап 1 — выделение чистого измерительного ядра и characterization tests

Приоритет: основа исправления P0.  
Зависит от: этапа 0.

### Новые контракты

Предлагаемые типы:

```cpp
struct GpsEpoch {
    GpsData data;
    uint32_t towMs;
    uint64_t arrivalUs;
    uint32_t sequence;
};

struct ImuSample {
    ImuData data;
    uint64_t sampleUs;
};

enum class RaceAbortReason : uint8_t {
    NONE,
    GPS_STALE,
    GPS_FIX_LOST,
    GPS_ACCURACY_BAD,
    IMU_FAILURE,
    USER_RESET
};
```

### Работы

- Разделить текущий `process(gps, imu)` на два входа:
  - `processImuSample(const ImuSample&)` для launch detection;
  - `processGpsEpoch(const GpsEpoch&)` для скорости, дистанции, splits и slope.
- Передавать время в engine аргументом; исключить прямые вызовы `millis()`/`micros()` из расчётного класса.
- Ввести отдельные величины:
  - абсолютное монотонное время выборки;
  - время физического launch;
  - rollout time offset;
  - display/race elapsed time.
- Сначала зафиксировать тестами текущее поведение, включая известные ошибки, затем менять ожидаемые значения отдельными коммитами.
- Создать генератор синтетических GNSS epochs 20 Гц с линейным и кусочно-линейным профилем скорости.

### Обязательные characterization tests

- stationary -> arm -> launch;
- 0-60 и 0-100 без rollout;
- 1-Foot Rollout с текущим разрывом времени;
- 402.336 м при постоянном ускорении;
- brake 100-0;
- finish каждой дисциплины;
- один и тот же PVT, поданный десять раз между epoch;
- отсутствие новых PVT при ненулевой последней скорости;
- завершение run и текущее нулевое `totalDurationSec`.

### Acceptance gate

- Чистая расчётная часть собирается отдельно от Arduino hardware API.
- Тесты воспроизводят P0/P1-1 до исправления.
- Производственная прошивка продолжает собираться без изменения внешнего BLE-протокола.

## 6. Этап 2 — исправление временной модели и P0-дефектов

Приоритет: максимальный.  
Закрывает: P0-1, P0-2, P0-3, P1-1.

### 2.1. Обработка только новых GNSS epochs

- `GpsEngine::update()` должен возвращать не общий `bool`, а событие/счётчик принятого PVT.
- `TelemetryTask` вызывает `processGpsEpoch()` ровно один раз на новый checksum-valid PVT.
- Дубликат `towMs/sequence` должен игнорироваться.
- Интерполяция использует timestamps двух реальных GNSS epochs.
- IMU launch остаётся на 200 Гц и не создаёт псевдо-GPS точки.

### 2.2. Корректный rollout

- Дистанция всегда интегрируется по одной непрерывной временной шкале.
- При пересечении 0.3048 м интерполируется точное время rollout crossing.
- Для временных splits применяется `elapsed = sampleTime - launchTime - rolloutOffset`.
- Финишная физическая дистанция остаётся 402.336 м от стартовой точки; интегратор не сбрасывается и не замораживается.

### 2.3. Политика stale/quality

- В `Config.h` ввести явно названные пороги:
  - максимум возраста PVT;
  - максимум пропущенных epochs;
  - допустимые `fixType`, `sAcc`, при необходимости `hAcc`;
  - grace period отдельно до launch и во время run.
- При нарушении качества прекращать интеграцию немедленно.
- После grace period переводить run в `ABORTED` с причиной.
- Невалидный/aborted run не обновляет PB и не сохраняется как валидный результат.

### 2.4. Finish snapshot

- Вычислять duration до смены state или хранить готовое значение в immutable result.
- Интерполировать finish time, speed и altitude на целевой отсечке.
- Не читать «предыдущую» высоту без привязки к фактической финишной точке.

### Acceptance gate

- Повторная подача одного PVT не меняет distance/time.
- При GNSS 20 Гц скоростная отсечка находится между двумя PVT timestamps.
- Rollout ON не создаёт паузу интеграции и проходит тесты 60 ft/402 м.
- После GPS stale дистанция остаётся неизменной; run получает ожидаемый abort reason.
- `totalDurationSec > 0` и совпадает с временем целевой отсечки в пределах заданного допуска.
- Все дисциплины закрыты unit tests.

## 7. Этап 3 — устранение гонок и назначение владельцев ресурсов

Приоритет: высокий.  
Закрывает: P1-2, P1-3, P1-4 и связанные data races.

### Работы

- В `BleEngine::onWrite()` оставить только:
  - bounded accumulation;
  - parse/validation;
  - неблокирующий `xQueueSend`;
  - error counter при переполнении.
- Запретить callback-коду вызывать `TelemetryEngine`, `Wire`, `Preferences`, LED, `delay()` и deep sleep.
- `TelemetryTask` становится единственным владельцем sensor/race state.
- `CommTask` становится единственным владельцем BLE TX и NVS.
- Заменить `safe*` globals и `newRunSaved` на:
  - versioned `TelemetrySnapshot` mailbox;
  - `CompletedRunEvent` queue;
  - `RaceEvent` queue для split/abort/state changes.
- Удалить неиспользуемые `safeCurrentRun/localCurr`.
- Настройки публиковать целиком с revision number; не менять отдельные поля одновременно из двух задач.
- Команды `calibrate_imu`, `set_disc`, `reset`, `arm` подтверждать ACK после фактического выполнения.
- `power_off` реализовать как оркестрированный shutdown:
  1. запрет новых команд;
  2. остановка/abort активного run;
  3. flush NVS/TX;
  4. GPS backup;
  5. LED animation;
  6. deep sleep из task-владельца.

### Нагрузочные тесты

- 1000 команд `ping/get_info` во время live 15 Гц;
- `reset/set_disc` на границе GPS epoch;
- попытка `calibrate_imu` во время run;
- finish одновременно с `get_history`;
- disconnect/reconnect во время chunked run response;
- заполненная TX queue и проверка политики drop/coalesce для live snapshots.

### Acceptance gate

- ThreadSanitizer применим к host-модели очередей либо все cross-task точки документированы и сведены к FreeRTOS queues/mailbox.
- Нет shared mutable `_txBuffer` между контекстами.
- Каждый completed run доставляется ровно один раз и содержит соответствующий snapshot.
- NVS и I2C никогда не вызываются одновременно из двух задач.

## 8. Этап 4 — надёжная инициализация IMU/GNSS и честная диагностика

Приоритет: высокий.  
Закрывает: P1-6, P1-7, P2-6, P2-7 и диагностические замечания.

### IMU

- Проверять `WHO_AM_I` только на 0x68/0x69 и принимать документированный набор MPU-6050/6500/9250 IDs.
- Не назначать неизвестное I2C-устройство как IMU.
- Проверять critical register writes read-back.
- `begin()` возвращает `false`, если устройство не подтверждено.
- Reinit выполнять вне активного run; при отказе во время run формировать `IMU_FAILURE`.
- Калибровка считает только успешные чтения, имеет minimum success ratio и возвращает typed result.
- Не сохранять offset при неуспешной калибровке.

### GNSS

- Инициализировать UART до wake burst.
- После wake выдерживать установленный период восстановления.
- Auto-baud подтверждать валидным UBX checksum или NMEA checksum, а не числом любых байт.
- Проверять ACK/NAK конфигурационных команд или подтверждать конфигурацию опросом значений.
- Нормализовать UBX/NMEA fix quality в один enum; Web и firmware должны одинаково трактовать 2D/3D/DR.
- Валидировать date/time flags до вычисления Unix timestamp.

### Диагностика

- Удалить hard-coded `storageOk=true` и безусловное `calibrated=true`.
- Передавать фактические baud/rate, состояние fix, возраст PVT, число ошибок UART/I2C/NVS и последнюю abort reason.
- RSSI либо измерять, либо передавать как unknown, но не фиксированные -55 dBm.

### Acceptance gate

- Нет IMU: setup и diagnostics показывают ошибку, launch по IMU запрещён.
- На I2C присутствует другое устройство: оно не принимается за MPU.
- После deep sleep GNSS стабильно просыпается в серии холодных/тёплых запусков.
- Неверный baud/noise не считается найденным потоком.
- Web и firmware одинаково показывают состояние fix.

## 9. Этап 5 — версионированное хранилище и идентичность заезда

Приоритет: высокий/средний.  
Закрывает: P1-8, P2-4, P2-5.

### Новый формат

Не сохранять raw layout `RunRecord`. Ввести storage DTO/serialized record:

```text
magic | schemaVersion | payloadLength | runId | payload | crc32
```

### Работы

- Добавить монотонный `nextRunId`; присваивать ID до публикации completed event.
- Сериализовать поля явно в фиксированном порядке и фиксированных типах.
- Проверять все результаты `Preferences::put/get/remove`.
- Хранить статус последней операции для диагностики.
- Реализовать чтение legacy raw records только как одноразовую миграцию, если надёжно определяется старый размер; иначе безопасно инвалидировать с понятным сообщением.
- Определить семантику `clear_history`:
  - либо очищает runs и PB;
  - либо добавить отдельные `clear_runs`/`clear_personal_bests`.
- Добавить тесты ring buffer на 0, 1, 20 и 21 запись.

### Acceptance gate

- Два runs в одну секунду имеют разные IDs и сохраняются отдельно.
- Повреждённая/усечённая запись отклоняется по length/CRC.
- Смена версии структуры не интерпретирует старые bytes как новый record.
- Ошибка NVS видна клиенту и не сопровождается ложным «успешно сохранено».

## 10. Этап 6 — BLE protocol v2 и управление потоком

Приоритет: средний.  
Закрывает: P2-1, P2-2, часть P1-4.

### Предлагаемые сообщения

- каждое сообщение содержит `v`, `type`, `seq`;
- команды получают `commandId` и ответ `ack/error`;
- `split` содержит enum/name, time, trap speed и runId;
- история передаётся как `history_begin(count)`, `history_item`, `history_end`;
- live snapshots допускают coalescing: при перегрузке отправляется самый свежий, а не очередь устаревших;
- completed run и command ACK не могут быть вытеснены live-трафиком;
- добавить явное `aborted` с причиной.

### Работы

- Заменить общий `bool _splitTriggered` на очередь конкретных `SplitEvent`.
- Определить максимальный размер одного JSON и проверить `snprintf` truncation.
- При сохранении chunking добавить message sequence/chunk index либо использовать payload, соответствующий negotiated MTU.
- Не запускать автоматическую историю одновременно с явным `get_history`; выбрать один handshake.
- На время миграции поддержать чтение v1 в web-клиенте, но firmware отправляет один согласованный формат.

### Acceptance gate

- Каждая достигнутая отсечка приходит клиенту ровно один раз.
- History sync не вызывает finish sound/toast и завершается показом самого нового run.
- Live/history/diagnostics не перемешивают JSON даже при параллельных запросах.
- Переполнение очереди имеет измеряемый counter и предсказуемую политику.

## 11. Этап 7 — восстановление и модульность веб-приложения

Приоритет: высокий для истории, средний для Garage.  
Закрывает: P1-5, P2-2, P2-3, P3-1, P3-3, P3-4.

### Решение по Garage

До начала выбрать один вариант:

1. Полностью реализовать `CarRepository`, active car, modal CRUD, фильтры и привязку `carId` к run.
2. Удалить все Garage controls/filters и выпустить эту функцию позже.

Оставлять текущий полуинтерфейс нельзя.

### Структура web-кода

```text
index.html
app.js
ble-client.js
protocol.js
history-store.js
race-ui.js
settings.js
sw.js
```

### Работы

- Оставить `index.html` canonical; `dragon_app.html` сделать redirect/генерируемой копией или удалить после обновления ссылок.
- Реализовать/удалить отсутствующие `openGarageModal`, `getActiveCar`, `renderCarFilters`, `recalculatePersonalBestsForFilter`.
- Реализовать единый `formatRaceTimestamp` и `setTimezoneSetting`.
- Различать live-completed run и history item.
- Очищать `currentRunSamples` при подтверждённом Arm и после завершения.
- Исправить filtered index: DOM должен хранить `runId`, а не индекс массива.
- Не очищать локальную историю до успешного ACK устройства.
- Удалить пустые `catch`; показывать контролируемую ошибку и писать диагностический log.
- Исправить Service Worker fallback через resolved cache result.
- Пересмотреть cache versioning и update UX.

### Автотесты Node

- parser фрагментированных BLE notifications;
- два JSON в одном chunk и один JSON в нескольких chunks;
- v1/v2 protocol normalization;
- history dedup по runId;
- timezone formatting;
- filtered history selection;
- sample buffer lifecycle;
- Service Worker asset list.

### Browser smoke tests

- загрузка без `ReferenceError`;
- все inline handlers существуют;
- подключение/отключение с mock BLE adapter;
- history begin/items/end;
- offline reload;
- Garage/Timezone согласно выбранному scope.

### Acceptance gate

- Нет undefined handlers/variables.
- Старая локальная история мигрирует или безопасно очищается с уведомлением.
- Повторная history sync не создаёт дубли и не меняет latest run на oldest.
- График одного run не содержит samples другого.
- Offline fallback работает для navigation request, отсутствующего в cache.

## 12. Этап 8 — безопасность, документация и совместимость core

Приоритет: перед публичным релизом.  
Закрывает: P2-8 и P3-долг документации.

### Работы

- Определить threat model BLE.
- Минимум запретить destructive-команды во время run и требовать физическое подтверждение для clear/power off, если это соответствует UX.
- Для продукта рассмотреть bonding/passkey.
- Обновить README/USER_MANUAL по фактическим:
  - имени BLE;
  - частоте задач;
  - дисциплинам;
  - slope semantics;
  - split notifications;
  - требованиям HTTPS/localhost для PWA/Web Bluetooth;
  - поведению clear history/PB.
- Выполнить отдельную compatibility-сборку на Arduino-ESP32 3.3.11.
- Не обновлять production core одновременно с изменением измерительной математики.

### Acceptance gate

- Документация перечисляет только работающие функции.
- Команды с внешним эффектом имеют определённую политику доступа/подтверждения.
- Сборка проходит на pinned 3.3.6; результат проверки 3.3.11 документирован отдельно.

## 13. Этап 9 — HIL и release gate

Приоритет: обязательный перед измерительным релизом.

### Стенд

- ESP32-S3 с реальными MPU и u-blox M10Q;
- возможность записать и воспроизвести raw UBX UART 20 Гц;
- эталонный профиль скорости/дистанции;
- управляемое отключение UART/I2C;
- Android Web Bluetooth client и desktop Chromium.

### Матрица

| Сценарий | Критерий |
|---|---|
| 0-60/0-100/0-200, rollout OFF | Ошибка соответствует заранее утверждённому бюджету |
| Те же профили, rollout ON | Корректный offset, непрерывная дистанция |
| 60 ft/1/8/1/4/1/2 mile | Пересечение физической дистанции интерполировано |
| Brake 100-0 | Старт и stop интерполированы, нет packet-arrival bias |
| Потеря 1-2 PVT | Поведение соответствует grace policy |
| Длительная потеря PVT | Abort, distance frozen, PB не обновлён |
| Потеря IMU | Нет конкурентного reinit во время run |
| 20+ runs и reboot | Ring order/IDs/CRC корректны |
| BLE stress | Нет битого JSON и пропавшего completed run |
| Deep sleep, 20 циклов | GNSS стабильно просыпается и конфигурируется |
| Offline PWA | Запуск и история доступны без сети |

### Release gate

- Все P0/P1 из `CODE_REVIEW.md` закрыты тестами.
- Нет известных data races по таблице владения.
- Нулевая ошибка компиляции и ноль warnings собственного кода.
- Все unit/integration/web tests зелёные.
- HIL-отчёт содержит raw входные данные, ожидаемые и фактические splits.
- Версия firmware, protocol version и storage schema согласованы.

## 14. Предлагаемая последовательность PR/коммитов

1. `build: pin esp32s3 baseline and add checks`
2. `test: characterize current race timing behavior`
3. `refactor: split imu samples from gps epochs`
4. `fix: make rollout and duration monotonic`
5. `fix: abort measurement on stale or invalid gps`
6. `refactor: queue ble commands and own sensor state in telemetry task`
7. `refactor: serialize ble tx and completed run events`
8. `fix: validate imu identity calibration and gps wakeup`
9. `feat: add versioned run storage and unique ids`
10. `feat: introduce ble protocol v2 and split events`
11. `fix: restore history and remove incomplete garage paths`
12. `refactor: modularize web client and add node tests`
13. `docs: align manual with verified behavior`
14. `test: add hil evidence and release checklist`

Каждый пункт должен собираться самостоятельно. Коммиты 3-5 нельзя squash с UI-изменениями: история измерительной математики должна оставаться отдельно проверяемой.

## 15. Оценка объёма

| Блок | Размер | Главный риск |
|---|---|---|
| Baseline и тестовый каркас | M | Отделить core warnings от project warnings |
| Чистое ядро и P0 timing | L | Не изменить физическую семантику rollout |
| Очереди и ownership | L | Shutdown и команды на границах state |
| IMU/GNSS reliability | M | Требует реального железа |
| Storage v2 | M | Миграция существующих records |
| BLE protocol v2 | M | Совместимость firmware/web |
| Web cleanup/Garage | M-L | Сначала требуется product decision по Garage |
| HIL/release | L | Нужен воспроизводимый источник UBX |

## 16. Первое безопасное действие

Начать с этапов 0 и 1: добавить воспроизводимые команды и тесты, которые красным подтверждают четыре главные ошибки — повторную обработку PVT, rollout freeze, stale-distance и нулевой duration. Только после этого менять `TelemetryEngine`.

Это сохраняет измеримый baseline и не позволяет «исправить» одну проблему ценой незаметного изменения других дисциплин.
