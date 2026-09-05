/**
 * RaceMetry Service Worker
 * Provides 100% offline capability and instant background auto-updates
 */

const CACHE_NAME = 'racemetry-v5.4';
const ASSETS_TO_CACHE = [
  './',
  './index.html',
  './dragon_app.html',
  './manifest.json',
  './icon.svg',
  './icon-192.png',
  './icon-512.png'
];

// Install: Cache core assets and activate immediately
self.addEventListener('install', (event) => {
  event.waitUntil(
    caches.open(CACHE_NAME).then((cache) => {
      return cache.addAll(ASSETS_TO_CACHE);
    }).then(() => self.skipWaiting())
  );
});

// Activate: clean old versions, claim clients and reload an already open PWA
// once so it cannot keep running stale inline JavaScript until a manual swipe.
self.addEventListener('activate', (event) => {
  event.waitUntil(
    caches.keys().then(async (keys) => {
      const oldRaceMetryCaches = keys.filter(
        (key) => key.startsWith('racemetry-') && key !== CACHE_NAME
      );
      await Promise.all(oldRaceMetryCaches.map((key) => caches.delete(key)));
      await self.clients.claim();

      if (oldRaceMetryCaches.length > 0) {
        const windows = await self.clients.matchAll({
          type: 'window',
          includeUncontrolled: true
        });
        await Promise.all(windows.map((client) => (
          typeof client.navigate === 'function' ? client.navigate(client.url) : Promise.resolve()
        )));
      }
    })
  );
});

// Fetch handler: Network-First for HTML documents, Cache-First for static assets
self.addEventListener('fetch', (event) => {
  if (event.request.method !== 'GET') return;

  // 1. Для HTML-документов: Всегда сначала загружаем свежую версию из сети
  if (event.request.mode === 'navigate' || event.request.destination === 'document') {
    event.respondWith(
      fetch(event.request, { cache: 'no-store' })
        .then((networkResponse) => {
          if (networkResponse && networkResponse.status === 200) {
            const responseToCache = networkResponse.clone();
            caches.open(CACHE_NAME).then((cache) => cache.put(event.request, responseToCache));
          }
          return networkResponse;
        })
        .catch(() => {
          // Если нет интернета - корректный промис-fallback на кэш
          return caches.match(event.request).then((cached) => {
            return cached || caches.match('./index.html');
          });
        })
    );
    return;
  }


  // 2. Для статических файлов (иконки, манифест): Stale-While-Revalidate
  event.respondWith(
    caches.match(event.request).then((cachedResponse) => {
      const fetchPromise = fetch(event.request)
        .then((networkResponse) => {
          if (networkResponse && networkResponse.status === 200) {
            const responseToCache = networkResponse.clone();
            caches.open(CACHE_NAME).then((cache) => cache.put(event.request, responseToCache));
          }
          return networkResponse;
        })
        .catch(() => {});

      return cachedResponse || fetchPromise;
    })
  );
});
