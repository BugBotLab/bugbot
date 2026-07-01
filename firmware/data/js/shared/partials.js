// js/shared/partials.js
// Loads HTML partial fragments into elements that have a data-partial attribute.
// Each element's innerHTML is replaced with the fetched HTML (up to 3 retries).
// Called once from index.html after the page skeleton is in place.
async function fetchWithRetry(url) {
  for (let i = 0; i < 3; i++) {
    if (i > 0) {
      await new Promise(function(r) { return setTimeout(r, 400 * i); });
    }
    try {
      const res = await fetch(url, { cache: 'no-store' });
      if (res.ok) return await res.text();
    } catch (err) { void err; }
  }
  throw new Error('Failed to load: ' + url);
}

export async function loadPartials() {
  const holders = Array.from(document.querySelectorAll('[data-partial]'));
  await Promise.all(holders.map(function(el) {
    return fetchWithRetry(el.getAttribute('data-partial'))
      .then(function(html) { el.innerHTML = html; })
      .catch(function(e) { console.error('partial failed:', e); });
  }));
}
