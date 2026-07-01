// Fetch and inject each panel’s HTML into its placeholder
export async function loadPartials() {
  const holders = [...document.querySelectorAll('[data-partial]')];
  await Promise.all(holders.map(async el => {
    const url = el.getAttribute('data-partial');
    const html = await fetch(url, { cache: 'no-store' }).then(r => r.text());
    el.innerHTML = html;
  }));
}
