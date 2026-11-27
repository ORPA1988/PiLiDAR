/* Worker für die Scan-Ausrichtung (rechnet ohne die Oberfläche zu blockieren). */
'use strict';
importScripts('registration.js' + self.location.search);   // gleiche Versionskennung

self.onmessage = (ev) => {
  const m = ev.data;
  if (m.type !== 'align') return;
  try {
    const res = Registration.align(m.ref, m.mov, m.opts || {}, (f, phase) =>
      self.postMessage({ type: 'progress', id: m.id, f, phase }));
    self.postMessage({ type: 'result', id: m.id, res });
  } catch (e) {
    self.postMessage({ type: 'error', id: m.id, error: String(e && e.message || e) });
  }
};
