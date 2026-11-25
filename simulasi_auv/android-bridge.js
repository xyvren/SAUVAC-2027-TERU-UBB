(() => {
  if (!window.AndroidWifiBridge) return;

  const nativeFetch = window.fetch.bind(window);
  const pending = new Map();
  let nextRequestId = 1;

  window.__sauvacNativeResolve = (requestId, rawResult) => {
    const resolve = pending.get(requestId);
    if (!resolve) return;
    pending.delete(requestId);

    let result;
    try {
      result = JSON.parse(rawResult);
    } catch {
      result = { success: false, error: 'Respons native Android tidak valid.' };
    }
    resolve(new Response(JSON.stringify(result), {
      status: result.success ? 200 : 409,
      headers: { 'Content-Type': 'application/json' }
    }));
  };

  window.fetch = (input, init = {}) => {
    const url = new URL(typeof input === 'string' ? input : input.url, window.location.href);
    const method = (init.method || (typeof input === 'string' ? 'GET' : input.method) || 'GET').toUpperCase();
    const isWifiEndpoint = [
      '/api/wifi/status',
      '/api/wifi/config',
      '/api/wifi/command',
      '/api/wifi/mission_upload'
    ].includes(url.pathname);
    if (!isWifiEndpoint) return nativeFetch(input, init);

    let payload = {};
    if (method === 'GET' && url.pathname === '/api/wifi/status') {
      payload.op = 'get';
    } else if (method === 'POST') {
      try {
        payload = JSON.parse(init.body || '{}');
      } catch {
        return Promise.resolve(new Response(
          JSON.stringify({ success: false, error: 'Request Wi-Fi bukan JSON yang valid.' }),
          { status: 400, headers: { 'Content-Type': 'application/json' } }
        ));
      }
      if (url.pathname === '/api/wifi/mission_upload' && !payload.op) {
        payload.op = 'mission_upload';
      }
    } else {
      return Promise.resolve(new Response(
        JSON.stringify({ success: false, error: 'Metode Wi-Fi tidak didukung.' }),
        { status: 405, headers: { 'Content-Type': 'application/json' } }
      ));
    }

    const requestId = String(nextRequestId++);
    return new Promise(resolve => {
      const timeout = setTimeout(() => {
        pending.delete(requestId);
        resolve(new Response(
          JSON.stringify({ success: false, error: 'Waktu tunggu respons Pico W habis.' }),
          { status: 504, headers: { 'Content-Type': 'application/json' } }
        ));
      }, 8000);
      pending.set(requestId, response => {
        clearTimeout(timeout);
        resolve(response);
      });
      try {
        window.AndroidWifiBridge.requestWifi(JSON.stringify(payload), requestId);
      } catch (error) {
        clearTimeout(timeout);
        pending.delete(requestId);
        resolve(new Response(
          JSON.stringify({ success: false, error: error.message || 'Jembatan Wi-Fi Android gagal.' }),
          { status: 500, headers: { 'Content-Type': 'application/json' } }
        ));
      }
    });
  };

  document.addEventListener('DOMContentLoaded', () => {
    document.body.classList.add('sauvac-android-app');
    const transport = document.getElementById('configTransportSelect');
    if (transport && typeof window.setConfigTransport === 'function') {
      transport.value = 'wifi';
      window.setConfigTransport('wifi');
    }
    const wifiCapabilityNote = document.querySelector('#wifiConfigPanel > p');
    if (wifiCapabilityNote) {
      wifiCapabilityNote.textContent =
        'Android mendukung readback, konfigurasi parameter, dan upload misi terbatas melalui UDP. Kontrol motor, emergency stop, terminal CLI, HIL, telemetri host, dan audit memerlukan laptop/USB.';
    }
    const emergencyStop = document.querySelector('.btn-estop');
    if (emergencyStop) {
      emergencyStop.disabled = true;
      emergencyStop.title = 'Emergency stop tidak tersedia dari Android. Gunakan prosedur stop USB yang telah diuji.';
      emergencyStop.setAttribute('aria-label', 'Emergency stop tidak tersedia dari Android');
      emergencyStop.textContent = 'STOP VIA USB';
    }
    document.querySelectorAll('#hilArmBtn, #hilStopBtn').forEach(button => {
      button.disabled = true;
    });
    document.querySelectorAll('#splitTerminalDrawer, [onclick*="toggleSplitTerminal"]').forEach(element => {
      element.hidden = true;
    });
    const firmwarePanel = document.querySelector('.wifi-firmware-update');
    if (firmwarePanel) {
      firmwarePanel.innerHTML =
        '<h3>Pembaruan firmware Android</h3>' +
        '<p>Gunakan tab Firmware pada aplikasi Android. Upload dikirim langsung ke Pico W melalui Wi-Fi dan tetap memeriksa kondisi aman firmware.</p>';
    }
  });
})();
