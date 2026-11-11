// Included only in the downloadable USB panel built by tools/build_usb_page.py.
(() => {
  'use strict';
  const dialog = document.getElementById('usbConnectDialog');
  const button = document.getElementById('usbConnectButton');
  const message = document.getElementById('usbMessage');
  const encoder = new TextEncoder();
  const nativeFetch = window.fetch.bind(window);
  window.gdl90Usb = true;

  let connection = null;
  let ready;
  let signalReady;
  let queue = Promise.resolve();

  function showConnect(reason) {
    ready = new Promise(resolve => { signalReady = resolve; });
    if (reason) message.textContent = reason;
    button.disabled = false;
    if (!dialog.open) dialog.showModal();
  }

  // Headers and newlines are ASCII. Reassemble payload bytes before UTF-8 decoding
  // because the firmware can split a multibyte character across two reply chunks.
  async function readPort(link) {
    const line = [];
    const reader = link.port.readable.getReader();
    link.reader = reader;
    try {
      for (;;) {
        const { value, done } = await reader.read();
        if (done) break;
        for (const byte of value) {
          if (byte === 10) {
            if (line.at(-1) === 13) line.pop();
            if (link.pending && line.length < 4096) link.pending.accept(line);
            line.length = 0;
          } else if (line.length < 4096) {
            line.push(byte);
          }
        }
      }
    } catch (error) {
      link.error = error;
    } finally {
      reader.releaseLock();
      link.reader = null;
      link.ended = true;
      link.pending?.reject(new Error('The USB connection closed.'));
      if (connection === link) disconnect(link, 'USB disconnected. Reconnect the board to resume.');
    }
  }

  async function closePort(link) {
    try { await link.reader?.cancel(); } catch {}
    try { await link.reading; } catch {}
    try { await link.port.close(); } catch {}
  }

  function disconnect(link, reason) {
    if (connection !== link) return;
    connection = null;
    showConnect(reason);
    button.disabled = true;
    closePort(link).finally(() => { button.disabled = false; });
  }

  async function request(link, method, path, body, timeout = 15000) {
    if (link.ended) throw new Error('The USB connection closed.');
    const id = crypto.getRandomValues(new Uint32Array(1))[0];
    const prefix = encoder.encode('G90! ' + id + ' ');
    const frame = encoder.encode('G90? ' + JSON.stringify({id, method, path, body}) + '\n');
    if (frame.length > 12 * 1024 + 500) return new Response('Request exceeds the USB limit', {status: 413});
    const writer = link.port.writable.getWriter();
    const chunks = [];
    let size = 0, status = 0, timer;
    const reply = new Promise((resolve, reject) => {
      timer = setTimeout(() => reject(new Error('The ESP32 did not reply. Check its firmware and USB cable.')), timeout);
      link.pending = { reject, accept(bytes) {
        if (bytes.length < prefix.length + 3 || !prefix.every((byte, i) => bytes[i] === byte)) return;
        const offset = prefix.length;
        const marker = String.fromCharCode(...bytes.slice(offset, offset + 3));
        if (marker === 'END' && bytes.length === offset + 3 && status) {
          resolve(new Response(new Blob(chunks, {type: 'application/json'}), {status}));
        } else if ((marker === '200' || marker === '400') && bytes[offset + 3] === 32) {
          status = Number(marker);
          const chunk = Uint8Array.from(bytes.slice(offset + 4));
          size += chunk.length;
          if (size > 512 * 1024) reject(new Error('The USB reply is too large.'));
          else chunks.push(chunk);
        }
      }};
    });
    try {
      const [, result] = await Promise.all([writer.write(frame), reply]);
      return result;
    } finally {
      clearTimeout(timer);
      writer.releaseLock();
      link.pending = null;
    }
  }

  dialog.addEventListener('cancel', event => event.preventDefault());
  showConnect();
  if (!('serial' in navigator)) {
    button.disabled = true;
    message.textContent = 'Open this downloaded file in desktop Chrome or Edge to use USB. You can also join GDL90-SIM and open http://192.168.4.1 in your browser.';
  }

  button.addEventListener('click', async () => {
    button.disabled = true;
    let link;
    try {
      const port = await navigator.serial.requestPort();
      await port.open({baudRate: 115200, bufferSize: 32768});
      link = {port};
      await port.setSignals({dataTerminalReady: false, requestToSend: false});
      link.reading = readPort(link);
      message.textContent = 'Waiting for the emulator…';
      // Some USB bridges reset the ESP32 on open. Probe only with a read.
      let catalog;
      for (let attempt = 0; attempt < 3; attempt++) {
        try {
          const response = await request(link, 'GET', '/api/catalog', null, 4500);
          catalog = await response.json();
          if (catalog.platform === 'esp32') break;
          throw new Error('This port did not identify a GDL 90 emulator.');
        } catch (error) {
          if (link.ended || attempt === 2) throw error;
        }
      }
      if (link.ended) throw new Error('The USB connection closed.');
      connection = link;
      dialog.close();
      signalReady();
    } catch (error) {
      if (link) await closePort(link);
      message.textContent = error.name === 'NotFoundError'
        ? 'Choose the ESP32 USB serial port when you are ready.'
        : error.message + ' Close any serial monitor using the port, then try again.';
      button.disabled = false;
    }
  });

  window.fetch = (path, options = {}) => {
    if (typeof path !== 'string' || !path.startsWith('/api/')) return nativeFetch(path, options);
    const method = options.method || 'GET';
    const originalLink = connection;
    const run = async () => {
      if (method !== 'GET' && (!connection || connection !== originalLink)) {
        throw new Error('Connect the ESP32 before changing settings.');
      }
      while (!connection) await ready;
      const link = connection;
      try {
        return await request(link, method, path, options.body ? JSON.parse(options.body) : null);
      } catch (error) {
        disconnect(link, error.message);
        // Boot reads can safely resume after reconnection. Never repeat a mutation.
        if (method === 'GET') return run();
        throw error;
      }
    };
    const result = queue.then(run);
    queue = result.catch(() => {});
    return result;
  };
})();
