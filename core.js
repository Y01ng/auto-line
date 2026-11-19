/* core.js - LEGO SPIKE Prime (official firmware, HubOS 3) over Web Bluetooth + camera line detection.
 * Hub protocol: https://lego.github.io/spike-prime-docs/  (same as the laptop's spike_ble.py)
 * Pure functions are exported for Node tests (test_core.js); SpikeHub needs a browser (Web Bluetooth).
 */
(function (root, factory) {
  const api = factory();
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.SpikeCore = api;
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  const SERVICE = '0000fd02-0000-1000-8000-00805f9b34fb';
  const RX_CHAR = '0000fd02-0001-1000-8000-00805f9b34fb'; // phone -> hub (write without response)
  const TX_CHAR = '0000fd02-0002-1000-8000-00805f9b34fb'; // hub -> phone (notify)

  const MSG = {
    INFO_REQ: 0x00, INFO_RESP: 0x01, UPLOAD_REQ: 0x0c, UPLOAD_RESP: 0x0d,
    CHUNK_REQ: 0x10, CHUNK_RESP: 0x11, FLOW_REQ: 0x1e, FLOW_RESP: 0x1f, FLOW_NOTIFY: 0x20,
    CONSOLE: 0x21, CLEAR_REQ: 0x46, CLEAR_RESP: 0x47,
  };

  // ---- framing: COBS variant escaping 0x00..0x02, XOR 0x03, 0x02 frame delimiter ----
  const DELIM = 0x02, NO_DELIM = 0xff, OFFSET = 0x02, MAXB = 84, XOR = 0x03;

  function cobsEncode(data) {
    const out = [];
    let codeIndex = 0, block = 0;
    const begin = () => { codeIndex = out.length; out.push(NO_DELIM); block = 1; };
    begin();
    for (const byte of data) {
      if (byte > DELIM) { out.push(byte); block++; }
      if (byte <= DELIM || block > MAXB) {
        if (byte <= DELIM) out[codeIndex] = byte * MAXB + block + OFFSET;
        begin();
      }
    }
    out[codeIndex] = block + OFFSET;
    return out;
  }

  function cobsDecode(data) {
    const out = [];
    if (!data.length) return out;
    const unesc = (code) => {
      if (code === NO_DELIM) return [null, MAXB + 1];
      let v = Math.floor((code - OFFSET) / MAXB), b = (code - OFFSET) % MAXB;
      if (b === 0) { b = MAXB; v -= 1; }
      return [v, b];
    };
    let [value, block] = unesc(data[0]);
    for (let i = 1; i < data.length; i++) {
      block--;
      if (block > 0) { out.push(data[i]); continue; }
      if (value !== null) out.push(value);
      [value, block] = unesc(data[i]);
    }
    return out;
  }

  function pack(msg) {
    const enc = cobsEncode(msg);
    const out = new Uint8Array(enc.length + 1);
    for (let i = 0; i < enc.length; i++) out[i] = enc[i] ^ XOR;
    out[enc.length] = DELIM;
    return out;
  }

  /** frame = bytes between delimiters (delimiter stripped) */
  function unpack(frame) {
    let start = frame.length && frame[0] === 0x01 ? 1 : 0; // optional priority byte
    const x = [];
    for (let i = start; i < frame.length; i++) x.push(frame[i] ^ XOR);
    return Uint8Array.from(cobsDecode(x));
  }

  // ---- CRC32 (zlib), padded to a multiple of 4 bytes as the hub expects ----
  const CRC_TABLE = (() => {
    const t = new Uint32Array(256);
    for (let n = 0; n < 256; n++) {
      let c = n;
      for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
      t[n] = c >>> 0;
    }
    return t;
  })();

  function crc32(bytes, seed = 0) {
    let c = (seed ^ 0xffffffff) >>> 0;
    for (let i = 0; i < bytes.length; i++) c = CRC_TABLE[(c ^ bytes[i]) & 0xff] ^ (c >>> 8);
    return (c ^ 0xffffffff) >>> 0;
  }

  function crc32Padded(bytes, seed = 0) {
    const pad = (4 - (bytes.length % 4)) % 4;
    if (!pad) return crc32(bytes, seed);
    const b = new Uint8Array(bytes.length + pad);
    b.set(bytes);
    return crc32(b, seed);
  }

  // ---- messages ----
  function msgStartUpload(name, slot, crc) {
    const n = new TextEncoder().encode(name);
    const m = new Uint8Array(n.length + 7);
    m[0] = MSG.UPLOAD_REQ;
    m.set(n, 1);
    m[n.length + 1] = 0; // null terminator
    m[n.length + 2] = slot;
    new DataView(m.buffer).setUint32(n.length + 3, crc >>> 0, true);
    return m;
  }

  function msgChunk(runningCrc, chunk) {
    const m = new Uint8Array(chunk.length + 7);
    const dv = new DataView(m.buffer);
    m[0] = MSG.CHUNK_REQ;
    dv.setUint32(1, runningCrc >>> 0, true);
    dv.setUint16(5, chunk.length, true);
    m.set(chunk, 7);
    return m;
  }

  function parseInfo(msg) {
    const d = new DataView(msg.buffer, msg.byteOffset, msg.byteLength);
    return {
      rpc: `${d.getUint8(1)}.${d.getUint8(2)}.${d.getUint16(3, true)}`,
      firmware: `${d.getUint8(5)}.${d.getUint8(6)}.${d.getUint16(7, true)}`,
      maxPacket: d.getUint16(9, true),
      maxMessage: d.getUint16(11, true),
      maxChunk: d.getUint16(13, true),
    };
  }

  const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

  // ---- the hub connection (browser only) ----
  class SpikeHub {
    constructor(handlers = {}) {
      this.h = handlers; // onLine(text), onDisconnect(), onProgramEnd()
      this.rxbuf = [];
      this.waiters = new Map();
      this.maxPacket = 20;
      this.maxChunk = 0;
      this.writeSize = 180; // drops to 20 if the phone's BLE link only allows small writes
      this.lineBuf = '';
      this.lock = Promise.resolve();
      this.device = null;
    }

    get connected() { return !!(this.device && this.device.gatt.connected); }

    /** Must be called from a user gesture (button tap). */
    async connect() {
      this.device = await navigator.bluetooth.requestDevice({ filters: [{ services: [SERVICE] }] });
      this.device.addEventListener('gattserverdisconnected', () => {
        for (const [, w] of this.waiters) w(null);          // fail pending requests right away
        this.waiters.clear();
        this.h.onDisconnect && this.h.onDisconnect();
      });
      return this._open();
    }

    /** Reconnect to the same hub after a drop (no chooser, no user gesture needed). */
    async reconnect() {
      if (!this.device) throw new Error('연결했던 허브가 없어요');
      if (this.connected) return null;
      this.rxbuf = []; this.lineBuf = ''; this.lock = Promise.resolve();
      return this._open();
    }

    async _open() {
      const server = await this.device.gatt.connect();
      const svc = await server.getPrimaryService(SERVICE);
      this.rx = await svc.getCharacteristic(RX_CHAR);
      this.tx = await svc.getCharacteristic(TX_CHAR);
      this.tx.addEventListener('characteristicvaluechanged', (e) => {
        const v = e.target.value;
        this._onNotify(new Uint8Array(v.buffer, v.byteOffset, v.byteLength));
      });
      await this.tx.startNotifications();
      const info = parseInfo(await this.request(Uint8Array.of(MSG.INFO_REQ), MSG.INFO_RESP));
      this.maxPacket = info.maxPacket;
      this.maxChunk = info.maxChunk;
      return info;
    }

    disconnect() { if (this.connected) this.device.gatt.disconnect(); }

    _onNotify(data) {
      for (let i = 0; i < data.length; i++) {
        const b = data[i];
        if (b !== DELIM) { this.rxbuf.push(b); continue; }
        const frame = Uint8Array.from(this.rxbuf);
        this.rxbuf = [];
        if (!frame.length) continue;
        let msg;
        try { msg = unpack(frame); } catch (e) { continue; }
        if (msg.length) this._handle(msg);
      }
    }

    _handle(msg) {
      const id = msg[0];
      if (id === MSG.CONSOLE) { // console text arrives in fragments: reassemble lines
        let end = msg.indexOf(0, 1);
        if (end < 0) end = msg.length;
        this.lineBuf += new TextDecoder().decode(msg.subarray(1, end));
        const parts = this.lineBuf.split('\n');
        this.lineBuf = parts.pop();
        for (const p of parts) this.h.onLine && this.h.onLine(p.replace(/\r$/, ''));
      } else if (id === MSG.FLOW_NOTIFY && msg.length > 1 && msg[1]) {
        this.h.onProgramEnd && this.h.onProgramEnd();
      }
      const w = this.waiters.get(id);
      if (w) { this.waiters.delete(id); w(msg); }
    }

    async _write(packet) {
      for (let i = 0; i < packet.length;) {
        const n = Math.min(this.writeSize, this.maxPacket || 20);
        const part = packet.slice(i, i + n);
        try {
          if (this.rx.writeValueWithoutResponse) await this.rx.writeValueWithoutResponse(part);
          else await this.rx.writeValue(part);
          i += n;
        } catch (e) {
          if (this.writeSize > 20) { this.writeSize = 20; continue; } // small MTU: retry smaller
          throw e;
        }
      }
    }

    send(msg) { // serialize all writes
      const p = pack(msg);
      const run = this.lock.then(() => this._write(p));
      this.lock = run.catch(() => {});
      return run;
    }

    request(msg, respId, timeoutMs = 5000) {
      return new Promise((resolve, reject) => {
        const t = setTimeout(() => {
          this.waiters.delete(respId);
          reject(new Error('허브 응답 없음 (0x' + respId.toString(16) + ')'));
        }, timeoutMs);
        this.waiters.set(respId, (m) => { clearTimeout(t); m ? resolve(m) : reject(new Error('허브 연결이 끊겼어요')); });
        this.send(msg).catch((e) => { clearTimeout(t); this.waiters.delete(respId); reject(e); });
      });
    }

    /** Upload a SPIKE-3 Python program (bytes) to `slot`. */
    async uploadProgram(code, slot) {
      await this.request(Uint8Array.of(MSG.CLEAR_REQ, slot), MSG.CLEAR_RESP); // NACK if empty: fine
      let r = await this.request(msgStartUpload('program.py', slot, crc32Padded(code)), MSG.UPLOAD_RESP);
      if (r[1] !== 0) throw new Error('허브가 업로드를 거부했어요');
      let running = 0;
      for (let i = 0; i < code.length; i += this.maxChunk) {
        const chunk = code.subarray(i, i + this.maxChunk);
        running = crc32Padded(chunk, running);
        r = await this.request(msgChunk(running, chunk), MSG.CHUNK_RESP);
        if (r[1] !== 0) throw new Error('허브가 데이터 조각을 거부했어요 (' + i + ')');
      }
    }

    /** Fast overwrite of a tiny file (the command channel). ~50-150 ms. */
    async writeFile(bytes, slot) {
      const crc = crc32Padded(bytes);
      let r = await this.request(msgStartUpload('program.py', slot, crc), MSG.UPLOAD_RESP);
      if (r[1] !== 0) throw new Error('명령 업로드 거부');
      r = await this.request(msgChunk(crc, bytes), MSG.CHUNK_RESP);
      if (r[1] !== 0) throw new Error('명령 조각 거부');
    }

    async startProgram(slot) {
      const r = await this.request(Uint8Array.of(MSG.FLOW_REQ, 0, slot), MSG.FLOW_RESP);
      if (r[1] !== 0) throw new Error('허브가 프로그램 시작을 거부했어요');
    }

    stopProgram(slot) { return this.request(Uint8Array.of(MSG.FLOW_REQ, 1, slot), MSG.FLOW_RESP); }
  }

  // ---- vision: find a dark tape line in a grayscale frame ----
  function otsu(hist, total) {
    let sum = 0;
    for (let i = 0; i < 256; i++) sum += i * hist[i];
    let sumB = 0, wB = 0, best = -1, thr = 128;
    for (let t = 0; t < 256; t++) {
      wB += hist[t];
      if (!wB) continue;
      const wF = total - wB;
      if (!wF) break;
      sumB += t * hist[t];
      const mB = sumB / wB, mF = (sum - sumB) / wF;
      const between = wB * wF * (mB - mF) * (mB - mF);
      if (between > best) { best = between; thr = t + 1; } // dark = gray < thr
    }
    return thr;
  }

  /**
   * gray: Uint8Array(W*H), row-major. Looks only below roiTop (fraction of height), split into
   * `bands` horizontal bands (far ... near). Per row the longest dark run is the tape; each band
   * takes the median of row centres. error: -1 (line far left) .. 0 (centre) .. +1 (far right).
   */
  function detectLine(gray, W, H, o = {}) {
    const roiTop = o.roiTop ?? 0.4, nb = o.bands ?? 3, minContrast = o.minContrast ?? 40;
    const y0 = Math.min(H - nb, Math.max(0, Math.floor(H * roiTop)));
    const hist = new Uint32Array(256);
    for (let y = y0; y < H; y++) {
      const row = y * W;
      for (let x = 0; x < W; x++) hist[gray[row + x]]++;
    }
    const n = (H - y0) * W;
    const thr = o.threshold == null ? otsu(hist, n) : o.threshold;
    let cd = 0, sd = 0, sb = 0;
    for (let v = 0; v < 256; v++) {
      if (v < thr) { cd += hist[v]; sd += v * hist[v]; } else sb += v * hist[v];
    }
    const cb = n - cd;
    const meanDark = cd ? sd / cd : 0, meanBright = cb ? sb / cb : 255;
    const res = { found: false, error: 0, thr, darkFrac: cd / n, contrast: meanBright - meanDark, bands: [], y0, reason: '' };
    if (!cd || !cb || meanBright - meanDark < minContrast) { res.reason = 'low-contrast'; return res; }
    if (res.darkFrac > 0.6) { res.reason = 'too-dark'; return res; }

    const bandH = (H - y0) / nb;
    // maxRunFrac: rows whose dark run is wider than this are side branches / crossings, not the line
    const maxRun = W * (o.maxRunFrac ?? 0.6), minRun = Math.max(2, Math.round(W * 0.015));
    for (let b = 0; b < nb; b++) {
      const ya = Math.round(y0 + b * bandH), yb = Math.round(y0 + (b + 1) * bandH);
      const centers = [];
      for (let y = ya; y < yb; y++) {
        const row = y * W;
        let best = 0, bestC = -1, run = 0;
        for (let x = 0; x <= W; x++) {
          if (x < W && gray[row + x] < thr) { run++; continue; }
          if (run > best) { best = run; bestC = x - (run + 1) / 2; }
          run = 0;
        }
        if (best >= minRun && best <= maxRun) centers.push(bestC);
      }
      let cx = null;
      if (centers.length >= Math.max(1, (yb - ya) * 0.3)) {
        centers.sort((p, q) => p - q);
        cx = centers[centers.length >> 1];
      }
      res.bands.push({ ya, yb, cx });
    }
    const weights = nb === 3 ? [0.2, 0.3, 0.5] : res.bands.map((_, i) => i + 1); // near counts most
    let ws = 0, es = 0;
    res.bands.forEach((bd, i) => {
      if (bd.cx == null) return;
      ws += weights[i];
      es += weights[i] * ((bd.cx - (W - 1) / 2) / (W / 2));
    });
    if (ws > 0) { res.found = true; res.error = Math.max(-1, Math.min(1, es / ws)); } else res.reason = 'no-line';
    return res;
  }

  // ---- control: PD steering -> wheel speeds (deg/s), + = car forward ----
  function control(err, prevErr, dtMs, c) {
    let d = 0;
    if (prevErr != null && dtMs > 0) d = (err - prevErr) * (66 / dtMs); // per ~15 fps frame
    let turn = c.kp * err + c.kd * d;
    if (c.steerInvert) turn = -turn;
    turn = Math.max(-1, Math.min(1, turn));
    const base = c.speed * (1 - (c.slowOnTurn ?? 0.5) * Math.abs(turn));
    return { left: base * (1 + turn), right: base * (1 - turn), turn };
  }

  function quantize(v) {
    const q = Math.round(v / 10) * 10;
    return Math.max(-1000, Math.min(1000, q)) || 0; // no "-0"
  }

  /** Car wheel speeds -> raw motor speeds for ports [A, B].
   *  Default guess (same as princess#1): right wheel on A, left on B, car forward = A +, B -. */
  function toPorts(left, right, c) {
    const onA = c.swapLR ? left : right, onB = c.swapLR ? right : left;
    const sA = c.invertFB ? -1 : 1;
    return [quantize(sA * onA), quantize(-sA * onB)];
  }

  return {
    SERVICE, RX_CHAR, TX_CHAR, MSG, cobsEncode, cobsDecode, pack, unpack, crc32, crc32Padded,
    msgStartUpload, msgChunk, parseInfo, SpikeHub, otsu, detectLine, control, quantize, toPorts, sleep,
  };
});
