/* Foot Recorder - record raw accelerometer + gyro samples from a Puck.js v2
 to a file in Storage.

 The Puck is found by name ("Puck.js ..."), and everything it runs is sent
 from here over its Nordic UART (its console): "reset()" to stop whatever
 program it's running, then PUCK_CODE, which streams the samples straight
 away and restarts the Puck (load(), so its saved program runs again) as
 soon as the connection ends. The Puck needs firmware 2v29+, and nothing
 else on it. Its frames are:
   0xA5 0x5A, uint16 sequence, uint8 n, uint8 flags (bit 0 = FIFO overrun),
   n * 12 bytes of int16 LE gx,gy,gz,ax,ay,az, uint8 checksum (sum & 255)
 Only the samples are saved, in the same format as raw_plot.py's recordings
 so plot_recording.py reads them directly:
   footrec.MMDDhhmmss.bin   int16 LE gx,gy,gz,ax,ay,az at the chosen rate
   footrec.MMDDhhmmss.json  description, including how many samples are valid

 StorageFile (the appendable kind) can't store byte 0xFF, so the .bin is a
 normal Storage file made the full size of the recording up front, and each
 frame written at its offset. If recording stops early, the rest of the file
 is unwritten (0xFF) and "samples" in the .json says how much is valid.
*/

var PREFIX = "footrec.";
var SETTINGS_FILE = "footrec.json";
var LENGTHS = [5, 10, 20, 30, 60, 120, 300]; // seconds
var BYTES_PER_SAMPLE = 12;
// Sample rates, in Hz. Index + 1 is the LSM6DS3TR-C's rate code (ODR_XL,
// ODR_G, ODR_FIFO all use 1 = 12.5Hz ... 7 = 833Hz). 12 bytes per sample, so
// 416Hz is ~5kB/s and 833Hz ~10kB/s over Bluetooth - the higher rates may
// lose frames (shown, and counted in the .json).
var RATES = [12.5, 26, 52, 104, 208, 416, 833];
var NUS = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
var NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"; // notifications from the Puck
var NUS_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"; // writes to the Puck's console
var NO_DATA_MS = 3000;

/* The code run on the Puck, sent as a single line - so no // comments, and
 every statement ends with ; or }. Same as raw_stream.js (for raw_plot.py):
 * The LSM6DS3TR-C fills its FIFO with gyro + accelerometer samples at RATE
   Hz, which has rate code ODR (both filled in by puckCode()):
   0x0D INT1_CTRL = 0 (no per-sample IRQ), 0x10 CTRL1_XL = ODR<<4 | 0x04
   (+/-16g), 0x11 CTRL2_G = ODR<<4 | 0x0C (+/-2000 deg/s), 0x15/0x16 = 0
   (high-performance mode), FIFO_CTRL1-4 0x06-0x09 = 0,0,9,0 (gyro + accel,
   no decimation), FIFO_CTRL5 0x0A = 0 (clear) then ODR<<3 | 6 (continuous)
 * Every 100ms, poll() reads FIFO_STATUS1-4 (0x3A) and drains whole samples
   from FIFO_DATA_OUT (0x3E) with multi-byte Puck.accelRd (firmware 2v29+),
   realigning to the start of a sample, resetting the FIFO if it overran,
   and reading less at once if there's not enough memory/stack
 * send() writes each chunk to Bluetooth as a frame (see above)
 * If the firmware is too old it prints an ERROR line, which is shown here
 Restarting the Puck afterwards is RESTORE, sent as its own line straight
 after reset() as the disconnect handler (so even a lost connection restarts
 it), and again by finish() before disconnecting. It turns the accelerometer
 off (reset() and load() don't) and runs load() - restart, and run the saved
 program if there is one. */
var PUCK_CODE = `
(function(){
var ODR = %ODR%, CHUNK = 20, FIFO_ON = (ODR << 3) | 6, frame = new Uint8Array(7 + CHUNK * 12), seq = 0, overrun = 0;
function send(d, n) {
  var len = n * 12;
  frame[0] = 0xA5; frame[1] = 0x5A; frame[2] = seq & 255; frame[3] = seq >> 8;
  frame[4] = n; frame[5] = overrun;
  frame.set(d, 6);
  frame[6 + len] = E.sum(d) & 255;
  Bluetooth.write(new Uint8Array(frame.buffer, 0, 7 + len));
  seq = (seq + 1) & 0xFFFF; overrun = 0;
}
function poll() {
  var st = Puck.accelRd(0x3A, 4);
  if (st[1] & 0x10) return;
  if (st[1] & 0x40) { overrun = 1; Puck.accelWr(0x0A, 0); Puck.accelWr(0x0A, FIFO_ON); return; }
  var words = st[0] | ((st[1] & 7) << 8), pattern = st[2] | ((st[3] & 3) << 8);
  if (pattern) {
    if (words < 6 - pattern) return;
    Puck.accelRd(0x3E, (6 - pattern) * 2); words -= 6 - pattern;
  }
  var samples = (words / 6) | 0;
  while (samples > 0) {
    var n = Math.min(samples, CHUNK), d;
    try { d = Puck.accelRd(0x3E, n * 12); } catch (e) {}
    if (d === undefined) { if (CHUNK == 1) return; CHUNK = CHUNK >> 1; continue; }
    send(d, n); samples -= n;
  }
}
Puck.accelOn(%RATE%);
if (!(Puck.accelRd(0x0F, 1) instanceof Uint8Array)) {
  Puck.accelOff();
  Bluetooth.println("ERROR: Puck firmware " + process.env.VERSION + " is too old - needs 2v29+");
  return;
}
[[0x0D, 0], [0x10, (ODR << 4) | 0x04], [0x11, (ODR << 4) | 0x0C], [0x15, 0], [0x16, 0], [0x0A, 0],
 [0x06, 0], [0x07, 0], [0x08, 0x09], [0x09, 0], [0x0A, FIFO_ON]].forEach(function(r) { Puck.accelWr(r[0], r[1]); });
setInterval(poll, 100);
})();
`.replace(/\n\s*/g, "");
var RESTORE = "Puck.accelOff();load();";

// PUCK_CODE for a sample rate from RATES
function puckCode(rate) {
  return PUCK_CODE.replace("%ODR%", RATES.indexOf(rate) + 1).replace("%RATE%", rate);
}

var storage = require("Storage");
var settings = Object.assign({ seconds: 10, rate: 208 }, storage.readJSON(SETTINGS_FILE, true) || {});
if (RATES.indexOf(settings.rate) < 0) settings.rate = 208;
var rec; // the recording in progress

function pad(n) { return ("0" + n).substr(-2); }

function show(lines) {
  var r = Bangle.appRect;
  g.reset().clearRect(r).setFont("6x8", 2).setFontAlign(0, 0);
  // Wrap long lines to the screen width
  var wrapped = [];
  lines.forEach(function(l) { wrapped = wrapped.concat(l ? g.wrapString(l, r.w - 4) : [""]); });
  var y = r.y + (r.h - wrapped.length * 20) / 2 + 10;
  wrapped.forEach(function(l) { g.drawString(l, r.x + r.w / 2, y); y += 20; });
}

// ---------------------------------------------------------------- Recording

function startRecording() {
  var rate = settings.rate;
  var target = Math.round(settings.seconds * rate);
  var size = target * BYTES_PER_SAMPLE;
  if (storage.getFree() < size + 4096)
    return finished(["Not enough", "storage for", settings.seconds + "s"]);
  var d = new Date();
  var r = rec = {
    name: PREFIX + pad(d.getMonth() + 1) + pad(d.getDate()) + pad(d.getHours()) +
          pad(d.getMinutes()) + pad(d.getSeconds()),
    started: d.getFullYear() + "-" + pad(d.getMonth() + 1) + "-" + pad(d.getDate()) + "T" +
             pad(d.getHours()) + ":" + pad(d.getMinutes()) + ":" + pad(d.getSeconds()), // local time
    rate: rate, seconds: settings.seconds, target: target, size: size,
    samples: 0, lostFrames: 0, overruns: 0, seq: -1,
    buf: new Uint8Array(2048), rd: 0, wr: 0, // received bytes not yet parsed
    lastData: 0, done: false,
    line: "", error: "" // console text from the Puck, and the last error line
  };
  Bangle.setUI({ mode: "custom", btn: function() { finish("Cancelled"); } });
  show(["Scanning for", "Puck.js...", "", "BTN: cancel"]);
  var service;
  // Each step checks this recording wasn't cancelled meanwhile - and if it
  // was, disconnects rather than leaving the Puck connected
  function check(x) {
    if (r.done) {
      disconnect(r);
      throw "Cancelled";
    }
    return x;
  }
  var rx;
  r.rx = function() { return rx; };
  function sendText(text) { return writeChunks(rx, text, check); }
  NRF.requestDevice({ filters: [{ namePrefix: "Puck.js" }], timeout: 10000 }).then(check).then(function(device) {
    device.on("gattserverdisconnected", function() { if (rec == r) finish("Puck disconnected"); });
    show(["Connecting to", device.name || "Puck.js", "", "BTN: cancel"]);
    // ~2.5kB/s, so ask for a short connection interval
    return device.gatt.connect({ minInterval: 7.5, maxInterval: 15 });
  }).then(function(gatt) {
    r.gatt = gatt;
    return check(gatt).getPrimaryService(NUS);
  }).then(check).then(function(s) {
    service = s;
    return service.getCharacteristic(NUS_TX);
  }).then(check).then(function(tx) {
    tx.on("characteristicvaluechanged", onData);
    return tx.startNotifications();
  }).then(check).then(function() {
    return service.getCharacteristic(NUS_RX);
  }).then(check).then(function(c) {
    rx = c;
    // Stop whatever the Puck is running (\x10 = don't echo the line back)...
    show(["Uploading", "code to Puck", "", "BTN: cancel"]);
    r.resetSent = true; // before it's sent, so a cancel meanwhile still restarts the Puck
    return sendText("\x10reset();\n");
  }).then(function() {
    return new Promise(function(resolve) { setTimeout(resolve, 1500); });
  }).then(function() {
    // ...restart it whenever the connection ends...
    return sendText("\x10NRF.on('disconnect',function(){" + RESTORE + "});\n");
  }).then(function() {
    // ...then send the streaming code, which starts straight away
    return sendText("\x10" + puckCode(r.rate) + "\n");
  }).then(check).then(function() {
    r.lastData = Date.now();
    r.progress = setInterval(progress, 500);
    progress();
  }).catch(function(e) {
    if (rec == r) finish("Error: " + e);
  });
}

function progress() {
  if (Date.now() - rec.lastData > NO_DATA_MS)
    return finish(rec.error || (rec.samples ? "Data stopped" : "No data from the Puck"));
  show(["Recording",
        (rec.samples / rec.rate).toFixed(1) + " / " + rec.seconds + "s @" + rec.rate + "Hz",
        rec.lostFrames || rec.overruns ? "lost " + rec.lostFrames + "/" + rec.overruns : "",
        "BTN: stop"]);
}

function onData(event) {
  var r = rec;
  if (!r || r.done) return;
  var dv = event.target.value;
  var v = new Uint8Array(dv.buffer, dv.byteOffset, dv.byteLength);
  if (r.wr + v.length > r.buf.length) {
    // Move the unparsed bytes to the start (a forward copy, so overlap is fine)
    r.buf.set(new Uint8Array(r.buf.buffer, r.rd, r.wr - r.rd), 0);
    r.wr -= r.rd;
    r.rd = 0;
    if (r.wr + v.length > r.buf.length) r.wr = 0; // garbage filled the buffer - drop it
  }
  r.buf.set(v, r.wr);
  r.wr += v.length;
  parse();
}

// Pull complete frames out of the buffer and save their samples
function parse() {
  var r = rec, b = r.buf;
  while (!r.done && r.rd < r.wr) {
    // Anything that isn't a frame is console text from the Puck
    if (b[r.rd] != 0xA5) { text(b[r.rd++]); continue; }
    if (r.wr - r.rd < 2) return;
    if (b[r.rd + 1] != 0x5A) { text(b[r.rd++]); continue; }
    if (r.wr - r.rd < 6) return;
    var n = b[r.rd + 4];
    if (n < 1 || n > 32) { text(b[r.rd++]); continue; }
    var len = 7 + n * BYTES_PER_SAMPLE;
    if (r.wr - r.rd < len) return; // wait for the rest
    var payload = new Uint8Array(b.buffer, r.rd + 6, n * BYTES_PER_SAMPLE);
    if ((E.sum(payload) & 255) != b[r.rd + len - 1]) { r.rd++; continue; } // corrupt - resync
    var seq = b[r.rd + 2] | (b[r.rd + 3] << 8);
    if (r.seq >= 0) r.lostFrames += (seq - r.seq - 1) & 0xFFFF;
    r.seq = seq;
    if (b[r.rd + 5] & 1) r.overruns++;
    save(payload, Math.min(n, r.target - r.samples));
    r.rd += len;
    r.lastData = Date.now();
    if (r.samples >= r.target) finish("Done");
  }
}

// Collect console text from the Puck (eg. its startup banner after reset(),
// or an error), keeping the last line that mentions an error
function text(c) {
  var r = rec;
  if (c == 10 || c == 13) {
    if (/ERROR|Error/.test(r.line)) r.error = r.line.replace(/^[>\s]+/, "").trim(); // without the > prompt
    r.line = "";
  } else if (c >= 32 && c < 127 && r.line.length < 80) {
    r.line += String.fromCharCode(c);
  }
}

function save(payload, n) {
  var data = E.toString(new Uint8Array(payload.buffer, payload.byteOffset, n * BYTES_PER_SAMPLE));
  var offset = rec.samples * BYTES_PER_SAMPLE;
  if (offset == 0) storage.write(rec.name + ".bin", data, 0, rec.size); // create full size
  else storage.write(rec.name + ".bin", data, offset);
  rec.samples += n;
}

function finish(reason) {
  var r = rec;
  if (!r || r.done) return;
  r.done = true;
  if (r.progress) clearInterval(r.progress);
  disconnect(r);
  var lines = [reason];
  if (r.samples) {
    storage.writeJSON(r.name + ".json", {
      file: r.name + ".bin", started: r.started, rate_hz: r.rate,
      dtype: "<i2", columns: ["gx", "gy", "gz", "ax", "ay", "az"],
      gyro_dps_per_lsb: 0.07, accel_g_per_lsb: 0.000488,
      samples: r.samples, seconds: Math.round(r.samples / r.rate * 100) / 100,
      lost_frames: r.lostFrames, fifo_overruns: r.overruns,
      note: "samples are contiguous; lost frames/overruns mean gaps in time that aren't " +
            "marked in the .bin. Only the first 'samples' samples of the .bin are valid.",
      recorded_by: "Bangle.js footrec"
    });
    lines.push((r.samples / r.rate).toFixed(1) + "s saved", r.name.substr(PREFIX.length));
    if (r.lostFrames || r.overruns) lines.push("lost " + r.lostFrames + "/" + r.overruns);
  } else {
    lines.push("Nothing saved");
  }
  rec = undefined;
  finished(lines, r.samples ? r.name : undefined);
}

// Send text to the Puck's console, 20 bytes (the most one Bluetooth write
// carries) at a time. before() is called before each write.
function writeChunks(rx, text, before) {
  var i = 0;
  function next() {
    if (before) before();
    if (i >= text.length) return;
    var chunk = text.substr(i, 20);
    i += 20;
    return rx.writeValue(chunk).then(next);
  }
  return Promise.resolve().then(next);
}

// Disconnect - but if the Puck was reset, first restart it: Ctrl-C (\x03)
// clears any half-sent line, then RESTORE (as the disconnect handler would,
// but that might not have been sent yet)
function disconnect(r) {
  function drop() { try { if (r.gatt && r.gatt.connected) r.gatt.disconnect(); } catch (e) {} }
  var rx = r.rx && r.rx();
  if (r.resetSent && rx && r.gatt && r.gatt.connected)
    writeChunks(rx, "\x03\x10" + RESTORE + "\n").then(drop, drop);
  else drop();
}

// Show a result until the button or screen is pressed
// Show a result until the button or screen is pressed - with a recording
// to show, the screen shows its graph
function finished(lines, name) {
  show(lines.concat(["", name ? "Tap: graph" : "", "BTN: menu"]));
  Bangle.setUI({ mode: "custom", btn: showMenu,
                 touch: name ? function() { showGraph(name, showMenu); } : showMenu });
}

// ---------------------------------------------------------------- Graph

var MAX_PER_COLUMN = 12; // samples read per pixel column - more are skipped (zoom in to see all)
var AXIS_COLOURS = ["#f00", "#0a0", "#00f"]; // x, y, z
var GYRO_DPS_PER_LSB = 0.07, ACCEL_G_PER_LSB = 0.000488;

/* For 'cols' pixel columns covering samples s0..s0+n of file, the minimum
 and maximum of each channel (gx,gy,gz,ax,ay,az) in each column - so short
 spikes still show. Reads at most MAX_PER_COLUMN samples per column, spread
 out, as reading every sample of a long recording is too slow. */
function envelope(file, s0, n, cols) {
  var lo = [], hi = [], c, k, j, mn, mx;
  for (k = 0; k < 6; k++) { lo.push(new Int16Array(cols)); hi.push(new Int16Array(cols)); }
  function add(d) { // samples d into this column's mn/mx
    for (var i = 0; i < d.length; i += 6)
      for (var ch = 0; ch < 6; ch++) {
        if (d[i + ch] < mn[ch]) mn[ch] = d[i + ch];
        if (d[i + ch] > mx[ch]) mx[ch] = d[i + ch];
      }
  }
  for (c = 0; c < cols; c++) {
    var a = s0 + Math.floor(c * n / cols), b = Math.max(a + 1, s0 + Math.floor((c + 1) * n / cols));
    mn = [32767, 32767, 32767, 32767, 32767, 32767];
    mx = [-32768, -32768, -32768, -32768, -32768, -32768];
    if (b - a <= MAX_PER_COLUMN) {
      add(new Int16Array(E.toArrayBuffer(storage.read(file, a * BYTES_PER_SAMPLE, (b - a) * BYTES_PER_SAMPLE))));
    } else {
      for (j = 0; j < MAX_PER_COLUMN; j++) {
        var i = a + Math.floor(j * (b - a) / MAX_PER_COLUMN);
        add(new Int16Array(E.toArrayBuffer(storage.read(file, i * BYTES_PER_SAMPLE, BYTES_PER_SAMPLE))));
      }
    }
    for (k = 0; k < 6; k++) { lo[k][c] = mn[k]; hi[k][c] = mx[k]; } // lo > hi: no samples
  }
  return { lo: lo, hi: hi };
}

/* Graph of a recording: accelerometer (g) above, gyro (deg/s) below, x/y/z
 in red/green/blue. Tap to zoom in around that point, drag sideways to pan,
 button to zoom out - and at full view, to go back. */
function showGraph(name, back) {
  var file = name + ".bin";
  var meta = storage.readJSON(name + ".json", true) || {};
  // Valid samples: as the .json says, but never more than the file holds
  var inFile = Math.floor((storage.read(file) || "").length / BYTES_PER_SAMPLE);
  var total = Math.min(meta.samples || inFile, inFile);
  if (!total) return E.showAlert("No samples").then(back);
  var rate = meta.rate_hz || 208;
  var R = Bangle.appRect;
  var view = { s0: 0, n: total }; // first sample, and number of samples, shown
  var dragged = 0, panned = 0;

  function clampView() {
    view.n = Math.max(Math.min(view.n, total), Math.min(total, 8));
    view.s0 = Math.max(0, Math.min(view.s0, total - view.n));
  }

  function draw() {
    g.reset().clearRect(R).setFont("6x8").setFontAlign(0, -1);
    g.drawString("...", R.x + R.w / 2, R.y + R.h / 2);
    var cols = R.w, env = envelope(file, view.s0, view.n, cols);
    g.clearRect(R);
    var top = R.y + 10, bottom = R.y2 - 10, mid = (top + bottom) >> 1;
    // Each panel: channels k0..k0+2, scaled to the largest value shown
    [[3, top, mid - 1, ACCEL_G_PER_LSB, "g"], [0, mid + 1, bottom, GYRO_DPS_PER_LSB, "dps"]].forEach(function(p) {
      var k0 = p[0], y0 = p[1], y1 = p[2], range = 1, k, c;
      for (k = k0; k < k0 + 3; k++)
        for (c = 0; c < cols; c++)
          if (env.lo[k][c] <= env.hi[k][c]) range = Math.max(range, -env.lo[k][c], env.hi[k][c]);
      var ym = (y0 + y1) / 2, scale = (y1 - y0) / 2 / range;
      g.setColor(g.theme.fg).setFontAlign(-1, -1);
      var v = range * p[3];
      g.drawString("+/-" + (v < 10 ? v.toFixed(1) : Math.round(v)) + p[4], R.x + 1, y0);
      g.setColor("#888").drawLine(R.x, ym, R.x2, ym);
      for (k = k0; k < k0 + 3; k++) {
        g.setColor(AXIS_COLOURS[k - k0]);
        for (c = 0; c < cols; c++) {
          // Extend each column to meet the previous one, so steep parts join up
          var lo = env.lo[k][c], hi = env.hi[k][c];
          if (lo > hi) continue; // no samples in this column
          if (c && env.lo[k][c - 1] <= env.hi[k][c - 1]) {
            lo = Math.min(lo, env.hi[k][c - 1]); hi = Math.max(hi, env.lo[k][c - 1]);
          }
          g.drawLine(R.x + c, ym - hi * scale, R.x + c, ym - lo * scale);
        }
      }
    });
    g.setColor(g.theme.fg).setFontAlign(-1, -1).drawString(name.substr(PREFIX.length), R.x + 1, R.y);
    g.setFontAlign(-1, 1).drawString((view.s0 / rate).toFixed(1) + "s", R.x + 1, R.y2);
    g.setFontAlign(1, 1).drawString(((view.s0 + view.n) / rate).toFixed(1) + "s", R.x2, R.y2);
    g.setFontAlign(0, 1).drawString(view.n < total ? "BTN: out" : "tap: zoom", R.x + R.w / 2, R.y2);
  }

  Bangle.setUI({
    mode: "custom",
    btn: function() {
      if (view.n >= total) return back();
      view.s0 -= view.n / 2; view.n *= 2; // zoom out around the middle
      clampView(); draw();
    },
    touch: function(_b, xy) {
      if (Date.now() - panned < 500) return; // the end of a drag, not a tap
      // zoom in x2, keeping the tapped point where it is
      var at = view.s0 + view.n * (xy.x - R.x) / R.w;
      view.n = Math.round(view.n / 2);
      view.s0 = Math.round(at - view.n * (xy.x - R.x) / R.w);
      clampView(); draw();
    },
    drag: function(e) {
      dragged += e.dx;
      if (!e.b) { // released - pan by how far it moved (if it did - a tap drags 0)
        var d = dragged;
        dragged = 0;
        if (Math.abs(d) < 8) return;
        view.s0 -= Math.round(d * view.n / R.w);
        panned = Date.now();
        clampView(); draw();
      }
    }
  });
  draw();
}

// ---------------------------------------------------------------- Menus

function showMenu() {
  Bangle.setUI();
  var i = LENGTHS.indexOf(settings.seconds);
  E.showMenu({
    "": { title: "Foot Recorder", back: load },
    "Record": function() { E.showMenu(); startRecording(); },
    "Length": {
      value: i < 0 ? 1 : i, min: 0, max: LENGTHS.length - 1,
      format: function(v) { return LENGTHS[v] + "s"; },
      onchange: function(v) {
        settings.seconds = LENGTHS[v];
        storage.writeJSON(SETTINGS_FILE, settings);
      }
    },
    "Rate": {
      value: RATES.indexOf(settings.rate), min: 0, max: RATES.length - 1,
      format: function(v) { return RATES[v] + "Hz"; },
      onchange: function(v) {
        settings.rate = RATES[v];
        storage.writeJSON(SETTINGS_FILE, settings);
      }
    },
    "Recordings": showRecordings
  });
}

function showRecordings() {
  var menu = { "": { title: "Recordings", back: showMenu } };
  storage.list(/^footrec\.\d+\.json$/).sort().reverse().forEach(function(f) {
    var meta = storage.readJSON(f, true) || {};
    var name = f.slice(0, -5);
    menu[name.substr(PREFIX.length) + " " + Math.round(meta.seconds || 0) + "s"] = function() {
      E.showMenu({
        "": { title: name.substr(PREFIX.length), back: showRecordings },
        "Graph": function() { E.showMenu(); showGraph(name, showRecordings); },
        "Delete": function() {
          E.showPrompt("Delete " + name.substr(PREFIX.length) + "?").then(function(yes) {
            if (yes) { storage.erase(name + ".bin"); storage.erase(name + ".json"); }
            showRecordings();
          });
        }
      });
    };
  });
  if (Object.keys(menu).length == 1) menu["(none)"] = function() {};
  E.showMenu(menu);
}

Bangle.loadWidgets();
Bangle.drawWidgets();
showMenu();
