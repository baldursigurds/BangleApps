/* Foot Recorder - record raw accelerometer + gyro samples from a Puck.js v2
 to a file in Storage.

 The Puck is found by name ("Puck.js ..."), and everything it runs is sent
 from here over its Nordic UART (its console): "reset()" to stop whatever
 program it's running, then PUCK_CODE, which defines functions this app
 calls. The Puck needs firmware 2v29+, and nothing else on it.

 The Puck records into its own flash (via a small RAM buffer), and only then
 are the samples sent - so Bluetooth never has to keep up with the sensor,
 and as the data stays on the Puck until it's all arrived, a transfer that
 fails can simply be done again:

 1. frInfo() - the Puck says "FR:HAVE <id> <n> <rate> <overruns> <started>"
    if a recording is waiting in its flash, "FR:BUSY <count>" if it's
    recording, or "FR:NONE". It's safe to call any time, so it's also what's
    sent if the Puck goes quiet (eg. a reply was lost)
 2. frRecord(id, started, rate, odr, want) - it makes a flash file for the
    samples wanted, or as many as fit, says "FR:START <n>", and records,
    saying "FR:REC <count>" about once a second. When it's done (or frStop()
    is called) it saves the description and says "FR:DONE <n> <overruns>" -
    and this app calls frInfo() to get the details and start fetching
 3. frSend(first, count) - this app asks for the samples, 8 frames of 20
    samples at a time, read from the Puck's flash:
      0xA5 0x5A, uint16 frame number, uint8 n, uint8 0,
      n * 12 bytes of int16 LE gx,gy,gz,ax,ay,az,
      uint32 CRC32 of everything after the 0xA5 0x5A
    Frames that don't arrive, or fail the CRC, are asked for again
 4. frDelete() - once every frame has arrived and been saved here, the
    Puck erases its copy and says "FR:DELETED"
 5. The Puck is restarted (load(), so its saved program runs again)

 If the connection is lost while recording, the Puck carries on, saves the
 recording, then restarts itself - "Fetch from Puck" gets it later.

 The samples are saved in the same format as raw_plot.py's recordings, so
 plot_recording.py reads them directly:
   footrec.MMDDhhmmss.bin   int16 LE gx,gy,gz,ax,ay,az at the chosen rate
   footrec.MMDDhhmmss.json  description, including how many samples are valid

 StorageFile (the appendable kind) can't store byte 0xFF, so the .bin is a
 normal Storage file made full size once the size is known, and each frame
 written at its place. Frames that never arrive are left unwritten (0xFF),
 and listed in the .json.
*/

var PREFIX = "footrec.";
var SETTINGS_FILE = "footrec.json";
var LENGTHS = [5, 10, 20, 30, 60, 120, 300]; // seconds - the Puck records as much as fits
var BYTES_PER_SAMPLE = 12, FRAME_SAMPLES = 20;
// Sample rates, in Hz. Index + 1 is the LSM6DS3TR-C's rate code (ODR_XL,
// ODR_G, ODR_FIFO all use 1 = 12.5Hz ... 7 = 833Hz). How long a recording
// can be depends on the Puck's free flash: ~94kB on a Puck.js v2 with
// nothing saved is ~37s at 208Hz.
var RATES = [12.5, 26, 52, 104, 208, 416, 833];
var NUS = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
var NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"; // notifications from the Puck
var NUS_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"; // writes to the Puck's console
var BATCH = 8;            // frames asked for at once (the first 8 still missing)
var MAX_TRIES = 20;       // times a frame is asked for before giving up on it
var PROBE_MS = 4000;      // if the Puck's quiet this long, ask again (frInfo(), or frDelete())
var MAX_PROBES = 5;       // ...and give up after this many unanswered
var TRANSFER_RETRY_MS = 400;   // re-ask for missing frames once data has stopped for this long

/* The code run on the Puck, sent as a single line - so no // comments, and
 every statement ends with ; or }. It defines the fr* functions above.
 * frRecord: erases any old frdata.bin/.json, works out how many samples fit
   in Storage (compacting it if that helps), makes frdata.bin that size (so
   its flash is erased before recording, not during) - with a 12 byte header,
   as on Espruino writing at offset 0 always makes a new file - and
   sets up the LSM6DS3TR-C to fill its FIFO with gyro + accelerometer
   samples: 0x0D INT1_CTRL = 0 (no per-sample IRQ), 0x10 CTRL1_XL =
   ODR<<4 | 0x04 (+/-16g), 0x11 CTRL2_G = ODR<<4 | 0x0C (+/-2000 deg/s),
   0x15/0x16 = 0 (high-performance mode), FIFO_CTRL1-4 0x06-0x09 = 0,0,9,0
   (gyro + accel, no decimation), FIFO_CTRL5 0x0A = 0 (clear) then
   ODR<<3 | 6 (continuous)
 * Every 100ms, poll() reads FIFO_STATUS1-4 (0x3A) and copies whole samples
   from FIFO_DATA_OUT (0x3E) into a 2400 byte RAM buffer with multi-byte
   Puck.accelRd (firmware 2v29+), realigning to the start of a sample, and
   counting and resetting the FIFO if it overran. Each time the buffer fills
   (~1s at 208Hz) it's written to frdata.bin after the header. If that fails,
   recording stops with an ERROR line
 * While recording, the red LED blinks briefly once a second (a 30ms flash,
   to save the battery)
 * At the end, frdata.json is written - it's what says a recording is complete
 * Nothing is printed unless connected, and if the connection's gone when it
   finishes, it restarts itself (load())
 * If the firmware is too old, or there's no room, it prints an ERROR line
 Restarting the Puck afterwards is RESTORE, sent as its own line straight
 after reset() as the disconnect handler (so even a lost connection restarts
 it - unless it's recording), and again by finish() before disconnecting.
 It turns the accelerometer off (reset() and load() don't) and runs load() -
 restart, and run the saved program if there is one. */
var PUCK_CODE = `
(function(){
var S = require("Storage"), F = "frdata.bin", J = "frdata.json";
var n = 0, got = 0, overruns = 0, last = 0, iv, info, ram, rpos = 0, flushed = 0, FIFO_ON = 0, failed, ticks = 0;
var frame = new Uint8Array(10 + 240);
function connected() { return NRF.getSecurityStatus().connected; }
function flush() {
  if (!rpos) return;
  try { S.write(F, E.toString(new Uint8Array(ram.buffer, 0, rpos)), 12 + flushed); }
  catch (e) { rpos = 0; failed = "ERROR: couldn't save to the Puck's flash - " + e; return done(); }
  flushed += rpos; rpos = 0;
}
function done() {
  if (!iv) return;
  clearInterval(iv);
  iv = undefined;
  digitalWrite(LED1, 0);
  Puck.accelOff();
  flush();
  if (failed) { S.erase(F); S.erase(J); if (connected()) Bluetooth.println(failed); return; }
  n = got;
  info.n = got; info.overruns = overruns;
  S.writeJSON(J, info);
  if (connected()) Bluetooth.println("FR:DONE " + n + " " + overruns);
  else load();
}
function poll() {
  if (++ticks % 10 == 0) digitalPulse(LED1, 1, 30);
  var st = Puck.accelRd(0x3A, 4);
  if (st[1] & 0x40) { overruns++; Puck.accelWr(0x0A, 0); Puck.accelWr(0x0A, FIFO_ON); return; }
  if (st[1] & 0x10) return;
  var words = st[0] | ((st[1] & 7) << 8), pattern = st[2] | ((st[3] & 3) << 8);
  if (pattern) {
    if (words < 6 - pattern) return;
    Puck.accelRd(0x3E, (6 - pattern) * 2); words -= 6 - pattern;
  }
  var s = Math.min((words / 6) | 0, n - got);
  while (s > 0) {
    var k = Math.min(s, 20, (ram.length - rpos) / 12), d;
    if (k < 1) return;
    try { d = Puck.accelRd(0x3E, k * 12); } catch (e) {}
    if (!(d instanceof Uint8Array)) return;
    ram.set(d, rpos); rpos += k * 12; got += k; s -= k;
    if (rpos == ram.length) flush();
  }
  if (got >= n || failed) return done();
  if (Date.now() - last > 1000) { last = Date.now(); if (connected()) Bluetooth.println("FR:REC " + got); }
}
global.frBusy = function() { return !!iv; };
global.frInfo = function() {
  if (iv) return Bluetooth.println("FR:BUSY " + got);
  info = S.readJSON(J, true);
  if (info && S.read(F)) {
    n = info.n;
    Bluetooth.println("FR:HAVE " + info.id + " " + info.n + " " + info.rate + " " + info.overruns + " " + info.started);
  } else {
    S.erase(F); S.erase(J); info = undefined;
    Bluetooth.println("FR:NONE");
  }
};
global.frDelete = function() { S.erase(F); S.erase(J); info = undefined; Bluetooth.println("FR:DELETED"); };
global.frStop = function() { if (iv) done(); };
global.frRecord = function(id, started, rate, odr, want) {
  S.erase(F); S.erase(J);
  var room = Math.floor((S.getFree() - 4096) / 12);
  if (room < want) { S.compact(); room = Math.floor((S.getFree() - 4096) / 12); }
  n = Math.min(want, room);
  if (n < 20) return Bluetooth.println("ERROR: no room in the Puck's flash");
  Puck.accelOn(rate);
  if (!(Puck.accelRd(0x0F, 1) instanceof Uint8Array)) {
    Puck.accelOff();
    return Bluetooth.println("ERROR: Puck firmware " + process.env.VERSION + " is too old - needs 2v29+");
  }
  S.write(F, "FRD1 footrec", 0, 12 + n * 12);
  info = { id: id, started: started, rate: rate, n: 0, overruns: 0 };
  got = 0; overruns = 0; rpos = 0; flushed = 0; last = 0; failed = undefined;
  ram = new Uint8Array(2400);
  FIFO_ON = (odr << 3) | 6;
  [[0x0D, 0], [0x10, (odr << 4) | 0x04], [0x11, (odr << 4) | 0x0C], [0x15, 0], [0x16, 0], [0x0A, 0],
   [0x06, 0], [0x07, 0], [0x08, 0x09], [0x09, 0], [0x0A, FIFO_ON]].forEach(function(r) { Puck.accelWr(r[0], r[1]); });
  Bluetooth.println("FR:START " + n);
  iv = setInterval(poll, 100);
};
global.frSend = function(first, count) {
  for (var k = first; k < first + count && k * 20 < n; k++) {
    var c = Math.min(20, n - k * 20), len = c * 12;
    frame[0] = 0xA5; frame[1] = 0x5A; frame[2] = k & 255; frame[3] = k >> 8; frame[4] = c; frame[5] = 0;
    frame.set(new Uint8Array(E.toArrayBuffer(S.read(F, 12 + k * 240, len))), 6);
    var crc = E.CRC32(new Uint8Array(frame.buffer, 2, 4 + len));
    frame[6 + len] = crc & 255; frame[7 + len] = (crc >> 8) & 255; frame[8 + len] = (crc >> 16) & 255; frame[9 + len] = (crc >> 24) & 255;
    Bluetooth.write(new Uint8Array(frame.buffer, 0, 10 + len));
  }
};
})();
`.replace(/\n\s*/g, "");
// Restart the Puck - unless it's recording, when it restarts itself afterwards
var RESTORE = "if(!(typeof frBusy=='function'&&frBusy())){Puck.accelOff();load();}";

var storage = require("Storage");
var settings = Object.assign({ seconds: 10, rate: 208 }, storage.readJSON(SETTINGS_FILE, true) || {});
if (RATES.indexOf(settings.rate) < 0) settings.rate = 208;
var rec; // the session in progress

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

// ---------------------------------------------------------------- Session

/* Connect to the Puck, upload the code, then fetch anything waiting on it
 and - if mode is "record" - make a new recording and fetch that. */
function startSession(mode) {
  if (mode == "record" && storage.getFree() < settings.seconds * settings.rate * BYTES_PER_SAMPLE + 4096)
    return finished(["Not enough", "storage for", settings.seconds + "s"]);
  var r = rec = {
    mode: mode,
    phase: "connect",   // connect, upload, recording, transfer, delete
    rate: settings.rate, seconds: settings.seconds, probes: 0,
    saved: [],          // names of recordings saved this session
    buf: new Uint8Array(2048), rd: 0, wr: 0, // received bytes not yet parsed
    lastData: Date.now(), lastGain: 0, done: false,
    line: "", error: "" // console text from the Puck, and the last error line
  };
  Bangle.setUI({ mode: "custom", btn: function() {
    if (r.phase == "recording") stopEarly(); else finish("Cancelled");
  } });
  show(["Scanning for", "Puck.js...", "", "BTN: cancel"]);
  var service;
  // Each step checks this session wasn't cancelled meanwhile - and if it
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
  NRF.requestDevice({ filters: [{ namePrefix: "Puck.js" }], timeout: 20000 }).then(check).then(function(device) {
    device.on("gattserverdisconnected", function() {
      if (rec != r) return;
      finish(r.phase == "recording" ? "Puck disconnected - it'll finish recording; then use Fetch from Puck" :
             r.phase == "transfer" ? "Puck disconnected - the data is still on it; use Fetch from Puck" :
             "Puck disconnected");
    });
    show(["Connecting to", device.name || "Puck.js", "", "BTN: cancel"]);
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
    // ...restart it whenever the connection ends (unless it's recording)...
    return sendText("\x10NRF.on('disconnect',function(){" + RESTORE + "});\n");
  }).then(function() {
    // ...send the code, and ask if there's a recording waiting on it
    r.phase = "upload";
    r.lastData = Date.now();
    return sendText("\x10" + PUCK_CODE + "\n\x10frInfo();\n");
  }).then(check).then(function() {
    r.progress = setInterval(progress, 500);
    progress();
  }).catch(function(e) {
    if (rec == r) finish("Error: " + e);
  });
}

// Send a command to the Puck's console
function command(cmd) {
  var r = rec, rx = r && r.rx && r.rx();
  r.lastData = Date.now();
  if (rx) writeChunks(rx, "\x10" + cmd + "\n").catch(function(e) { if (rec == r) finish("Error: " + e); });
}

// Start a new recording on the Puck
function recordNew() {
  var r = rec, d = new Date();
  r.newId = r.id = pad(d.getMonth() + 1) + pad(d.getDate()) + pad(d.getHours()) + pad(d.getMinutes()) + pad(d.getSeconds());
  r.started = d.getFullYear() + "-" + pad(d.getMonth() + 1) + "-" + pad(d.getDate()) + "T" +
              pad(d.getHours()) + ":" + pad(d.getMinutes()) + ":" + pad(d.getSeconds()); // local time
  r.recorded = 0;
  r.n = 0;
  r.phase = "upload";
  r.probes = 0;
  r.rate = settings.rate;
  r.want = Math.round(r.seconds * r.rate);
  command("frRecord(" + r.id + "," + JSON.stringify(r.started) + "," + r.rate + "," +
          (RATES.indexOf(r.rate) + 1) + "," + r.want + ");");
}

// Ask the Puck to stop recording now; it keeps and sends what it has
function stopEarly() {
  command("frStop();");
  show(["Stopping..."]);
}

// Every 500ms: show progress, and notice if the Puck has gone quiet
function progress() {
  var r = rec, quiet = Date.now() - r.lastData;
  if (r.phase != "transfer" && quiet > PROBE_MS) {
    // A reply may have been lost - ask again with something safe to repeat
    if (++r.probes > MAX_PROBES)
      return finish(r.error || (r.phase == "recording" ? "Lost touch with the Puck - it'll finish recording; then use Fetch from Puck" : "No reply from the Puck"));
    command(r.phase == "delete" ? "frDelete();" : "frInfo();");
  }
  if (r.phase == "upload" || r.phase == "delete") {
    show([r.phase == "delete" ? "Tidying up..." : "Starting...", "", "BTN: cancel"]);
  } else if (r.phase == "recording") {
    var full = r.n && r.want && r.n < r.want ? " (all that fits)" : "";
    show([r.newId ? "Recording" : "Puck is recording",
          (r.recorded / r.rate).toFixed(1) + (r.n ? " / " + (r.n / r.rate).toFixed(1) : "") + "s" + full,
          "@" + r.rate + "Hz",
          "BTN: stop & save"]);
  } else if (r.phase == "transfer") {
    if (Date.now() - r.lastGain > 15000) return finish("Transfer stopped - the data is still on the Puck; use Fetch from Puck");
    show([r.fetchingOld && r.mode == "record" ? "Fetching earlier recording" : "Transferring",
          Math.floor(100 * r.received / r.frames) + "%",
          "", "BTN: cancel"]);
  }
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
  r.lastData = Date.now();
  r.probes = 0;
  // While transferring, once data stops coming, ask again for what's missing
  if (r.phase == "transfer") {
    if (r.retry) clearTimeout(r.retry);
    r.retry = setTimeout(requestFrames, TRANSFER_RETRY_MS);
  }
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
    var len = 10 + n * BYTES_PER_SAMPLE, c = r.rd + len - 4;
    if (r.wr - r.rd < len) return; // wait for the rest
    // CRC32 of the header (frame number, n, flags) and samples
    var crc = (b[c] | (b[c + 1] << 8) | (b[c + 2] << 16) | (b[c + 3] << 24)) >>> 0;
    if (E.CRC32(new Uint8Array(b.buffer, r.rd + 2, len - 6)) != crc) { r.rd++; continue; } // corrupt - resync
    var payload = new Uint8Array(b.buffer, r.rd + 6, n * BYTES_PER_SAMPLE);
    var k = b[r.rd + 2] | (b[r.rd + 3] << 8);
    if (r.phase == "transfer" && k < r.frames && !r.have[k] &&
        n == Math.min(FRAME_SAMPLES, r.n - k * FRAME_SAMPLES)) save(k, payload);
    r.rd += len;
  }
}

// Start fetching the recording the Puck has (n samples)
function startTransfer(id, n, rate, overruns, started) {
  var r = rec;
  r.id = id; r.name = PREFIX + id; r.n = n; r.rate = rate; r.overruns = overruns; r.started = started;
  r.frames = Math.ceil(n / FRAME_SAMPLES);
  r.have = new Uint8Array(r.frames);   // 1 = received
  r.tries = new Uint8Array(r.frames);  // times asked for
  r.asked = new Uint8Array(r.frames);  // asked for in the current request
  r.received = 0;
  r.lastGain = Date.now();
  // Start afresh (eg. after an earlier transfer failed). On Espruino, writing
  // at offset 0 always makes a new file, so the file is made full size when
  // frame 0 is saved - and that's asked for on its own first
  storage.erase(r.name + ".bin");
  storage.erase(r.name + ".json");
  r.phase = "transfer";
  requestFrames();
}

// Save frame k where it belongs in the file
function save(k, payload) {
  var r = rec;
  if (k == 0) storage.write(r.name + ".bin", E.toString(payload), 0, r.n * BYTES_PER_SAMPLE); // makes the file
  else if (!r.have[0]) return; // the file doesn't exist yet
  else storage.write(r.name + ".bin", E.toString(payload), k * FRAME_SAMPLES * BYTES_PER_SAMPLE);
  r.have[k] = 1;
  r.received++;
  r.lastGain = Date.now();
  if (r.received == r.frames) return transferDone();
  // Once the frames asked for are all here, ask for the next ones
  if (r.asked[k]) { r.asked[k] = 0; if (--r.waiting == 0) requestFrames(); }
}

// Ask the Puck for the first BATCH frames not yet received (or given up
// on), as runs: "frSend(1,1);frSend(3,2);frSend(8,5);"
function requestFrames() {
  var r = rec;
  if (!r || r.done || r.phase != "transfer") return;
  if (r.retry) clearTimeout(r.retry);
  // If nothing comes back at all, ask again
  r.retry = setTimeout(requestFrames, TRANSFER_RETRY_MS * 3);
  var list = [], k, cmd = "", i = 0;
  for (k = 0; k < r.frames && list.length < (r.have[0] ? BATCH : 1); k++)
    if (!r.have[k] && r.tries[k] < MAX_TRIES) list.push(k);
  if (!r.have[0] && list[0] != 0) list = []; // frame 0 never came - nothing can be saved
  if (!list.length) return transferDone();
  r.asked.fill(0);
  list.forEach(function(k) { r.tries[k]++; r.asked[k] = 1; });
  r.waiting = list.length;
  while (i < list.length) {
    var j = i;
    while (j + 1 < list.length && list[j + 1] == list[j] + 1) j++;
    cmd += "frSend(" + list[i] + "," + (j - i + 1) + ");";
    i = j + 1;
  }
  command(cmd);
}

// Every frame has arrived (or been given up on): save the description, and
// if nothing's missing, tell the Puck it can delete its copy
function transferDone() {
  var r = rec, missing = [];
  if (r.retry) clearTimeout(r.retry);
  if (!r.have[0]) return finish("Transfer failed - the data is still on the Puck; use Fetch from Puck");
  for (var k = 0; k < r.frames; k++) if (!r.have[k]) missing.push(k);
  // Standard JSON (Storage.writeJSON would leave keys unquoted), so it can
  // be read on a computer as it is
  storage.write(r.name + ".json", JSON.stringify({
    file: r.name + ".bin", started: r.started, rate_hz: r.rate,
    dtype: "<i2", columns: ["gx", "gy", "gz", "ax", "ay", "az"],
    gyro_dps_per_lsb: 0.07, accel_g_per_lsb: 0.000488,
    samples: r.n, seconds: Math.round(r.n / r.rate * 100) / 100,
    lost_frames: missing.length, missing_frames: missing, frame_samples: FRAME_SAMPLES,
    fifo_overruns: r.overruns,
    note: "recorded on the Puck, then transferred. Frames (of frame_samples samples) in " +
          "missing_frames never arrived and are left as 0xFF bytes in the .bin. A FIFO " +
          "overrun means a gap in time that isn't marked in the .bin.",
    recorded_by: "Bangle.js footrec"
  }));
  r.saved.push(r.name);
  r.lost = missing.length;
  if (missing.length) return finish("Some data lost - it's still on the Puck; use Fetch from Puck to try again");
  r.phase = "delete";
  command("frDelete();");
}

// A line of console text from the Puck: its FR: messages, or an error
function onLine(line) {
  var r = rec, m;
  line = line.replace(/^[>\s]+/, "").trim(); // without the > prompt
  if (line == "FR:NONE") {
    if (r.phase == "transfer" || r.phase == "delete") return; // a late reply to a probe
    if (r.newId) return finish("The Puck lost the recording");
    if (r.mode == "record") recordNew();
    else finish("Nothing waiting on the Puck");
  } else if ((m = /^FR:HAVE (\d+) (\d+) ([\d.]+) (\d+) (\S*)/.exec(line))) {
    if (r.phase == "transfer" || r.phase == "delete") return; // a late reply to a probe
    // The new recording (once FR:DONE), or one waiting from before (eg. an
    // earlier transfer failed) - which is fetched before recording another
    r.fetchingOld = m[1] != r.newId;
    var old = storage.readJSON(PREFIX + m[1] + ".json", true);
    if (old && old.samples == +m[2] && !old.lost_frames) {
      r.alreadyHad = PREFIX + m[1];
      r.phase = "delete"; // already got it all - just tidy up the Puck
      command("frDelete();");
    } else {
      startTransfer(m[1], +m[2], +m[3], +m[4], m[5]);
    }
  } else if ((m = /^FR:START (\d+)/.exec(line))) {
    r.n = +m[1];
    r.phase = "recording";
  } else if ((m = /^FR:(REC|BUSY) (\d+)/.exec(line))) {
    // Recording - maybe started from an earlier session (Fetch from Puck)
    r.recorded = +m[2];
    if (r.phase == "upload") r.phase = "recording";
  } else if (/^FR:DONE/.test(line)) {
    command("frInfo();"); // for the details, then FR:HAVE starts the transfer
  } else if (line == "FR:DELETED") {
    if (r.phase != "delete") return;
    // An earlier recording fetched - now make the new one
    if (r.mode == "record" && !r.newId) recordNew();
    else if (!r.saved.length && r.alreadyHad) finish("Already saved - cleared it from the Puck");
    else finish("Done");
  } else if (/ERROR|Error/.test(line)) {
    r.error = line;
    if (/^ERROR:/.test(line)) finish(line.substr(7)); // the Puck gave up
  }
}

// Collect console text from the Puck a byte at a time, a line at a time
function text(c) {
  var r = rec;
  if (c == 10 || c == 13) {
    if (r.line) onLine(r.line);
    if (rec == r) r.line = "";
  } else if (c >= 32 && c < 127 && r.line.length < 80) {
    r.line += String.fromCharCode(c);
  }
}

function finish(reason) {
  var r = rec;
  if (!r || r.done) return;
  r.done = true;
  if (r.progress) clearInterval(r.progress);
  if (r.retry) clearTimeout(r.retry);
  disconnect(r);
  // A transfer that didn't finish: drop the partial file - the data's still on the Puck
  if (r.phase == "transfer" && r.saved.indexOf(r.name) < 0) {
    storage.erase(r.name + ".bin");
    storage.erase(r.name + ".json");
  }
  var lines = [reason], last = r.saved[r.saved.length - 1] || r.alreadyHad;
  if (last) {
    var meta = storage.readJSON(last + ".json", true) || {};
    lines.push((meta.seconds || 0).toFixed(1) + "s saved", last.substr(PREFIX.length));
    if (meta.lost_frames || meta.fifo_overruns) lines.push("lost " + meta.lost_frames + "/" + meta.fifo_overruns);
  } else {
    lines.push("Nothing saved");
  }
  rec = undefined;
  finished(lines, last);
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
    "Record": function() { E.showMenu(); startSession("record"); },
    "Fetch from Puck": function() { E.showMenu(); startSession("fetch"); },
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
