/* Foot Run - stride by stride feedback from a Puck.js v2 on a shoe.

 The Puck runs accel_fifo.js (foot-mounted inertial navigation: it counts
 strides and measures each one's displacement), which this app sends it:

 1. Connect to the nearest Puck.js. If it already has the stride service
    (0xBCDE), it's running the code - skip to 4
 2. Over its Nordic UART (its console): reset(), then make sure its Storage
    has footrun.js (the code) and footrun.bin (the compiled C), writing them
    if their CRC32s differ, and run eval(require("Storage").read("footrun.js")).
    Code run from Storage keeps its functions in flash, which is what lets it
    fit in the Puck's RAM
 3. The stride service only appears once the Puck's Bluetooth restarts, so
    disconnect, and connect to the same Puck again
 4. Subscribe to the stride characteristic (0xABCE): 5 little-endian float32s,
    stride count, dx, dy, dz (m), duration (s; 0 = first after standing).
    It's notified on every stride, and repeated every second when nothing's
    happening (the same count - a heartbeat). Each new stride buzzes and is
    shown; heartbeats just show the Puck's still there

 One stride is that foot's movement from landing to landing, ie. two steps.
 The code for the Puck is in puck.js and puck.c; build.py makes the module
 footrun.puck (puckcode.js) from them, with the files to put on the Puck.
*/


var NUS = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
var NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"; // notifications from the Puck's console
var NUS_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"; // writes to the Puck's console
var STRIDE_SERVICE = "bcde", STRIDE_CHAR = "abce";
var SETTINGS_FILE = "footrun.json";
var BUZZ_MS = 60;
var QUIET_MS = 3000; // heartbeats come every second - this long without one is a problem

var session;  // the current connection: { device, gatt, rx, phase, ... }
var run = {   // what's been seen
  last: undefined,   // the Puck's stride count from the last notification
  strides: 0,        // strides counted while connected
  length: 0,         // last stride's horizontal length (m)
  cadence: 0,        // steps per minute from the last stride's duration
  distance: 0,       // total horizontal distance (m)
  heard: 0           // Date.now() of the last notification
};

// The settings from footrun.json (see settings.js), as sent to the Puck as
// FR_SETTINGS - and in the form the Puck reports back the ones it's using
function puckSettings() {
  var s = Object.assign({ rate: 208, detector: "sample", accTh: 0.3, gyroTh: 1.5, stanceMin: 10, window: 5,
                          vcheck: false, vmax: 1 }, require("Storage").readJSON(SETTINGS_FILE, true) || {});
  return { rate: s.rate, accTh: s.accTh, gyroTh: s.gyroTh, detector: s.detector, stanceMin: s.stanceMin,
           window: s.detector == "window" ? Math.min(32, s.window) : 0, vmax: s.vcheck ? s.vmax : 0 };
}

// Are the settings a Puck says it's using (JSON) the ones we want?
function sameSettings(json) {
  var want = puckSettings(), have;
  try { have = JSON.parse(json); } catch (e) { return false; }
  for (var k in want) {
    if (typeof want[k] == "number" ? !(Math.abs(want[k] - have[k]) < 1e-4) : want[k] != have[k]) return false;
  }
  return true;
}

// A short description of the settings, for the screen
function settingsText() {
  var p = puckSettings();
  return p.rate + "Hz " + (p.window ? "window " + p.window : "still " + p.stanceMin) + (p.vmax ? " v<" + p.vmax : "");
}

function show(lines) {
  var r = Bangle.appRect;
  g.reset().clearRect(r).setFont("6x8", 2).setFontAlign(0, 0);
  var wrapped = [];
  lines.forEach(function(l) { wrapped = wrapped.concat(l ? g.wrapString(l, r.w - 4) : [""]); });
  var y = r.y + (r.h - wrapped.length * 20) / 2 + 10;
  wrapped.forEach(function(l) { g.drawString(l, r.x + r.w / 2, y); y += 20; });
}

// ---------------------------------------------------------------- Running

function draw() {
  var r = Bangle.appRect, x = r.x + r.w / 2, quiet = Date.now() - run.heard;
  g.reset().clearRect(r).setFontAlign(0, 0);
  g.setFont("Vector", 44).drawString(run.strides, x, r.y + 30);
  g.setFont("6x8", 1).drawString("strides", x, r.y + 56);
  g.setFont("6x8", 2);
  g.drawString(run.length ? run.length.toFixed(2) + " m" : "-", x, r.y + 78);
  g.drawString(run.cadence ? run.cadence.toFixed(0) + " steps/min" : "-", x, r.y + 100);
  g.drawString(run.distance.toFixed(0) + " m total", x, r.y + 122);
  g.setFont("6x8", 1);
  g.drawString(settingsText(), x, r.y2 - 20);
  g.drawString(quiet > QUIET_MS ? "no data for " + Math.round(quiet / 1000) + "s" : "connected - BTN: menu", x, r.y2 - 8);
}

// The stride characteristic's value when (re)connecting: the Puck's count
// now. Strides it counted while we weren't connected are added (silently -
// their lengths aren't known)
function syncStrides(d) {
  var n = d.getFloat32(0, true);
  run.heard = Date.now();
  if (run.last === undefined || n < run.last) run.last = n; // first time, or the Puck restarted
  else if (n > run.last) { run.strides += n - run.last; run.last = n; }
}

// A notification from the stride characteristic
function onStride(event) {
  var d = event.target.value;
  var n = d.getFloat32(0, true), dx = d.getFloat32(4, true), dy = d.getFloat32(8, true);
  var dt = d.getFloat32(16, true);
  run.heard = Date.now();
  if (run.last === undefined || n < run.last) {
    // Before the first read, or the Puck restarted: start counting from here
    run.last = n;
  } else if (n > run.last) {
    // New stride(s) - normally one; more if notifications were missed
    run.strides += n - run.last;
    run.last = n;
    run.length = Math.sqrt(dx * dx + dy * dy);
    run.cadence = dt > 0 ? 120 / dt : 0; // a stride is two steps
    run.distance += run.length;
    Bangle.buzz(BUZZ_MS);
  }
  // n == run.last: a heartbeat - nothing new
  if (session && session.phase == "running" && !session.menu) draw();
}

// ---------------------------------------------------------------- Connecting

/* Connect to the Puck, uploading the code first if it's not running it, and
 subscribe to its strides. */
function connect() {
  var s = session = { phase: "connect", done: false, line: "", lines: [] };
  Bangle.setUI({ mode: "custom", btn: function() { stop("Cancelled"); } });
  show(["Scanning for", "Puck.js...", "", "BTN: cancel"]);
  function check(x) { if (s.done) throw "Cancelled"; return x; }
  NRF.requestDevice({ filters: [{ namePrefix: "Puck.js" }], timeout: 20000 }).then(check).then(function(device) {
    s.device = device;
    device.on("gattserverdisconnected", function() { onDisconnect(s); });
    show(["Connecting to", device.name || "Puck.js", "", "BTN: cancel"]);
    return device.gatt.connect();
  }).then(check).then(function(gatt) {
    s.gatt = gatt;
    // Already running the code? Then it has the stride service - but is it
    // using these settings? If not, it's restarted with them
    return gatt.getPrimaryService(STRIDE_SERVICE).then(function() {
      return openConsole(s).then(function() { return ask(s, "footrunSettings"); }).then(sameSettings);
    }, function() { return false; });
  }).then(check).then(function(ready) {
    if (ready) return;
    return upload(s).then(check).then(function() {
      // The stride service only appears once the Puck's Bluetooth restarts
      show(["Restarting", "Puck Bluetooth", "", "BTN: cancel"]);
      s.reconnecting = true;
      s.gatt.disconnect();
      return new Promise(function(resolve) { setTimeout(resolve, 2500); });
    }).then(check).then(function() {
      return s.device.gatt.connect();
    }).then(check).then(function(gatt) {
      s.gatt = gatt;
      s.reconnecting = false;
      s.console = false; // a new connection
    });
  }).then(function() {
    return s.gatt.getPrimaryService(STRIDE_SERVICE);
  }).then(check).then(function(service) {
    return service.getCharacteristic(STRIDE_CHAR);
  }).then(check).then(function(c) {
    c.on("characteristicvaluechanged", onStride);
    return c.startNotifications().then(function() { return c.readValue(); }).then(syncStrides);
  }).then(check).then(function() {
    return s.gatt.getPrimaryService(NUS).then(function(nus) { return nus.getCharacteristic(NUS_RX); })
      .then(function(rx) { s.rx = rx; }, function() {}); // for "Stop Puck" - not essential
  }).then(check).then(function() {
    s.phase = "running";
    Bangle.setUI({ mode: "custom", btn: showRunMenu });
    s.timer = setInterval(function() { if (!s.menu) draw(); }, 1000);
    draw();
  }).catch(function(e) {
    if (session == s && !s.done) stop("Error: " + e);
  });
}

// Open the Puck's console (its Nordic UART), once per connection
function openConsole(s) {
  if (s.console) return Promise.resolve();
  var nus;
  return s.gatt.getPrimaryService(NUS).then(function(service) {
    nus = service;
    return nus.getCharacteristic(NUS_TX);
  }).then(function(tx) {
    tx.on("characteristicvaluechanged", function(e) { onText(s, e.target.value); });
    return tx.startNotifications();
  }).then(function() {
    return nus.getCharacteristic(NUS_RX);
  }).then(function(rx) {
    s.rx = rx;
    s.console = true;
  });
}

/* Put footrun.js and footrun.bin in the Puck's Storage (if they're not
 already there) and run the code, with the settings. */
function upload(s) {
  // The files for the Puck: { js, jsCrc, bin (base64), binCrc } - only loaded
  // when needed, as they're ~15kB
  var puck = require("footrun.puck");
  show(["Starting", "code on Puck", "", "BTN: cancel"]);
  return openConsole(s).then(function() {
    // Stop whatever the Puck is running (\x10 = don't echo the line back)
    return send(s, "\x10reset();\n");
  }).then(function() {
    return new Promise(function(resolve) { setTimeout(resolve, 1500); });
  }).then(function() {
    return ask(s, "E.CRC32(require('Storage').read('footrun.js')||'')+' '+E.CRC32(require('Storage').read('footrun.bin')||'')");
  }).then(function(reply) {
    var crcs = reply.split(" "), cmds = [], i, len;
    if (+crcs[1] != puck.binCrc) {
      // In chunks of 384 bytes = 512 base64 characters
      len = atob(puck.bin).length;
      for (i = 0; i < puck.bin.length; i += 512)
        cmds.push("require('Storage').write('footrun.bin',atob('" + puck.bin.substr(i, 512) + "')," +
                  (i / 4 * 3) + (i ? "" : "," + len) + ");");
    }
    if (+crcs[0] != puck.jsCrc) {
      for (i = 0; i < puck.js.length; i += 384)
        cmds.push("require('Storage').write('footrun.js'," + JSON.stringify(puck.js.substr(i, 384)) + "," +
                  i + (i ? "" : "," + puck.js.length) + ");");
    }
    // One at a time, showing progress
    var k = 0;
    function next() {
      if (s.done) throw "Cancelled";
      if (k >= cmds.length) return;
      show(["Uploading", "code to Puck", Math.round(100 * k / cmds.length) + "%", "BTN: cancel"]);
      return send(s, "\x10" + cmds[k++] + "\n").then(next);
    }
    return next();
  }).then(function() {
    return ask(s, "E.CRC32(require('Storage').read('footrun.js'))+' '+E.CRC32(require('Storage').read('footrun.bin'))");
  }).then(function(reply) {
    if (reply != puck.jsCrc + " " + puck.binCrc) throw "the code didn't arrive intact - try again";
    s.lines = [];
    return send(s, "\x10global.FR_SETTINGS=" + JSON.stringify(puckSettings()) + ";\n" +
                   "\x10eval(require('Storage').read('footrun.js'));\n");
  }).then(function() {
    return new Promise(function(resolve) { setTimeout(resolve, 2000); });
  }).then(function() {
    var err = s.lines.filter(function(l) { return /Error|ERROR|MEMORY/.test(l); })[0];
    if (err) throw "Puck: " + err;
  });
}

// Console text from the Puck, a line at a time
function onText(s, dv) {
  for (var i = 0; i < dv.byteLength; i++) {
    var c = dv.getUint8(i);
    if (c == 10 || c == 13) {
      var l = s.line.replace(/^[>\s]+/, "").trim();
      if (l) s.lines.push(l);
      if (s.lines.length > 20) s.lines.shift();
      s.line = "";
    } else if (c >= 32 && c < 127 && s.line.length < 100) s.line += String.fromCharCode(c);
  }
}

// Send text to the Puck's console, 20 bytes (one Bluetooth write) at a time
function send(s, text) {
  var i = 0;
  function next() {
    if (i >= text.length) return;
    var chunk = text.substr(i, 20);
    i += 20;
    return s.rx.writeValue(chunk).then(next);
  }
  return Promise.resolve().then(next);
}

// The result of a JS expression on the Puck, as a string
function ask(s, expr) {
  var tag = "FR" + Math.floor(Math.random() * 1e6);
  s.lines = [];
  return send(s, "\x10Bluetooth.println('" + tag + " '+(" + expr + "))\n").then(function() {
    return new Promise(function(resolve, reject) {
      var t0 = Date.now(), iv = setInterval(function() {
        for (var i = 0; i < s.lines.length; i++)
          if (s.lines[i].indexOf(tag + " ") == 0) { clearInterval(iv); return resolve(s.lines[i].substr(tag.length + 1)); }
        if (Date.now() - t0 > 5000 || s.done) { clearInterval(iv); reject("no reply from the Puck"); }
      }, 100);
    });
  });
}

function onDisconnect(s) {
  if (session != s || s.done || s.reconnecting) return;
  if (s.phase == "running") {
    stop("Puck disconnected - it keeps counting; tap to reconnect", true);
  } else stop("Puck disconnected");
}

// ---------------------------------------------------------------- Stopping

// End the session; with the Puck left running, or restarted (stopPuck)
function stop(reason, canReconnect, stopPuck) {
  var s = session;
  if (!s || s.done) return;
  s.done = true;
  if (s.timer) clearInterval(s.timer);
  function drop() { try { if (s.gatt && s.gatt.connected) s.gatt.disconnect(); } catch (e) {} }
  if (stopPuck && s.rx && s.gatt && s.gatt.connected)
    send(s, "\x03\x10Puck.accelOff();load();\n").then(drop, drop); // back to its saved program
  else drop();
  var lines = [reason, ""];
  if (run.strides) lines.push(run.strides + " strides", run.distance.toFixed(0) + " m", "");
  lines.push(canReconnect ? "Tap: reconnect" : "", "BTN: menu");
  show(lines);
  Bangle.setUI({ mode: "custom", btn: showMenu, touch: canReconnect ? connect : showMenu });
}

function showRunMenu() {
  var s = session;
  s.menu = true;
  E.showMenu({
    "": { title: "Foot Run", back: function() { s.menu = false; E.showMenu(); Bangle.setUI({ mode: "custom", btn: showRunMenu }); draw(); } },
    "Disconnect (Puck keeps counting)": function() { E.showMenu(); stop("Disconnected", true); },
    "Stop Puck": function() { E.showMenu(); stop("Puck stopped", false, true); },
    "Reset counts": function() {
      run.strides = 0; run.distance = 0; run.length = 0; run.cadence = 0;
      s.menu = false; E.showMenu(); Bangle.setUI({ mode: "custom", btn: showRunMenu }); draw();
    }
  });
}

function showMenu() {
  Bangle.setUI();
  E.showMenu({
    "": { title: "Foot Run", back: load },
    "Start": function() { E.showMenu(); connect(); },
    "Settings": function() { eval(require("Storage").read("footrun.settings.js"))(showMenu); },
    "Reset counts": function() { run.strides = 0; run.distance = 0; run.length = 0; run.cadence = 0; showMenu(); }
  });
}

// Don't leave the Puck connected if the app is closed
E.on("kill", function() {
  if (session && session.gatt && session.gatt.connected) try { session.gatt.disconnect(); } catch (e) {}
});

Bangle.loadWidgets();
Bangle.drawWidgets();
showMenu();
