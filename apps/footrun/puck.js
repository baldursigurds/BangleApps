/* footrun's code for the Puck.js v2 on a shoe (its own copy of ../accel_fifo.js,
 with LED blinks added at the end). The compiled part is in puck.c, and
 build.py turns the two into the module footrun uploads from.

 Puck.js v2 on a shoe - LSM6DS3TR-C accelerometer + gyro FIFO (208 Hz unless set),
 plus the magnetometer, with foot-mounted inertial navigation in compiled C:
 orientation (Madgwick AHRS), double integration of acceleration during each
 swing, and a zero velocity update whenever the foot is on the ground, giving
 the displacement of every stride (one stride = two steps, as the sensor is
 on one foot).

 The IMU fills its own FIFO, and every POLL_MS we pull everything in it with
 multi-byte Puck.accelRd(reg, cnt) reads (firmware 2v29+), then hand each
 chunk to the C code which runs the AHRS on every sample.

 The magnetometer runs at MAG_HZ. On each new reading we first drain the
 FIFO (so earlier samples use the previous reading), then hand the new
 reading to the AHRS, which uses the latest one for heading correction. It
 needs calibrating once - run calibrateMag() and turn Puck.js in every
 direction. Until then (or with no calibration saved) only accel+gyro are
 used and heading is relative to start-up, drifting slowly.

 footrun uploads this to the Puck (see build.py) - it isn't uploaded from the
 Web IDE, as the compiled part (puck.c) is compiled by build.py.

 Docs: https://www.espruino.com/Reference#l_Puck_accelRd
       https://www.espruino.com/Reference#l_Puck_magOn
       https://www.espruino.com/InlineC
       https://espruino.com/datasheets/LSM6DS3TR-C.pdf (sections 5.5, 9.4-9.8, 9.53-9.58)
*/

/* Settings: footrun sets FR_SETTINGS before running this (see its Settings
 menu); anything missing gets the default.
   rate       samples/s: 12.5, 26, 52, 104, 208, 416 or 833
   detector   "sample" or "window" - how the foot is found to be down (still):
   accTh      ...if | |a| - 1g | < accTh (g)
   gyroTh     ...and |rotation| < gyroTh (rad/s)
   stanceMin  ...for stanceMin samples in a row ("sample"), or
   window     ...over the last window samples, sum((|a|-1g)²) < window*accTh²
              and sum(|rotation|²) < window*gyroTh² ("window", at most 32)
   vmax       if > 0, the foot isn't down while the speed integrated so far is
              over vmax (m/s) - except once the test above has said down for
              ~60ms in a row (in case a swing drifted over vmax)
 Tune if strides are missed (merged into one long stride) or the foot is
 wrongly found to be down during the swing. */
var FR = typeof FR_SETTINGS == "object" ? FR_SETTINGS : {};
var RATES = [12.5, 26, 52, 104, 208, 416, 833];
var RATE = RATES.indexOf(FR.rate) >= 0 ? FR.rate : 208;
var ODR = RATES.indexOf(RATE) + 1; // the LSM6DS3TR-C's code for RATE
var ACC_TH = FR.accTh > 0 ? FR.accTh : 0.3;       // g
var GYRO_TH = FR.gyroTh > 0 ? FR.gyroTh : 1.5;    // rad/s (86 deg/s)
var STANCE_MIN = FR.stanceMin > 0 ? FR.stanceMin : 10;
var WINDOW = FR.detector == "window" ? Math.min(32, FR.window > 0 ? FR.window : 5) : 0;
var VMAX = FR.vmax > 0 ? FR.vmax : 0;             // m/s, 0 = no velocity check
// The FIFO holds 2047 words (6 a sample): 1.6s at 208Hz, 0.4s at 833Hz
var POLL_MS = RATE > 416 ? 100 : RATE > 208 ? 150 : 250;
var CHUNK_SAMPLES = 20; // samples per Puck.accelRd call (it allocates, and uses the stack)
var BETA = 0.3;         // Madgwick gain (rad/s) - only applied while the foot is down,
                        // which is brief when running, so higher than usual
var MAG_HZ = 20;        // magnetometer rate: 80, 40, 20, 10, 5, 2.5, 1.25 or 0.63 Hz
// The Earth's field points downwards in the northern hemisphere, upwards in
// the southern. Used by calibrateMag() to work out the magnetometer's axes.
var NORTHERN_HEMISPHERE = true;
// Magnetometer calibration file in flash (Puck.js has no external flash -
// this is the Storage area of the nRF52's internal flash, which survives
// resets and battery changes). Written by calibrateMag(), used at startup.
var MAG_CAL_FILE = "puckmag.json";

// LSM6DS3TR-C registers
var FIFO_CTRL1 = 0x06, FIFO_CTRL2 = 0x07, FIFO_CTRL3 = 0x08,
    FIFO_CTRL4 = 0x09, FIFO_CTRL5 = 0x0A, INT1_CTRL = 0x0D,
    CTRL1_XL = 0x10, CTRL2_G = 0x11, CTRL6_C = 0x15, CTRL7_G = 0x16,
    FIFO_STATUS1 = 0x3A, FIFO_DATA_OUT_L = 0x3E;
var FIFO_CTRL5_ON = (ODR << 3) | 6; // FIFO at RATE, continuous mode

// FIFO stores a gyro data set then an accel data set: gx,gy,gz,ax,ay,az
var WORDS_PER_SAMPLE = 6, BYTES_PER_SAMPLE = 12;

// The compiled puck.c - a placeholder build.py replaces with E.nativeCall()s for its
// functions, which run its machine code from the Puck's Storage (footrun.bin)
var c = PUCK_C;

// State shared with C - see struct State for the layout
var S_BETA = 4, S_MAG = 5, S_USEMAG = 8, S_ACC = 9, S_ACC_TH = 12, S_GYRO_TH = 13,
    S_BIAS = 22, S_STRIDES = 25, S_STRIDE = 26, S_STRIDE_TIME = 29,
    S_TOTAL = 31, S_DISTANCE = 34, S_SATURATED = 35, S_STANCE = 36,
    S_DT = 37, S_STANCE_MIN = 38, S_WINDOW = 39, S_VMAX = 40, S_VMAX_OVERRIDE = 41, S_BIAS_AFTER = 42;
var state = new Float32Array(112);
var stateAddr = E.getAddressOf(state, true);
// C reads samples from here - must be flat memory so it has an address
var sampleBuf = new Uint8Array(CHUNK_SAMPLES * BYTES_PER_SAMPLE);
var sampleAddr = E.getAddressOf(sampleBuf, true);
if (!stateAddr || !sampleAddr) throw new Error("Couldn't get flat buffers for compiled code");

var count = 0;    // samples processed so far
var overruns = 0; // FIFO overruns (samples lost) - see status()

// Called with Uint8Array d holding n raw samples from the FIFO
function onSample(d, n) {
  sampleBuf.set(d);
  c.process(sampleAddr, n, stateAddr);
  count += n;
}

// Heading (yaw) in degrees from the AHRS quaternion. With the magnetometer
// this is counter-clockwise from magnetic north, otherwise from start-up.
function getHeading() {
  var q0 = state[0], q1 = state[1], q2 = state[2], q3 = state[3];
  return Math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)) * 180 / Math.PI;
}

function poll() {
  // FIFO_STATUS1..4: unread word count, flags, position in the 6 word pattern
  var st = Puck.accelRd(FIFO_STATUS1, 4);
  if (st[1] & 0x10) return; // FIFO_EMPTY
  if (st[1] & 0x40) {
    // OVER_RUN - FIFO full. DIFF_FIFO is only 11 bits so may not be able
    // to report a full FIFO, so reset it (which also realigns the pattern)
    overruns++;
    Puck.accelWr(FIFO_CTRL5, 0);
    Puck.accelWr(FIFO_CTRL5, FIFO_CTRL5_ON);
    return;
  }
  var words = st[0] | ((st[1] & 7) << 8);
  var pattern = st[2] | ((st[3] & 3) << 8);
  if (pattern) { // not at the start of a gyro+accel sample - discard to realign
    var skip = WORDS_PER_SAMPLE - pattern;
    if (words < skip) return;
    Puck.accelRd(FIFO_DATA_OUT_L, skip * 2);
    words -= skip;
  }
  var samples = (words / WORDS_PER_SAMPLE) | 0;
  while (samples > 0) {
    var n = Math.min(samples, CHUNK_SAMPLES);
    var d;
    try {
      d = Puck.accelRd(FIFO_DATA_OUT_L, n * BYTES_PER_SAMPLE);
    } catch (e) {
      // Throws if there's no memory for the result
    }
    if (d === undefined) {
      // Too big for the stack or no memory - try smaller reads, and if even
      // one sample fails give up until the next poll
      if (CHUNK_SAMPLES == 1) return;
      CHUNK_SAMPLES = CHUNK_SAMPLES >> 1;
      continue;
    }
    onSample(d, n);
    samples -= n;
  }
}

// ---------------------------------------------------------------- Magnetometer

// Calibration {offset:[x,y,z], matrix:[9 numbers, row-major]}: a calibrated
// reading in IMU axes is matrix * (raw - offset). Loaded from MAG_CAL_FILE
// by start(), replaced (and saved) only when calibrateMag() succeeds.
// (Not loaded here: the IDE sends code to the REPL a statement at a time, so
// functions defined further down don't exist yet.)
var magCal;
var magCalPrev; // calibration in use before calibrateMag(), restored if it fails
var magCalData; // data being collected while calibrateMag() runs

function loadMagCal() {
  var cal = require("Storage").readJSON(MAG_CAL_FILE, true);
  // Ignore missing or damaged files, and the older min/max format
  if (!cal || !cal.offset || !cal.matrix || cal.offset.length != 3 || cal.matrix.length != 9 ||
      cal.offset.concat(cal.matrix).some(function(v) { return typeof v != "number" || !isFinite(v); }))
    return undefined;
  return cal;
}

// Forget the calibration (deletes MAG_CAL_FILE) - heading becomes relative
function clearMagCal() {
  require("Storage").erase(MAG_CAL_FILE);
  magCal = undefined;
  state[S_USEMAG] = 0;
  print("Magnetometer calibration deleted");
}
var MAG_CAL_SAMPLES = 300; // max samples kept - 15s at 20Hz, 2.7kB

function onMag(m) {
  poll(); // bring the AHRS up to now using the previous reading first
  if (magCalData) addMagCalSample(m);
  if (!magCal) return;
  var o = magCal.offset, T = magCal.matrix;
  var x = m.x - o[0], y = m.y - o[1], z = m.z - o[2];
  state[S_MAG] = T[0] * x + T[1] * y + T[2] * z;
  state[S_MAG + 1] = T[3] * x + T[4] * y + T[5] * z;
  state[S_MAG + 2] = T[6] * x + T[7] * y + T[8] * z;
  state[S_USEMAG] = 1;
}

/* Turn Puck.js slowly through every direction (each face pointing up and
 down, and spinning round) for 'seconds'.

 Hard and soft iron: the readings lie on an ellipsoid rather than a sphere
 centred on 0. fitEllipsoid() in the C code fits it by least squares and
 finds the affine map W(x - c) that moves it onto the unit sphere.

 Axis mapping: the magnetometer's axes aren't documented relative to the
 accelerometer's, so try every axis order and sign and pick the one where
 the calibrated field keeps the most constant angle to 'down' (from the
 accelerometer, using only samples where Puck.js was fairly still).
*/
function calibrateMag(seconds) {
  if (magCalData) return print("Already calibrating");
  seconds = seconds || 15;
  // Stop using the old calibration while collecting, but keep it in case
  // this calibration fails
  magCalPrev = magCal;
  magCal = undefined;
  state[S_USEMAG] = 0;
  magCalData = {
    n: 0,                                    // samples kept
    still: 0,                                // ...of which Puck.js was still
    m: new Int16Array(MAG_CAL_SAMPLES * 3),  // raw magnetometer
    d: new Int8Array(MAG_CAL_SAMPLES * 3)    // 'down' unit vector * 127, or 0 if moving
  };
  print("Magnetometer calibration: slowly turn Puck.js in every direction for " + seconds + "s");
  setTimeout(finishMagCal, seconds * 1000);
}

function addMagCalSample(m) {
  var cal = magCalData;
  if (cal.n >= MAG_CAL_SAMPLES) return;
  var i = cal.n * 3;
  cal.m.set([m.x, m.y, m.z], i);
  cal.n++;
  // For the axis mapping, note 'down' when Puck.js is fairly still so the
  // accelerometer reads gravity (it measures 'up' at rest, so down is -a)
  var ax = state[S_ACC], ay = state[S_ACC + 1], az = state[S_ACC + 2];
  var a = Math.sqrt(ax * ax + ay * ay + az * az);
  if (a < 0.9 || a > 1.1) return; // in g
  cal.d[i] = -127 * ax / a;
  cal.d[i + 1] = -127 * ay / a;
  cal.d[i + 2] = -127 * az / a;
  cal.still++;
}

function finishMagCal() {
  var cal = magCalData, i, j, k;
  magCalData = undefined;
  function fail(why) {
    magCal = magCalPrev;
    print("Magnetometer calibration failed - " + why +
          (magCal ? ". Keeping the previous calibration" : ""));
  }
  if (cal.n < 100 || cal.still < 50)
    return fail("turn Puck.js through more directions (" + cal.n + " samples, " + cal.still + " still)");

  // Ellipsoid fit in C: fit = offset[3], W[9], radii[3]
  var fit = new Float32Array(15);
  var magAddr = E.getAddressOf(cal.m, true), fitAddr = E.getAddressOf(fit, true);
  if (!magAddr || !fitAddr) return fail("couldn't get flat buffers for compiled code");
  if (!c.fitEllipsoid(magAddr, cal.n, fitAddr))
    return fail("couldn't fit an ellipsoid - turn Puck.js through more directions");
  var off = [fit[0], fit[1], fit[2]], W = new Float32Array(fit.buffer, 12, 9), radiiRaw = new Float32Array(fit.buffer, 48, 3);

  // Calibrated unit field for each still sample, and how far it is off the
  // unit sphere. C[j*3+k] = sum of down[j] * unit field[k]
  var C = new Float64Array(9), err = 0;
  for (i = 0; i < cal.n * 3; i += 3) {
    if (i % 60 == 0) poll();
    var x = cal.m[i] - off[0], y = cal.m[i + 1] - off[1], z = cal.m[i + 2] - off[2];
    var u = [W[0] * x + W[1] * y + W[2] * z, W[3] * x + W[4] * y + W[5] * z, W[6] * x + W[7] * y + W[8] * z];
    var len = Math.sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
    err += (len - 1) * (len - 1);
    if (!cal.d[i] && !cal.d[i + 1] && !cal.d[i + 2]) continue; // moving
    for (j = 0; j < 3; j++)
      for (k = 0; k < 3; k++)
        C[j * 3 + k] += cal.d[i + j] * u[k] / len;
  }
  // For each axis order the best signs make every term positive, so the
  // order with the largest sum lines the field up best with 'down'
  var perms = [[0, 1, 2], [1, 2, 0], [2, 0, 1], [0, 2, 1], [1, 0, 2], [2, 1, 0]];
  var best, bestIdx, bestScore = -1;
  perms.forEach(function(p, idx) {
    var score = Math.abs(C[p[0]]) + Math.abs(C[3 + p[1]]) + Math.abs(C[6 + p[2]]);
    if (score > bestScore) { bestScore = score; best = p; bestIdx = idx; }
  });
  var hemi = NORTHERN_HEMISPHERE ? 1 : -1;
  var sign = best.map(function(k, j) { return C[j * 3 + k] * hemi < 0 ? -1 : 1; });
  // Fold the axis mapping into W: IMU axis j = sign[j] * calibrated axis best[j]
  var T = [];
  for (j = 0; j < 3; j++) for (k = 0; k < 3; k++) T[j * 3 + k] = sign[j] * W[best[j] * 3 + k];
  magCal = { offset: off, matrix: T };
  try {
    require("Storage").writeJSON(MAG_CAL_FILE, magCal);
  } catch (e) {
    print("Warning: couldn't save " + MAG_CAL_FILE + " (" + e + ") - calibration will be lost on restart");
  }

  // Average angle between the field and horizontal - should be close to the
  // magnetic inclination (dip) where you are
  var dip = Math.asin(E.clip(bestScore / (127 * cal.still), -1, 1)) * 180 / Math.PI;
  var radii = [radiiRaw[0].toFixed(0), radiiRaw[1].toFixed(0), radiiRaw[2].toFixed(0)];
  print("Magnetometer calibrated from " + cal.n + " samples (" + cal.still + " still)");
  print(" centre " + off.map(function(v) { return v.toFixed(0); }).join(",") +
        ", ellipsoid radii " + radii.join(",") +
        ", RMS distance from unit sphere " + (100 * Math.sqrt(err / cal.n)).toFixed(1) + "%");
  print(" IMU x,y,z = mag " + best.map(function(k, j) { return (sign[j] < 0 ? "-" : "+") + "xyz"[k]; }).join(",") +
        ", dip " + dip.toFixed(1) + " deg");
  // Both chips use right-handed axes, so the mapping should be a rotation
  var det = (bestIdx < 3 ? 1 : -1) * sign[0] * sign[1] * sign[2];
  if (det < 0) print("Warning: axis mapping is a reflection - calibration is probably poor, try again");
}

// ---------------------------------------------------------------- Start

function start() {
  Puck.accelOn(RATE);             // power up, accel+gyro at RATE, +/-4g, 250dps, BDU, auto-increment
  // Older firmware ignores cnt and returns a single number
  if (!(Puck.accelRd(0x0F, 1) instanceof Uint8Array)) {
    Puck.accelOff();
    throw new Error("Firmware " + process.env.VERSION + " too old - Puck.accelRd(reg,cnt) needs 2v29+");
  }
  Puck.accelWr(INT1_CTRL, 0);     // no data-ready IRQ, so no 'accel' event per sample
  // A running foot sees well over 4g and 250 deg/s, so use the full ranges,
  // and high-performance mode (lower noise, more power) for integration
  Puck.accelWr(CTRL1_XL, (ODR << 4) | 0x04);   // RATE, +/-16g
  Puck.accelWr(CTRL2_G, (ODR << 4) | 0x0C);    // RATE, +/-2000 deg/s
  Puck.accelWr(CTRL6_C, 0x00);    // accelerometer high-performance mode
  Puck.accelWr(CTRL7_G, 0x00);    // gyro high-performance mode
  Puck.accelWr(FIFO_CTRL5, 0);    // bypass mode - clears the FIFO
  Puck.accelWr(FIFO_CTRL1, 0);    // no threshold
  Puck.accelWr(FIFO_CTRL2, 0);
  Puck.accelWr(FIFO_CTRL3, 0x09); // gyro and accelerometer in FIFO, no decimation
  Puck.accelWr(FIFO_CTRL4, 0);
  c.reset(stateAddr);
  state[S_BETA] = BETA;
  state[S_ACC_TH] = ACC_TH;
  state[S_GYRO_TH] = GYRO_TH;
  state[S_DT] = 1 / RATE;
  state[S_STANCE_MIN] = STANCE_MIN;
  state[S_WINDOW] = WINDOW;
  state[S_VMAX] = VMAX;
  state[S_VMAX_OVERRIDE] = Math.max(3, Math.round(0.06 * RATE)); // ~60ms
  state[S_BIAS_AFTER] = Math.round(0.5 * RATE);                  // 0.5s
  // The settings in use, for footrun to check
  global.footrunSettings = JSON.stringify({ rate: RATE, accTh: ACC_TH, gyroTh: GYRO_TH,
    detector: WINDOW ? "window" : "sample", stanceMin: STANCE_MIN, window: WINDOW, vmax: VMAX });
  Puck.accelWr(FIFO_CTRL5, FIFO_CTRL5_ON);
  setInterval(poll, POLL_MS);
  Puck.on('mag', onMag);
  Puck.magOn(MAG_HZ);
  magCal = loadMagCal();
  if (magCal) print("Using magnetometer calibration from " + MAG_CAL_FILE);
  else print("Magnetometer not calibrated - run calibrateMag() and turn Puck.js in every direction");
}

// Bluetooth: one characteristic, STRIDE_CHAR, holding the last stride as 5
// little-endian Float32s: stride count, dx, dy, dz (m, earth axes: x magnetic
// north if calibrated, z up), duration (s, 0 = first stride after standing).
// It's notified on each new stride, and resent every HEARTBEAT_MS when
// nothing is happening (same count) so the receiver knows we're alive.
// 16 bit UUIDs so the service fits in the advertising packet with the name.
// Nothing is print()ed while running, as the console moves to Bluetooth when
// something connects - use status() from the IDE instead.
var STRIDE_SERVICE = "BCDE";
var STRIDE_CHAR = "ABCE";
var CHECK_MS = 100;      // how often to check for a new stride
var HEARTBEAT_MS = 1000;
// View onto the C state, so always the latest stride
var strideBytes = new Uint8Array(state.buffer, S_STRIDES * 4, 20);
var lastStrides = 0, lastSent = 0;

function setupBluetooth() {
  var s = {};
  s[STRIDE_SERVICE] = {};
  s[STRIDE_SERVICE][STRIDE_CHAR] = {
    value: strideBytes,
    maxLen: 20,
    readable: true,
    notify: true,
    description: "Stride n,dx,dy,dz,dt (float32)"
  };
  NRF.setServices(s, { advertise: [STRIDE_SERVICE], uart: true });
}

function checkStride() {
  var now = Date.now();
  var isNew = state[S_STRIDES] != lastStrides;
  if (!isNew && now - lastSent < HEARTBEAT_MS) return;
  lastStrides = state[S_STRIDES];
  lastSent = now;
  var u = {};
  u[STRIDE_SERVICE] = {};
  u[STRIDE_SERVICE][STRIDE_CHAR] = { value: strideBytes, notify: true };
  try {
    NRF.updateServices(u);
  } catch (e) {
    // setServices while connected (eg. to the IDE) only takes effect after
    // disconnecting - until then there's nothing to update
  }
}

// Print a summary - call from the IDE
function status() {
  var dx = state[S_STRIDE], dy = state[S_STRIDE + 1], dt = state[S_STRIDE_TIME];
  print(count + " samples, " + overruns + " FIFO overruns, " + state[S_SATURATED] +
        " clipped samples (+/-16g or 2000 deg/s)");
  print("Heading " + getHeading().toFixed(1) + (state[S_USEMAG] ? " (magnetic)" : " (relative)") +
        ", foot " + (state[S_STANCE] ? "down" : "moving"));
  print("Settings " + footrunSettings);
  print(state[S_STRIDES] + " strides, " + state[S_DISTANCE].toFixed(1) + "m. Last " +
        Math.sqrt(dx * dx + dy * dy).toFixed(2) + "m" +
        (dt ? " in " + dt.toFixed(2) + "s (" + (120 / dt).toFixed(0) + " steps/min)" : ""));
}

start();
setupBluetooth();
setInterval(checkStride, CHECK_MS);

// footrun: blink green three times once running (LED1 red, LED2 green, LED3 blue)...
digitalPulse(LED2, 1, [120, 150, 120, 150, 120]);
// ...and blue once for every stride (the count kept in state)
var footrunStrides = state[S_STRIDES];
setInterval(function() {
  if (state[S_STRIDES] != footrunStrides) {
    footrunStrides = state[S_STRIDES];
    digitalPulse(LED3, 1, 80);
  }
}, 100);
