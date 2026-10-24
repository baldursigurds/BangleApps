// Foot Run settings: in the Settings app (Apps -> Foot Run), and footrun's own
// Settings menu. Kept in footrun.json; footrun sends them to the Puck as
// FR_SETTINGS (see puck.js), restarting its code if they've changed.
(function(back) {
  var FILE = "footrun.json";
  var RATES = [52, 104, 208, 416, 833];
  var DETECTORS = ["sample", "window"];
  var s = Object.assign({ rate: 208, detector: "sample", accTh: 0.3, gyroTh: 1.5, stanceMin: 10, window: 5,
                          vcheck: false, vmax: 1 }, require("Storage").readJSON(FILE, true) || {});
  function save() { require("Storage").writeJSON(FILE, s); }
  // Menus add steps in floating point (0.05*6 = 0.30000000000000004)
  function round(v, step) { return +(Math.round(v / step) * step).toFixed(2); }
  E.showMenu({
    "": { title: "Foot Run", back: back },
    // How often the Puck samples the accelerometer and gyro
    "Rate": {
      value: Math.max(0, RATES.indexOf(s.rate)), min: 0, max: RATES.length - 1,
      format: function(v) { return RATES[v] + "Hz"; },
      onchange: function(v) { s.rate = RATES[v]; save(); }
    },
    // Foot down (still) when acceleration is within Acc thresh of 1g and rotation under Gyro thresh -
    // "sample": for Still N samples in a row; "window": summed (squared) over the last Window W samples
    "Detector": {
      value: Math.max(0, DETECTORS.indexOf(s.detector)), min: 0, max: 1,
      format: function(v) { return DETECTORS[v]; },
      onchange: function(v) { s.detector = DETECTORS[v]; save(); }
    },
    "Acc thresh": {
      value: s.accTh, min: 0.05, max: 1, step: 0.05,
      format: function(v) { return v.toFixed(2) + "g"; },
      onchange: function(v) { s.accTh = round(v, 0.05); save(); }
    },
    "Gyro thresh": {
      value: s.gyroTh, min: 0.25, max: 5, step: 0.25,
      format: function(v) { return v.toFixed(2) + "r/s"; },
      onchange: function(v) { s.gyroTh = round(v, 0.25); save(); }
    },
    "Still N": {
      value: s.stanceMin, min: 1, max: 40,
      format: function(v) { return v + (s.detector == "sample" ? "" : " (unused)"); },
      onchange: function(v) { s.stanceMin = v; save(); }
    },
    "Window W": {
      value: s.window, min: 2, max: 32,
      format: function(v) { return v + (s.detector == "window" ? "" : " (unused)"); },
      onchange: function(v) { s.window = v; save(); }
    },
    // Not down while the speed integrated so far is over Max speed
    "Speed check": {
      value: !!s.vcheck,
      onchange: function(v) { s.vcheck = v; save(); }
    },
    "Max speed": {
      value: s.vmax, min: 0.25, max: 5, step: 0.25,
      format: function(v) { return v.toFixed(2) + "m/s"; },
      onchange: function(v) { s.vmax = round(v, 0.25); save(); }
    }
  });
})
