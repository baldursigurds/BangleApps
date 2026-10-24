// Foot Recorder settings, in the Settings app (Apps -> Foot Recorder).
// The same settings as in the app's own menu, kept in footrec.json.
(function(back) {
  var FILE = "footrec.json";
  var LENGTHS = [5, 10, 20, 30, 60, 120, 300]; // seconds - as in app.js
  var RATES = [12.5, 26, 52, 104, 208, 416, 833]; // Hz - as in app.js
  var settings = Object.assign({ seconds: 10, rate: 208 }, require("Storage").readJSON(FILE, true) || {});
  function save() { require("Storage").writeJSON(FILE, settings); }
  var l = LENGTHS.indexOf(settings.seconds), r = RATES.indexOf(settings.rate);
  E.showMenu({
    "": { title: "Foot Recorder", back: back },
    "Length": {
      value: l < 0 ? 1 : l, min: 0, max: LENGTHS.length - 1,
      format: function(v) { return LENGTHS[v] + "s"; },
      onchange: function(v) { settings.seconds = LENGTHS[v]; save(); }
    },
    "Rate": {
      value: r < 0 ? 4 : r, min: 0, max: RATES.length - 1,
      format: function(v) { return RATES[v] + "Hz"; },
      onchange: function(v) { settings.rate = RATES[v]; save(); }
    }
  });
})
