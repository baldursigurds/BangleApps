# Foot Recorder

Records raw accelerometer and gyro samples from a Puck.js v2 to a file on
the Bangle.js - eg. with the Puck on a shoe, to look at running data later.

Nothing needs to be installed on the Puck - the app sends it all the code
it runs. The Puck needs to be a Puck.js v2 with firmware 2v29 or later
(for multi-byte `Puck.accelRd`), with its Bluetooth UART enabled (the
default).

What happens to the Puck:

1. The app connects to its Bluetooth UART (its JavaScript console) and
   sends `reset()`, stopping whatever program it's running.
2. It sets a disconnect handler that turns the accelerometer off and runs
   `load()` - restarting the Puck and its saved program, if it has one.
3. It sends the streaming code (the same as `raw_stream.js` in the folder
   above, but at the chosen rate): ±16 g, ±2000 deg/s, streamed straight
   away.
4. When recording ends (or is cancelled) the app sends the same restart
   before disconnecting - and if the connection is just lost, the Puck's
   disconnect handler restarts it anyway.

So afterwards the Puck is running its saved program again (eg. a saved
`accel_fifo.js`). Anything uploaded to it but **not saved** with `save()`
is lost, just as when uploading from the Web IDE.

## Usage

* **Record** - finds the nearest Puck.js advertising its name, connects,
  uploads the code, and records for the chosen length. The button stops
  early (what's been recorded is kept). Make sure the Puck isn't connected
  to anything else (eg. the Espruino Web IDE), as then it isn't advertising.
  If the Puck reports an error (eg. firmware too old) it's shown.
* **Length** - 5 s to 5 minutes, default 10 s.
* **Rate** - samples per second: 12.5, 26, 52, 104, 208 (default), 416 or
  833 Hz. Each sample is 12 bytes, so 416 Hz is ~5 kB/s and 833 Hz ~10 kB/s
  over Bluetooth, handled in JavaScript at both ends - the higher rates may
  not keep up. If frames are lost, it's shown at the end of recording and
  counted in the `.json`.

Length and Rate are remembered in `footrec.json`, and can also be set in
the Settings app (Apps → Foot Recorder).
* **Recordings** - lists recordings, newest first; select one to see its
  graph or delete it. After recording, tap the screen to see its graph.

## Graph

Accelerometer (g) at the top, gyro (deg/s) below, with x, y, z in red,
green, blue, each scaled to the largest value shown (the +/- label).
Each pixel column shows the minimum to maximum of the samples it covers, so
short spikes (eg. foot strikes) aren't lost.

* **Tap** - zoom in x2, keeping the tapped moment under your finger
* **Drag** sideways - pan
* **Button** - zoom out, and at full view go back

To keep redrawing quick, each column reads at most 12 samples, spread
across it - so in long recordings (over ~10 s per screen) a short spike can
be missed; zoom in to see every sample.

Recording also stops if the Puck disconnects, or no data arrives for 3 s.

## Files

Each recording is two files in Storage, in the same format as `raw_plot.py`
recordings:

* `footrec.MMDDhhmmss.bin` - int16 little-endian `gx,gy,gz,ax,ay,az` per
  sample, at the chosen rate. Gyro 0.07 deg/s per unit, accelerometer
  0.000488 g per unit.
* `footrec.MMDDhhmmss.json` - when it was recorded, the rate (`rate_hz`),
  the scales, how many
  `samples` were recorded, and any lost frames / FIFO overruns (gaps in
  time that aren't marked in the `.bin`).

The `.bin` is created at the full size of the chosen length (Storage files
can't grow), so if recording stopped early only the first `samples`
samples are valid - the rest is unwritten. `plot_recording.py` reads the
`.json` and ignores the unwritten part.

To get them onto a computer, use the Espruino Web IDE: connect to the
Bangle, open the Storage menu (the icon of 3 discs), and save both files
for a recording. Then:

    python3 plot_recording.py footrec.1024183005.bin

or in Python:

    raw = numpy.fromfile("footrec.1024183005.bin", dtype="<i2").reshape(-1, 6)
    raw = raw[:json.load(open("footrec.1024183005.json"))["samples"]]

## Notes

* Uses the Bangle's Bluetooth as a central at the same time as any phone
  connection, at ~2.5 kB/s.
* Each 10 s takes ~25 kB of storage.
* Only tested on paper / in simulation so far - not yet on a real Bangle.js 2.
