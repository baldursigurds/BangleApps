# Foot Recorder

Records raw accelerometer and gyro samples from a Puck.js v2 to a file on
the Bangle.js - eg. with the Puck on a shoe, to look at running data later.

Nothing needs to be installed on the Puck - the app sends it all the code
it runs. The Puck needs to be a Puck.js v2 with firmware 2v29 or later
(for multi-byte `Puck.accelRd`), with its Bluetooth UART enabled (the
default).

The Puck records into its own flash first, and only then are the samples
sent to the Bangle - so Bluetooth never has to keep up with the sensor. The
recording stays on the Puck until the Bangle has all of it, so a transfer
that fails can simply be done again. What happens to the Puck:

1. The app connects to its Bluetooth UART (its JavaScript console) and
   sends `reset()`, stopping whatever program it's running.
2. It sets a disconnect handler that turns the accelerometer off and runs
   `load()` - restarting the Puck and its saved program, if it has one
   (but not while it's recording).
3. It sends the code, and asks the Puck if it has a recording waiting from
   before (eg. a transfer that failed) - if so, that's fetched first.
4. The Puck makes a file in its flash for the samples asked for - or as
   many as fit - and records at the chosen rate (±16 g, ±2000 deg/s),
   through a small RAM buffer that's written to flash about once a second.
   When it's done it says so.
5. The app fetches the samples, 8 frames of 20 samples at a time, each
   checked with a CRC32; missing or damaged frames are asked for again.
   Only when it has every frame does it tell the Puck to delete its copy.
6. The app sends the same restart before disconnecting.

If the connection is lost **while recording**, the Puck carries on, saves
the recording to its flash, and then restarts itself. If it's lost **while
transferring**, the recording is still on the Puck. Either way, **Fetch from
Puck** (or the next **Record**) gets it.

Afterwards the Puck is running its saved program again (eg. a saved
`accel_fifo.js`). Anything uploaded to it but **not saved** with `save()`
is lost, just as when uploading from the Web IDE.

## Usage

* **Record** - finds the nearest Puck.js advertising its name, connects,
  uploads the code, records, and fetches the samples. While recording, the
  button stops early and saves what's been recorded. Make sure the Puck
  isn't connected to anything else (eg. the Espruino Web IDE), as then it
  isn't advertising. If the Puck reports an error (eg. firmware too old)
  it's shown.
* **Fetch from Puck** - gets a recording left on the Puck (eg. after a
  transfer failed, or the connection was lost while recording). If the Puck
  is still recording, it waits for it to finish.
* **Length** - 5 s to 5 minutes, default 10 s. The Puck records as much of
  this as fits in its free flash - shown as "(all that fits)". With nothing
  else saved on it, a Puck.js v2 (firmware 2v29) has ~94 kB, which is 7850
  samples of 12 bytes (measured):

  | Rate | 12.5 Hz | 26 Hz | 52 Hz | 104 Hz | 208 Hz | 416 Hz | 833 Hz |
  |---|---|---|---|---|---|---|---|
  | Longest recording | 5 min (setting) | 5 min | 2.5 min | 75 s | 37 s | 18 s | 9 s |

  Programs saved on the Puck take some of that space.
* **Rate** - samples per second: 12.5, 26, 52, 104, 208 (default), 416 or
  833 Hz.
* **Recordings** - lists recordings, newest first; select one to see its
  graph or delete it. After recording, tap the screen to see its graph.

Length and Rate are remembered in `footrec.json`, and can also be set in
the Settings app (Apps → Foot Recorder).

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
  the scales, how many `samples` were recorded, any frames that never
  arrived (`missing_frames`, of `frame_samples` samples each - left as
  0xFF bytes in their place in the `.bin`, so timing is kept), and how many
  FIFO overruns there were on the Puck (each a gap in time that isn't
  marked in the `.bin` - rare now, as the Puck does nothing else while
  recording).

The `.bin` is created at the size the Puck has room for (Storage files
can't grow), so if recording was stopped early only the first `samples`
samples are valid - the rest is unwritten. `plot_recording.py` and
`fetch_footrec.py` read the `.json` and ignore the unwritten part.

To get them onto a computer, run `fetch_footrec.py` (in the folder above),
which downloads any new ones into `foot_data/`. Or use the Espruino Web IDE:
connect to the Bangle, open the Storage menu (the icon of 3 discs), and save both files
for a recording. Then:

    python3 plot_recording.py footrec.1024183005.bin

or in Python:

    raw = numpy.fromfile("footrec.1024183005.bin", dtype="<i2").reshape(-1, 6)
    raw = raw[:json.load(open("footrec.1024183005.json"))["samples"]]

## Notes

* Uses the Bangle's Bluetooth as a central at the same time as any phone
  connection. Transfers run at ~6 kB/s, so a 37 s recording at 208 Hz
  (94 kB) takes ~16 s.
* On the Puck the recording is kept in `frdata.bin` / `frdata.json` until
  it's been fetched.
* The Puck side is tested on a real Puck.js v2: 37 s at 208 Hz with no FIFO
  overruns, a transfer cut off and done again, and the connection lost while
  recording. The whole app is tested in simulation, including heavy
  Bluetooth loss.
