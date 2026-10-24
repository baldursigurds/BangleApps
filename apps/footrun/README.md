# Foot Run

Stride by stride feedback from a Puck.js v2 on your shoe: each stride's
length and cadence on the screen, and a short buzz for every stride.

The Puck runs `puck.js`/`puck.c` (footrun's copy of `accel_fifo.js` in the
folder above): foot-mounted inertial
navigation that integrates the accelerometer during each swing and resets
the velocity whenever the foot is on the ground, giving the displacement of
every stride. One stride is that foot's movement from landing to landing -
two steps.

Nothing needs to be installed on the Puck - the app sends it the code. It
needs to be a Puck.js v2 with firmware 2v29 or later.

## Usage

* **Start** - connects to the nearest Puck.js. If it isn't already running
  the code, the app uploads it (the first time this takes ~15 s; after that
  the files are already on the Puck and it's quick), the Puck restarts its
  Bluetooth, and the app connects again.
* Then the screen shows the strides counted, the last stride's length, the
  cadence (steps per minute) and the total distance. Every stride gives a
  short buzz. The Puck also sends a "nothing new" message every second -
  these don't buzz; they just show it's still connected (the bottom line
  says if nothing's been heard for 3 s).
* The Puck blinks **green three times** when it starts running the code,
  and **blue once** for every stride it measures (not for the "nothing new"
  messages) - so you can see it's working without looking at the watch.
* **Button** - a menu:
  * **Disconnect** - the Puck keeps counting; reconnect later with Start
    (strides it counted meanwhile are added to the count, but not their
    distance)
  * **Stop Puck** - restarts the Puck, back to its saved program
  * **Reset counts**

If the connection is lost, tap the screen to reconnect.

## Settings

In the app's **Settings** menu, or the Settings app (Apps → Foot Run). The
running screen shows the ones in use (eg. `208Hz still 10 v<1`). If the Puck
is already running with different settings, Start restarts its code with
the new ones (which takes a few seconds; its counts start again).

* **Rate** - samples per second: 52, 104, 208 (default), 416 or 833 Hz.
* **Detector** - how the Puck decides the foot is down (still), which is
  when it resets the velocity (and ends a stride). With a = acceleration (g)
  and w = rotation rate (rad/s):
  * **sample** (default) - | |a| - 1 | < **Acc thresh** and |w| < **Gyro
    thresh**, for **Still N** samples in a row (default 10)
  * **window** - over the last **Window W** samples (default 5, at most
    32), the sum of (|a| - 1)² < W × Acc thresh², and the sum of |w|² <
    W × Gyro thresh² (like the SHOE detector)
* **Acc thresh** - default 0.3 g. **Gyro thresh** - default 1.5 rad/s.
* **Speed check** (default off) - the foot isn't counted as down while the
  speed integrated so far is over **Max speed** (default 1 m/s): a foot
  that's really still has only drifted that fast, so it's a false stillness
  in mid-swing. As drift could still exceed it, the check is dropped once
  the detector has said down for ~60 ms in a row (otherwise the velocity
  would never be reset and every stride after would merge into one).

Still N and Window W are in samples, so the same number is a different time
at a different rate (10 samples is 48 ms at 208 Hz, 24 ms at 416 Hz).

If strides get merged (about double length and time, half the cadence),
the foot isn't being found to be still - raise the thresholds, or lower
N/W. If a stride is split into two short ones, the foot is wrongly found to
be still in mid-swing - lower the thresholds, raise N/W, or turn on Speed
check.

Make sure the Puck isn't connected to anything else (eg. the Espruino Web
IDE), as then it isn't advertising.

## How the code gets onto the Puck

The Puck's code is in two files here:

* `puck.js` - footrun's own copy of `accel_fifo.js` (in the folder above),
  plus the LED blinks. `PUCK_C` marks where the compiled C goes.
* `puck.c` - its compiled part. It starts with the functions `puck.js` can
  call, as for Espruino's `E.compiledC`: `// void process(int, int, int)`.

`puck.c` would normally be compiled online by the Espruino Web IDE - a
watch can't do that. So `build.py` compiles it ahead of time, exactly as
Espruino's compiler does, puts the `E.nativeCall()`s for its functions in
place of `PUCK_C`, removes comments and spaces from `puck.js`, and writes
`puckcode.js`: the module `footrun.puck` (the App Loader installs it with the
app), exporting the two files for the Puck and their CRC32s:

* `footrun.js` - the code (~8 kB)
* `footrun.bin` - the compiled C (~5 kB). It doesn't use any Espruino
  functions, so it works on any firmware version.

The app only loads the module when it needs to upload. It writes the files
to the Puck's Storage (only if their CRC32s don't match what's there), and
runs `eval(require("Storage").read("footrun.js"))`. Code run from Storage
keeps its functions in flash rather than RAM - sent as one line, the code
doesn't fit in the Puck's memory. They take ~13 kB of the Puck's flash (and
so a little of what `footrec` can record).

The stride service it sets up only appears once the Puck's Bluetooth
restarts, which is why the app disconnects and connects again.

After changing `puck.js` or `puck.c`, run `python3 build.py` (needs
`arm-none-eabi-gcc`, `node` and EspruinoTools: `npm install -g espruino`),
then reinstall the app. Changes to `../accel_fifo.js` aren't picked up -
copy them into `puck.js`/`puck.c`.

## Magnetometer calibration

Everything is as in `puck.js` (as in `accel_fifo.js`) - including its magnetometer
calibration (`puckmag.json` on the Puck, made with `calibrateMag()` from the
Web IDE), which is used if it's there, so the stride directions are relative
to magnetic north. The stride lengths don't need it.
