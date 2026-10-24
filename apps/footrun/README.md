# Foot Run

Stride by stride feedback from a Puck.js v2 on your shoe: each stride's
length and cadence on the screen, and a short buzz for every stride.

The Puck runs `accel_fifo.js` (in the folder above): foot-mounted inertial
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
* **Button** - a menu:
  * **Disconnect** - the Puck keeps counting; reconnect later with Start
    (strides it counted meanwhile are added to the count, but not their
    distance)
  * **Stop Puck** - restarts the Puck, back to its saved program
  * **Reset counts**

If the connection is lost, tap the screen to reconnect.

Make sure the Puck isn't connected to anything else (eg. the Espruino Web
IDE), as then it isn't advertising.

## How the code gets onto the Puck

`accel_fifo.js` does its sums in compiled C (`E.compiledC`), which is
normally compiled online by the Espruino Web IDE - a watch can't do that.
So `build.py` compiles it ahead of time, exactly as Espruino's compiler
does, and builds two files for the Puck, which it puts into `app.js`:

* `footrun.js` - `accel_fifo.js` with comments and spaces removed (~8 kB)
* `footrun.bin` - the compiled C (~5 kB). It doesn't use any Espruino
  functions, so it works on any firmware version.

The app writes these to the Puck's Storage (only if their CRC32s don't
match what's there), and runs `eval(require("Storage").read("footrun.js"))`.
Code run from Storage keeps its functions in flash rather than RAM - sent
as one line, `accel_fifo.js` doesn't fit in the Puck's memory. They take
~13 kB of the Puck's flash (and so a little of what `footrec` can record).

The stride service it sets up only appears once the Puck's Bluetooth
restarts, which is why the app disconnects and connects again.

After changing `accel_fifo.js`, run `python3 build.py` (needs
`arm-none-eabi-gcc`, `node` and EspruinoTools: `npm install -g espruino`),
then reinstall the app.

## The Puck's settings

Everything is as in `accel_fifo.js` - including its magnetometer
calibration (`puckmag.json` on the Puck, made with `calibrateMag()` from the
Web IDE), which is used if it's there, so the stride directions are relative
to magnetic north. The stride lengths don't need it.
