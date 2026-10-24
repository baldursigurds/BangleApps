// void reset(int)
// void process(int, int, int)
// int fitEllipsoid(int, int, int)

/* footrun: the compiled part of the code the Puck runs - foot-mounted
 inertial navigation (orientation, double integration of each swing, zero
 velocity updates). build.py compiles it as Espruino's E.compiledC would.

 The lines above are the functions puck.js can call, with their types (as
 for E.compiledC: they must be the first lines). Compiled with -nostdlib and
 -fpreprocessed: there's no C library (so no sqrtf/memset - and only single
 precision float maths), and no preprocessor (#define etc.). */
// Packed so access is safe whatever the alignment of Espruino's buffers.
// Shared with JS as a Float32Array - keep the S_* indices there in sync.
struct __attribute__((packed)) State {
  float q0, q1, q2, q3;  // 0  orientation, sensor -> earth (x north, z up)
  float beta;            // 4  Madgwick gain, applied only while the foot is down
  float mx, my, mz;      // 5  latest magnetometer reading, calibrated, IMU axes
  float useMag;          // 8  1 = correct heading with mx,my,mz
  float ax, ay, az;      // 9  last accelerometer sample in g (for mag calibration)
  float accTh, gyroTh;   // 12 foot down if ||a|-1g| < accTh (g) and |w| < gyroTh (rad/s)
  float vx, vy, vz;      // 14 velocity during the swing (m/s, earth axes)
  float px, py, pz;      // 17 displacement since the foot left the ground (m)
  float swingTime;       // 20 duration of the current swing (s)
  float stillCount;      // 21 consecutive samples meeting the stance thresholds
  float bx, by, bz;      // 22 gyro bias, learned while still (rad/s)
  float strides;         // 25 stride count
  float sx, sy, sz;      // 26 last stride displacement (m, earth axes)
  float strideTime;      // 29 last stride duration (s), 0 = first after standing
  float sinceStride;     // 30 time since the last stride ended (s)
  float tx, ty, tz;      // 31 total position (m, earth axes)
  float distance;        // 34 total horizontal distance (m)
  float saturated;       // 35 samples where the accelerometer or gyro clipped
  float stance;          // 36 1 while the foot is down
  // Settings (set by puck.js)
  float dt;              // 37 sample period (s)
  float stanceMin;       // 38 sample detector: still samples in a row for the foot to be down
  float window;          // 39 window detector: samples summed (0 = use the sample detector)
  float vmax;            // 40 not down while the integrated speed is over this (m/s; 0 = no check)...
  float vmaxOverride;    // 41 ...unless the stillness test has said down this many samples in a row
  float biasAfter;       // 42 samples still before learning the gyro bias
  // Stillness detection state
  float imuRun;          // 43 samples in a row the stillness test has said down
  float ringPos;         // 44 window detector: next slot in the rings,
  float ringCount;       // 45 how many samples are in them (up to window),
  float accSum, gyroSum; // 46 and the sums of (|a|-1g)² and |w|² in them
  float accRing[32];     // 48 (|a|-1g)² of the last samples
  float gyroRing[32];    // 80 |w|² of the last samples
};                       // 112 floats
struct __attribute__((packed)) Sample { short gx, gy, gz, ax, ay, az; };

// No libm here (-nostdlib), so use fast inverse square root + 2 Newton steps
float invSqrt(float x) {
  union { float f; int i; } u;
  u.f = x;
  u.i = 0x5f3759df - (u.i >> 1);
  float y = u.f;
  y = y * (1.5f - 0.5f * x * y * y);
  y = y * (1.5f - 0.5f * x * y * y);
  return y;
}
float absf(float x) { return x < 0.0f ? -x : x; }
float sqrtf_(float x) { return x > 0.0f ? x * invSqrt(x) : 0.0f; }
bool clipped(short v) { return v > 32700 || v < -32700; }

void reset(int stateAddr) {
  State *s = (State*)stateAddr;
  s->q0 = 1.0f; s->q1 = 0.0f; s->q2 = 0.0f; s->q3 = 0.0f;
  s->vx = s->vy = s->vz = 0.0f;
  s->px = s->py = s->pz = 0.0f;
  s->swingTime = 0.0f; s->stillCount = 0.0f;
  s->strides = 0.0f; s->sx = s->sy = s->sz = 0.0f;
  s->strideTime = 0.0f; s->sinceStride = 0.0f;
  s->tx = s->ty = s->tz = 0.0f; s->distance = 0.0f;
  s->saturated = 0.0f;
  s->stance = 1.0f; // assume we start with the foot on the ground
  s->imuRun = 0.0f; s->ringPos = 0.0f; s->ringCount = 0.0f; s->accSum = 0.0f; s->gyroSum = 0.0f;
}

/* Foot-mounted inertial navigation over 'count' raw FIFO samples:
 * Orientation: the gyro is integrated all the time. While the foot is down
   (and only then - in the swing the accelerometer measures motion too) it's
   corrected towards gravity, and magnetic north if useMag, by Madgwick's
   MadgwickAHRSupdate() / MadgwickAHRSupdateIMU() gradient step.
 * Swing: acceleration is rotated to earth axes, gravity removed, and
   integrated to velocity then displacement.
 * Landing (zero velocity update): the true velocity is 0, so what's left
   is drift. Assuming it grew linearly through the swing, its effect on
   displacement is v*T/2 - remove it, and that's the stride.
 * Still for over biasAfter samples: learn the gyro bias.
 The foot is down (stillness) if, with a = acceleration (g), w = rotation
 rate (rad/s):
 * sample detector (window = 0): | |a|-1 | < accTh and |w| < gyroTh, for
   stanceMin samples in a row
 * window detector: over the last window samples, sum((|a|-1)²) <
   window * accTh² and sum(|w|²) < window * gyroTh² (like the SHOE detector)
 * and if vmax > 0, the speed integrated so far is under vmax - a foot
   that's really still has only drifted that fast. As drift could exceed
   vmax, the check is dropped once the test has said down for vmaxOverride
   samples in a row (or the velocity would never reset). */
void process(int dataAddr, int count, int stateAddr) {
  State *st = (State*)stateAddr;
  Sample *d = (Sample*)dataAddr;
  // +/-2000dps = 70 mdps/LSB, +/-16g = 0.488 mg/LSB (datasheet)
  const float GYRO_SCALE = 70e-3f * 3.14159265f / 180.0f;
  const float ACC_SCALE = 0.488e-3f;
  const float G = 9.80665f;
  const float BIAS_RATE = 0.005f;  // per sample
  const float MIN_SWING = 0.1f;    // shorter swings (shuffling) aren't counted as strides
  const int WMAX = 32;             // the most samples the window detector sums
  float dt = st->dt;
  int stanceMin = (int)st->stanceMin, biasAfter = (int)st->biasAfter;
  int window = (int)st->window;
  if (window > WMAX) window = WMAX;
  float vmax2 = st->vmax * st->vmax;
  int vmaxOverride = (int)st->vmaxOverride, imuRun = (int)st->imuRun;
  int ringPos = (int)st->ringPos, ringCount = (int)st->ringCount;
  float accSum = st->accSum, gyroSum = st->gyroSum;
  float q0 = st->q0, q1 = st->q1, q2 = st->q2, q3 = st->q3, beta = st->beta;
  float accTh = st->accTh, gyroTh2 = st->gyroTh * st->gyroTh;
  float vx = st->vx, vy = st->vy, vz = st->vz;
  float px = st->px, py = st->py, pz = st->pz;
  float bx = st->bx, by = st->by, bz = st->bz;
  float swingTime = st->swingTime, sinceStride = st->sinceStride;
  int stillCount = (int)st->stillCount;
  bool stance = st->stance > 0.5f;
  // Latest magnetometer reading, normalised once for the whole chunk
  float mx = st->mx, my = st->my, mz = st->mz;
  float mNorm = mx * mx + my * my + mz * mz;
  bool useMag = st->useMag > 0.5f && mNorm > 0.0f;
  if (useMag) {
    float r = invSqrt(mNorm);
    mx *= r; my *= r; mz *= r;
  }
  for (int n = 0; n < count; n++, d++) {
    if (clipped(d->gx) || clipped(d->gy) || clipped(d->gz) ||
        clipped(d->ax) || clipped(d->ay) || clipped(d->az)) st->saturated += 1.0f;
    float rgx = d->gx * GYRO_SCALE, rgy = d->gy * GYRO_SCALE, rgz = d->gz * GYRO_SCALE;
    float gx = rgx - bx, gy = rgy - by, gz = rgz - bz;
    float ax = d->ax * ACC_SCALE, ay = d->ay * ACC_SCALE, az = d->az * ACC_SCALE; // g
    float aNorm = ax * ax + ay * ay + az * az;
    float aMag = sqrtf_(aNorm);
    // Stillness (foot down) detection
    float accDev = aMag - 1.0f, w2 = gx * gx + gy * gy + gz * gz;
    if (absf(accDev) < accTh && w2 < gyroTh2) stillCount++;
    else stillCount = 0;
    bool imuDown;
    if (window > 0) {
      // Sums over the last window samples, kept up to date with rings
      float a2 = accDev * accDev;
      if (ringCount >= window) { accSum -= st->accRing[ringPos]; gyroSum -= st->gyroRing[ringPos]; }
      else ringCount++;
      st->accRing[ringPos] = a2; st->gyroRing[ringPos] = w2;
      accSum += a2; gyroSum += w2;
      if (++ringPos >= window) ringPos = 0;
      imuDown = accSum < ringCount * accTh * accTh && gyroSum < ringCount * gyroTh2;
    } else {
      imuDown = stillCount >= stanceMin;
    }
    imuRun = imuDown ? imuRun + 1 : 0;
    bool down = imuDown;
    // Velocity check: not still while moving faster than drift could explain
    if (down && vmax2 > 0.0f && vx * vx + vy * vy + vz * vz > vmax2 && imuRun < vmaxOverride) down = false;
    if (stillCount > biasAfter) { // still for a while - the gyro should read 0
      bx += BIAS_RATE * (rgx - bx); by += BIAS_RATE * (rgy - by); bz += BIAS_RATE * (rgz - bz);
    }

    // Orientation: rate of change of quaternion from gyroscope
    float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    float qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    float qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
    // Correct with the accelerometer (and magnetometer) only while the foot is down
    if (down && aNorm > 0.0f) {
      float r = 1.0f / aMag;
      float nax = ax * r, nay = ay * r, naz = az * r;
      float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
      float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;
      float s0, s1, s2, s3;
      if (useMag) {
        float _2q0mx = 2.0f * q0 * mx, _2q0my = 2.0f * q0 * my, _2q0mz = 2.0f * q0 * mz;
        float _2q1mx = 2.0f * q1 * mx, _2q0q2 = 2.0f * q0 * q2, _2q2q3 = 2.0f * q2 * q3;
        float q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
        float q1q2 = q1 * q2, q1q3 = q1 * q3, q2q3 = q2 * q3;
        // Reference direction of Earth's magnetic field
        float hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        float hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        float _2bx = sqrtf_(hx * hx + hy * hy);
        float _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        float _4bx = 2.0f * _2bx, _4bz = 2.0f * _2bz;
        // Gradient descent corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - nax) + _2q1 * (2.0f * q0q1 + _2q2q3 - nay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - nax) + _2q0 * (2.0f * q0q1 + _2q2q3 - nay) - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - naz) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - nax) + _2q3 * (2.0f * q0q1 + _2q2q3 - nay) - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - naz) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - nax) + _2q2 * (2.0f * q0q1 + _2q2q3 - nay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
      } else {
        float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1, _4q2 = 4.0f * q2;
        float _8q1 = 8.0f * q1, _8q2 = 8.0f * q2;
        // Gradient descent corrective step
        s0 = _4q0 * q2q2 + _2q2 * nax + _4q0 * q1q1 - _2q1 * nay;
        s1 = _4q1 * q3q3 - _2q3 * nax + 4.0f * q0q0 * q1 - _2q0 * nay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * naz;
        s2 = 4.0f * q0q0 * q2 + _2q0 * nax + _4q2 * q3q3 - _2q3 * nay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * naz;
        s3 = 4.0f * q1q1 * q3 - _2q1 * nax + 4.0f * q2q2 * q3 - _2q2 * nay;
      }
      float sNorm = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
      if (sNorm > 0.0f) {
        r = invSqrt(sNorm);
        qDot1 -= beta * s0 * r;
        qDot2 -= beta * s1 * r;
        qDot3 -= beta * s2 * r;
        qDot4 -= beta * s3 * r;
      }
    }
    // Integrate and normalise
    q0 += qDot1 * dt; q1 += qDot2 * dt; q2 += qDot3 * dt; q3 += qDot4 * dt;
    float r = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= r; q1 *= r; q2 *= r; q3 *= r;

    sinceStride += dt;
    if (!down) {
      if (stance) { // foot just left the ground - start a new swing
        stance = false;
        vx = vy = vz = 0.0f; px = py = pz = 0.0f; swingTime = 0.0f;
      }
      // Acceleration in earth axes (rotate by q), minus gravity, in m/s²
      float ex = (1.0f - 2.0f * (q2 * q2 + q3 * q3)) * ax + 2.0f * (q1 * q2 - q0 * q3) * ay + 2.0f * (q1 * q3 + q0 * q2) * az;
      float ey = 2.0f * (q1 * q2 + q0 * q3) * ax + (1.0f - 2.0f * (q1 * q1 + q3 * q3)) * ay + 2.0f * (q2 * q3 - q0 * q1) * az;
      float ez = 2.0f * (q1 * q3 - q0 * q2) * ax + 2.0f * (q2 * q3 + q0 * q1) * ay + (1.0f - 2.0f * (q1 * q1 + q2 * q2)) * az;
      ex *= G; ey *= G; ez = (ez - 1.0f) * G;
      // Trapezoidal integration to velocity and displacement
      float nvx = vx + ex * dt, nvy = vy + ey * dt, nvz = vz + ez * dt;
      px += 0.5f * (vx + nvx) * dt; py += 0.5f * (vy + nvy) * dt; pz += 0.5f * (vz + nvz) * dt;
      vx = nvx; vy = nvy; vz = nvz;
      swingTime += dt;
    } else if (!stance) {
      // Foot just landed: velocity should be 0, so what's left is drift.
      // If it grew linearly through the swing it added v*T/2 to displacement.
      stance = true;
      float h = 0.5f * swingTime;
      float dx = px - vx * h, dy = py - vy * h, dz = pz - vz * h;
      float dist = sqrtf_(dx * dx + dy * dy);
      st->tx += dx; st->ty += dy; st->tz += dz;
      st->distance += dist;
      if (swingTime >= MIN_SWING) {
        st->sx = dx; st->sy = dy; st->sz = dz;
        st->strideTime = sinceStride < 2.0f ? sinceStride : 0.0f;
        sinceStride = 0.0f;
        st->strides += 1.0f; // last, so JS sees a complete stride when this changes
      }
      vx = vy = vz = 0.0f;
    }
  }
  st->q0 = q0; st->q1 = q1; st->q2 = q2; st->q3 = q3;
  st->vx = vx; st->vy = vy; st->vz = vz;
  st->px = px; st->py = py; st->pz = pz;
  st->bx = bx; st->by = by; st->bz = bz;
  st->swingTime = swingTime; st->sinceStride = sinceStride;
  st->stillCount = (float)stillCount;
  st->stance = stance ? 1.0f : 0.0f;
  st->imuRun = (float)imuRun;
  st->ringPos = (float)ringPos; st->ringCount = (float)ringCount;
  st->accSum = accSum; st->gyroSum = gyroSum;
  if (count > 0) {
    d--;
    st->ax = d->ax * ACC_SCALE; st->ay = d->ay * ACC_SCALE; st->az = d->az * ACC_SCALE;
  }
}

// ------------------------------------------------ Magnetometer ellipsoid fit

struct __attribute__((packed)) Mag { short x, y, z; };
// Result: calibrated = W * (raw - offset), plus ellipsoid radii in raw counts
struct __attribute__((packed)) Fit { float offset[3], W[9], radii[3]; };


// Solve the n x n system A x = b in place (A row-major, x ends up in b) by
// Gaussian elimination with partial pivoting. Columns are scaled by
// 1/sqrt(A[i][i]) first, as x² and x terms differ in magnitude.
bool solve(float *A, float *b, int n) {
  float s[9];
  int i, j, k;
  for (i = 0; i < n; i++) {
    if (!(A[i * n + i] > 0.0f)) return false;
    s[i] = invSqrt(A[i * n + i]);
  }
  for (i = 0; i < n; i++) {
    for (j = 0; j < n; j++) A[i * n + j] *= s[i] * s[j];
    b[i] *= s[i];
  }
  for (k = 0; k < n; k++) {
    int p = k;
    for (i = k + 1; i < n; i++) if (absf(A[i * n + k]) > absf(A[p * n + k])) p = i;
    if (absf(A[p * n + k]) < 1e-6f) return false; // singular
    if (p != k) {
      for (j = 0; j < n; j++) { float t = A[k * n + j]; A[k * n + j] = A[p * n + j]; A[p * n + j] = t; }
      float t = b[k]; b[k] = b[p]; b[p] = t;
    }
    for (i = k + 1; i < n; i++) {
      float f = A[i * n + k] / A[k * n + k];
      for (j = k; j < n; j++) A[i * n + j] -= f * A[k * n + j];
      b[i] -= f * b[k];
    }
  }
  for (i = n - 1; i >= 0; i--) {
    float sum = b[i];
    for (j = i + 1; j < n; j++) sum -= A[i * n + j] * b[j];
    b[i] = sum / A[i * n + i];
  }
  for (i = 0; i < n; i++) b[i] *= s[i];
  return true;
}

// Eigen-decomposition of symmetric 3x3 a (row-major) by Jacobi rotations.
// a ends up diagonal (the eigenvalues), eigenvector k in column k of V.
void eigSym3(float *a, float *V) {
  int k;
  for (k = 0; k < 9; k++) V[k] = (k % 4 == 0) ? 1.0f : 0.0f;
  for (int sweep = 0; sweep < 20; sweep++) {
    if (a[1] * a[1] + a[2] * a[2] + a[5] * a[5] < 1e-14f * (a[0] * a[0] + a[4] * a[4] + a[8] * a[8])) break;
    for (int p = 0; p < 2; p++) for (int q = p + 1; q < 3; q++) {
      float apq = a[p * 3 + q];
      if (apq == 0.0f) continue;
      float theta = (a[q * 3 + q] - a[p * 3 + p]) / (2.0f * apq);
      float t = absf(theta) > 1e6f ? 0.5f / theta : // avoid theta² overflow
                (theta >= 0.0f ? 1.0f : -1.0f) / (absf(theta) + sqrtf_(theta * theta + 1.0f));
      float cs = invSqrt(t * t + 1.0f), sn = t * cs;
      for (k = 0; k < 3; k++) { // A = A J
        float akp = a[k * 3 + p], akq = a[k * 3 + q];
        a[k * 3 + p] = cs * akp - sn * akq; a[k * 3 + q] = sn * akp + cs * akq;
      }
      for (k = 0; k < 3; k++) { // A = J' A
        float apk = a[p * 3 + k], aqk = a[q * 3 + k];
        a[p * 3 + k] = cs * apk - sn * aqk; a[q * 3 + k] = sn * apk + cs * aqk;
      }
      for (k = 0; k < 3; k++) { // V = V J
        float vkp = V[k * 3 + p], vkq = V[k * 3 + q];
        V[k * 3 + p] = cs * vkp - sn * vkq; V[k * 3 + q] = sn * vkp + cs * vkq;
      }
    }
  }
}

/* Least squares fit of the ellipsoid  x'Ax + 2u'x = 1  (linear in its 9
 coefficients) to n raw magnetometer readings, then the affine map
 W(x - c) that moves it onto the unit sphere: centre c = -inv(A)u,
 shape M = A / (1 + c'Ac), W = sqrt(M) = V diag(sqrt(eigenvalues)) V'.
 The fit is done relative to the middle of the min/max range, scaled by
 the half-range, as the '= 1' form can't fit an ellipsoid through (or
 near) the origin. Returns 1 on success, 0 if the data doesn't fit an
 ellipsoid (eg. not enough directions covered).
 The attribute stops GCC turning the zeroing loops into memset() calls,
 as there's no C library to link against. */
__attribute__((optimize("no-tree-loop-distribute-patterns")))
int fitEllipsoid(int magAddr, int n, int fitAddr) {
  Mag *m = (Mag*)magAddr;
  Fit *fit = (Fit*)fitAddr;
  int i, j, k, s;
  if (n < 10) return 0;
  float lo[3], hi[3], ref[3], sc = 0.0f;
  lo[0] = hi[0] = m[0].x; lo[1] = hi[1] = m[0].y; lo[2] = hi[2] = m[0].z;
  for (s = 1; s < n; s++) {
    if (m[s].x < lo[0]) lo[0] = m[s].x; if (m[s].x > hi[0]) hi[0] = m[s].x;
    if (m[s].y < lo[1]) lo[1] = m[s].y; if (m[s].y > hi[1]) hi[1] = m[s].y;
    if (m[s].z < lo[2]) lo[2] = m[s].z; if (m[s].z > hi[2]) hi[2] = m[s].z;
  }
  for (k = 0; k < 3; k++) { ref[k] = (lo[k] + hi[k]) * 0.5f; sc += (hi[k] - lo[k]) * (1.0f / 6.0f); }
  if (!(sc > 0.0f)) return 0;
  float isc = 1.0f / sc;
  // Each sample gives a row [x²,y²,z²,2yz,2xz,2xy,2x,2y,2z] of D,
  // solve (D'D) coeffs = D'1
  float N[81], e[9], v[9];
  for (i = 0; i < 81; i++) N[i] = 0.0f;
  for (i = 0; i < 9; i++) e[i] = 0.0f;
  for (s = 0; s < n; s++) {
    float x = (m[s].x - ref[0]) * isc, y = (m[s].y - ref[1]) * isc, z = (m[s].z - ref[2]) * isc;
    v[0] = x * x; v[1] = y * y; v[2] = z * z;
    v[3] = 2.0f * y * z; v[4] = 2.0f * x * z; v[5] = 2.0f * x * y;
    v[6] = 2.0f * x; v[7] = 2.0f * y; v[8] = 2.0f * z;
    for (i = 0; i < 9; i++) {
      e[i] += v[i];
      for (j = 0; j <= i; j++) N[i * 9 + j] += v[i] * v[j];
    }
  }
  for (i = 0; i < 9; i++) for (j = 0; j < i; j++) N[j * 9 + i] = N[i * 9 + j];
  if (!solve(N, e, 9)) return 0;
  // A = [a h g; h b f; g f c], u = [p q r]
  float A[9], Ac[9], c[3];
  A[0] = e[0]; A[1] = e[5]; A[2] = e[4];
  A[3] = e[5]; A[4] = e[1]; A[5] = e[3];
  A[6] = e[4]; A[7] = e[3]; A[8] = e[2];
  for (i = 0; i < 9; i++) Ac[i] = A[i];
  c[0] = -e[6]; c[1] = -e[7]; c[2] = -e[8];
  if (!solve(Ac, c, 3)) return 0; // centre = -inv(A) u
  float kk = 1.0f;
  for (i = 0; i < 3; i++) for (j = 0; j < 3; j++) kk += c[i] * A[i * 3 + j] * c[j];
  if (!(kk > 0.0f)) return 0;
  for (i = 0; i < 9; i++) A[i] /= kk; // A is now M
  float V[9];
  eigSym3(A, V);
  float val[3];
  val[0] = A[0]; val[1] = A[4]; val[2] = A[8];
  if (!(val[0] > 0.0f && val[1] > 0.0f && val[2] > 0.0f)) return 0;
  // W = V diag(sqrt(val)) V', divided by sc to work on raw counts
  for (i = 0; i < 3; i++) for (j = 0; j < 3; j++) {
    float sum = 0.0f;
    for (k = 0; k < 3; k++) sum += V[i * 3 + k] * sqrtf_(val[k]) * V[j * 3 + k];
    fit->W[i * 3 + j] = sum * isc;
  }
  for (k = 0; k < 3; k++) {
    fit->offset[k] = ref[k] + c[k] * sc;
    fit->radii[k] = sc * invSqrt(val[k]);
  }
  return 1;
}
