/* Delta robot controller with WASD + SPACE/Z control over Serial
   - w/a/s/d move in X/Y (forward/back/left/right)
   - SPACE (press space then Enter) moves UP
   - z moves DOWN
   - Uppercase W/A/S/D/Z moves a larger step
   - Also supports: "x y z" absolute coordinates and "TEST"
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ------ Geometry (from your message) ------
const float f_mm  = 120.0;   // top plate triangle side
const float e_mm  = 60.0;    // end effector triangle side
const float rf_mm = 100.0;   // upper arm length
const float re_mm = 200.0;   // lower arm length

// ------ Servo / PCA9685 setup ------
const int PWM_FREQ = 50; // Hz for servos
const uint8_t SERVO_CH0 = 0;
const uint8_t SERVO_CH1 = 1;
const uint8_t SERVO_CH2 = 2;

// Typical hobby servo pulse limits (microseconds) - tune to your servos
const int SERVO_US_MIN = 700;
const int SERVO_US_MAX = 2300;

// Per-servo trim (microseconds)
const int SERVO_TRIM0 = 0;
const int SERVO_TRIM1 = 0;
const int SERVO_TRIM2 = 0;

// Movement steps
const float STEP_SMALL = 5.0f;   // mm for lowercase wasd
const float STEP_LARGE = 15.0f;  // mm for uppercase WASD

// Safe Z limits (tweak for your robot)
const float Z_MAX = -20.0f;   // highest (near top)
const float Z_MIN = -300.0f;  // lowest (down)


// Helpers
int usToTicks(int us) {
  float period_us = 1000000.0f / PWM_FREQ;
  float ticks = ((float)us / period_us) * 4096.0f;
  if (ticks < 0) ticks = 0;
  if (ticks > 4095) ticks = 4095;
  return (int)roundf(ticks);
}

int angleToUs(float angleDeg, int trim_us=0) {
  if (angleDeg < 0.0f) angleDeg = 0.0f;
  if (angleDeg > 180.0f) angleDeg = 180.0f;
  float us = SERVO_US_MIN + (angleDeg / 180.0f) * (SERVO_US_MAX - SERVO_US_MIN);
  us += trim_us;
  return (int)roundf(us);
}

void setServoAngle(uint8_t channel, float angleDeg, int trim_us) {
  int us = angleToUs(angleDeg, trim_us);
  int ticks = usToTicks(us);
  pwm.setPWM(channel, 0, ticks);
}

// ---- Delta kinematics ----
const float sqrt3 = 1.7320508075688772935;
const float pi = 3.14159265358979323846;
const float sin120 = sqrt3/2.0;
const float cos120 = -0.5;

bool delta_calcAngleYZ_reliable(float x0, float y0, float z0, float &theta) {
  float y1 = - (0.5f * 0.57735026919f * f_mm); // f * (sqrt(3)/6)
  float y0p = y0 - (0.5f * 0.57735026919f * e_mm);
  float a = (x0*x0 + y0p*y0p + z0*z0 + rf_mm*rf_mm - re_mm*re_mm - y1*y1) / (2.0f * z0);
  float b = (y1 - y0p) / z0;
  float disc = rf_mm*rf_mm*(b*b + 1) - (a + b*y1)*(a + b*y1);
  if (disc < 0.0f) return false;
  float yj = (y1 - a*b - sqrt(disc)) / (b*b + 1.0f);
  float zj = a + b*yj;
  theta = atan2(-zj, y1 - yj) * 180.0f / pi + 90.0f;
  return true;
}

bool delta_calcInverse(float x0, float y0, float z0, float &theta1, float &theta2, float &theta3) {
  if (!delta_calcAngleYZ_reliable(x0, y0, z0, theta1)) return false;
  // arm2 rotate -120
  float x1 = x0 * cos120 + y0 * sin120;
  float y1 = -x0 * sin120 + y0 * cos120;
  if (!delta_calcAngleYZ_reliable(x1, y1, z0, theta2)) return false;
  // arm3 rotate +120
  float x2 = x0 * cos120 - y0 * sin120;
  float y2 = x0 * sin120 + y0 * cos120;
  if (!delta_calcAngleYZ_reliable(x2, y2, z0, theta3)) return false;
  return true;
}

// ---- Serial helpers ----
String readSerialRaw() {
  if (!Serial.available()) return "";
  // read until newline (or timeout)
  String s = Serial.readStringUntil('\n'); // does not block forever if no data
  // keep raw (do NOT trim here because space-only lines would be lost)
  return s;
}

// parse three floats from trimmed copy. returns count parsed (0..3)
int parseThreeFloatsTrimmed(const String &trimmed, float &outx, float &outy, float &outz) {
  if (trimmed.length() == 0) return 0;
  char buf[80];
  trimmed.toCharArray(buf, sizeof(buf));
  for (size_t i = 0; i < sizeof(buf); ++i) if (buf[i] == ',') buf[i] = ' ';
  char *tok = strtok(buf, " \t\r\n");
  int idx = 0;
  float vals[3];
  while (tok != NULL && idx < 3) {
    vals[idx] = atof(tok);
    idx++;
    tok = strtok(NULL, " \t\r\n");
  }
  if (idx < 3) return idx;
  outx = vals[0];
  outy = vals[1];
  outz = vals[2];
  return 3;
}

// ---- Position state ----
float curX = 0.0f;
float curY = 0.0f;
float curZ = -150.0f; // start down (home)

void moveTo(float x, float y, float z) {
  float t1,t2,t3;
  if (!delta_calcInverse(x, y, z, t1, t2, t3)) {
    Serial.print(F("Point unreachable: "));
    Serial.print(x); Serial.print(' '); Serial.print(y); Serial.print(' '); Serial.println(z);
    return;
  }
  // bounds check angles
  if (t1 < 0 || t1 > 180 || t2 < 0 || t2 > 180 || t3 < 0 || t3 > 180) {
    Serial.print(F("Warning: angles outside 0-180 deg (may be mechanical). Angles: "));
    Serial.print(t1); Serial.print(' '); Serial.print(t2); Serial.print(' '); Serial.println(t3);
  } else {
    Serial.print(F("Angles (deg): "));
    Serial.print(t1); Serial.print(' ');
    Serial.print(t2); Serial.print(' ');
    Serial.println(t3);
  }
  // command servos
  setServoAngle(SERVO_CH0, t1, SERVO_TRIM0);
  setServoAngle(SERVO_CH1, t2, SERVO_TRIM1);
  setServoAngle(SERVO_CH2, t3, SERVO_TRIM2);
  // update state
  curX = x; curY = y; curZ = z;
  Serial.print(F("Moved to: "));
  Serial.print(curX); Serial.print(' ');
  Serial.print(curY); Serial.print(' ');
  Serial.println(curZ);
}

void printHelp() {
  Serial.println(F("Controls:"));
  Serial.println(F("  w/a/s/d  -> move forward/left/back/right (small step)"));
  Serial.println(F("  W/A/S/D  -> larger step"));
  Serial.println(F("  SPACE    -> move UP (press Space then Enter)"));
  Serial.println(F("  z        -> move DOWN"));
  Serial.println(F("  x y z    -> jump to absolute coordinates (e.g. 0 0 -150)"));
  Serial.println(F("  TEST     -> demo"));
  Serial.print(F("Current pos: ")); Serial.print(curX); Serial.print(' '); Serial.print(curY); Serial.print(' '); Serial.println(curZ);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println(F("Delta controller starting..."));

  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ);

  // Home to 90 deg (but keep curX/Y/Z as defined)
  setServoAngle(SERVO_CH0, 90.0f, SERVO_TRIM0);
  setServoAngle(SERVO_CH1, 90.0f, SERVO_TRIM1);
  setServoAngle(SERVO_CH2, 90.0f, SERVO_TRIM2);
  Serial.println(F("Servos homed to 90 deg"));
  printHelp();
}

void loop() {
  String raw = readSerialRaw();
  if (raw.length() == 0) return;

  // raw may contain trailing \r. We'll keep raw to detect a line that is only a space.
  // Create trimmed version for parsing multi-token input:
  String trimmed = raw;
  trimmed.trim();

  // handle TEST command
  String upTrim = trimmed;
  upTrim.toUpperCase();
  if (upTrim == "TEST") {
    Serial.println(F("Running TEST move"));
    float tx=0, ty=0, tz=-150;
    moveTo(tx,ty,tz);
    delay(1500);
    // back to home (slightly up)
    moveTo(0,0,-120);
    delay(800);
    moveTo(0,0,curZ);
    Serial.println(F("TEST finished."));
    return;
  }

  // If trimmed empty but raw contains space(s), interpret as SPACE command (move up)
  if (trimmed.length() == 0 && raw.indexOf(' ') >= 0) {
    // SPACE pressed
    float step = STEP_SMALL;
    float newZ = curZ + step;
    if (newZ > Z_MAX) newZ = Z_MAX;
    moveTo(curX, curY, newZ);
    return;
  }

  // If trimmed length == 1 treat as single-key command (w/a/s/d/z etc.)
  if (trimmed.length() == 1) {
    char c = trimmed.charAt(0);
    // determine step size by case
    bool large = (c >= 'A' && c <= 'Z');
    char lower = tolower(c);
    float step = large ? STEP_LARGE : STEP_SMALL;

    if (lower == 'w') {
      moveTo(curX, curY + step, curZ);
      return;
    } else if (lower == 's') {
      moveTo(curX, curY - step, curZ);
      return;
    } else if (lower == 'a') {
      moveTo(curX - step, curY, curZ);
      return;
    } else if (lower == 'd') {
      moveTo(curX + step, curY, curZ);
      return;
    } else if (lower == 'z') {
      // down
      float newZ = curZ - step;
      if (newZ < Z_MIN) newZ = Z_MIN;
      moveTo(curX, curY, newZ);
      return;
    } else if (lower == 'h') {
      printHelp();
      return;
    }
    // fallthrough to try parse as coordinates if not recognized
  }

  // Otherwise try parse as "x y z" absolute coords
  float x,y,z;
  int parsed = parseThreeFloatsTrimmed(trimmed, x, y, z);
  if (parsed == 3) {
    // ensure z within bounds
    if (z > Z_MAX) z = Z_MAX;
    if (z < Z_MIN) z = Z_MIN;
    moveTo(x,y,z);
    return;
  } else {
    Serial.print(F("Could not parse input: '"));
    Serial.print(raw);
    Serial.println(F("'. Use WASD, SPACE (up), z (down), or 'x y z'. Send 'h' for help."));
    return;
  }
}
