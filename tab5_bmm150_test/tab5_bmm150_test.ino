// tab5_bmm150_test.ino
// Standalone test: reach the M135's BMM150 magnetometer through the BMI270
// at 0x69, on the Tab5's internal I2C bus, without disturbing M5Unified.
//
// WHY THIS IS AWKWARD
//   The BMM150 has no direct connection to the host bus. Its I2C lines go to
//   the BMI270's auxiliary interface, so every access is a proxied read/write
//   through BMI270 registers. And the BMI270 itself does nothing at all until
//   its ~8KB config blob has been uploaded.
//
// APPROACH
//   Use Bosch's BMI270 API (bundled inside the SparkFun library) for the blob
//   and the aux helpers, but supply our OWN bus callbacks that go through
//   M5.In_I2C. That way we get the config file without pulling raw Wire onto
//   the bus, which is what broke everything previously.
//
//   M5.Imu keeps running the Tab5's onboard BMI270 at 0x68, untouched.
//   This code only ever talks to 0x69.
//
// DEPENDENCY
//   arduino-cli lib install "SparkFun BMI270 Arduino Library"
//   (used only for the bundled Bosch sources - none of its Wire code runs)

#include <M5Unified.h>
// Public header - pulls in the nested Bosch sources (bmi2_dev, bmi270_init,
// the config blob) regardless of how they are laid out under src/.
// None of the library's own Wire code runs; we only use the Bosch layer.
#include <SparkFun_BMI270_Arduino_Library.h>

constexpr uint8_t  ADDR_BMI270 = 0x69;   // M135's IMU (0x68 is the Tab5's own)
constexpr uint8_t  ADDR_BMM150 = 0x10;   // behind the BMI270 aux interface
constexpr uint32_t I2C_FREQ    = 100000;

// ---- BMM150 register map ----
constexpr uint8_t BMM_PWR_CTRL   = 0x4B;  // bit0 = power on (out of suspend)
constexpr uint8_t BMM_OP_MODE    = 0x4C;
constexpr uint8_t BMM_REP_XY     = 0x51;
constexpr uint8_t BMM_REP_Z      = 0x52;
constexpr uint8_t BMM_DATA_START = 0x42;
constexpr uint8_t BMM_CHIP_ID    = 0x40;  // reads 0x32
constexpr uint8_t BMM_TRIM_X1    = 0x5D;
constexpr uint8_t BMM_TRIM_XY2   = 0x64;
constexpr uint8_t BMM_TRIM_Z1    = 0x68;

struct bmi2_dev bmi;
uint8_t devAddr = ADDR_BMI270;
bool bmiReady = false, bmmReady = false;

// ============================================================================
// Bus callbacks - Bosch API calls these, we route them through M5.In_I2C
// ============================================================================
BMI2_INTF_RETURN_TYPE bmiRead(uint8_t reg, uint8_t* data, uint32_t len, void* intf) {
  uint8_t addr = *(uint8_t*)intf;
  return M5.In_I2C.readRegister(addr, reg, data, len, I2C_FREQ) ? BMI2_OK : BMI2_E_COM_FAIL;
}

BMI2_INTF_RETURN_TYPE bmiWrite(uint8_t reg, const uint8_t* data, uint32_t len, void* intf) {
  uint8_t addr = *(uint8_t*)intf;
  return M5.In_I2C.writeRegister(addr, reg, data, len, I2C_FREQ) ? BMI2_OK : BMI2_E_COM_FAIL;
}

void bmiDelayUs(uint32_t period, void* intf) {
  delayMicroseconds(period);
}

// ============================================================================
// BMM150 trim coefficients and compensation
// ============================================================================
struct BmmTrim {
  int8_t   x1, y1, x2, y2;
  uint16_t z1;
  int16_t  z2, z3, z4;
  uint8_t  xy1;
  int8_t   xy2;
  uint16_t xyz1;
} trim;

// Proxied single-register access to the BMM150 via the BMI270 aux interface.
int8_t bmmWrite(uint8_t reg, uint8_t val) {
  return bmi2_write_aux_man_mode(reg, &val, 1, &bmi);
}
int8_t bmmRead(uint8_t reg, uint8_t* buf, uint16_t len) {
  return bmi2_read_aux_man_mode(reg, buf, len, &bmi);
}

bool readTrim() {
  uint8_t a[2], b[4], c[10];
  if (bmmRead(BMM_TRIM_X1, a, 2) != BMI2_OK) return false;
  trim.x1 = (int8_t)a[0];
  trim.y1 = (int8_t)a[1];

  // dig_x2 = 0x64, dig_y2 = 0x65. 0x66/0x67 are reserved - do NOT skip ahead.
  if (bmmRead(BMM_TRIM_XY2, b, 2) != BMI2_OK) return false;
  trim.x2 = (int8_t)b[0];
  trim.y2 = (int8_t)b[1];

  if (bmmRead(BMM_TRIM_Z1, c, 10) != BMI2_OK) return false;
  trim.z2   = (int16_t)(((uint16_t)c[1] << 8) | c[0]);
  trim.z1   = (uint16_t)(((uint16_t)c[3] << 8) | c[2]);
  trim.xyz1 = (uint16_t)(((uint16_t)c[5] << 8) | c[4]);
  trim.z3   = (int16_t)(((uint16_t)c[7] << 8) | c[6]);
  trim.xy2  = (int8_t)c[8];
  trim.xy1  = c[9];

  // dig_z4 lives just below the block above
  uint8_t d[2];
  if (bmmRead(0x62, d, 2) != BMI2_OK) return false;
  trim.z4 = (int16_t)(((uint16_t)d[1] << 8) | d[0]);

  Serial.printf("BMM150 trim: x1=%d y1=%d x2=%d y2=%d z1=%u z2=%d z3=%d z4=%d xy1=%u xy2=%d xyz1=%u\n",
                trim.x1, trim.y1, trim.x2, trim.y2, trim.z1,
                trim.z2, trim.z3, trim.z4, trim.xy1, trim.xy2, trim.xyz1);

  // xyz1 and z1 are never zero on a good part - guards a bad trim read
  return (trim.xyz1 != 0 && trim.z1 != 0);
}

// Bosch BMM150 reference compensation (float variant). Returns microtesla.
// The trailing /16 converts the sensor's 1/16 uT LSB into uT.
float compensateXY(int16_t raw, uint16_t rhall, int8_t t1, int8_t t2) {
  if (raw == -4096 || rhall == 0 || trim.xyz1 == 0) return NAN;   // overflow marker

  float c0 = ((float)trim.xyz1) * 16384.0f / (float)rhall;
  float r  = c0 - 16384.0f;
  float c1 = ((float)trim.xy2) * ((r * r) / 268435456.0f);
  float c2 = c1 + r * ((float)trim.xy1) / 16384.0f;
  float c3 = ((float)t2) + 160.0f;
  float c4 = ((float)raw) * ((c2 + 256.0f) * c3);
  return ((c4 / 8192.0f) + (((float)t1) * 8.0f)) / 16.0f;
}

float compensateZ(int16_t raw, uint16_t rhall) {
  if (raw == -16384) return NAN;                                  // overflow marker
  if (trim.z2 == 0 || trim.z1 == 0 || trim.xyz1 == 0 || rhall == 0) return NAN;

  float z0 = ((float)raw) - ((float)trim.z4);
  float z1 = ((float)rhall) - ((float)trim.xyz1);
  float z2 = ((float)trim.z3) * z1;
  float z3 = ((float)trim.z1) * ((float)rhall) / 32768.0f;
  float z4 = ((float)trim.z2) + z3;
  float z5 = (z0 * 131072.0f) - z2;
  return (z5 / (z4 * 4.0f)) / 16.0f;
}

// ============================================================================
// Calibration
//
// The readings carry a large fixed offset - on this hardware the module sits
// a couple of mm from the Tab5's speaker magnet, which the M135 docs warn
// about explicitly. That is hard iron: a constant vector added to every
// sample. It is removed by finding the centre of the sphere the readings
// trace out as the device is rotated.
//
// Soft iron (the sphere being squashed into an ellipsoid) is corrected here
// only crudely, by equalising the per-axis ranges. A full ellipsoid fit would
// do better but needs far more maths than this warrants.
// ============================================================================
struct MagCal {
  float minX =  1e9, minY =  1e9, minZ =  1e9;
  float maxX = -1e9, maxY = -1e9, maxZ = -1e9;
  float offX = 0, offY = 0, offZ = 0;
  float sclX = 1, sclY = 1, sclZ = 1;
  bool  valid = false;
  uint32_t samples = 0;

  void reset() {
    minX = minY = minZ =  1e9;
    maxX = maxY = maxZ = -1e9;
    valid = false; samples = 0;
  }

  void feed(float x, float y, float z) {
    if (isnan(x) || isnan(y) || isnan(z)) return;
    if (x < minX) minX = x;  if (x > maxX) maxX = x;
    if (y < minY) minY = y;  if (y > maxY) maxY = y;
    if (z < minZ) minZ = z;  if (z > maxZ) maxZ = z;
    samples++;
    solve();
  }

  void solve() {
    float rx = (maxX - minX) / 2.0f;
    float ry = (maxY - minY) / 2.0f;
    float rz = (maxZ - minZ) / 2.0f;
    // need a meaningful swing on every axis before the fit means anything
    if (rx < 5 || ry < 5 || rz < 5) { valid = false; return; }

    offX = (maxX + minX) / 2.0f;
    offY = (maxY + minY) / 2.0f;
    offZ = (maxZ + minZ) / 2.0f;

    float avg = (rx + ry + rz) / 3.0f;
    sclX = avg / rx;  sclY = avg / ry;  sclZ = avg / rz;
    valid = true;
  }

  void apply(float x, float y, float z, float& ox, float& oy, float& oz) const {
    ox = (x - offX) * sclX;
    oy = (y - offY) * sclY;
    oz = (z - offZ) * sclZ;
  }

  // rough quality signal: how close the three radii are to each other
  float coverage() const {
    float rx = (maxX - minX) / 2.0f;
    float ry = (maxY - minY) / 2.0f;
    float rz = (maxZ - minZ) / 2.0f;
    if (rx <= 0 || ry <= 0 || rz <= 0) return 0;
    float lo = fminf(rx, fminf(ry, rz));
    float hi = fmaxf(rx, fmaxf(ry, rz));
    return lo / hi;      // 1.0 = perfectly spherical coverage
  }
} cal;

bool calibrating = false;

// ============================================================================
bool startBmm() {
  // Route aux traffic to the BMM150's address and enter manual mode, so we
  // can address arbitrary registers rather than letting the BMI270 auto-poll.
  struct bmi2_sens_config cfg;
  cfg.type = BMI2_AUX;
  int8_t rc = bmi2_get_sensor_config(&cfg, 1, &bmi);
  if (rc != BMI2_OK) { Serial.printf("aux get_config rc=%d\n", rc); return false; }

  cfg.cfg.aux.aux_en      = BMI2_ENABLE;
  cfg.cfg.aux.i2c_device_addr = ADDR_BMM150;
  cfg.cfg.aux.manual_en   = BMI2_ENABLE;      // manual, not auto-poll
  cfg.cfg.aux.fcu_write_en= BMI2_ENABLE;
  cfg.cfg.aux.man_rd_burst= BMI2_AUX_READ_LEN_3;   // 8-byte bursts
  cfg.cfg.aux.aux_rd_burst= BMI2_AUX_READ_LEN_3;
  cfg.cfg.aux.odr         = BMI2_AUX_ODR_100HZ;

  rc = bmi2_set_sensor_config(&cfg, 1, &bmi);
  if (rc != BMI2_OK) { Serial.printf("aux set_config rc=%d\n", rc); return false; }

  uint8_t sens = BMI2_AUX;
  rc = bmi2_sensor_enable(&sens, 1, &bmi);
  if (rc != BMI2_OK) { Serial.printf("aux enable rc=%d\n", rc); return false; }
  delay(10);

  // BMM150 boots in suspend - bit0 of 0x4B lifts it into sleep mode
  if (bmmWrite(BMM_PWR_CTRL, 0x01) != BMI2_OK) {
    Serial.println("BMM150 power-on write failed");
    return false;
  }
  delay(10);

  uint8_t id = 0;
  if (bmmRead(BMM_CHIP_ID, &id, 1) != BMI2_OK) {
    Serial.println("BMM150 chip id read failed");
    return false;
  }
  Serial.printf("BMM150 chip id: 0x%02X (expect 0x32)\n", id);
  if (id != 0x32) return false;

  if (!readTrim()) { Serial.println("BMM150 trim read failed"); return false; }

  // regular preset: 9 XY repetitions, 15 Z repetitions, forced/normal mode
  bmmWrite(BMM_REP_XY, 0x04);      // (2*0x04)+1 = 9
  bmmWrite(BMM_REP_Z,  0x0E);      // 0x0E+1 = 15
  bmmWrite(BMM_OP_MODE, 0x00);     // normal mode, ODR 10Hz
  delay(10);

  return true;
}

int16_t lastRx = 0, lastRy = 0, lastRz = 0;
uint16_t lastRh = 0;

bool readMag(float& mx, float& my, float& mz) {
  uint8_t d[8];
  if (bmmRead(BMM_DATA_START, d, 8) != BMI2_OK) return false;

  // X and Y are 13-bit, Z is 15-bit, RHALL is 14-bit - all left-padded
  int16_t rx = (int16_t)(((int16_t)((int8_t)d[1]) << 5) | (d[0] >> 3));
  int16_t ry = (int16_t)(((int16_t)((int8_t)d[3]) << 5) | (d[2] >> 3));
  int16_t rz = (int16_t)(((int16_t)((int8_t)d[5]) << 7) | (d[4] >> 1));
  uint16_t rh = (uint16_t)(((uint16_t)d[7] << 6) | (d[6] >> 2));

  lastRx = rx; lastRy = ry; lastRz = rz; lastRh = rh;

  mx = compensateXY(rx, rh, trim.x1, trim.x2);
  my = compensateXY(ry, rh, trim.y1, trim.y2);
  mz = compensateZ(rz, rh);
  return true;
}

// ============================================================================
void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(3);
  M5.Display.setTextSize(2);
  M5.Display.fillScreen(TFT_BLACK);

  Serial.begin(115200);
  M5.Power.setExtOutput(true);
  delay(1000);
  Serial.println("\n=== BMM150 via BMI270 aux, on M135 @0x69 ===");

  // Onboard IMU is separate and stays with M5Unified
  Serial.printf("Onboard IMU (0x68) type=%d\n", (int)M5.Imu.getType());

  // ---- bring up the M135's BMI270 at 0x69 with our own bus callbacks ----
  bmi.intf      = BMI2_I2C_INTF;
  bmi.read      = bmiRead;
  bmi.write     = bmiWrite;
  bmi.delay_us  = bmiDelayUs;
  bmi.intf_ptr  = &devAddr;
  bmi.read_write_len = 32;
  bmi.config_file_ptr = NULL;      // use the library's bundled blob

  int8_t rc = bmi270_init(&bmi);
  Serial.printf("bmi270_init rc=%d (0 = OK) chip_id=0x%02X\n", rc, bmi.chip_id);
  bmiReady = (rc == BMI2_OK);

  if (bmiReady) {
    bmmReady = startBmm();
    Serial.println(bmmReady ? "BMM150 ready" : "BMM150 bring-up failed");
  }
}

uint32_t lastDraw = 0;

void loop() {
  M5.update();

  // touch anywhere to start/stop calibration
  if (M5.Touch.getCount() > 0 && M5.Touch.getDetail().wasPressed()) {
    calibrating = !calibrating;
    if (calibrating) { cal.reset(); Serial.println("--- calibration started ---"); }
    else Serial.printf("--- calibration stopped: off %.1f %.1f %.1f  cov %.2f ---\n",
                       cal.offX, cal.offY, cal.offZ, cal.coverage());
  }

  float mx, my, mz;
  bool got = (bmiReady && bmmReady && readMag(mx, my, mz));

  if (got && calibrating) cal.feed(mx, my, mz);

  if (millis() - lastDraw > 200) {
    lastDraw = millis();
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setCursor(10, 10);

    if (!bmiReady) {
      M5.Display.setTextColor(TFT_RED);
      M5.Display.printf("BMI270 @0x69 init failed\n");
    } else if (!bmmReady) {
      M5.Display.setTextColor(TFT_RED);
      M5.Display.printf("BMI270 OK, BMM150 failed\n");
    } else if (!got) {
      M5.Display.setTextColor(TFT_RED);
      M5.Display.printf("aux read failed\n");
    } else {
      float cx = mx, cy = my, cz = mz;
      if (cal.valid) cal.apply(mx, my, mz, cx, cy, cz);

      M5.Display.setTextColor(TFT_WHITE);
      M5.Display.printf("BMM150 magnetometer\n\n");
      M5.Display.printf("  raw  %7.1f %7.1f %7.1f\n", mx, my, mz);

      if (cal.valid) {
        M5.Display.setTextColor(TFT_GREENYELLOW);
        M5.Display.printf("  cal  %7.1f %7.1f %7.1f uT\n", cx, cy, cz);
      } else {
        M5.Display.setTextColor(TFT_DARKGREY);
        M5.Display.printf("  cal  (not calibrated)\n");
      }

      float heading = atan2f(cy, cx) * 57.2958f;
      if (heading < 0) heading += 360.0f;
      float mag = sqrtf(cx*cx + cy*cy + cz*cz);

      M5.Display.setTextColor(TFT_WHITE);
      M5.Display.printf("\n  heading %6.1f deg\n", heading);

      // a correctly calibrated reading sits in Earth's 25-65 uT band
      bool sane = (mag > 20 && mag < 75);
      M5.Display.setTextColor(sane ? TFT_GREEN : TFT_ORANGE);
      M5.Display.printf("  |B|     %6.1f uT\n\n", mag);

      M5.Display.setTextColor(TFT_WHITE);
      if (calibrating) {
        M5.Display.setTextColor(TFT_YELLOW);
        M5.Display.printf("  CALIBRATING - rotate slowly\n");
        M5.Display.printf("  through all orientations\n");
        M5.Display.printf("  samples %lu  coverage %.2f\n",
                          (unsigned long)cal.samples, cal.coverage());
        M5.Display.printf("\n  tap to finish\n");
      } else {
        M5.Display.setTextColor(TFT_DARKGREY);
        M5.Display.printf("  offsets %.1f %.1f %.1f\n", cal.offX, cal.offY, cal.offZ);
        M5.Display.printf("\n  tap to calibrate\n");
      }

      Serial.printf("raw %6d %6d %6d rh %5u | cal %7.2f %7.2f %7.2f uT | |B| %5.1f | hdg %.1f\n",
                    lastRx, lastRy, lastRz, lastRh, cx, cy, cz, mag, heading);
    }
  }
}
