// tab5_gnss_sensors.ino
// M5Stack Module GNSS (M135) on Tab5 (ESP32-P4) - GNSS + onboard sensors.
//
// DIP: TX pos 1, RX pos 1 -> module TX = G7, module RX = G6
//      PPS pos 3 (optional) -> G51
//
// Sensors on the M135, all on the Tab5 internal I2C (G31/G32):
//   BMP280  0x76  - pressure + temperature   (hand-written, via M5.In_I2C)
//   BMI270  0x69  - 6-axis IMU               (M5Unified's own IMU_Class)
//   BMM150  0x10  - magnetometer, sits behind the BMI270 aux bus, NOT direct
//
// No external libraries, and no raw Wire anywhere. Everything goes through
// M5.In_I2C, which is the same accessor M5Unified uses internally for touch,
// the RTC and the onboard IMU. Calling Wire.begin() on G31/G32 tears that
// setup down and breaks every device on the bus, so it is deliberately absent.

#define USE_BMI270 1

#include <M5Unified.h>

constexpr int  PIN_GNSS_TX = 7;
constexpr int  PIN_GNSS_RX = 6;
constexpr int  PIN_PPS     = 51;
constexpr uint32_t GNSS_BAUD = 38400;

constexpr int  PIN_SDA = 31;
constexpr int  PIN_SCL = 32;
constexpr uint8_t ADDR_BMP280 = 0x76;
constexpr uint8_t ADDR_BMI270 = 0x69;   // M135 module (0x68 is the Tab5's own)
constexpr uint32_t I2C_FREQ   = 100000; // shared bus - keep it slow

// ============================ BMP280 =========================================
// Direct driver. Reads factory calibration, applies Bosch compensation.
struct Bmp280 {
  uint16_t T1; int16_t T2, T3;
  uint16_t P1; int16_t P2, P3, P4, P5, P6, P7, P8, P9;
  int32_t  t_fine = 0;
  bool     ok = false;

  // All access via M5.In_I2C - the arbitrated path shared with touch/RTC.
  uint8_t lastErr = 0;
  int     lastGot = 0;

  uint8_t r8(uint8_t reg) {
    uint8_t v = 0;
    if (!M5.In_I2C.readRegister(ADDR_BMP280, reg, &v, 1, I2C_FREQ)) return 0;
    return v;
  }
  uint16_t r16le(uint8_t reg) {
    uint8_t b[2] = {0, 0};
    if (!M5.In_I2C.readRegister(ADDR_BMP280, reg, b, 2, I2C_FREQ)) return 0;
    return ((uint16_t)b[1] << 8) | b[0];
  }
  void w8(uint8_t reg, uint8_t val) {
    M5.In_I2C.writeRegister8(ADDR_BMP280, reg, val, I2C_FREQ);
  }

  bool begin() {
    uint8_t id = r8(0xD0);
    Serial.printf("BMP280 chip id: 0x%02X (expect 0x58)\n", id);
    if (id != 0x58) return false;

    // soft reset, then wait for the NVM copy to finish
    w8(0xE0, 0xB6);
    delay(10);
    for (int i = 0; i < 20 && (r8(0xF3) & 0x01); i++) delay(5);

    T1 = r16le(0x88); T2 = (int16_t)r16le(0x8A); T3 = (int16_t)r16le(0x8C);
    P1 = r16le(0x8E);
    P2 = (int16_t)r16le(0x90); P3 = (int16_t)r16le(0x92);
    P4 = (int16_t)r16le(0x94); P5 = (int16_t)r16le(0x96);
    P6 = (int16_t)r16le(0x98); P7 = (int16_t)r16le(0x9A);
    P8 = (int16_t)r16le(0x9C); P9 = (int16_t)r16le(0x9E);

    // T1/P1 are unsigned and never zero on a good part - catches a bad read
    if (T1 == 0 || P1 == 0) {
      Serial.println("BMP280 calibration read looks wrong (T1/P1 zero)");
      return false;
    }
    Serial.printf("BMP280 calib T1=%u P1=%u\n", T1, P1);

    // config: standby 0.5ms, IIR filter 16 ; ctrl_meas: osrs_t x2, osrs_p x16, normal
    w8(0xF5, (0 << 5) | (4 << 2));
    w8(0xF4, (2 << 5) | (5 << 2) | 3);
    delay(50);

    uint8_t ctrl = r8(0xF4);
    Serial.printf("BMP280 ctrl_meas readback: 0x%02X (mode bits %d)\n", ctrl, ctrl & 3);
    if ((ctrl & 3) != 3) {
      Serial.println("BMP280 did not enter normal mode");
      return false;
    }
    ok = true;
    return true;
  }

  // NOTE: the byte reads MUST be sequenced explicitly. Writing this as
  //   (Wire.read() << 12) | (Wire.read() << 4) | (Wire.read() >> 4)
  // is undefined behaviour - C++ does not order the operands of '|', so the
  // bytes get assembled in arbitrary order and the result is garbage.
  bool readRaw(int32_t& adcT, int32_t& adcP) {
    uint8_t b[6];
    if (!M5.In_I2C.readRegister(ADDR_BMP280, 0xF7, b, 6, I2C_FREQ)) {
      lastErr = 1; lastGot = 0;
      return false;
    }
    lastErr = 0; lastGot = 6;
    adcP = ((uint32_t)b[0] << 12) | ((uint32_t)b[1] << 4) | (b[2] >> 4);
    adcT = ((uint32_t)b[3] << 12) | ((uint32_t)b[4] << 4) | (b[5] >> 4);
    // 0x80000 in both is the reset value - sensor is in sleep, not measuring
    if (adcP == 0x80000 && adcT == 0x80000) return false;
    return true;
  }

  // Single burst read -> both values. Avoids the double-read the old
  // temperature()/pressure() pair did, which sampled the sensor twice.
  bool read(float& tempC, float& hPa) {
    int32_t adcT, adcP;
    if (!readRaw(adcT, adcP)) return false;

    // --- temperature, Bosch datasheet 3.11.3 ---
    int32_t v1 = ((((adcT >> 3) - ((int32_t)T1 << 1))) * ((int32_t)T2)) >> 11;
    int32_t v2 = (((((adcT >> 4) - ((int32_t)T1)) * ((adcT >> 4) - ((int32_t)T1))) >> 12)
                 * ((int32_t)T3)) >> 14;
    t_fine = v1 + v2;
    tempC = ((t_fine * 5 + 128) >> 8) / 100.0f;

    // --- pressure, needs t_fine from above ---
    int64_t w1 = ((int64_t)t_fine) - 128000;
    int64_t w2 = w1 * w1 * (int64_t)P6;
    w2 += ((w1 * (int64_t)P5) << 17);
    w2 += (((int64_t)P4) << 35);
    w1 = ((w1 * w1 * (int64_t)P3) >> 8) + ((w1 * (int64_t)P2) << 12);
    w1 = (((((int64_t)1) << 47) + w1)) * ((int64_t)P1) >> 33;
    if (w1 == 0) { hPa = 0; return false; }
    int64_t p = 1048576 - adcP;
    p = (((p << 31) - w2) * 3125) / w1;
    w1 = (((int64_t)P9) * (p >> 13) * (p >> 13)) >> 25;
    w2 = (((int64_t)P8) * p) >> 19;
    p = ((p + w1 + w2) >> 8) + (((int64_t)P7) << 4);
    hPa = (float)p / 25600.0f;
    return true;
  }

} bmp;

// pressure altitude, ISA standard atmosphere
float pressureAltitude(float hPa, float seaLevel = 1013.25f) {
  return 44330.0f * (1.0f - powf(hPa / seaLevel, 0.1903f));
}

// ============================ IMU ============================================
// NOTE: the M135 carries its own BMI270 at 0x69, but M5Unified's IMU_Class
// auto-scans and always binds 0x68 first - begin()'s second argument is a
// board_t, not an address, so there is no supported way to target 0x69.
// Since the Tab5 has an identical BMI270 onboard at 0x68, we simply use that.
// The module's IMU is redundant here; it exists for Cores that lack one.
// (To use the module's instead you would need a hand-written BMI270 driver
// over M5.In_I2C, including the ~8KB config blob upload.)
#if USE_BMI270
bool imuOk = false;
float imuAx = 0, imuAy = 0, imuAz = 0;
float imuGx = 0, imuGy = 0, imuGz = 0;
#endif

// ============================ GNSS parse =====================================
struct Fix {
  char   status = 'V';
  int    mode = 1, sats = 0;
  double lat = 0, lon = 0, altitude = 0, hdop = 99.99;
  double speedKmh = 0, course = 0;
  char   utc[16] = "", date[16] = "";
  uint32_t lastSentence = 0;
} fix;

struct Con { const char* name; int visible; int bestSnr; };
Con cons[4] = { {"GPS",0,0}, {"GLO",0,0}, {"GAL",0,0}, {"BDS",0,0} };
uint32_t sentenceCount = 0;

int splitFields(char* s, char** f, int maxf) {
  int n = 0; f[n++] = s;
  for (char* p = s; *p && n < maxf; p++) {
    if (*p == ',') { *p = 0; f[n++] = p + 1; }
    else if (*p == '*') { *p = 0; break; }
  }
  return n;
}
double nmeaCoord(const char* v, const char* h) {
  if (!v || !*v) return 0;
  double raw = atof(v); int deg = (int)(raw / 100);
  double d = deg + (raw - deg * 100) / 60.0;
  if (h && (*h == 'S' || *h == 'W')) d = -d;
  return d;
}
int conIndex(const char* t) {
  if (!strncmp(t,"GP",2)) return 0;
  if (!strncmp(t,"GL",2)) return 1;
  if (!strncmp(t,"GA",2)) return 2;
  if (!strncmp(t,"GB",2)) return 3;
  return -1;
}
void parseSentence(char* s) {
  if (s[0] != '$') return;
  sentenceCount++; fix.lastSentence = millis();
  char talker[3] = { s[1], s[2], 0 };
  char type[4]   = { s[3], s[4], s[5], 0 };
  char* f[24];
  int n = splitFields(s, f, 24);

  if (!strcmp(type,"RMC") && n > 9) {
    fix.status = f[2][0] ? f[2][0] : 'V';
    strncpy(fix.utc, f[1], sizeof(fix.utc)-1);
    strncpy(fix.date, f[9], sizeof(fix.date)-1);
    if (fix.status == 'A') {
      fix.lat = nmeaCoord(f[3], f[4]);
      fix.lon = nmeaCoord(f[5], f[6]);
    }
  } else if (!strcmp(type,"VTG") && n > 7) {
    fix.course   = atof(f[1]);
    fix.speedKmh = atof(f[7]);          // km/h directly, no conversion needed
  } else if (!strcmp(type,"GGA") && n > 9) {
    fix.sats = atoi(f[7]);
    fix.hdop = f[8][0] ? atof(f[8]) : 99.99;
    fix.altitude = atof(f[9]);
  } else if (!strcmp(type,"GSA") && n > 17) {
    int m = atoi(f[2]); if (m > fix.mode || m == 1) fix.mode = m;
  } else if (!strcmp(type,"GSV") && n >= 4) {
    int ci = conIndex(talker);
    if (ci >= 0) {
      if (atoi(f[2]) == 1) { cons[ci].bestSnr = 0; }
      cons[ci].visible = atoi(f[3]);
      for (int i = 4; i + 3 < n; i += 4) {
        int snr = atoi(f[i+3]);
        if (snr > cons[ci].bestSnr) cons[ci].bestSnr = snr;
      }
    }
  }
}

// ============================ display ========================================
M5Canvas canvas(&M5.Display);

// PPS is edge-counted in an ISR. Polling from loop() misses pulses, because
// the draw path can occupy far longer than the ~100ms PPS pulse width.
volatile uint32_t ppsCount = 0, ppsLast = 0, ppsInterval = 0;

void IRAM_ATTR ppsIsr() {
  uint32_t now = millis();
  if (ppsLast) ppsInterval = now - ppsLast;
  ppsLast = now;
  ppsCount++;
}

float tempC = 0, presHpa = 0, baroAlt = 0;
bool  bmpRead = false;

#if USE_BMI270
float  imuPrevX = 0, imuPrevY = 0, imuPrevZ = 0;
int    imuStaleCount = 0;      // consecutive byte-identical samples
#endif

void drawScreen() {
  canvas.fillSprite(TFT_BLACK);
  canvas.setTextDatum(top_left);
  int W = canvas.width();
  char buf[96];

  bool have = (fix.status == 'A');
  canvas.fillRect(0, 0, W, 56, have ? TFT_DARKGREEN : 0x6000);
  canvas.setTextColor(TFT_WHITE); canvas.setTextSize(3);
  canvas.drawString("Module GNSS  M135", 16, 14);
  canvas.setTextColor(have ? TFT_GREENYELLOW : TFT_ORANGE);
  canvas.drawString(have ? "FIX" : "NO FIX", W - 160, 14);

  // ---- left column: GNSS ----
  int y = 74, x = 16;
  canvas.setTextSize(2);
  canvas.setTextColor(TFT_CYAN); canvas.drawString("GNSS", x, y); y += 30;
  canvas.setTextColor(TFT_WHITE);
  if (have) {
    snprintf(buf,sizeof(buf),"lat %.6f", fix.lat); canvas.drawString(buf,x+12,y); y+=24;
    snprintf(buf,sizeof(buf),"lon %.6f", fix.lon); canvas.drawString(buf,x+12,y); y+=24;
    snprintf(buf,sizeof(buf),"alt %.1f m", fix.altitude); canvas.drawString(buf,x+12,y); y+=24;
    snprintf(buf,sizeof(buf),"spd %.1f km/h", fix.speedKmh); canvas.drawString(buf,x+12,y); y+=24;
    snprintf(buf,sizeof(buf),"cog %.0f deg", fix.course); canvas.drawString(buf,x+12,y); y+=24;
  } else {
    canvas.setTextColor(TFT_DARKGREY);
    canvas.drawString("no fix - needs open sky", x+12, y); y += 24;
    canvas.drawString("30-90s cold start", x+12, y); y += 24;
  }
  canvas.setTextColor(TFT_WHITE);
  const char* ms = fix.mode==3?"3D":fix.mode==2?"2D":"none";
  snprintf(buf,sizeof(buf),"%s  sats %d  HDOP %.2f", ms, fix.sats, fix.hdop);
  canvas.drawString(buf,x+12,y); y+=24;
  if (fix.utc[0]) {
    snprintf(buf,sizeof(buf),"UTC %s  %s", fix.utc, fix.date);
    canvas.drawString(buf,x+12,y);
  }
  y += 40;

  // satellites
  canvas.setTextColor(TFT_CYAN); canvas.drawString("Satellites", x, y); y += 28;
  int bw = 180;
  for (int i = 0; i < 4; i++) {
    canvas.setTextColor(TFT_WHITE);
    canvas.drawString(cons[i].name, x+12, y+3);
    int fw = (cons[i].bestSnr > 50 ? 50 : cons[i].bestSnr) * bw / 50;
    canvas.drawRect(x+70, y, bw, 20, TFT_DARKGREY);
    uint16_t bc = cons[i].bestSnr>=35?TFT_GREEN:cons[i].bestSnr>=25?TFT_YELLOW:TFT_RED;
    if (fw > 2) canvas.fillRect(x+71, y+1, fw-2, 18, bc);
    snprintf(buf,sizeof(buf),"%d  %ddB", cons[i].visible, cons[i].bestSnr);
    canvas.drawString(buf, x+70+bw+10, y+3);
    y += 26;
  }

  // ---- right column: sensors ----
  int rx = W/2 + 20; y = 74;
  canvas.setTextColor(TFT_CYAN); canvas.drawString("Barometer  BMP280", rx, y); y += 30;
  canvas.setTextColor(TFT_WHITE);
  if (bmp.ok && bmpRead) {
    // sanity range: anything outside this means a bad read, not real weather
    bool sane = (presHpa > 800 && presHpa < 1100 && tempC > -40 && tempC < 85);
    canvas.setTextColor(sane ? TFT_WHITE : TFT_RED);
    snprintf(buf,sizeof(buf),"temp  %.2f C", tempC);      canvas.drawString(buf,rx+12,y); y+=24;
    snprintf(buf,sizeof(buf),"pres  %.2f hPa", presHpa);  canvas.drawString(buf,rx+12,y); y+=24;
    snprintf(buf,sizeof(buf),"alt   %.1f m (ISA)", baroAlt); canvas.drawString(buf,rx+12,y); y+=24;
    canvas.setTextColor(TFT_DARKGREY);
    if (sane) canvas.drawString("ISA alt assumes 1013.25 hPa", rx+12, y);
    else      canvas.drawString("out of range - suspect bad read", rx+12, y);
    y += 24;
  } else if (bmp.ok) {
    canvas.setTextColor(TFT_RED); canvas.drawString("read failed", rx+12, y); y += 24;
  } else {
    canvas.setTextColor(TFT_RED); canvas.drawString("not found at 0x76", rx+12, y); y += 24;
  }
  y += 24;

  canvas.setTextColor(TFT_CYAN); canvas.drawString("IMU  BMI270 @0x68 (Tab5)", rx, y); y += 30;
#if USE_BMI270
  if (imuOk) {
    M5.Imu.update();
    auto d = M5.Imu.getImuData();
    imuAx = d.accel.x; imuAy = d.accel.y; imuAz = d.accel.z;
    imuGx = d.gyro.x;  imuGy = d.gyro.y;  imuGz = d.gyro.z;

    float mag = sqrtf(imuAx*imuAx + imuAy*imuAy + imuAz*imuAz);

    // A live BMI270 always dithers by a few LSB. Byte-identical samples mean
    // the read is failing and we are seeing the previous values again.
    bool changed = (imuAx != imuPrevX || imuAy != imuPrevY || imuAz != imuPrevZ);
    if (changed) { imuStaleCount = 0; imuPrevX = imuAx; imuPrevY = imuAy; imuPrevZ = imuAz; }
    else         { imuStaleCount++; }

    bool live = (mag > 0.05f) && (imuStaleCount < 6);

    canvas.setTextColor(live ? TFT_WHITE : TFT_RED);
    snprintf(buf,sizeof(buf),"acc  %6.2f %6.2f %6.2f g", imuAx, imuAy, imuAz);
    canvas.drawString(buf,rx+12,y); y+=24;
    snprintf(buf,sizeof(buf),"gyr  %6.1f %6.1f %6.1f dps", imuGx, imuGy, imuGz);
    canvas.drawString(buf,rx+12,y); y+=24;

    if (live) {
      float pitch = atan2f(-imuAx, sqrtf(imuAy*imuAy + imuAz*imuAz)) * 57.2958f;
      float roll  = atan2f(imuAy, imuAz) * 57.2958f;
      snprintf(buf,sizeof(buf),"pitch %.1f  roll %.1f   |a| %.2fg", pitch, roll, mag);
      canvas.drawString(buf,rx+12,y); y+=24;
    } else {
      canvas.setTextColor(TFT_RED);
      snprintf(buf,sizeof(buf),"stale x%d", imuStaleCount);
      canvas.drawString(buf,rx+12,y); y+=24;
    }
  } else {
    canvas.setTextColor(TFT_RED);
    canvas.drawString("not detected", rx+12, y); y += 24;
  }
#else
  canvas.setTextColor(TFT_DARKGREY);
  canvas.drawString("disabled (USE_BMI270 0)", rx+12, y); y += 24;
#endif
  y += 24;

  canvas.setTextColor(TFT_CYAN); canvas.drawString("Magnetometer  BMM150", rx, y); y += 30;
  canvas.setTextColor(TFT_DARKGREY);
  canvas.drawString("behind BMI270 aux bus,", rx+12, y); y += 22;
  canvas.drawString("not on main I2C - see notes", rx+12, y); y += 22;

  // footer
  y = canvas.height() - 60;
  canvas.setTextColor(TFT_DARKGREY);
  uint32_t age = millis() - fix.lastSentence;
  snprintf(buf,sizeof(buf),"UART G%d/G%d @%lu   %lu sentences",
           PIN_GNSS_TX, PIN_GNSS_RX, (unsigned long)GNSS_BAUD,
           (unsigned long)sentenceCount);
  canvas.drawString(buf,16,y); y+=24;
  if (age > 3000) {
    canvas.setTextColor(TFT_RED); canvas.drawString("NO GNSS DATA", 16, y);
  } else if (ppsCount) {
    snprintf(buf,sizeof(buf),"PPS %lu edges, %lu ms",
             (unsigned long)ppsCount, (unsigned long)ppsInterval);
    canvas.drawString(buf,16,y);
  } else {
    canvas.drawString("PPS idle", 16, y);
  }
  canvas.pushSprite(0,0);
}

// ============================ setup / loop ===================================
void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(3);            // landscape, 180 from rotation 1
  M5.Display.fillScreen(TFT_BLACK);

  Serial.begin(115200);
  M5.Power.setExtOutput(true);
  delay(500);

  canvas.setColorDepth(16);
  canvas.createSprite(M5.Display.width(), M5.Display.height());

  // NO Wire.begin() here. M5.begin() has already configured the internal I2C
  // bus (G31/G32). Calling Wire.begin() on those pins reconfigures the
  // peripheral out from under M5Unified and breaks every device on the bus.

  if (!bmp.begin()) Serial.println("BMP280 not found at 0x76");
  else              Serial.println("BMP280 ready");

#if USE_BMI270
  // M5.Imu is already initialised by M5.begin() - this just checks it bound.
  imuOk = (M5.Imu.getType() != m5::imu_none);
  Serial.printf("Onboard IMU: %s (type=%d)\n",
                imuOk ? "OK" : "not detected", (int)M5.Imu.getType());
  if (imuOk) {
    M5.Imu.update();
    auto d = M5.Imu.getImuData();
    Serial.printf("IMU first read: a=%.2f %.2f %.2f\n",
                  d.accel.x, d.accel.y, d.accel.z);
  }
#endif

  // Must precede begin() - ignored once the port is open. The default 256B
  // FIFO overflows during a full canvas push at 38400 baud.
  Serial1.setRxBufferSize(2048);
  Serial1.begin(GNSS_BAUD, SERIAL_8N1, PIN_GNSS_TX, PIN_GNSS_RX);

  pinMode(PIN_PPS, INPUT_PULLDOWN);
  attachInterrupt(PIN_PPS, ppsIsr, RISING);

  fix.lastSentence = millis();
}

char line[128]; int linePos = 0;
uint32_t lastDraw = 0, lastBaro = 0;

// Drain the UART. Called from several places, including mid-draw, because a
// full canvas push takes long enough to overflow the RX FIFO at 38400 baud.
void pumpGnss() {
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') { line[linePos] = 0; parseSentence(line); linePos = 0; }
    else if (c != '\r' && linePos < (int)sizeof(line) - 1) line[linePos++] = c;
  }
}

void loop() {
  pumpGnss();

  // Safe to call every iteration again: everything now shares M5.In_I2C,
  // so touch polling no longer collides with the sensor reads.
  M5.update();

  pumpGnss();

  if (bmp.ok && millis() - lastBaro > 250) {
    bmpRead = bmp.read(tempC, presHpa);      // one burst, temp and pressure
    if (bmpRead) baroAlt = pressureAltitude(presHpa);
    else Serial.printf("BMP280 read fail: err=%u got=%d\n", bmp.lastErr, bmp.lastGot);
    lastBaro = millis();
  }

  if (millis() - lastDraw > 500) { drawScreen(); lastDraw = millis(); }

  pumpGnss();
}
