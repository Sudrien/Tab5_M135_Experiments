// tab5_gnss_display.ino
// M5Stack Module GNSS (M135) on Tab5 (ESP32-P4) - on-screen status.
//
// DIP: TX block pos 1, RX block pos 1  -> module TX = G7, module RX = G6
//      PPS block pos 3 (optional)      -> PPS = G51
//
// Parses NMEA directly (no external library) and renders a status page.

#include <M5Unified.h>

constexpr int  PIN_GNSS_TX = 7;    // bus 15 - Tab5 receives here
constexpr int  PIN_GNSS_RX = 6;    // bus 16 - Tab5 transmits here
constexpr int  PIN_PPS     = 51;   // bus 26 - set PPS DIP to pos 3, or ignore
constexpr uint32_t GNSS_BAUD = 38400;

// ---- parsed state ----
struct Fix {
  char     status   = 'V';   // A = valid, V = void
  int      quality  = 0;     // GGA fix quality
  int      mode     = 1;     // GSA: 1 none, 2 = 2D, 3 = 3D
  int      sats     = 0;     // satellites used
  double   lat = 0, lon = 0;
  double   altitude = 0;
  double   hdop = 99.99;
  double   speedKn = 0;
  char     utc[16] = "";
  char     date[16] = "";
  uint32_t lastSentence = 0;
} fix;

// per-constellation visible counts and best SNR
struct Con { const char* name; int visible; int bestSnr; };
Con cons[4] = { {"GPS",0,0}, {"GLO",0,0}, {"GAL",0,0}, {"BDS",0,0} };

uint32_t sentenceCount = 0, byteCount = 0;
uint32_t ppsCount = 0, ppsLast = 0, ppsInterval = 0;
int      ppsPrev = 0;

// ---- tiny CSV field splitter ----
int splitFields(char* s, char** f, int maxf) {
  int n = 0;
  f[n++] = s;
  for (char* p = s; *p && n < maxf; p++) {
    if (*p == ',') { *p = 0; f[n++] = p + 1; }
    else if (*p == '*') { *p = 0; break; }
  }
  return n;
}

double nmeaCoord(const char* v, const char* hemi) {
  if (!v || !*v) return 0;
  double raw = atof(v);
  int deg = (int)(raw / 100);
  double min = raw - deg * 100;
  double d = deg + min / 60.0;
  if (hemi && (*hemi == 'S' || *hemi == 'W')) d = -d;
  return d;
}

int conIndex(const char* talker) {   // talker = 2 chars after '$'
  if (!strncmp(talker, "GP", 2)) return 0;
  if (!strncmp(talker, "GL", 2)) return 1;
  if (!strncmp(talker, "GA", 2)) return 2;
  if (!strncmp(talker, "GB", 2)) return 3;
  return -1;
}

void parseSentence(char* s) {
  if (s[0] != '$') return;
  sentenceCount++;
  fix.lastSentence = millis();

  char talker[3] = { s[1], s[2], 0 };
  char type[4]   = { s[3], s[4], s[5], 0 };

  char* f[24];
  int n = splitFields(s, f, 24);

  if (!strcmp(type, "RMC") && n > 9) {
    fix.status = f[2][0] ? f[2][0] : 'V';
    strncpy(fix.utc,  f[1], sizeof(fix.utc) - 1);
    strncpy(fix.date, f[9], sizeof(fix.date) - 1);
    if (fix.status == 'A') {
      fix.lat = nmeaCoord(f[3], f[4]);
      fix.lon = nmeaCoord(f[5], f[6]);
      fix.speedKn = atof(f[7]);
    }
  }
  else if (!strcmp(type, "GGA") && n > 9) {
    fix.quality  = atoi(f[6]);
    fix.sats     = atoi(f[7]);
    fix.hdop     = f[8][0] ? atof(f[8]) : 99.99;
    fix.altitude = atof(f[9]);
  }
  else if (!strcmp(type, "GSA") && n > 17) {
    int m = atoi(f[2]);
    if (m > fix.mode || m == 1) fix.mode = m;
  }
  else if (!strcmp(type, "GSV") && n >= 4) {
    int ci = conIndex(talker);
    if (ci >= 0) {
      int msgNum = atoi(f[2]);
      if (msgNum == 1) { cons[ci].visible = 0; cons[ci].bestSnr = 0; }
      cons[ci].visible = atoi(f[3]);
      for (int i = 4; i + 3 < n; i += 4) {
        int snr = atoi(f[i + 3]);
        if (snr > cons[ci].bestSnr) cons[ci].bestSnr = snr;
      }
    }
  }
}

// ---- drawing ----
M5Canvas canvas(&M5.Display);

void drawScreen() {
  canvas.fillSprite(TFT_BLACK);
  canvas.setTextDatum(top_left);

  int W = canvas.width();

  // header
  bool have = (fix.status == 'A');
  uint16_t hc = have ? TFT_DARKGREEN : 0x6000;
  canvas.fillRect(0, 0, W, 64, hc);
  canvas.setTextColor(TFT_WHITE);
  canvas.setTextSize(3);
  canvas.drawString("Module GNSS  -  M135", 16, 18);
  canvas.setTextSize(4);
  canvas.setTextColor(have ? TFT_GREENYELLOW : TFT_ORANGE);
  canvas.drawString(have ? "FIX" : "NO FIX", W - 180, 14);

  int y = 88;
  canvas.setTextSize(3);

  // position
  canvas.setTextColor(TFT_CYAN);
  canvas.drawString("Position", 16, y); y += 40;
  canvas.setTextColor(TFT_WHITE);
  char buf[96];
  if (have) {
    snprintf(buf, sizeof(buf), "lat  %.6f", fix.lat);  canvas.drawString(buf, 32, y); y += 34;
    snprintf(buf, sizeof(buf), "lon  %.6f", fix.lon);  canvas.drawString(buf, 32, y); y += 34;
    snprintf(buf, sizeof(buf), "alt  %.1f m", fix.altitude); canvas.drawString(buf, 32, y); y += 34;
    snprintf(buf, sizeof(buf), "spd  %.1f kn", fix.speedKn); canvas.drawString(buf, 32, y); y += 44;
  } else {
    canvas.setTextColor(TFT_DARKGREY);
    canvas.drawString("waiting for fix", 32, y); y += 34;
    canvas.drawString("needs open sky, 30-90s cold", 32, y); y += 44;
  }

  // fix quality
  canvas.setTextColor(TFT_CYAN);
  canvas.drawString("Fix", 16, y); y += 40;
  canvas.setTextColor(TFT_WHITE);
  const char* modeStr = (fix.mode == 3) ? "3D" : (fix.mode == 2) ? "2D" : "none";
  snprintf(buf, sizeof(buf), "mode %s   sats %d   HDOP %.2f", modeStr, fix.sats, fix.hdop);
  canvas.drawString(buf, 32, y); y += 34;
  if (fix.utc[0]) {
    snprintf(buf, sizeof(buf), "UTC  %s   %s", fix.utc, fix.date);
    canvas.drawString(buf, 32, y);
  }
  y += 48;

  // constellation bars
  canvas.setTextColor(TFT_CYAN);
  canvas.drawString("Satellites visible", 16, y); y += 42;
  canvas.setTextSize(2);
  for (int i = 0; i < 4; i++) {
    canvas.setTextColor(TFT_WHITE);
    canvas.drawString(cons[i].name, 32, y + 4);
    // bar scaled to SNR (0-50)
    int bw = (W - 300);
    int fillw = (cons[i].bestSnr > 50 ? 50 : cons[i].bestSnr) * bw / 50;
    canvas.drawRect(110, y, bw, 24, TFT_DARKGREY);
    uint16_t bc = cons[i].bestSnr >= 35 ? TFT_GREEN
                : cons[i].bestSnr >= 25 ? TFT_YELLOW : TFT_RED;
    if (fillw > 2) canvas.fillRect(111, y + 1, fillw - 2, 22, bc);
    snprintf(buf, sizeof(buf), "%d vis  %d dB", cons[i].visible, cons[i].bestSnr);
    canvas.drawString(buf, 120 + bw, y + 4);
    y += 32;
  }

  // link status footer
  y += 16;
  canvas.setTextColor(TFT_DARKGREY);
  uint32_t age = millis() - fix.lastSentence;
  snprintf(buf, sizeof(buf), "UART G%d/G%d @%lu   %lu sentences   %lu B",
           PIN_GNSS_TX, PIN_GNSS_RX, (unsigned long)GNSS_BAUD,
           (unsigned long)sentenceCount, (unsigned long)byteCount);
  canvas.drawString(buf, 16, y); y += 26;
  if (age > 3000) {
    canvas.setTextColor(TFT_RED);
    canvas.drawString("NO DATA - check DIP / seating", 16, y);
  } else if (ppsCount) {
    snprintf(buf, sizeof(buf), "PPS %lu edges, %lu ms interval",
             (unsigned long)ppsCount, (unsigned long)ppsInterval);
    canvas.drawString(buf, 16, y);
  } else {
    canvas.drawString("PPS idle (starts after fix, needs DIP pos 3)", 16, y);
  }

  canvas.pushSprite(0, 0);
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(1);
  M5.Display.fillScreen(TFT_BLACK);

  Serial.begin(115200);
  M5.Power.setExtOutput(true);
  delay(500);

  canvas.setColorDepth(16);
  canvas.createSprite(M5.Display.width(), M5.Display.height());

  Serial1.begin(GNSS_BAUD, SERIAL_8N1, PIN_GNSS_TX, PIN_GNSS_RX);
  pinMode(PIN_PPS, INPUT_PULLDOWN);

  fix.lastSentence = millis();
}

char line[128];
int  linePos = 0;
uint32_t lastDraw = 0;

void loop() {
  M5.update();

  while (Serial1.available()) {
    char c = Serial1.read();
    byteCount++;
    Serial.write(c);                 // keep the serial mirror
    if (c == '\n') {
      line[linePos] = 0;
      parseSentence(line);
      linePos = 0;
    } else if (c != '\r') {
      if (linePos < (int)sizeof(line) - 1) line[linePos++] = c;
    }
  }

  int p = digitalRead(PIN_PPS);
  if (ppsPrev == 0 && p == 1) {
    uint32_t now = millis();
    if (ppsLast) ppsInterval = now - ppsLast;
    ppsLast = now;
    ppsCount++;
  }
  ppsPrev = p;

  if (millis() - lastDraw > 500) { drawScreen(); lastDraw = millis(); }
}
