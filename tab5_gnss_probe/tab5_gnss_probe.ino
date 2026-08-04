// tab5_gnss_probe.ino
// Safe bring-up probe for M5Stack Module GNSS (M135) on Tab5 (ESP32-P4) M-Bus.
//
// Stage 1: I2C scan  - confirms module is powered and seated (no DIP involvement)
// Stage 2: RX sweep   - finds which pin carries NEO-M9N TX, with NO output driven
// Stage 3: full port  - only run after stage 2 identifies the pin
//
// Safety notes:
//   * Stage 2 passes -1 as the TX pin so the P4 never drives the bus.
//     This avoids output-vs-output contention if the DIP selection is wrong.
//   * G35 (bus pin 24) is the P4 BOOT strap. It is deliberately NOT probed.
//   * G38/G37 (bus pins 13/14) are UART0. Not touched.

#include <M5Unified.h>
#include <Wire.h>

// ---- Tab5 M-Bus pin map (verified against M5Stack Stack Compatibility tool) ----
constexpr int PIN_SDA      = 31;   // bus 17, internal I2C
constexpr int PIN_SCL      = 32;   // bus 18, internal I2C

// Candidate pins the module may drive (module UART_TX side)
// bus 15 -> G7, bus 2 -> G16, bus 22 -> G48, bus 26 -> G51
const int rxCandidates[]   = { 7, 16, 48, 51 };
const char* rxLabels[]     = { "G7  (bus 15)", "G16 (bus 2)", "G48 (bus 22)", "G51 (bus 26)" };
constexpr int N_RX = sizeof(rxCandidates) / sizeof(rxCandidates[0]);

// Expected pairing for the recommended DIP setting (TX-4 / RX-4)
constexpr int PIN_GNSS_TX  = 7;    // Tab5 receives here
constexpr int PIN_GNSS_RX  = 6;    // Tab5 transmits here
constexpr int PIN_PPS      = 51;   // bus 26, if PPS block position 1

constexpr uint32_t GNSS_BAUD = 38400;   // NEO-M9N-00B factory default, 8N1

// -----------------------------------------------------------------------------

void i2cScan() {
  Serial.println("\n--- Stage 1: I2C scan on G31/G32 ---");
  Wire.begin(PIN_SDA, PIN_SCL, 100000);
  int found = 0;
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  0x%02X", addr);
      switch (addr) {
        case 0x69: Serial.print("  <- BMI270 (M135 module)");   break;
        case 0x76: Serial.print("  <- BMP280 (M135 module)");   break;
        case 0x68: Serial.print("  <- BMI270 (Tab5 onboard)");  break;
        case 0x43: case 0x44: Serial.print("  <- PI4IOE5V6408"); break;
        case 0x41: Serial.print("  <- INA226");                 break;
        case 0x32: Serial.print("  <- RX8130 RTC");             break;
        default: break;
      }
      Serial.println();
      found++;
    }
  }
  Serial.printf("  %d device(s).\n", found);
  Serial.println("  Expect 0x69 and 0x76 if the M135 is seated and powered.");
  Serial.println("  (BMM150 sits behind the BMI270 and will NOT appear.)");
}

// Listen on one candidate pin with TX unbound. Returns bytes seen.
int sniffPin(int pin, const char* label, uint32_t ms) {
  Serial.printf("  %-14s ... ", label);
  Serial1.end();
  delay(20);
  Serial1.begin(GNSS_BAUD, SERIAL_8N1, pin, -1);   // -1 = do not drive any TX
  delay(20);
  while (Serial1.available()) Serial1.read();      // flush

  uint32_t t0 = millis();
  int count = 0;
  String line;
  String firstSentence;
  while (millis() - t0 < ms) {
    while (Serial1.available()) {
      char c = Serial1.read();
      count++;
      if (c == '\n') {
        if (firstSentence.length() == 0 && line.startsWith("$")) firstSentence = line;
        line = "";
      } else if (c != '\r') {
        if (line.length() < 120) line += c;
      }
    }
    delay(1);
  }
  Serial1.end();

  Serial.printf("%4d bytes", count);
  if (firstSentence.length()) Serial.printf("   %s", firstSentence.c_str());
  Serial.println();
  return count;
}

void rxSweep() {
  Serial.println("\n--- Stage 2: RX sweep (no output driven) ---");
  Serial.printf("    %lu baud 8N1, 2s per pin\n", (unsigned long)GNSS_BAUD);
  int best = -1, bestCount = 0;
  for (int i = 0; i < N_RX; i++) {
    int n = sniffPin(rxCandidates[i], rxLabels[i], 2000);
    if (n > bestCount) { bestCount = n; best = i; }
  }
  Serial.println();
  if (bestCount > 0) {
    Serial.printf("  >> Module TX appears on %s\n", rxLabels[best]);
    Serial.println("     Set your UART RX to that pin.");
  } else {
    Serial.println("  >> Nothing received. Check, in order:");
    Serial.println("     - 5V enabled (setExtOutput) and module seated");
    Serial.println("     - Stage 1 found 0x69 / 0x76");
    Serial.println("     - DIP TX/RX blocks: exactly one switch ON in each");
    Serial.println("     - baud: try 9600 if the module was reconfigured");
  }
}

void streamNmea() {
  Serial.println("\n--- Stage 3: full port ---");
  Serial.printf("    RX=G%d  TX=G%d  @%lu\n",
                PIN_GNSS_TX, PIN_GNSS_RX, (unsigned long)GNSS_BAUD);
  Serial.println("    Streaming NMEA. Needs sky view for a fix.");
  Serial.println("    RMC field 2 = 'A' means valid fix, 'V' means no fix yet.\n");
  Serial1.begin(GNSS_BAUD, SERIAL_8N1, PIN_GNSS_TX, PIN_GNSS_RX);
  pinMode(PIN_PPS, INPUT);
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);

  Serial.begin(115200);
  delay(1500);
  Serial.println("\n\n=== Tab5 + Module GNSS (M135) probe ===");

  M5.Power.setExtOutput(true);          // enable EXT_5V_BUS to the M-Bus
  Serial.println("EXT 5V enabled. Waiting for module...");
  delay(1000);

  i2cScan();
  rxSweep();
  streamNmea();
}

uint32_t lastPps = 0;
int ppsPrev = -1;

void loop() {
  while (Serial1.available()) Serial.write(Serial1.read());

  int p = digitalRead(PIN_PPS);
  if (ppsPrev == 0 && p == 1) {
    uint32_t now = millis();
    if (lastPps) Serial.printf("\n[PPS edge, %lu ms since last]\n", (unsigned long)(now - lastPps));
    lastPps = now;
  }
  ppsPrev = p;

  M5.update();
}
