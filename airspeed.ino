#include <Wire.h>
#include <bluefruit.h>
#include "ms4525do.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// PCA9546 I²C multiplexer
#define PCAADDR       0x70   // multiplexer address
#define SENSOR_PORT     0    // sensor on port 0
#define DISPLAY_PORT    1    // display on port 1

void pcaselect(uint8_t port) {
  if (port > 3) return;  
  Wire.beginTransmission(PCAADDR);
  Wire.write(1 << port);
  Wire.endTransmission();
  Serial.print("Switched to I2C port ");
  Serial.println(port);
}

#define OFFSET_ADDR (0x7F000)  // last 4KB page of flash
float offset = 0.0f;

unsigned long buttonPressTime = 0;
bool buttonHeld = false;
#define HOLD_TIME 2000
#define AVG_WINDOW 50

float avgBuffer[AVG_WINDOW] = {0.0f};
int avgIndex = 0;
bool avgFilled = false;

Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);

#define SENSOR_ADDR   0x28
#define SCREEN_ADDR   0x3C
#define VBATPIN       A6

const float ATM_PRESSURE  = 101325.0f;
const float GAS_CONST_AIR = 287.05f;

BLEUart bleuart;
bfs::Ms4525do pres;

int uiMode = 0; // 0: bold speed, 1: debug, 2: graph
bool lastButtonState = false;

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define PLOT_HEIGHT   54  // leave 10px for labels

float speedBuffer[SCREEN_WIDTH] = {0};  // one speed sample per x pixel

void setup() {
  Serial.begin(115200);
  while (!Serial); // wait for USB serial
  delay(100);

  // Load offset from flash
  offset = *((float*)OFFSET_ADDR);
  if (isnan(offset) || offset < -500 || offset > 500) {
    offset = 0.0f; // sanity check
  }
  Serial.print("Loaded offset: ");
  Serial.println(offset);

  Wire.begin();

  // Initialize sensor on its port
  pcaselect(SENSOR_PORT);
  Wire.setClock(400000);

  pres.Config(&Wire, SENSOR_ADDR, 1.0f, -1.0f);
  if (!pres.Begin()) {
    Serial.println("Error: sensor not found");
    while (1);
  }
  Serial.println("Sensor initialized.");

  // Initialize display on its port
  pcaselect(DISPLAY_PORT);
  if (!display.begin(SCREEN_ADDR, true)) {
    Serial.println("Error: display not found");
    while (1);
  }
  display.setRotation(3);
  display.clearDisplay();
  display.display();         
  delay(1000);    
  Serial.println("Display initialized.");

  Bluefruit.begin();
  Bluefruit.setName("nRF52840_Airspeed");
  bleuart.begin();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.start(0);

  pinMode(PIN_BUTTON1, INPUT_PULLUP);

  Serial.println("BLE initialized.");
  Serial.println("Setup complete.");
}

void drawBluetoothSymbol(int x, int y, uint16_t color) {
  display.drawLine(x, y, x, y + 9, color);
  display.drawLine(x, y, x + 3, y + 3, color);
  display.drawLine(x + 3, y + 3, x, y + 6, color);
  display.drawLine(x, y + 6, x + 3, y + 9, color);
  display.drawLine(x + 3, y + 9, x, y + 12, color);
}

void loop() {
  bool buttonPressed = digitalRead(PIN_BUTTON1) == LOW;
  if (buttonPressed && !lastButtonState) {
    uiMode = (uiMode + 1) % 3;
    Serial.print("UI mode changed to ");
    Serial.println(uiMode);
    delay(200);
  }
  lastButtonState = buttonPressed;

  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.6;  // Multiply by 3.6V reference
  measuredvbat /= 1024; // convert to voltage
  Serial.print("Battery voltage: ");
  Serial.print(measuredvbat, 2);
  Serial.println(" V");

  bool buttonDown = digitalRead(PIN_BUTTON1) == LOW;
  if (buttonDown) {
    if (!buttonHeld) {
      buttonHeld = true;
      buttonPressTime = millis();
      Serial.println("Button hold started");
    } else if (millis() - buttonPressTime >= HOLD_TIME) {
      Serial.println("Calibrating offset...");

      // Draw calibrating screen
      pcaselect(DISPLAY_PORT);
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(28, 20);
      display.print("Calibrating...");
      display.setCursor(28, 36);
      display.print("Hold steady...");
      display.display(); 

      float sum = 0.0f;
      int count = avgFilled ? AVG_WINDOW : avgIndex;

      for (int i = 0; i < count; ++i) {
        sum += avgBuffer[i];
        delay(30);
      }

      float newOffset = sum / count;
      offset = newOffset;

      // Save to flash
      sd_flash_page_erase(OFFSET_ADDR / 4096);
      while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
      sd_flash_write((uint32_t*)OFFSET_ADDR, (uint32_t*)&offset, 1);
      while (NRF_NVMC->READY == NVMC_READY_READY_Busy);

      Serial.print("New offset saved: ");
      Serial.println(offset);

      display.clearDisplay();
      display.setCursor(20, 24);
      display.print("New offset: ");
      display.print(offset,0);
      display.display();
      delay(2000);

      buttonHeld = false;
    }
  } else {
    if (buttonHeld) {
      Serial.println("Button released before calibration");
    }
    buttonHeld = false;
  }

  // Read sensor data on its port
  pcaselect(SENSOR_PORT);
  if (pres.Read()) {
    float pressureRaw = pres.pres_pa();
    Serial.print("Raw pressure: ");
    Serial.print(pressureRaw, 2);
    Serial.println(" Pa");

    // Add to average buffer
    avgBuffer[avgIndex++] = pressureRaw;
    if (avgIndex >= AVG_WINDOW) {
      avgIndex = 0;
      avgFilled = true;
    }

    float pressurePa = pressureRaw - offset;
    Serial.print("Adjusted pressure: ");
    Serial.print(pressurePa, 2);
    Serial.println(" Pa");

    float tempC = pres.die_temp_c();
    Serial.print("Temp (C): ");
    Serial.println(tempC, 2);

    float tempK = tempC + 273.15f;
    float rho = ATM_PRESSURE / (GAS_CONST_AIR * tempK);
    Serial.print("Air density: ");
    Serial.println(rho, 4);

    float v = 0.0f;
    if (fabsf(pressurePa) > 5.0f) {
      v = 3.6f * sqrtf(2.0f * fabsf(pressurePa) / rho);
      if (pressurePa < 0) v = -v;
    }
    Serial.print("Computed speed: ");
    Serial.print(v, 2);
    Serial.println(" km/h");

    for (int i = 0; i < SCREEN_WIDTH - 1; ++i) {
      speedBuffer[i] = speedBuffer[i + 1];
    }
    speedBuffer[SCREEN_WIDTH - 1] = v;

    // Draw on display port
    pcaselect(DISPLAY_PORT);
    display.clearDisplay();

    if (uiMode == 0) {
      display.setTextSize(2);
      char speedStr[16];
      snprintf(speedStr, sizeof(speedStr), "%.1f km/h", v);
      int16_t x1, y1;
      uint16_t w, h;
      display.getTextBounds(speedStr, 0, 0, &x1, &y1, &w, &h);
      display.setCursor(128 - w - 2, 20);
      display.print(speedStr);
    }
    else if (uiMode == 1) {
      display.setTextSize(1);
      display.setTextColor(SH110X_WHITE);

      display.fillRect(0, 0, 128, 12, SH110X_WHITE);

      int bx = 2, by = 3, bw = 10, bh = 6;
      display.drawRect(bx, by, bw, bh, SH110X_BLACK);
      display.fillRect(bx + bw, by + 2, 2, 2, SH110X_BLACK);

      display.setCursor(bx + bw + 6, by);
      display.setTextColor(SH110X_BLACK);
      display.print(measuredvbat, 1);
      display.print(" V");

      if (Bluefruit.connected()) {
        drawBluetoothSymbol(118, 1, SH110X_BLACK);
      }

      display.setTextColor(SH110X_WHITE);

      display.drawRect(0, 14, 64, 20, SH110X_WHITE);
      display.setCursor(2, 16);
      display.print("P:");
      display.print(pressureRaw, 0);
      display.print(" Pa");

      display.drawRect(66, 14, 62, 20, SH110X_WHITE);
      display.setCursor(68, 16);
      display.print(tempC, 1);
      display.print("C ");
      display.setCursor(68, 24);
      display.print(rho, 1);
      display.print("kg/m3");

      display.drawRect(0, 36, 128, 28, SH110X_WHITE);

      display.setTextSize(2);
      String speedStr = String(v, 1);
      int speedWidth = speedStr.length() * 6 * 2;
      int unitWidth = 5 * 6;
      int startX = 128 - (speedWidth + unitWidth) - 2;

      display.setCursor(startX, 42);
      display.print(speedStr);
      display.setTextSize(1);
      display.print(" km/h");
    }
    else if (uiMode == 2) {
      display.clearDisplay();
      display.drawLine(0, SCREEN_HEIGHT - 1,
                       SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1,
                       SH110X_WHITE);
      display.drawLine(0, SCREEN_HEIGHT - PLOT_HEIGHT,
                       0, SCREEN_HEIGHT - 1,
                       SH110X_WHITE);

      float maxSpeed = 80.0f;
      for (int x = 1; x < SCREEN_WIDTH; ++x) {
        int y0 = SCREEN_HEIGHT - 1 - (int)(PLOT_HEIGHT *
                  constrain(speedBuffer[x - 1] / maxSpeed, 0.0f, 1.0f));
        int y1 = SCREEN_HEIGHT - 1 - (int)(PLOT_HEIGHT *
                  constrain(speedBuffer[x] / maxSpeed, 0.0f, 1.0f));
        display.drawLine(x - 1, y0, x, y1, SH110X_WHITE);
      }

      display.setTextSize(1);
      char speedStr[16];
      snprintf(speedStr, sizeof(speedStr), "%.1f km/h", v);
      display.setCursor(SCREEN_WIDTH - 6 * strlen(speedStr), 0);
      display.print(speedStr);
      display.display();
    }

    display.display();

    if (Bluefruit.connected()) {
      bleuart.print(v, 3);
      bleuart.print("\r\n");
    }
  } else {
    Serial.println("Warning: sensor read failed");
  }

  delay(100);
}