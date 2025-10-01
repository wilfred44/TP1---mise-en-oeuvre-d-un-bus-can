// ===== ESP32 CAN (ACAN_ESP32) + OLED + POTENTIOMETRE + BOUTON =====
// Envoi statut potentiomètre et bouton vers Arduino UNO
// Affichage détaillé réception ID 0x101

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ACAN_ESP32.h>

// ========== CONFIGURATION OLED ==========
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SDA_PIN 21
#define SCL_PIN 22
#define SCREEN_ADDRESS 0x3C  // Changez en 0x3D si nécessaire

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ========== CONFIGURATION CAN ==========
#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4

#define MY_TX_ID    0x101  // ESP32 envoie sur 0x101
#define REMOTE_RX_ID 0x100 // ESP32 reçoit 0x100 d'Arduino
#define LISTEN_ID   0x101  // ID à écouter (messages de notre propre ESP32 ou d'un autre ESP32)

// ========== COMMANDES CAN ==========
#define CMD_POT_STATUS    0x10  // Commande: Statut potentiomètre
#define CMD_BUTTON_STATUS 0x11  // Commande: Statut bouton
#define CMD_ALL_STATUS    0x12  // Commande: Tous les statuts

// ========== CONFIGURATION ENTRÉES ==========
#define POT_PIN 36        // Potentiomètre sur GPIO 36 (ADC1_CH0)
#define BUTTON_PIN 34     // Bouton sur GPIO 34 (INPUT_ONLY)

// ========== VARIABLES ==========
static uint32_t lastSend = 0;
static uint32_t lastDisplayUpdate = 0;
static uint32_t txCount = 0;
static uint32_t rxCount = 0;
static uint32_t rx101Count = 0;  // Compteur spécifique ID 0x101
static bool arduinoConnected = false;
static uint32_t lastRxTime = 0;

// Variables potentiomètre
static uint16_t potValue = 0;
static uint16_t lastPotValue = 0;
static uint8_t potPercent = 0;

// Variables bouton
static bool buttonState = false;
static bool lastButtonState = false;
static uint32_t buttonPressCount = 0;
static uint32_t lastDebounceTime = 0;
static const uint32_t debounceDelay = 50;

// Dernières données reçues
static uint8_t lastRxData[8] = {0};
static uint8_t lastRxDLC = 0;

// Seuil de changement pour envoi potentiomètre
#define POT_CHANGE_THRESHOLD 50  // Envoyer si changement > 50 (~1.2%)

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n========================================");
  Serial.println("  ESP32 ACAN + OLED + POT + BUTTON");
  Serial.println("  Envoi statut vers Arduino");
  Serial.println("  Ecoute ID: 0x101");
  Serial.println("========================================");

  // ========== INITIALISATION ENTRÉES ==========
  pinMode(BUTTON_PIN, INPUT);  // GPIO 34 est INPUT_ONLY
  
  Serial.println("\nConfiguration GPIO:");
  Serial.print("  Potentiometre : GPIO ");
  Serial.println(POT_PIN);
  Serial.print("  Bouton        : GPIO ");
  Serial.println(BUTTON_PIN);

  // ========== INITIALISATION I2C + OLED ==========
  Serial.println("\nInitialisation OLED...");
  Wire.begin(SDA_PIN, SCL_PIN);
  
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("❌ Erreur OLED!");
    Serial.println("Verifiez le cablage et l'adresse I2C");
  } else {
    Serial.println("✅ OLED OK");
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("ESP32 ACAN"));
    display.println(F("Initialisation..."));
    display.display();
  }

  // ========== INITIALISATION CAN (ACAN_ESP32) ==========
  Serial.println("\nInitialisation CAN (ACAN_ESP32)...");
  Serial.print("  TX : GPIO ");
  Serial.println(CAN_TX_PIN);
  Serial.print("  RX : GPIO ");
  Serial.println(CAN_RX_PIN);
  Serial.println("  Vitesse : 500 kbps");
  Serial.print("  TX ID   : 0x");
  Serial.println(MY_TX_ID, HEX);
  Serial.print("  RX ID   : 0x");
  Serial.println(REMOTE_RX_ID, HEX);
  Serial.print("  LISTEN  : 0x");
  Serial.println(LISTEN_ID, HEX);

  // Configuration ACAN_ESP32
  ACAN_ESP32_Settings settings(500 * 1000);  // 500 kbps
  settings.mRxPin = CAN_RX_PIN;
  settings.mTxPin = CAN_TX_PIN;
  settings.mRequestedCANMode = ACAN_ESP32_Settings::NormalMode;

  // Début CAN
  const uint32_t errorCode = ACAN_ESP32::can.begin(settings);
  
  if (errorCode == 0) {
    Serial.println("✅ CAN OK");
    Serial.print("  Bitrate: ");
    Serial.print(settings.actualBitRate() / 1000);
    Serial.println(" kbps");
    Serial.print("  Sample point: ");
    Serial.print(settings.samplePointFromBitStart());
    Serial.println("%");
  } else {
    Serial.print("❌ Erreur CAN: 0x");
    Serial.println(errorCode, HEX);
    while(1) delay(1000);
  }

  Serial.println("\n========================================");
  Serial.println("Systeme pret!");
  Serial.println("En attente de messages ID 0x101...");
  Serial.println("========================================\n");

  delay(1000);
}

// ========== FONCTION: DECODER MESSAGE ID 0x101 ==========
void decodeMessage101(CANMessage &frame) {
  Serial.println("\n╔════════════════════════════════════════════════╗");
  Serial.println("║     MESSAGE REÇU DE ID 0x101                  ║");
  Serial.println("╚════════════════════════════════════════════════╝");
  
  // Afficher données brutes
  Serial.print("📦 Données brutes (");
  Serial.print(frame.len);
  Serial.print(" octets): ");
  for (uint8_t i = 0; i < frame.len; i++) {
    if (frame.data[i] < 0x10) Serial.print("0");
    Serial.print(frame.data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  
  // Vérifier longueur minimale
  if (frame.len < 1) {
    Serial.println("⚠️  Message trop court (DLC < 1)");
    Serial.println("─────────────────────────────────────────────────\n");
    return;
  }
  
  uint8_t cmd = frame.data[0];
  Serial.print("🔧 Commande: 0x");
  if (cmd < 0x10) Serial.print("0");
  Serial.print(cmd, HEX);
  Serial.print(" (");
  
  // Décoder selon la commande
  switch(cmd) {
    case CMD_POT_STATUS: {  // 0x10
      Serial.println("POT_STATUS)");
      
      if (frame.len >= 8) {
        uint16_t potVal = (frame.data[1] << 8) | frame.data[2];
        uint8_t percent = frame.data[3];
        uint8_t pwm = frame.data[4];
        uint16_t timestamp = (frame.data[5] << 8) | frame.data[6];
        uint8_t checksum = frame.data[7];
        
        // Vérifier checksum
        uint8_t calcChecksum = 0;
        for (int i = 0; i < 7; i++) {
          calcChecksum ^= frame.data[i];
        }
        
        Serial.println("\n📊 DÉCODAGE POTENTIOMÈTRE:");
        Serial.print("   • Valeur brute  : ");
        Serial.print(potVal);
        Serial.println(" (0-4095)");
        
        Serial.print("   • Pourcentage   : ");
        Serial.print(percent);
        Serial.println(" %");
        
        Serial.print("   • Valeur PWM    : ");
        Serial.print(pwm);
        Serial.println(" (0-255)");
        
        Serial.print("   • Timestamp     : ");
        Serial.print(timestamp);
        Serial.println(" (x100ms)");
        
        Serial.print("   • Checksum      : 0x");
        if (checksum < 0x10) Serial.print("0");
        Serial.print(checksum, HEX);
        
        if (checksum == calcChecksum) {
          Serial.println(" ✅ VALIDE");
        } else {
          Serial.print(" ❌ INVALIDE (attendu: 0x");
          if (calcChecksum < 0x10) Serial.print("0");
          Serial.print(calcChecksum, HEX);
          Serial.println(")");
        }
        
        // Affichage graphique
        Serial.print("\n   📈 ");
        int bars = map(percent, 0, 100, 0, 40);
        Serial.print("[");
        for (int i = 0; i < 40; i++) {
          if (i < bars) Serial.print("█");
          else Serial.print("·");
        }
        Serial.println("]");
      } else {
        Serial.println("\n⚠️  Données incomplètes");
      }
      break;
    }
    
    case CMD_BUTTON_STATUS: {  // 0x11
      Serial.println("BUTTON_STATUS)");
      
      if (frame.len >= 8) {
        bool state = frame.data[1];
        uint16_t count = (frame.data[2] << 8) | frame.data[3];
        uint16_t timestamp = (frame.data[4] << 8) | frame.data[5];
        uint8_t checksum = frame.data[7];
        
        // Vérifier checksum
        uint8_t calcChecksum = 0;
        for (int i = 0; i < 7; i++) {
          calcChecksum ^= frame.data[i];
        }
        
        Serial.println("\n🔘 DÉCODAGE BOUTON:");
        Serial.print("   • État          : ");
        Serial.println(state ? "APPUYÉ (ON)" : "RELÂCHÉ (OFF)");
        
        Serial.print("   • Compteur      : ");
        Serial.print(count);
        Serial.println(" appuis");
        
        Serial.print("   • Timestamp     : ");
        Serial.print(timestamp);
        Serial.println(" (x100ms)");
        
        Serial.print("   • Checksum      : 0x");
        if (checksum < 0x10) Serial.print("0");
        Serial.print(checksum, HEX);
        
        if (checksum == calcChecksum) {
          Serial.println(" ✅ VALIDE");
        } else {
          Serial.print(" ❌ INVALIDE (attendu: 0x");
          if (calcChecksum < 0x10) Serial.print("0");
          Serial.print(calcChecksum, HEX);
          Serial.println(")");
        }
        
        // Affichage visuel
        Serial.print("\n   🔵 Bouton: ");
        if (state) {
          Serial.println("[●●●●●] APPUYÉ");
        } else {
          Serial.println("[○○○○○] RELÂCHÉ");
        }
      } else {
        Serial.println("\n⚠️  Données incomplètes");
      }
      break;
    }
    
    case CMD_ALL_STATUS: {  // 0x12
      Serial.println("ALL_STATUS)");
      
      if (frame.len >= 8) {
        uint16_t potVal = (frame.data[1] << 8) | frame.data[2];
        uint8_t percent = frame.data[3];
        bool btnState = (frame.data[4] & 0x80) ? true : false;
        uint8_t btnCount = frame.data[4] & 0x7F;
        uint8_t pwm = frame.data[5];
        uint8_t timestamp = frame.data[6];
        uint8_t checksum = frame.data[7];
        
        // Vérifier checksum
        uint8_t calcChecksum = 0;
        for (int i = 0; i < 7; i++) {
          calcChecksum ^= frame.data[i];
        }
        
        Serial.println("\n📊 DÉCODAGE COMPLET:");
        Serial.println("   ┌─ POTENTIOMÈTRE ─────────────");
        Serial.print("   │ Valeur brute  : ");
        Serial.print(potVal);
        Serial.println(" (0-4095)");
        
        Serial.print("   │ Pourcentage   : ");
        Serial.print(percent);
        Serial.println(" %");
        
        Serial.print("   │ Valeur PWM    : ");
        Serial.print(pwm);
        Serial.println(" (0-255)");
        
        Serial.println("   ├─ BOUTON ────────────────────");
        Serial.print("   │ État         : ");
        Serial.println(btnState ? "APPUYÉ (ON)" : "RELÂCHÉ (OFF)");
        
        Serial.print("   │ Compteur     : ");
        Serial.print(btnCount);
        Serial.println(" appuis");
        
        Serial.println("   └─ SYSTÈME ───────────────────");
        Serial.print("     Timestamp   : ");
        Serial.print(timestamp);
        Serial.println(" s");
        
        Serial.print("     Checksum    : 0x");
        if (checksum < 0x10) Serial.print("0");
        Serial.print(checksum, HEX);
        
        if (checksum == calcChecksum) {
          Serial.println(" ✅ VALIDE");
        } else {
          Serial.print(" ❌ INVALIDE (attendu: 0x");
          if (calcChecksum < 0x10) Serial.print("0");
          Serial.print(calcChecksum, HEX);
          Serial.println(")");
        }
        
        // Affichage graphique
        Serial.print("\n   📈 POT: ");
        int bars = map(percent, 0, 100, 0, 30);
        Serial.print("[");
        for (int i = 0; i < 30; i++) {
          if (i < bars) Serial.print("█");
          else Serial.print("·");
        }
        Serial.print("] ");
        Serial.print(percent);
        Serial.println("%");
        
        Serial.print("   🔵 BTN: ");
        if (btnState) {
          Serial.print("[●●●●●] ON ");
        } else {
          Serial.print("[○○○○○] OFF");
        }
        Serial.print("  (Appuis: ");
        Serial.print(btnCount);
        Serial.println(")");
      } else {
        Serial.println("\n⚠️  Données incomplètes");
      }
      break;
    }
    
    default:
      Serial.print("INCONNUE - 0x");
      if (cmd < 0x10) Serial.print("0");
      Serial.print(cmd, HEX);
      Serial.println(")");
      Serial.println("\n⚠️  Commande non reconnue");
      
      // Afficher quand même les données
      Serial.println("\n📦 Données (interprétation décimale):");
      for (uint8_t i = 0; i < frame.len; i++) {
        Serial.print("   Byte[");
        Serial.print(i);
        Serial.print("] = ");
        Serial.print(frame.data[i]);
        Serial.print(" (0x");
        if (frame.data[i] < 0x10) Serial.print("0");
        Serial.print(frame.data[i], HEX);
        Serial.println(")");
      }
      break;
  }
  
  Serial.println("─────────────────────────────────────────────────\n");
}

// ========== FONCTION: ENVOYER STATUT POTENTIOMETRE ==========
void sendPotStatus() {
  CANMessage txFrame;
  txFrame.id = MY_TX_ID;
  txFrame.len = 8;
  txFrame.ext = false;
  txFrame.rtr = false;

  txFrame.data[0] = CMD_POT_STATUS;
  txFrame.data[1] = (potValue >> 8) & 0xFF;
  txFrame.data[2] = potValue & 0xFF;
  txFrame.data[3] = potPercent;
  
  uint8_t pwmValue = map(potValue, 0, 4095, 0, 255);
  txFrame.data[4] = pwmValue;
  
  uint16_t timestamp = (millis() / 100) & 0xFFFF;
  txFrame.data[5] = (timestamp >> 8) & 0xFF;
  txFrame.data[6] = timestamp & 0xFF;
  
  uint8_t checksum = 0;
  for (int i = 0; i < 7; i++) {
    checksum ^= txFrame.data[i];
  }
  txFrame.data[7] = checksum;

  bool success = ACAN_ESP32::can.tryToSend(txFrame);

  if (success) {
    txCount++;
    Serial.print("📤 TX POT [0x");
    if (MY_TX_ID < 0x100) Serial.print("0");
    if (MY_TX_ID < 0x10) Serial.print("0");
    Serial.print(MY_TX_ID, HEX);
    Serial.print("] Val=");
    Serial.print(potValue);
    Serial.print(" (");
    Serial.print(potPercent);
    Serial.print("%) PWM=");
    Serial.println(pwmValue);
  }
}

// ========== FONCTION: ENVOYER STATUT BOUTON ==========
void sendButtonStatus() {
  CANMessage txFrame;
  txFrame.id = MY_TX_ID;
  txFrame.len = 8;
  txFrame.ext = false;
  txFrame.rtr = false;

  txFrame.data[0] = CMD_BUTTON_STATUS;
  txFrame.data[1] = buttonState ? 0x01 : 0x00;
  txFrame.data[2] = (buttonPressCount >> 8) & 0xFF;
  txFrame.data[3] = buttonPressCount & 0xFF;
  
  uint16_t timestamp = (millis() / 100) & 0xFFFF;
  txFrame.data[4] = (timestamp >> 8) & 0xFF;
  txFrame.data[5] = timestamp & 0xFF;
  txFrame.data[6] = 0x00;
  
  uint8_t checksum = 0;
  for (int i = 0; i < 7; i++) {
    checksum ^= txFrame.data[i];
  }
  txFrame.data[7] = checksum;

  bool success = ACAN_ESP32::can.tryToSend(txFrame);

  if (success) {
    txCount++;
    Serial.print("📤 TX BTN [0x");
    if (MY_TX_ID < 0x100) Serial.print("0");
    if (MY_TX_ID < 0x10) Serial.print("0");
    Serial.print(MY_TX_ID, HEX);
    Serial.print("] State=");
    Serial.print(buttonState ? "ON" : "OFF");
    Serial.print(" Count=");
    Serial.println(buttonPressCount);
  }
}

// ========== FONCTION: ENVOYER TOUS LES STATUTS ==========
void sendAllStatus() {
  CANMessage txFrame;
  txFrame.id = MY_TX_ID;
  txFrame.len = 8;
  txFrame.ext = false;
  txFrame.rtr = false;

  txFrame.data[0] = CMD_ALL_STATUS;
  txFrame.data[1] = (potValue >> 8) & 0xFF;
  txFrame.data[2] = potValue & 0xFF;
  txFrame.data[3] = potPercent;
  txFrame.data[4] = (buttonState ? 0x80 : 0x00) | (buttonPressCount & 0x7F);
  
  uint8_t pwmValue = map(potValue, 0, 4095, 0, 255);
  txFrame.data[5] = pwmValue;
  txFrame.data[6] = (millis() / 1000) & 0xFF;
  
  uint8_t checksum = 0;
  for (int i = 0; i < 7; i++) {
    checksum ^= txFrame.data[i];
  }
  txFrame.data[7] = checksum;

  bool success = ACAN_ESP32::can.tryToSend(txFrame);

  if (success) {
    txCount++;
    Serial.print("📤 TX ALL [0x");
    if (MY_TX_ID < 0x100) Serial.print("0");
    if (MY_TX_ID < 0x10) Serial.print("0");
    Serial.print(MY_TX_ID, HEX);
    Serial.print("] POT=");
    Serial.print(potValue);
    Serial.print(" (");
    Serial.print(potPercent);
    Serial.print("%) BTN=");
    Serial.print(buttonState ? "ON" : "OFF");
    Serial.print(" PWM=");
    Serial.println(pwmValue);
  }
}

void loop() {
  // ========== LECTURE POTENTIOMÈTRE ==========
  potValue = analogRead(POT_PIN);
  potPercent = map(potValue, 0, 4095, 0, 100);

  if (abs(potValue - lastPotValue) > POT_CHANGE_THRESHOLD) {
    Serial.print("🎛️  POT changé: ");
    Serial.print(lastPotValue);
    Serial.print(" → ");
    Serial.print(potValue);
    Serial.print(" (");
    Serial.print(potPercent);
    Serial.println("%)");
    
    sendPotStatus();
    lastPotValue = potValue;
  }

  // ========== LECTURE BOUTON ==========
  bool reading = digitalRead(BUTTON_PIN);
  
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      
      if (buttonState == HIGH) {
        buttonPressCount++;
        Serial.println("🔘 BOUTON APPUYE!");
        Serial.print("   Nombre d'appuis: ");
        Serial.println(buttonPressCount);
        
        sendButtonStatus();
      }
    }
  }
  lastButtonState = reading;

  // ========== RÉCEPTION CAN ==========
  CANMessage rxFrame;
  
  while (ACAN_ESP32::can.receive(rxFrame)) {
    rxCount++;
    
    // Traiter message de l'ID 0x101
    if (rxFrame.id == LISTEN_ID) {
      rx101Count++;
      decodeMessage101(rxFrame);
    }
    // Traiter message de l'Arduino (0x100)
    else if (rxFrame.id == REMOTE_RX_ID) {
      lastRxTime = millis();
      
      if (!arduinoConnected) {
        arduinoConnected = true;
        Serial.println("\n✅ ARDUINO CONNECTE!\n");
      }

      lastRxDLC = rxFrame.len;
      for (uint8_t i = 0; i < lastRxDLC && i < 8; i++) {
        lastRxData[i] = rxFrame.data[i];
      }

      Serial.print("📥 RX Arduino [0x");
      if (rxFrame.id < 0x100) Serial.print("0");
      if (rxFrame.id < 0x10) Serial.print("0");
      Serial.print(rxFrame.id, HEX);
      Serial.print("] DLC=");
      Serial.print(rxFrame.len);
      Serial.print(" Data: ");
      for (uint8_t i = 0; i < rxFrame.len; i++) {
        if (rxFrame.data[i] < 0x10) Serial.print("0");
        Serial.print(rxFrame.data[i], HEX);
        Serial.print(" ");
      }

      if (rxFrame.len >= 3) {
        uint8_t cmd = rxFrame.data[0];
        uint16_t value = (rxFrame.data[1] << 8) | rxFrame.data[2];
        Serial.print(" | CMD=0x");
        if (cmd < 0x10) Serial.print("0");
        Serial.print(cmd, HEX);
        Serial.print(" VAL=");
        Serial.print(value);
      }
      Serial.println();
    }
    // Autre ID
    else {
      Serial.print("📥 RX Autre [0x");
      if (rxFrame.id < 0x100) Serial.print("0");
      if (rxFrame.id < 0x10) Serial.print("0");
      Serial.print(rxFrame.id, HEX);
      Serial.print("] DLC=");
      Serial.print(rxFrame.len);
      Serial.print(" Data: ");
      for (uint8_t i = 0; i < rxFrame.len; i++) {
        if (rxFrame.data[i] < 0x10) Serial.print("0");
        Serial.print(rxFrame.data[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
  }

  // Vérifier timeout Arduino
  if (arduinoConnected && (millis() - lastRxTime > 3000)) {
    arduinoConnected = false;
    Serial.println("\n⚠️  ARDUINO DECONNECTE\n");
  }

  // ========== ENVOI CAN PÉRIODIQUE ==========
  if (millis() - lastSend >= 1000) {
    lastSend = millis();
    sendAllStatus();
  }

  // ========== MISE À JOUR OLED ==========
  if (millis() - lastDisplayUpdate >= 200) {
    lastDisplayUpdate = millis();

    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);

    display.setTextSize(1);
    display.println(F("ESP32->Arduino"));
    display.drawLine(0, 9, 127, 9, SSD1306_WHITE);

    display.setCursor(0, 12);
    display.print(F("UNO: "));
    if (arduinoConnected) {
      display.print(F("ON  "));
      uint32_t secAgo = (millis() - lastRxTime) / 1000;
      if (secAgo < 10) display.print(" ");
      display.print(secAgo);
      display.print(F("s"));
    } else {
      display.println(F("WAIT..."));
    }

    display.setCursor(0, 22);
    display.print(F("TX:"));
    display.print(txCount);
    display.print(F(" RX:"));
    display.print(rxCount);
    display.print(F(" [101]:"));
    display.print(rx101Count);

    display.drawLine(0, 31, 127, 31, SSD1306_WHITE);

    display.setCursor(0, 34);
    display.print(F("POT: "));
    display.print(potPercent);
    display.print(F("% ("));
    display.print(potValue);
    display.print(F(")"));

    display.drawRect(0, 44, 128, 8, SSD1306_WHITE);
    int barWidth = map(potPercent, 0, 100, 0, 126);
    display.fillRect(1, 45, barWidth, 6, SSD1306_WHITE);

    display.setCursor(0, 54);
    display.print(F("BTN: "));
    if (buttonState) {
      display.print(F("[ON]"));
    } else {
      display.print(F(" OFF"));
    }
    display.print(F(" #"));
    display.print(buttonPressCount);

    display.display();
  }

  delay(10);
}
