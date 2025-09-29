#ifndef ARDUINO_ARCH_ESP32
#error "Select an ESP32 board"
#endif

//----------------------------------------------------------------------------------------
//   Include files
//----------------------------------------------------------------------------------------
#include <ACAN2515.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//----------------------------------------------------------------------------------------
//   OLED Configuration
//----------------------------------------------------------------------------------------
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C // See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//----------------------------------------------------------------------------------------
//   MCP2515 Configuration
//----------------------------------------------------------------------------------------
static const byte MCP2515_SCK  = 18 ; // SCK input of MCP2515 (GPIO18)
static const byte MCP2515_MOSI = 23 ; // SDI input of MCP2515 (GPIO23)
static const byte MCP2515_MISO = 19 ; // SDO output of MCP2515 (GPIO19)
static const byte MCP2515_CS   = 5 ;  // CS input of MCP2515 (GPIO5)

ACAN2515 can (MCP2515_CS, SPI, 255) ; // Last parameter is interrupt pin (255 = not used)

//----------------------------------------------------------------------------------------
//   Variables
//----------------------------------------------------------------------------------------
static const int LED_BUILTIN = 2; // LED connectée au GPIO2
static const uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL ; // 8 MHz

// Compteurs pour l'affichage
uint32_t messagesSent = 0;
uint32_t messagesReceived = 0;
String lastSentData = "";
String lastReceivedData = "";
unsigned long lastReceivedTime = 0;

//----------------------------------------------------------------------------------------
//   SETUP
//----------------------------------------------------------------------------------------
void setup()
{
  //--- Switch on builtin led
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  //--- Start serial
  Serial.begin(115200);
  delay(500);

  //--- Initialize OLED display
  Serial.println("=== ESP32 + MCP2515 CAN + OLED Monitor ===");
  Serial.println("Initialisation de l'afficheur OLED...");

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("❌ ERREUR: Échec initialisation OLED SSD1306!");
    for(;;); // Boucle infinie
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("ESP32+MCP2515 CAN");
  display.println("Initialisation...");
  display.display();

  Serial.println("✅ OLED initialisé avec succès!");

  //--- Configure SPI
  Serial.println("Configuration SPI...");
  SPI.begin(MCP2515_SCK, MCP2515_MISO, MCP2515_MOSI);

  //--- Configure MCP2515
  Serial.println("Configuration du MCP2515...");
  Serial.print("Pins utilisées - CS: GPIO");
  Serial.print(MCP2515_CS);
  Serial.print(", SCK: GPIO");
  Serial.print(MCP2515_SCK);
  Serial.print(", MOSI: GPIO");
  Serial.print(MCP2515_MOSI);
  Serial.print(", MISO: GPIO");
  Serial.println(MCP2515_MISO);

  ACAN2515Settings settings(QUARTZ_FREQUENCY, 500UL * 1000UL) ; // 500 kbps

  // Optionnel: configurer les modes
  settings.mRequestedMode = ACAN2515Settings::NormalMode;
  // settings.mRequestedMode = ACAN2515Settings::LoopBackMode; // Pour test sans bus CAN

  const uint16_t errorCode = can.begin(settings, [] {
    // Fonction d'interruption (vide car on utilise le polling)
  });

  if (errorCode == 0) {
    Serial.print("✅ Configuration MCP2515 réussie!");
    Serial.print(" Débit réel: ");
    Serial.print(settings.actualBitRate() / 1000);
    Serial.println(" kbps");

    updateOLED("MCP2515 OK!", "500 kbps", "Attente comm...", "");
  } else {
    Serial.print("❌ Erreur configuration MCP2515: Code ");
    Serial.println(errorCode);

    updateOLED("ERREUR MCP2515!", "Code: " + String(errorCode), "", "");

    // Version corrigée pour ACAN2515 - codes d'erreur simples
    Serial.print("   └─ Description: ");
    switch(errorCode) {
      case 1:
        Serial.println("SPI non configuré");
        break;
      case 2:
        Serial.println("MCP2515 non trouvé - Vérifier les connexions");
        break;
      case 3:
        Serial.println("Paramètres de débit incorrects");
        break;
      case 4:
        Serial.println("Débit trop éloigné de la valeur demandée");
        break;
      case 5:
        Serial.println("Configuration du MCP2515 échouée");
        break;
      default:
        Serial.println("Erreur inconnue");
        break;
    }
    
    // Conseils de dépannage
    Serial.println("🔧 Vérifications suggérées:");
    Serial.println("   • Connexions SPI (CS, SCK, MOSI, MISO)");
    Serial.println("   • Alimentation du MCP2515 (3.3V ou 5V)");
    Serial.println("   • Fréquence du quartz (8MHz, 16MHz, 20MHz)");
    Serial.println("   • Terminaison du bus CAN si nécessaire");
  }

  Serial.println("=== Début de la communication CAN ===");
}

//----------------------------------------------------------------------------------------
//   Fonction pour mettre à jour l'OLED
//----------------------------------------------------------------------------------------
void updateOLED(String line1, String line2, String line3, String line4) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println(line1);

  display.setCursor(0, 12);
  display.println(line2);

  display.setCursor(0, 24);
  display.println(line3);

  display.setCursor(0, 36);
  display.println(line4);

  // Ligne de statistiques
  display.setCursor(0, 48);
  display.print("TX:");
  display.print(messagesSent);
  display.print(" RX:");
  display.print(messagesReceived);

  // Indicateur de connexion
  display.setCursor(0, 56);
  if (millis() - lastReceivedTime < 5000 || messagesReceived == 0) {
    if (messagesReceived > 0) {
      display.print("Status: CONNECTE");
    } else {
      display.print("Status: EN ATTENTE");
    }
  } else {
    display.print("Status: TIMEOUT");
  }

  display.display();
}

//----------------------------------------------------------------------------------------
//   LOOP
//----------------------------------------------------------------------------------------
void loop()
{
  static uint32_t lastSend = 0;
  static uint32_t lastDisplayUpdate = 0;

  // === ENVOI DE MESSAGES ===
  if (millis() - lastSend > 2000) { // envoi toutes les 2 secondes
    lastSend = millis();

    CANMessage frame;
    frame.id = 0x123; // ID pour ESP32 -> Arduino
    frame.len = 8;    // 8 octets de données
    frame.ext = false; // trame standard (11 bits)
    frame.rtr = false; // data frame

    // Données dynamiques pour test
    frame.data[0] = 0x10 + (messagesSent % 16);
    frame.data[1] = 0x20 + (messagesSent % 16);
    frame.data[2] = (millis() >> 24) & 0xFF;
    frame.data[3] = (millis() >> 16) & 0xFF;
    frame.data[4] = (millis() >> 8) & 0xFF;
    frame.data[5] = millis() & 0xFF;
    frame.data[6] = 0xAA;
    frame.data[7] = 0xBB;

    const bool frameEnvoye = can.tryToSend(frame);

    if (frameEnvoye) {
      messagesSent++;

      // Préparer la chaîne des données envoyées
      lastSentData = "";
      for (int i = 0; i < frame.len; i++) {
        if (i > 0) lastSentData += " ";
        if (frame.data[i] < 16) lastSentData += "0";
        lastSentData += String(frame.data[i], HEX);
      }
      lastSentData.toUpperCase();

      // Message dans le terminal
      Serial.print("📤 ENVOI #");
      Serial.print(messagesSent);
      Serial.print(" - ID: 0x");
      if (frame.id < 16) Serial.print("0");
      Serial.print(frame.id, HEX);
      Serial.print(" - Données: ");
      Serial.print(lastSentData);
      Serial.print(" - Longueur: ");
      Serial.print(frame.len);
      Serial.println(" octets");

      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Blink LED
    }
    else {
      Serial.println("❌ Échec d'envoi - Buffer plein ou erreur MCP2515");
    }
  }

  // === RÉCEPTION DE MESSAGES ===
  CANMessage frameReceived;
  if (can.receive(frameReceived)) {
    messagesReceived++;
    lastReceivedTime = millis();

    // Préparer la chaîne des données reçues
    lastReceivedData = "";
    for (int i = 0; i < frameReceived.len; i++) {
      if (i > 0) lastReceivedData += " ";
      if (frameReceived.data[i] < 16) lastReceivedData += "0";
      lastReceivedData += String(frameReceived.data[i], HEX);
    }
    lastReceivedData.toUpperCase();

    // Message dans le terminal
    Serial.print("📥 RÉCEPTION #");
    Serial.print(messagesReceived);
    Serial.print(" - ID: 0x");
    if (frameReceived.id < 16) Serial.print("0");
    Serial.print(frameReceived.id, HEX);
    Serial.print(" - Données: ");
    Serial.print(lastReceivedData);
    Serial.print(" - Longueur: ");
    Serial.print(frameReceived.len);
    Serial.println(" octets");

    // Traitement spécifique selon l'ID
    if (frameReceived.id == 0x124) {
      Serial.println("   └─ 💬 Réponse de l'Arduino détectée!");
    } else if (frameReceived.id == 0x123) {
      Serial.println("   └─ 🔄 Message en boucle détecté (LoopBack?)");
    }

    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Blink LED
  }

  // === MISE À JOUR DE L'AFFICHEUR ===
  if (millis() - lastDisplayUpdate > 500) { // Mise à jour toutes les 500ms
    lastDisplayUpdate = millis();

    String line1 = "MCP2515 CAN Mon.";
    String line2 = "TX: " + (lastSentData.length() > 0 ? lastSentData.substring(0, 14) : "Aucun");
    String line3 = "RX: " + (lastReceivedData.length() > 0 ? lastReceivedData.substring(0, 14) : "Aucun");
    String line4 = "Up: " + String(millis()/1000) + "s";

    updateOLED(line1, line2, line3, line4);
  }

  // === MESSAGES DE STATUT PÉRIODIQUES ===
  static uint32_t lastStatusMessage = 0;
  if (millis() - lastStatusMessage > 15000) { // Toutes les 15 secondes
    lastStatusMessage = millis();

    Serial.println("📊 === RAPPORT DE STATUT MCP2515 ===");
    Serial.print("   Messages envoyés: ");
    Serial.println(messagesSent);
    Serial.print("   Messages reçus: ");
    Serial.println(messagesReceived);
    Serial.print("   Temps de fonctionnement: ");
    Serial.print(millis()/1000);
    Serial.println(" secondes");

    if (messagesReceived > 0) {
      Serial.print("   Dernière réception il y a: ");
      Serial.print((millis() - lastReceivedTime)/1000);
      Serial.println(" secondes");
    } else {
      Serial.println("   ⚠️  Aucune réception - Vérifier le bus CAN");
    }

    // Statistiques du MCP2515 (fonctions corrigées)
    Serial.print("   Erreurs de transmission: ");
    Serial.println(can.transmitErrorCounter());
    Serial.print("   Erreurs de réception: ");
    Serial.println(can.receiveErrorCounter());

    Serial.println("===================================");
  }
}
