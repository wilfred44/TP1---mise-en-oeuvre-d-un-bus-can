// ===== ESP32 (ACAN2515) =====
// Modifié pour utiliser ACAN2515 au lieu de ACAN_ESP32
// Mode NORMAL, 500 kb/s, MCP2515 sur SPI

#include <ACAN2515.h>
#include <SPI.h>

//----------------------------------------------------------------------------------------
//   Configuration
//----------------------------------------------------------------------------------------
static const uint32_t DESIRED_BIT_RATE = 500UL * 1000UL;
static const uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL; // 8 MHz quartz

// Pins
static const byte MCP2515_CS = 5;     // Chip Select
static const byte ESP32_LED = 2;      // ⚡ CHANGÉ: LED ESP32 (évite le conflit)

// Objets
ACAN2515 can(MCP2515_CS, SPI, 255);   // CS=5, SPI, pas d'interruption

// Variables
static uint32_t lastSend = 0;

//----------------------------------------------------------------------------------------
//   SETUP
//----------------------------------------------------------------------------------------
void setup() {
  pinMode(ESP32_LED, OUTPUT);          // ⚡ CHANGÉ: ESP32_LED au lieu de LED_BUILTIN
  digitalWrite(ESP32_LED, HIGH);       // ⚡ CHANGÉ: ESP32_LED au lieu de LED_BUILTIN
  Serial.begin(115200);
  delay(100);

  Serial.println("Configure ESP32 + MCP2515 CAN (normal mode, 500kbps)");
  
  // Configuration ESP32 SPI
  SPI.begin();
  
  // Configuration ACAN2515
  ACAN2515Settings settings(QUARTZ_FREQUENCY, DESIRED_BIT_RATE);
  
  // Filtre : accepter seulement les trames standard
  const ACAN2515Mask mask = standard2515Mask(0x7FF, 0x000, 0x000);
  const ACAN2515AcceptanceFilter filters[] = {
    {standard2515Filter(0x000, 0x000, 0x000)}  // Accepte tout
  };

  const uint16_t errorCode = can.begin(settings, [] {}, mask, filters, 1);
  
  if (errorCode == 0) {
    Serial.println("Configuration OK:");
    Serial.print("  Actual bit rate: "); 
    Serial.print(settings.actualBitRate()); 
    Serial.println(" bit/s");
    Serial.print("  Exact bit rate? "); 
    Serial.println(settings.exactBitRate() ? "yes" : "no");
    Serial.print("  Sample point:    "); 
    Serial.print(settings.samplePointFromBitStart()); 
    Serial.println("%");
    Serial.print("  MCP2515 CS Pin:  "); 
    Serial.println(MCP2515_CS);
  } else {
    Serial.print("Configuration error 0x");
    Serial.println(errorCode, HEX);
    Serial.println("Vérifiez les connexions MCP2515:");
    Serial.println("  CS  -> GPIO 5");
    Serial.println("  SCK -> GPIO 18");
    Serial.println("  MOSI-> GPIO 23");
    Serial.println("  MISO-> GPIO 19");
    Serial.println("  VCC -> 3.3V");
    Serial.println("  GND -> GND");
    while (1) { 
      digitalWrite(ESP32_LED, !digitalRead(ESP32_LED)); // ⚡ CHANGÉ
      delay(200); 
    }
  }
}

//----------------------------------------------------------------------------------------
//   LOOP
//----------------------------------------------------------------------------------------
void loop() {
  // ========== RÉCEPTION NON BLOQUANTE ==========
  CANMessage rx;
  if (can.receive(rx)) {
    Serial.print("📥 RX ");
    Serial.print(rx.ext ? "EXT " : "STD ");
    Serial.print(rx.rtr ? "RTR " : "DATA ");
    Serial.print("ID=0x"); 
    Serial.print(rx.id, HEX);
    Serial.print(" DLC="); 
    Serial.print(rx.len);
    Serial.print(" DATA=");
    
    for (uint8_t i = 0; i < rx.len; i++) { 
      Serial.print("0x");
      if (rx.data[i] < 0x10) Serial.print("0");
      Serial.print(rx.data[i], HEX); 
      Serial.print(" "); 
    }
    Serial.println();
    
    // Petit écho LED
    digitalWrite(ESP32_LED, !digitalRead(ESP32_LED)); // ⚡ CHANGÉ
    
    // Traitement spécifique par ID
    if (rx.id == 0x100) {
      Serial.println("  -> Message Arduino reçu!");
    }
  }

  // ========== ENVOI PÉRIODIQUE ==========
  if (millis() - lastSend >= 700) {
    lastSend = millis();
    
    CANMessage tx;
    tx.id  = 0x101;      // Notre ID TX côté ESP32
    tx.ext = false;      // Trame standard 11 bits
    tx.rtr = false;      // Data frame
    tx.len = 8;
    
    // Données exemple (modifiables)
    tx.data[0] = 0x02;   // CMD
    tx.data[1] = 0x04;   // VALUE_HI
    tx.data[2] = 0x56;   // VALUE_LO
    tx.data[3] = (millis() >> 24) & 0xFF; // Timestamp
    tx.data[4] = (millis() >> 16) & 0xFF;
    tx.data[5] = (millis() >> 8) & 0xFF;
    tx.data[6] = millis() & 0xFF;
    tx.data[7] = 0xAA;   // Marqueur

    const bool ok = can.tryToSend(tx);
    if (ok) {
      Serial.println("📤 TX ESP32 -> ID 0x101 OK");
      Serial.print("   Data: ");
      for (uint8_t i = 0; i < 8; i++) {
        Serial.print("0x");
        if (tx.data[i] < 0x10) Serial.print("0");
        Serial.print(tx.data[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    } else {
      Serial.println("❌ TX ESP32 FAILED - Buffer plein?");
    }
  }
  
  // Petit délai pour stabilité
  delay(10);
}
