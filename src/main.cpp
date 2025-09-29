#include <ACAN2515.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>

//----------------------------------------------------------------------------------------
//   Configuration
//----------------------------------------------------------------------------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// Pins
#define MCP2515_CS 5
#define LED_PIN 2
#define BUTTON_PIN 4
#define ADC_PIN A0
#define DHT_PIN 15

// Objets
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
ACAN2515 can(MCP2515_CS, SPI, 255);
DHT dht(DHT_PIN, DHT11);

// Variables
const uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL;
bool buttonState = false;
int adcValue = 0;
float temperature = 0;

//----------------------------------------------------------------------------------------
//   ⚡ AJOUTEZ CETTE LIGNE - Déclaration de la fonction
//----------------------------------------------------------------------------------------
void updateDisplay(String l1, String l2, String l3, String l4);

//----------------------------------------------------------------------------------------
//   SETUP
//----------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  
  // Init pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // Init périphériques
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("Erreur OLED!");
    while(1);
  }
  
  dht.begin();
  SPI.begin();
  
  // Config CAN 125kbps
  ACAN2515Settings settings(QUARTZ_FREQUENCY, 125UL * 1000UL);
  settings.mRequestedMode = ACAN2515Settings::NormalMode;
  
  if (can.begin(settings, []{}) == 0) {
    Serial.println("✅ CAN 125kbps OK");
    updateDisplay("CAN 125kbps", "ESP32 Ready", "", "");
  } else {
    Serial.println("❌ Erreur CAN");
    updateDisplay("ERREUR CAN", "Verif connect", "", "");
  }
}

//----------------------------------------------------------------------------------------
//   LOOP
//----------------------------------------------------------------------------------------
void loop() {
  static unsigned long lastSend = 0;
  
  // Lecture capteurs
  buttonState = !digitalRead(BUTTON_PIN);
  adcValue = analogRead(ADC_PIN);
  temperature = dht.readTemperature();
  
  // Envoi périodique (toutes les secondes)
  if (millis() - lastSend > 1000) {
    lastSend = millis();
    
    CANMessage message;
    message.id = 0x100;
    message.len = 8;
    message.data[0] = buttonState ? 1 : 0;
    message.data[1] = (adcValue >> 8) & 0xFF;
    message.data[2] = adcValue & 0xFF;
    message.data[3] = (int)temperature;
    message.data[4] = 0; // LED state
    message.data[5] = 0;
    message.data[6] = 0;
    message.data[7] = 0; // Checksum
    
    if (can.tryToSend(message)) {
      Serial.println("📤 Données envoyées");
    }
  }
  
  // Réception
  CANMessage received;
  if (can.receive(received)) {
    if (received.id == 0x200) {
      if (received.data[0] == 1) {
        digitalWrite(LED_PIN, received.data[1]);
        Serial.print("LED=");
        Serial.println(received.data[1] ? "ON" : "OFF");
      }
    }
  }
  
  // Mise à jour affichage
  String status = buttonState ? "BP:ON" : "BP:OFF";
  String sensors = "ADC:" + String(adcValue) + " T:" + String(temperature,1) + "C";
  String canInfo = "CAN: 125kbps";
  String uptime = "Up:" + String(millis()/1000) + "s";
  
  updateDisplay(canInfo, status, sensors, uptime);
  
  delay(100);
}

//----------------------------------------------------------------------------------------
//   ⚡ DÉFINITION DE LA FONCTION - Placez à la fin
//----------------------------------------------------------------------------------------
void updateDisplay(String l1, String l2, String l3, String l4) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  display.setCursor(0, 0);  display.println(l1);
  display.setCursor(0, 12); display.println(l2);
  display.setCursor(0, 24); display.println(l3);
  display.setCursor(0, 36); display.println(l4);
  
  display.display();
}
