// ===== TEST DIAGNOSTIC OLED ESP32 =====
// Test complet pour identifier le problème OLED

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ========== CONFIGURATION ==========
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SDA_PIN 21
#define SCL_PIN 22

// Testez les 2 adresses courantes
#define SCREEN_ADDRESS_1 0x3C
#define SCREEN_ADDRESS_2 0x3D

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Serial.begin(115200);
  delay(2000); // Attendre ouverture Serial Monitor
  
  Serial.println("\n\n========================================");
  Serial.println("DIAGNOSTIC OLED ESP32");
  Serial.println("========================================\n");

  // ========== TEST 1: INITIALISATION I2C ==========
  Serial.println("TEST 1: Initialisation I2C...");
  Serial.print("  SDA = GPIO ");
  Serial.println(SDA_PIN);
  Serial.print("  SCL = GPIO ");
  Serial.println(SCL_PIN);
  
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("  ✅ I2C initialisé\n");
  delay(500);

  // ========== TEST 2: SCAN I2C ==========
  Serial.println("TEST 2: Scan du bus I2C...");
  byte deviceCount = 0;
  byte foundAddress = 0;
  
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("  ✅ Device trouvé à l'adresse 0x");
      if (addr < 16) Serial.print("0");
      Serial.print(addr, HEX);
      
      if (addr == 0x3C || addr == 0x3D) {
        Serial.print(" <- OLED détecté!");
        foundAddress = addr;
      }
      Serial.println();
      deviceCount++;
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("  ❌ AUCUN device I2C détecté!");
    Serial.println("\n⚠️ PROBLÈME DÉTECTÉ:");
    Serial.println("  1. Vérifiez le câblage:");
    Serial.println("     OLED VCC -> ESP32 3.3V");
    Serial.println("     OLED GND -> ESP32 GND");
    Serial.println("     OLED SDA -> ESP32 GPIO 21");
    Serial.println("     OLED SCL -> ESP32 GPIO 22");
    Serial.println("  2. Vérifiez l'alimentation OLED");
    Serial.println("  3. Essayez un autre module OLED");
    while(1) { delay(1000); }
  } else {
    Serial.print("\n  Total: ");
    Serial.print(deviceCount);
    Serial.println(" device(s) trouvé(s)\n");
  }

  // ========== TEST 3: INITIALISATION OLED ==========
  Serial.println("TEST 3: Initialisation OLED...");
  
  bool oledOK = false;
  byte workingAddress = 0;

  // Test adresse 0x3C
  Serial.print("  Test adresse 0x3C... ");
  if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS_1)) {
    Serial.println("✅ OK");
    oledOK = true;
    workingAddress = SCREEN_ADDRESS_1;
  } else {
    Serial.println("❌ Échec");
    
    // Test adresse 0x3D
    Serial.print("  Test adresse 0x3D... ");
    if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS_2)) {
      Serial.println("✅ OK");
      oledOK = true;
      workingAddress = SCREEN_ADDRESS_2;
    } else {
      Serial.println("❌ Échec");
    }
  }

  if (!oledOK) {
    Serial.println("\n❌ ÉCHEC INITIALISATION OLED");
    Serial.println("\n⚠️ CAUSES POSSIBLES:");
    Serial.println("  1. Mauvaise résolution (essayez 128x32 au lieu de 128x64)");
    Serial.println("  2. Module OLED défectueux");
    Serial.println("  3. Alimentation insuffisante");
    
    if (foundAddress > 0) {
      Serial.print("  4. Device I2C détecté à 0x");
      Serial.print(foundAddress, HEX);
      Serial.println(" mais pas compatible SSD1306");
    }
    
    while(1) { delay(1000); }
  }

  // ========== TEST 4: AFFICHAGE ==========
  Serial.println("\nTEST 4: Test affichage...");
  Serial.print("  Adresse utilisée: 0x");
  Serial.println(workingAddress, HEX);
  
  // Test 1: Effacer écran
  Serial.println("  - Effacement écran");
  display.clearDisplay();
  display.display();
  delay(500);

  // Test 2: Afficher un pixel
  Serial.println("  - Affichage pixel");
  display.drawPixel(64, 32, SSD1306_WHITE);
  display.display();
  delay(1000);

  // Test 3: Afficher texte
  Serial.println("  - Affichage texte");
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("OLED"));
  display.println(F("OK!"));
  display.setTextSize(1);
  display.print(F("Addr: 0x"));
  display.println(workingAddress, HEX);
  display.display();
  delay(2000);

  // Test 4: Animation
  Serial.println("  - Test animation");
  for (int i = 0; i < 3; i++) {
    display.invertDisplay(true);
    delay(200);
    display.invertDisplay(false);
    delay(200);
  }

  // ========== RÉSULTAT FINAL ==========
  Serial.println("\n========================================");
  Serial.println("✅ DIAGNOSTIC TERMINÉ - OLED OK!");
  Serial.println("========================================");
  Serial.print("Adresse I2C fonctionnelle: 0x");
  Serial.println(workingAddress, HEX);
  Serial.println("\nCopiez cette ligne dans votre code:");
  Serial.print("#define SCREEN_ADDRESS 0x");
  Serial.println(workingAddress, HEX);
  Serial.println("========================================\n");
}

void loop() {
  // Animation de test continue
  static unsigned long lastUpdate = 0;
  static int counter = 0;
  
  if (millis() - lastUpdate > 1000) {
    lastUpdate = millis();
    counter++;
    
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println(F("TEST"));
    display.setTextSize(1);
    display.print(F("Count: "));
    display.println(counter);
    display.print(F("Millis: "));
    display.println(millis() / 1000);
    display.display();
    
    Serial.print("Loop: ");
    Serial.println(counter);
  }
}
