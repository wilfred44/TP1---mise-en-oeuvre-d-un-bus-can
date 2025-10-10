#include <ACAN_ESP32.h>
#include <SSD1306Wire.h>

#define BOOT_BUTTON_PIN 0
#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4
#define LED_PIN 19

const int PWM_CHANNEL = 0;
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;

SSD1306Wire display(0x3C, 21, 22);

uint8_t brightness = 0;
bool lastButtonState = HIGH;
bool buttonPressed = false;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 50;

void updateLED(uint8_t intensity);
void updateDisplay();

void setup() {
  Serial.begin(115200);
  
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(LED_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 0);
  
  display.init();
  display.flipScreenVertically();
  
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, "ESP32");
  display.drawString(0, 20, "RECEPTEUR");
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 45, "Pret !");
  display.display();
  delay(1000);
  
  ACAN_ESP32_Settings settings(500 * 1000);
  settings.mRequestedCANMode = ACAN_ESP32_Settings::NormalMode;
  settings.mRxPin = CAN_RX_PIN;
  settings.mTxPin = CAN_TX_PIN;
  
  ACAN_ESP32::can.begin(settings);
  Serial.println("CAN OK");
  
  updateDisplay();
  lastButtonState = digitalRead(BOOT_BUTTON_PIN);
}

void updateLED(uint8_t intensity) {
  ledcWrite(PWM_CHANNEL, intensity);
}

void updateDisplay() {
  display.clear();
  
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "ESP32 RECEPTEUR");
  display.drawHorizontalLine(0, 12, 128);
  display.drawString(0, 18, "Luminosite:");
  
  display.setFont(ArialMT_Plain_24);
  int percentage = (brightness * 100) / 255;
  String percentStr = String(percentage) + "%";
  display.drawString(15, 32, percentStr);
  
  display.display();
}

void loop() {
  bool reading = digitalRead(BOOT_BUTTON_PIN);
  
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
    lastButtonState = reading;
  }
  
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (reading == LOW && !buttonPressed) {
      buttonPressed = true;
      
      CANMessage messageToSend;
      messageToSend.id = 0x200;
      messageToSend.len = 1;
      messageToSend.ext = false;
      messageToSend.rtr = false;
      messageToSend.data[0] = 0x01;
      
      if (ACAN_ESP32::can.tryToSend(messageToSend)) {
        Serial.println("Bouton: Message envoye");
      }
    }
    
    if (reading == HIGH) {
      buttonPressed = false;
    }
  }
  
  CANMessage receivedMessage;
  if (ACAN_ESP32::can.receive(receivedMessage)) {
    if (receivedMessage.id == 0x100 && receivedMessage.len >= 1) {
      brightness = receivedMessage.data[0];
      updateLED(brightness);
      updateDisplay();
      
      Serial.print("Lum: ");
      Serial.println(brightness);
    }
  }
  
  delay(10);
}