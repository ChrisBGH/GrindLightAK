ARDUINO

#include <WiFi.h>
#include <MQTT.h>
#include <FastLED.h>

#define LED_PIN 16          // Data pin connected to the NeoPixel strip
#define NUM_LEDS 11         // Total number of LEDs in the strip
#define BRIGHTNESS 100      // Brightness level (0 - 255)
#define WIFI_SSID "Noah - iPhone"   // Replace with your WiFi SSID
#define WIFI_PASSWORD "IDS12345"    // Replace with your WiFi password
#define BUTTON_PIN 22       // GPIO pin for the physical button

#define COLOR_ON CRGB::Blue
#define COLOR_OFF CRGB::Black

WiFiClient net;
MQTTClient client;

CRGB leds[NUM_LEDS];
unsigned long lastMillis = 0;

bool allLedsOn = false;  // Track the state of all LEDs
bool buttonPressed = false;
bool ledStates[NUM_LEDS] = {false};  // Track the on/off state of each LED

void connectWiFi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\nWiFi connected!");
}

void connectMQTT() {
  Serial.print("Connecting to MQTT...");
  while (!client.connect("arduino", "public", "public")) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\nMQTT connected!");

  client.subscribe("/led1");
  client.subscribe("/led2");
  client.subscribe("/led3");
  client.subscribe("/led4");
  client.subscribe("/led5");
}

void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);

  int ledIndex = -1;
  if (topic.equals("/led1")) ledIndex = 1;
  else if (topic.equals("/led2")) ledIndex = 3;
  else if (topic.equals("/led3")) ledIndex = 5;
  else if (topic.equals("/led4")) ledIndex = 7;
  else if (topic.equals("/led5")) ledIndex = 9;

  if (ledIndex != -1) {
    ledStates[ledIndex] = (payload.equals("on"));  // Update state in array
    leds[ledIndex] = ledStates[ledIndex] ? COLOR_ON : COLOR_OFF;
    FastLED.show();
  }
}

void setup() {
  Serial.begin(9600);
  connectWiFi();

  client.begin("public.cloud.shiftr.io", net);
  client.onMessage(messageReceived);

  connectMQTT();

  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();
  FastLED.show();

  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Set button pin as input with pull-up resistor
}

void loop() {
  client.loop();
  delay(10);

  if (!client.connected()) {
    connectMQTT();
  }

  if (millis() - lastMillis > 1000) {
    lastMillis = millis();
    client.publish("/idsesp32", "I am online");
  }

  checkButton();  // Check button state
}

// Function to check the button state and toggle all LEDs
void checkButton() {
  if (digitalRead(BUTTON_PIN) == LOW) {  // Button pressed (assuming active low)
    delay(50);  // Debounce delay
    if (!buttonPressed) {  // Only toggle if previously not pressed
      buttonPressed = true;
      allLedsOn = !allLedsOn;  // Toggle the state
      setAllLeds(allLedsOn);
    }
  } else {
    buttonPressed = false;  // Reset button state when released
  }
}

// Function to set all LEDs based on the current state
void setAllLeds(bool turnOn) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = (turnOn && ledStates[i]) ? COLOR_ON : COLOR_OFF;  // Use stored state if turning on
  }
  FastLED.show();
}
