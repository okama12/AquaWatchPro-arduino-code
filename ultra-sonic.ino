#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>

// WiFi credentials
const char* ssid = "undercover";
const char* password = "miriam2014";

// WebSocket server address and port
const char* serverAddress = "192.168.1.103";
const int serverPort = 8765;

WebSocketsClient webSocket;

// Ultrasonic sensor pins
#define TRIG_PIN 12
#define ECHO_PIN 13

// Relay pin
#define RELAY_PIN 19

// Tank dimensions in cm
#define TANK_HEIGHT 20         // Height of the tank in cm
#define PUMP_ON_LEVEL 15       // Distance from sensor to water surface to turn pump on
#define PUMP_OFF_LEVEL 3       // Distance from sensor to water surface to turn pump off
#define NUM_SAMPLES 20         // Number of samples for averaging
#define MIN_VALID_DISTANCE 2   // Minimum valid distance to consider

float distanceSamples[NUM_SAMPLES]; // Array to hold distance samples

LiquidCrystal_I2C lcd(0x27, 16, 2);

enum RelayState { RELAY_OFF, RELAY_ON }; // Define relay states
RelayState relayState = RELAY_OFF; // Initialize relay state

// Connection flag
bool isConnected = false;

// WebSocket event handler
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
    switch (type) {
        case WStype_DISCONNECTED:
            Serial.println("WebSocket disconnected");
            isConnected = false;
            break;
        case WStype_CONNECTED:
            Serial.println("Connected to WebSocket server");
            isConnected = true;
            break;
        case WStype_TEXT:
            Serial.printf("Received text: %s\n", payload);
            break;
    }
}

void connectToWiFi() {
    Serial.print("Connecting to WiFi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Connected!");
}

void connectToWebSocket() {
    Serial.print("Connecting to WebSocket server...");
    webSocket.begin(serverAddress, serverPort, "/");
    webSocket.onEvent(webSocketEvent);
    webSocket.setReconnectInterval(5000);
    while (!isConnected) {
        webSocket.loop();
        delay(100);
    }
    Serial.println("Connected to WebSocket!");
}

void setup() {
    Serial.begin(115200);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);  // Initially turn off the pump
    
    lcd.init();  // Initialize LCD
    lcd.backlight();  // Turn on backlight
    lcd.begin(16, 2);  // Set number of columns and rows
    
    lcd.clear();  // Clear the display
    lcd.setCursor(0, 0);  // Set cursor position
    lcd.print("Distance:");

    connectToWiFi();
    connectToWebSocket();

    // Initialize distance samples
    for (int i = 0; i < NUM_SAMPLES; i++) {
        distanceSamples[i] = 0;
    }
}

float measureDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 25000); // 25ms timeout
    float distance = (duration * 0.0343) / 2;
    return distance;
}

float getAverageDistance() {
    float distances[NUM_SAMPLES];
    for (int i = 0; i < NUM_SAMPLES; i++) {
        distances[i] = measureDistance();
        delay(50);
    }

    // Sort distances array to find median
    for (int i = 0; i < NUM_SAMPLES - 1; i++) {
        for (int j = i + 1; j < NUM_SAMPLES; j++) {
            if (distances[i] > distances[j]) {
                float temp = distances[i];
                distances[i] = distances[j];
                distances[j] = temp;
            }
        }
    }

    if (NUM_SAMPLES % 2 == 0) {
        return (distances[NUM_SAMPLES / 2 - 1] + distances[NUM_SAMPLES / 2]) / 2;
    } else {
        return distances[NUM_SAMPLES / 2];
    }
}

void loop() {
    webSocket.loop();
    
    float distance = getAverageDistance();
    
    // Filter out invalid readings (below the minimum valid distance)
    if (distance >= MIN_VALID_DISTANCE) {
        Serial.print("Distance: ");
        Serial.print(distance);
        Serial.println(" cm");
        
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Distance: ");
        lcd.print(distance);
        lcd.print(" cm");
        
        lcd.setCursor(0, 1);
        if (distance >= PUMP_ON_LEVEL && relayState != RELAY_ON) { 
            lcd.print("Pump: ON");
            digitalWrite(RELAY_PIN, HIGH);  
            relayState = RELAY_ON; 
        } else if (distance <= PUMP_OFF_LEVEL && relayState != RELAY_OFF) { 
            lcd.print("Pump: OFF");
            digitalWrite(RELAY_PIN, LOW);  
            relayState = RELAY_OFF; 
        } else {
            lcd.print("Pump: WAIT");
        }

        // Send data to WebSocket server
        if (isConnected) {
            DynamicJsonDocument doc(200);
            doc["type"] = "sensor";
            doc["distance"] = distance;
            doc["status"] = relayState == RELAY_ON ? "Filling" : "Full";
            String message;
            serializeJson(doc, message);
            webSocket.sendTXT(message);
        }
    }
    
    delay(1000);
}
