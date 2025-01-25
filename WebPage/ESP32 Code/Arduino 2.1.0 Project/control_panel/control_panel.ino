#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <FS.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <time.h>

// Set your WiFi credentials
const char* ssid = "BartonFirtop-Control";
const char* password = "123456789";
float valveX = 100; // Initial X position
float valveY = 100; // Initial Y position
bool openingA = false;
bool openingB = false;
float positionA = 0;
float positionB = 200;
unsigned long startTime = 0; // Start time of the movement



// Create an instance of the web server
AsyncWebServer server(80);

// Functions triggered by button presses
void handleOpenA() {
    Serial.println("Open A pressed");
    // Add your logic here
    openingA = true;
    openingB = false;
}

void handleAuto() {
    Serial.println("Auto pressed");
    // Add your logic here
}

void handleOpenB() {
    Serial.println("Open B pressed");
    // Add your logic here
    openingA = false;
    openingB = true;
}

void handleStop() {
    Serial.println("STOP pressed");
    // Add your logic here
    openingA = false;
    openingB = false;
}

// Function to handle the "/valve-position" endpoint
void handleValvePosition(AsyncWebServerRequest *request) {
    Serial.print("Handling Valve position");
    String json = "{ \"x\": " + String(valveX) + ", \"y\": " + String(valveY) + " }";
    Serial.println(json);
    request->send(200, "application/json", json);
}

void setup() {
    Serial.begin(115200);

    // Set up the ESP32 as an access point
    WiFi.softAP(ssid, password);

    // Initialize SPIFFS
    if (!SPIFFS.begin(true)) {
        Serial.println("Failed to mount SPIFFS");
        return;
    }

    // Serve static files from SPIFFS
    server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");

      // Handle POST requests to /action
    server.on("/action", HTTP_POST, [](AsyncWebServerRequest *request) {}, 
        NULL,  // No body handler for multipart
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
            String body = String((char*)data).substring(0, len);
            Serial.println("Received POST body: " + body);

            // Parse JSON
            StaticJsonDocument<200> doc;
            DeserializationError error = deserializeJson(doc, body);
            if (error) {
                Serial.println("Failed to parse JSON");
                request->send(400, "text/plain", "Bad Request: Invalid JSON");
                return;
            }

            String action = doc["action"]; // Extract the "action" field
            Serial.println("Action: " + action);

            // Perform actions based on the "action" field
            if (action == "openA") {
                Serial.println("Opening Chamber A...");
                startTime = millis(); // Record the start time
                // Add your code to open Chamber A
                handleOpenA();
            } else if (action == "openB") {
                Serial.println("Opening Chamber B...");
                startTime = millis(); // Record the start time
                // Add your code to open Chamber B
                handleOpenB();
            } else if (action == "stop") {
                Serial.println("Stopping system...");
                // Add your code to stop the system
                handleStop();
            } else {
                Serial.println("Unknown action received");
            }

            request->send(200, "text/plain", "Action received: " + action);
        }
    );

    // Define the "/valve-position" route
    server.on("/valve-position", HTTP_GET, handleValvePosition);

    // Start the server
    server.begin();
    Serial.println("Server started");
}

void loop() {
    if(openingA)
    {
      if(valveX > positionA)valveX -= 1;
      if(valveX < positionA)valveX += 1;
    }
    if(openingB)
    {
      if(valveX > positionB)valveX -= 1;
      if(valveX < positionB)valveX += 1;
    }

}
