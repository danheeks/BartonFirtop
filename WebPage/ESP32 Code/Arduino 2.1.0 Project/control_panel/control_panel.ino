#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <FS.h>
#include <SPIFFS.h>

// Set your WiFi credentials
const char* ssid = "BartonFirtop-Control";
const char* password = "123456789";
float valveX = 100; // Initial X position
float valveY = 100; // Initial Y position

// Create an instance of the web server
AsyncWebServer server(80);

// Functions triggered by button presses
void handleOpenA() {
    Serial.println("Open A pressed");
    // Add your logic here
}

void handleAuto() {
    Serial.println("Auto pressed");
    // Add your logic here
}

void handleOpenB() {
    Serial.println("Open B pressed");
    // Add your logic here
}

void handleStop() {
    Serial.println("STOP pressed");
    // Add your logic here
}

// Handle POST requests
void handleAction(AsyncWebServerRequest *request) {
    if (request->hasParam("plain", true)) { // Check for a plain-text body
        String body = request->getParam("plain", true)->value(); // Retrieve the body content
        Serial.println("Received action: " + body);

        // Perform actions based on the received body
        if (body == "openA") {
            Serial.println("Opening Chamber A...");
            // Add your code to open Chamber A
        } else if (body == "openB") {
            Serial.println("Opening Chamber B...");
            // Add your code to open Chamber B
        } else if (body == "stop") {
            Serial.println("Stopping system...");
            // Add your code to stop the system
        } else {
            Serial.println("Unknown action received");
        }

        request->send(200, "text/plain", "Action received: " + body);
    } else {
        request->send(400, "text/plain", "Bad Request: Missing body");
    }
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

    server.on("/action", HTTP_POST, handleAction);

    // Define the "/valve-position" route
    server.on("/valve-position", HTTP_GET, handleValvePosition);

    // Start the server
    server.begin();
    Serial.println("Server started");
}

void loop() {
    // Nothing to do here, everything is handled by the AsyncWebServer
}
