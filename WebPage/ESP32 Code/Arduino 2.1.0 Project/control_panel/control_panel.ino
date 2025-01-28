#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <FS.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <ModbusMaster.h>
#include <HardwareSerial.h>
#include "soc/rtc_wdt.h"

// connections RS485 module to Arduino
// GND to GND
// VCC to 5V
// RXD to D2 ( set as RX_PIN below )
// TXD to D4 ( set as TX_PIN below )


// type these letters, then Enter to do:
// o - move to open position
// c - move to closed position
// g50 - move to 50% open (for example)
// s - stop move
// b - print battery voltage
// S - print status
// w - print working angle
// p - print position (as a percentage)
// d - print all input registers
// O - print 1 if open relay is activated, else 0
// C - print 1 if closed relay is activated, else 0
// m - print 1 if actuator is moving, else 0
// t - print 1 if failed to complete a move, else 0
// l - print 1 if battery to low to start a move, else 0
// D - print all discrete inputs


// Set your WiFi credentials
const char* ssid = "BartonFirtop-Control";
const char* password = "123456789";
float valveX = 100; // Initial X position
float valveY = 100; // Initial Y position

// Create an instance of the web server
AsyncWebServer server(80);

HardwareSerial rs485(1); // Use UART1

#define RX_PIN 17
#define TX_PIN 16

// instantiate ModbusMaster object
ModbusMaster node;
char reading_number = 0;
char num[64];
int i = 0;

// Functions triggered by button presses
void handleOpenA() {
    Serial.println("Open A pressed");
    // Add your logic here
    // tell actuator to close
    node.writeSingleCoil(1, 1);
}

void handleAuto() {
    Serial.println("Auto pressed");
    // Add your logic here
}

void handleOpenB() {
    Serial.println("Open B pressed");
    // Add your logic here
    // tell actuator to open
    node.writeSingleCoil(0, 1);
}

void handleStop() {
    Serial.println("STOP pressed");
    // Add your logic here
    // tell actuator to stop
      node.writeSingleCoil(3, 1);
}

// Function to handle the "/valve-position" endpoint
void handleValvePosition(AsyncWebServerRequest *request) {
    Serial.print("Handling Valve position");
    String json = "{ \"x\": " + String(valveX) + ", \"y\": " + String(valveY) + " }";
    Serial.println(json);
    request->send(200, "application/json", json);
}

void printInputRegister(uint16_t u16ReadAddress, char* str)
{
  uint8_t result = node.readInputRegisters(u16ReadAddress, 1);
        
  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    Serial.print(str);
    Serial.println((uint16_t)(node.getResponseBuffer(0)));
  }
  else
  {
    Serial.print("Error code: ");
    Serial.println(result);
  }
}

void printDiscreteInput(uint16_t u16ReadAddress, char* str)
{
    uint8_t result = node.readDiscreteInputs(u16ReadAddress, 1);
    
    // do something with data if read is successful
    if (result == node.ku8MBSuccess)
    {
        Serial.print(str);
        Serial.println((int)(node.getResponseBuffer(0)));
  }
  else
  {
    Serial.print("Error code: ");
    Serial.println(result);
  }
}

void setup() {
    rtc_wdt_protect_off();    // Turns off the automatic wdt service
rtc_wdt_enable();         // Turn it on manually
rtc_wdt_set_time(RTC_WDT_STAGE0, 5000);  // Define how long you desire to let dog wait.
    
    rs485.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

    // communicate with Modbus slave ID 1 over RS485
    node.begin(1, rs485);

    Serial.begin(115200);

    delay(1000);

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
                // Add your code to open Chamber A
                handleOpenA();
            } else if (action == "openB") {
                Serial.println("Opening Chamber B...");
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
  uint8_t result;
  uint16_t data;
  
  //Serial.println("transmit low...");
  // set word 0 of TX buffer to least-significant word of counter (bits 15..0)
  //node.setTransmitBuffer(0, lowWord(i));
  
  //Serial.println("transmit high...");
  // set word 1 of TX buffer to most-significant word of counter (bits 31..16)
  //node.setTransmitBuffer(1, highWord(i));
  
  //Serial.println("write...");
  // slave: write TX buffer to (2) 16-bit registers starting at register 0
  //result = node.writeMultipleRegisters(0, 2);

  while (Serial.available() > 0) {
    // cjheck serial from console

    char c = Serial.read();
    if(reading_number != 0)
    {
    switch(c)
    {
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
    case '.':
    case '-':
      num[i] = c;
      i++;
      break;
      default:
      num[i] = 0;
      reading_number = 0;
    uint16_t percentage = atoi(num);
    i = 0;
    node.writeSingleRegister(0,percentage);
    Serial.print("go to percentage: ");
    Serial.println(percentage);

    delay(100);
    
      // tell actuator to move to that percentage
      node.writeSingleCoil(2, 1);
      
      break;
    }
    }
    else
    { // not reading_number
    if (c == 'o') {
      // tell actuator to open
      node.writeSingleCoil(0, 1);
    }
    if (c == 'c') {
      // tell actuator to close
      node.writeSingleCoil(1, 1);
    }
    if(c == 'g')
    {
      reading_number = 'g';
   }
    if (c == 's') {
      // tell actuator to stop
      node.writeSingleCoil(3, 1);
    }
    if (c == 'b') {
        printInputRegister(0, "  battery = ");
    }
    if (c == 'p') {
        printInputRegister(1, "  position = ");
    }
    if (c == 'S') {
        printInputRegister(2, "  status = ");
    }
    if (c == 'w') {
        printInputRegister(3, "  working angle = ");
    }
    if (c == 'd') {
        result = node.readInputRegisters(0, 4);
        
        // do something with data if read is successful
        if (result == node.ku8MBSuccess)
        {
            Serial.print("  battery = ");
            float battery = 0.001 * (float)((uint16_t)(node.getResponseBuffer(0)));
            Serial.println(battery);
            Serial.print("  position = ");
            Serial.println(node.getResponseBuffer(1));
            Serial.print("  status = ");
            Serial.println((uint16_t)(node.getResponseBuffer(2)));
            Serial.print("  working angle = ");
            Serial.println((uint16_t)(node.getResponseBuffer(3)));
        }
        else
        {
          Serial.print("Error code: ");
          Serial.println(result);
        }
    }
    if (c == 'O') printDiscreteInput(0, "Open Relay = ");
    if (c == 'C') printDiscreteInput(1, "Closed Relay = ");
    if (c == 'm') printDiscreteInput(2, "moving = ");
    if (c == 't') printDiscreteInput(3, "failed to complete = ");
    if (c == 'l') printDiscreteInput(4, "too low to move = ");
    if (c == 'D') {
        result = node.readDiscreteInputs(0, 5);
        
        // do something with data if read is successful
        if (result == node.ku8MBSuccess)
        {
            uint8_t result = node.getResponseBuffer(0);
            Serial.print("Open Relay = ");
            Serial.println((int)((result & 1) != 0));
            Serial.print("Closed Relay = ");
            Serial.println((int)((result & 2) != 0));
            Serial.print("moving = ");
            Serial.println((int)((result & 4) != 0));
            Serial.print("failed to complete = ");
            Serial.println((int)((result & 8) != 0));
            Serial.print("too low to move = ");
            Serial.println((int)((result & 16) != 0));
       }
      else
      {
        Serial.print("Error code: ");
        Serial.println(result);
      }
    }

    }
  }

   // Reset the watchdog timer
    rtc_wdt_feed();
}
