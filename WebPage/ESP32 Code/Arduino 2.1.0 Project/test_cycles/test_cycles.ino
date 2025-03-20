#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <FS.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <ModbusMaster.h>
#include <HardwareSerial.h>
#include "soc/rtc_wdt.h"


#define USE_MODBUS
#define USE_WIFI
#define USE_WATCHDOG


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

// Create an instance of the web server
#ifdef USE_WIFI
AsyncWebServer server(80);
#endif

AsyncWebSocket ws("/ws");

#define STATUS_READY 0
#define STATUS_CLOSING 1
#define STATUS_OPENING 2
#define STATUS_WAITING 3

int cycleCount = 0;
int maxCycles = 0;
int waitTime = 20; // Default wait time in seconds
int status = STATUS_READY;
bool next_move_is_open = false; // start with close move
int timeWaited = 0;

void notifyClients() {
  String status_string = "";
  switch(status)
  {
    case STATUS_READY:
      status_string = "Ready...";
      break;
    case STATUS_OPENING:
      status_string = "Opening...";
      break;
    case STATUS_CLOSING:
      status_string = "Closing...";
      break;
    case STATUS_WAITING:
      status_string = "Waiting time, time waited = " + String(timeWaited/2);
      break;
  }
    String message = String("{\"cycleDone\":") + cycleCount + ", \"status\":\"" + status_string + "\"}";
    ws.textAll(message);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->opcode == WS_TEXT) {
        String msg = String((char*)data).substring(0, len);
        if (msg.startsWith("start:")) {
            maxCycles = msg.substring(6).toInt();
            status = STATUS_WAITING;
            next_move_is_open = true;
            timeWaited = waitTime * 2;
        }
        else if (msg == "stop") {
            stopActuator();
            Serial.println("stop");
            status = STATUS_READY;
        } 
        else if (msg.startsWith("waitTime:")) {
            waitTime = msg.substring(9).toInt();
        }
        notifyClients();
    }
}

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_DATA) {
        handleWebSocketMessage(arg, data, len);
    }
}

#ifdef USE_MODBUS
HardwareSerial rs485(1); // Use UART1

#define RX_PIN 17
#define TX_PIN 16

// instantiate ModbusMaster object
ModbusMaster node;
#endif
char reading_number = 0;
char num[64];
int i = 0;

void openActuator()
{
#ifdef USE_MODBUS  
    // tell actuator to open
    node.writeSingleCoil(0, 1);
#endif
}

void closeActuator()
{
#ifdef USE_MODBUS  
    // tell actuator to close
    node.writeSingleCoil(1, 1);
#endif
}

void stopActuator()
{
#ifdef USE_MODBUS  
    // tell actuator to stop
    node.writeSingleCoil(3, 1);
#endif
}

// Functions triggered by button presses
void handleOpenA() {
#ifdef USE_MODBUS  
    Serial.println("Open A pressed");
    // Add your logic here
    // tell actuator to close
    node.writeSingleCoil(1, 1);
#endif
}

void handleAuto() {
    Serial.println("Auto pressed");
    // Add your logic here
}

void handleOpenB() {
    Serial.println("Open B pressed");
    // Add your logic here
    // tell actuator to open
#ifdef USE_MODBUS  
    node.writeSingleCoil(0, 1);
#endif
}

void handleStop() {
    Serial.println("STOP pressed");
    // Add your logic here
    // tell actuator to stop
#ifdef USE_MODBUS  
      node.writeSingleCoil(3, 1);
#endif
}

void printInputRegister(uint16_t u16ReadAddress, char* str)
{
#ifdef USE_MODBUS
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
#endif
}

void printDiscreteInput(uint16_t u16ReadAddress, char* str)
{
#ifdef USE_MODBUS
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
#endif
}

void setup() {
#ifdef USE_WATCHDOG
    rtc_wdt_protect_off();    // Turns off the automatic wdt service
rtc_wdt_enable();         // Turn it on manually
rtc_wdt_set_time(RTC_WDT_STAGE0, 5000);  // Define how long you desire to let dog wait.
#endif


#ifdef USE_MODBUS  
    rs485.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

    // communicate with Modbus slave ID 1 over RS485
    node.begin(1, rs485);
#endif

    Serial.begin(115200);

    delay(1000);

#ifdef USE_WIFI
    // Set up the ESP32 as an access point
    WiFi.softAP(ssid, password);
#endif

    // Initialize SPIFFS
    if (!SPIFFS.begin(true)) {
        Serial.println("Failed to mount SPIFFS");
        return;
    }

    ws.onEvent(onWebSocketEvent);
    server.addHandler(&ws);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/index.html", "text/html");
    });

    server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/script.js", "application/javascript");
    });

    server.begin();

delay(1000);

#ifdef USE_WIFI
    Serial.println("Server started");
#endif
}

bool isDiscreteInputTrue(uint16_t u16ReadAddress)
{
#ifdef USE_MODBUS  
    uint8_t result = node.readDiscreteInputs(u16ReadAddress, 1);
    
    // do something with data if read is successful
    if (result == node.ku8MBSuccess)
    {
        return (int)(node.getResponseBuffer(0)) != 0;
    }
  else
  {
    Serial.print("Error code: ");
    Serial.println(result);
  }
#endif
  return false;
}

bool isOpen()
{
  return isDiscreteInputTrue(0);
}

bool isClosed()
{
  return isDiscreteInputTrue(1);
}

void loop() {
  switch(status)
  {
    case STATUS_OPENING:
    {
      if(isOpen())
      {
          status = STATUS_WAITING;
          timeWaited = 0;
      }
    }
    break;

    case STATUS_CLOSING:
    {
      if(isClosed())
      {
        Serial.println("is Closed");
        cycleCount++;
        if(cycleCount >= maxCycles)
        {
          status = STATUS_READY;
        }
        else
        {
          status = STATUS_WAITING;
          timeWaited = 0;
        }
      }
    }
    break;

    case STATUS_WAITING:
    {
      timeWaited++;
      if(timeWaited >= waitTime * 2)
      {
            if(next_move_is_open)
            {
              openActuator();
              status = STATUS_OPENING;
              next_move_is_open = false;
           }
            else
            {
              closeActuator();
              status = STATUS_CLOSING;
              next_move_is_open = true;
            }
      }
    }
    break;
  }

  notifyClients();
  delay(500);

#ifdef USE_WATCHDOG
   // Reset the watchdog timer
    rtc_wdt_feed();
    #endif
}
