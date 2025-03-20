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
#define USE_LEDS
//#define USE_WATCHDOG

#ifdef USE_LEDS
#define LED1 13
#define LED2 12
#define LED3 14
#define LED4 27
#define LED5 26
#define LED6 25
#define LED7 33
#define LED8 32
#define LED9 23
#define LED10 22
#define LED11 15
#define LED12 2
#define LED13 5
#define LED14 18
#define LED15 19
#define LED16 21
#endif

// connections RS485 module to Arduino
// GND to GND
// VCC to 5V
// RXD to D2 ( set as RX_PIN below )
// TXD to D4 ( set as TX_PIN below )

#define STATUS_OK 0
#define STATUS_MODBUS_ERROR 1

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
uint16_t angle = 0;
uint8_t status = STATUS_OK;
uint8_t modbus_error = 0;

unsigned long myTime;

myTime = millis();

// Create an instance of the web server
#ifdef USE_WIFI
AsyncWebServer server(80);
#endif

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
    Serial.println("Open A pressed");
    // Add your logic here
    // tell actuator to close
    closeActuator();
}

void handleAuto() {
    Serial.println("Auto pressed");
    // Add your logic here
}

void handleOpenB() {
    Serial.println("Open B pressed");
    // Add your logic here
    // tell actuator to open
    openActuator();
}

void handleStop() {
    Serial.println("STOP pressed");
    // Add your logic here
    // tell actuator to stop
    stopActuator();
}

void handleBatteryVoltage(AsyncWebServerRequest *request) {
    request->send(200, "text/plain", String(14.0));
}

void handleAngle(AsyncWebServerRequest *request) {
    request->send(200, "text/plain", String(angle));
}

void handleStatus(AsyncWebServerRequest *request) {
  String status_str = "OK";
  switch(status)
  {
    case STATUS_MODBUS_ERROR:
      status_str = String("Modbus Error: ") + String(modbus_error);
      break;
  }
    request->send(200, "text/plain", status_str);
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

uint16_t getInputRegister(uint16_t u16ReadAddress)
{
#ifdef USE_MODBUS
  modbus_error = 0;
  uint8_t result = node.readInputRegisters(u16ReadAddress, 1);
        
  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    return (uint16_t)(node.getResponseBuffer(0));
  }
  else
  {
    Serial.print("Error code: ");
    Serial.println(result);
    modbus_error = result;
    return 0;
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

#ifdef USE_LEDS
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(LED5, OUTPUT);
  pinMode(LED6, OUTPUT);
  pinMode(LED7, OUTPUT);
  pinMode(LED8, OUTPUT);
  pinMode(LED9, OUTPUT);
  pinMode(LED10, OUTPUT);
  pinMode(LED11, OUTPUT);
  pinMode(LED12, OUTPUT);
  pinMode(LED13, OUTPUT);
  pinMode(LED14, OUTPUT);
  pinMode(LED15, OUTPUT);
  pinMode(LED16, OUTPUT);
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

#ifdef USE_WIFI
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

    server.on("/battery-voltage", HTTP_GET, handleBatteryVoltage);
    server.on("/angle", HTTP_GET, handleAngle);
    server.on("/status", HTTP_GET, handleStatus);

    // Start the server
    server.begin();
  #endif
delay(1000);

#if 1
   pinMode(LED12, OUTPUT);
    digitalWrite(LED12, LOW);
    pinMode(LED6, OUTPUT);
    digitalWrite(LED6, LOW);
#endif
    
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

#ifdef USE_LEDS
int delay_count = 0;
int led_test_count = 0;
#endif

void SetLEDs(uint16_t value)
{
#ifdef USE_LEDS
    digitalWrite(LED1, (value & 1) != 0 ? HIGH:LOW);
    digitalWrite(LED2, (value & 2) != 0 ? HIGH:LOW);
    digitalWrite(LED3, (value & 4) != 0 ? HIGH:LOW);
    digitalWrite(LED4, (value & 8) != 0 ? HIGH:LOW);
    digitalWrite(LED5, (value & 16) != 0 ? HIGH:LOW);
    digitalWrite(LED6, (value & 32) != 0 ? HIGH:LOW);
    digitalWrite(LED7, (value & 64) != 0 ? HIGH:LOW);
    digitalWrite(LED8, (value & 128) != 0 ? HIGH:LOW);
    digitalWrite(LED9, (value & 0x100) != 0 ? HIGH:LOW);
    digitalWrite(LED10, (value & 0x200) != 0 ? HIGH:LOW);
    digitalWrite(LED11, (value & 0x400) != 0 ? HIGH:LOW);
    digitalWrite(LED12, (value & 0x800) != 0 ? HIGH:LOW);
    digitalWrite(LED13, (value & 0x1000) != 0 ? HIGH:LOW);
    digitalWrite(LED14, (value & 0x2000) != 0 ? HIGH:LOW);
    digitalWrite(LED15, (value & 0x4000) != 0 ? HIGH:LOW);
    digitalWrite(LED16, (value & 0x8000) != 0 ? HIGH:LOW);
#endif
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
      if(reading_number == 'g')
      {
    uint16_t percentage = atoi(num);
    i = 0;
#ifdef USE_MODBUS  
    node.writeSingleRegister(0,percentage);
#endif
    Serial.print("go to percentage: ");
    Serial.println(percentage);
      }
      else if(reading_number == 'L')
      {
        SetLEDs(atoi(num));
      }
      reading_number = 0;

    delay(100);
    
#ifdef USE_MODBUS  
      // tell actuator to move to that percentage
      node.writeSingleCoil(2, 1);
#endif      
      break;
    }
    }
    else
    { // not reading_number
    if (c == 'o') {
      // tell actuator to open
#ifdef USE_MODBUS  
      node.writeSingleCoil(0, 1);
#endif
    }
    if (c == 'c') {
      // tell actuator to close
#ifdef USE_MODBUS  
      node.writeSingleCoil(1, 1);
#endif
    }
    if(c == 'g')
    {
      reading_number = 'g';
   }
    if (c == 's') {
      // tell actuator to stop
#ifdef USE_MODBUS  
      node.writeSingleCoil(3, 1);
#endif
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
#ifdef USE_MODBUS  
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
#endif
    }
#ifdef USE_MODBUS
    if (c == 'O') printDiscreteInput(0, "Open Relay = ");
    if (c == 'C') printDiscreteInput(1, "Closed Relay = ");
    if (c == 'm') printDiscreteInput(2, "moving = ");
    if (c == 't') printDiscreteInput(3, "failed to complete = ");
    if (c == 'l') printDiscreteInput(4, "too low to move = ");
#endif
    if (c == 'L')
          reading_number = 'L';

    if (c == 'D') {
#ifdef USE_MODBUS
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
#endif
    }

    }
  }

  angle = getInputRegister(1);
if(modbus_error == 0)
{
    Serial.print("Modbus Success Angle = ");
    Serial.println(angle);
}  
  status = STATUS_OK;
  if(modbus_error != 0)
  {
    status = STATUS_MODBUS_ERROR;
  }

#ifdef USE_LEDS
if(delay_count == 0)
{
  uint16_t led_value = 0x0001;
  for(int i = 0; i<16;i++)
  {
  // led test
  SetLEDs(led_value);
  led_value <<=1;
  delay(200);
  }

  led_test_count++;
  Serial.print("LED test ");
  Serial.println(led_test_count);
}
#endif

  delay(2000);
  delay_count++;
  if(delay_count == 30)delay_count = 0;

#ifdef USE_WATCHDOG
   // Reset the watchdog timer
    rtc_wdt_feed();
    #endif
}
