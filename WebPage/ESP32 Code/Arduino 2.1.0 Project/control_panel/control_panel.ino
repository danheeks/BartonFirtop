#include <WiFi.h>
#include <WebServer.h>
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

#define AB_PIN 34

// connections RS485 module to Arduino
// GND to GND
// VCC to 5V
// RXD to D2 ( set as RX_PIN below )
// TXD to D4 ( set as TX_PIN below )

#define STATUS_OK 0
#define STATUS_MODBUS_ERROR 1

#define SEQ_B_STANDBY 1
#define SEQ_DP_HIGH_A 2
#define SEQ_MOVE_TO_B 3
#define SEQ_WAIT_AFTER_MOVE_B 4
#define SEQ_A_STANDBY 5
#define SEQ_DP_HIGH_B 6
#define SEQ_MOVE_TO_A 7
#define SEQ_WAIT_AFTER_MOVE_A 8

// Set your WiFi credentials
const char* ssid = "BartonFirtop-Control";
const char* password = "123456789";
uint16_t position = 0;
uint8_t status = STATUS_OK;
uint8_t modbus_error = 0;
unsigned long oldTime;
uint16_t led_value = 0;
bool automan = false;
uint8_t auto_seq = SEQ_B_STANDBY;
uint8_t countdown = 0;
int waitTime1 = 30; // Default value
int waitTime2 = 15; // Default value
int waitTime3 = 60; // Default value
int waitTime4 = 300; // Default value
int AB_switch = 0;

// Create an instance of the web server
#ifdef USE_WIFI
WebServer server(80);
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
    automan = !automan;
    if(automan)
    {
      // auto sequence start
      auto_seq = SEQ_B_STANDBY;
      countdown = waitTime1;
    }
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
    automan = false;
}

void handleBatteryVoltage() {
    server.send(200, "text/plain", String(14.0));
}

void handlePosition() {
    server.send(200, "text/plain", String(position));
}

void handleSeq() {
  // return sequence string. Empty if in Manual Mode
  String seqstr;
  if(automan)
  {
  switch(auto_seq)
  {
    case SEQ_B_STANDBY:
    case SEQ_A_STANDBY:
        seqstr = "DP High in " + String(countdown) + " seconds";
        break;
    case SEQ_DP_HIGH_A:
    case SEQ_DP_HIGH_B:
        seqstr = "Move in " + String(countdown) + " seconds";
        break;
    case SEQ_WAIT_AFTER_MOVE_B:
    case SEQ_WAIT_AFTER_MOVE_A:
        seqstr = "Waiting for " + String(countdown) + " seconds";
        break;
    case SEQ_MOVE_TO_A:
        seqstr = "Moving to Chamber A";
        break;
    case SEQ_MOVE_TO_B:
        seqstr = "Moving to Chamber B";
        break;
  }
  }
    server.send(200, "text/plain", seqstr);
}

void handleAutoMan() {
    server.send(200, "text/plain", String(automan));
}

void handleLeds() {
    server.send(200, "text/plain", String(led_value));
}

void handleStatus() {
  String status_str = "OK";
  switch(status)
  {
    case STATUS_MODBUS_ERROR:
      status_str = String("Modbus Error: ") + String(modbus_error);
      break;
  }
    server.send(200, "text/plain", status_str);
}

void handleGetWaitTime1() {
    server.send(200, "text/plain", String(waitTime1));
}

void handleGetWaitTime2() {
    server.send(200, "text/plain", String(waitTime2));
}

void handleGetWaitTime3() {
    server.send(200, "text/plain", String(waitTime3));
}

void handleGetWaitTime4() {
    server.send(200, "text/plain", String(waitTime4));
}

void handleSetWaitTime1() {
    if (server.hasArg("plain")) {
        StaticJsonDocument<200> doc;
        deserializeJson(doc, server.arg("plain"));
        waitTime1 = doc["waitTime1"];
        server.send(200, "text/plain", "Wait1 time updated");
    } else {
        server.send(400, "text/plain", "Bad Request");
    }
}

void handleSetWaitTime2() {
    if (server.hasArg("plain")) {
        StaticJsonDocument<200> doc;
        deserializeJson(doc, server.arg("plain"));
        waitTime2 = doc["waitTime2"];
        server.send(200, "text/plain", "Wait2 time updated");
    } else {
        server.send(400, "text/plain", "Bad Request");
    }
}

void handleSetWaitTime3() {
    if (server.hasArg("plain")) {
        StaticJsonDocument<200> doc;
        deserializeJson(doc, server.arg("plain"));
        waitTime3 = doc["waitTime3"];
        server.send(200, "text/plain", "Wait3 time updated");
    } else {
        server.send(400, "text/plain", "Bad Request");
    }
}

void handleSetWaitTime4() {
    if (server.hasArg("plain")) {
        StaticJsonDocument<200> doc;
        deserializeJson(doc, server.arg("plain"));
        waitTime4 = doc["waitTime4"];
        server.send(200, "text/plain", "Wait4 time updated");
    } else {
        server.send(400, "text/plain", "Bad Request");
    }
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

void handleRoot() {
    File file = SPIFFS.open("/index.html", "r");
    if (!file) {
        server.send(500, "text/plain", "Failed to open file");
        return;
    }
    server.streamFile(file, "text/html");
    file.close();
}

// Serve static files (CSS, JS, images)
void handleFileRequest() {
    String path = server.uri();
    Serial.println("Requested file: " + path);

    if (path == "/") {
        path = "/index.html";  // Default to index.html
    }

    File file = SPIFFS.open(path, "r");
    if (!file) {
        server.send(404, "text/plain", "File Not Found");
        return;
    }

    // Determine content type
    String contentType = "text/plain";
    if (path.endsWith(".html")) contentType = "text/html";
    else if (path.endsWith(".css")) contentType = "text/css";
    else if (path.endsWith(".js")) contentType = "application/javascript";
    else if (path.endsWith(".png")) contentType = "image/png";
    else if (path.endsWith(".jpg") || path.endsWith(".jpeg")) contentType = "image/jpeg";
    else if (path.endsWith(".gif")) contentType = "image/gif";
    else if (path.endsWith(".ico")) contentType = "image/x-icon";
    else if (path.endsWith(".svg")) contentType = "image/svg+xml";

    server.streamFile(file, contentType);
    file.close();
}

void handlePostAction() {
    if (server.method() != HTTP_POST) {
        server.send(405, "text/plain", "Method Not Allowed");
        return;
    }

    // Read the request body
    String body = server.arg(0);
    Serial.println("Received POST body: " + body);

    // Parse JSON
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, body);
    if (error) {
        Serial.println("Failed to parse JSON");
        server.send(400, "text/plain", "Bad Request: Invalid JSON");
        return;
    }

    String action = doc["action"];
    Serial.println("Action: " + action);

    // Perform actions based on the "action" field
    if (action == "openA") {
        Serial.println("Opening Chamber A...");
        handleOpenA();
    } else if (action == "auto") {
        Serial.println("Auto Pressed...");
        handleAuto();
    } else if (action == "openB") {
        Serial.println("Opening Chamber B...");
        handleOpenB();
    } else if (action == "stop") {
        Serial.println("Stopping system...");
        handleStop();
    } else {
        Serial.println("Unknown action received");
        server.send(400, "text/plain", "Bad Request: Unknown action");
        return;
    }

    server.send(200, "text/plain", "Action received: " + action);
}

// Handle 404 (Not Found)
void handleNotFound() {
    server.send(404, "text/plain", "404: Not Found");
}

void setup() {
#ifdef USE_WATCHDOG
  rtc_wdt_protect_off();    // Turns off the automatic wdt service
  rtc_wdt_enable();         // Turn it on manually
  rtc_wdt_set_time(RTC_WDT_STAGE0, 5000);  // Define how long you desire to let dog wait.
#endif

  oldTime = millis();

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
  pinMode(AB_PIN, INPUT_PULLUP);
  pinMode(35, INPUT_PULLUP);

  while(1)
  {

  }

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

    server.on("/", HTTP_GET, handleRoot);
     server.on("/action", HTTP_POST, handlePostAction);
   server.on("/battery-voltage", HTTP_GET, handleBatteryVoltage);
    server.on("/position", HTTP_GET, handlePosition);
    server.on("/seq", HTTP_GET, handleSeq);
    server.on("/automan", HTTP_GET, handleAutoMan);
    server.on("/leds", HTTP_GET, handleLeds);
    server.on("/status", HTTP_GET, handleStatus);
    server.on("/getWaitTime1", HTTP_GET, handleGetWaitTime1);
    server.on("/setWaitTime1", HTTP_POST, handleSetWaitTime1);
    server.on("/getWaitTime2", HTTP_GET, handleGetWaitTime2);
    server.on("/setWaitTime2", HTTP_POST, handleSetWaitTime2);
    server.on("/getWaitTime3", HTTP_GET, handleGetWaitTime3);
    server.on("/setWaitTime3", HTTP_POST, handleSetWaitTime3);
    server.on("/getWaitTime4", HTTP_GET, handleGetWaitTime4);
    server.on("/setWaitTime4", HTTP_POST, handleSetWaitTime4);
    server.onNotFound(handleFileRequest);  // Serve static files from SPIFFS

    // Start the server
    server.begin();
  #endif
delay(1000);

    // set these LEDs as output again, after they were messed up by the wifi initialisation

   pinMode(LED12, OUTPUT);
    digitalWrite(LED12, LOW);
    pinMode(LED6, OUTPUT);
    digitalWrite(LED6, LOW);
    
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

void SetLEDs(uint16_t value)
{
#ifdef USE_LEDS
    digitalWrite(LED1, (value & 0x0001) != 0 ? HIGH:LOW);
    digitalWrite(LED2, (value & 0x0002) != 0 ? HIGH:LOW);
    digitalWrite(LED3, (value & 0x0004) != 0 ? HIGH:LOW);
    digitalWrite(LED4, (value & 0x0008) != 0 ? HIGH:LOW);
    digitalWrite(LED5, (value & 0x0010) != 0 ? HIGH:LOW);
    digitalWrite(LED6, (value & 0x0020) != 0 ? HIGH:LOW);
    digitalWrite(LED7, (value & 0x0040) != 0 ? HIGH:LOW);
    digitalWrite(LED8, (value & 0x0080) != 0 ? HIGH:LOW);
    digitalWrite(LED9, (value & 0x0100) != 0 ? HIGH:LOW);
    digitalWrite(LED10, (value & 0x0200) != 0 ? HIGH:LOW);
    digitalWrite(LED11, (value & 0x0400) != 0 ? HIGH:LOW);
    digitalWrite(LED12, (value & 0x0800) != 0 ? HIGH:LOW);
    digitalWrite(LED13, (value & 0x1000) != 0 ? HIGH:LOW);
    digitalWrite(LED14, (value & 0x2000) != 0 ? HIGH:LOW);
    digitalWrite(LED15, (value & 0x4000) != 0 ? HIGH:LOW);
    digitalWrite(LED16, (value & 0x8000) != 0 ? HIGH:LOW);
#endif
    led_value = value;
}

bool every_other = false;

void loop() {
  server.handleClient();  // Handle incoming requests

  unsigned long newTime = millis();
  if(newTime > oldTime + 1000)
  {
    every_other = !every_other;
    oldTime = newTime;

    position = getInputRegister(1);
    if(modbus_error == 0)
    {
      status = STATUS_OK;
    }  
    else
    {
      status = STATUS_MODBUS_ERROR;
    }

    // check state of AB switch
    AB_switch = digitalRead(AB_PIN);

    uint16_t new_led_state = 0x0000;
    new_led_state |= 0x8000; // eXTERNAL pOWWER

    if(modbus_error != 0)
    {
      new_led_state |= 0x4000; // system fault
    }
    else
    {
      // a, b, online
      if(position == 0)
      {
        new_led_state |= 0x0001; // A On line
        new_led_state |= 0x0200; // B Off Line
      }
      else if(position == 100)
      {
        new_led_state |= 0x0100; // B On line
        new_led_state |= 0x0002; // A Off Line
      }
      else
      {
        if(every_other)new_led_state |= 0x2000; // Changeover Active Flashing
      }
    }

    // debug the AB switch
    if(AB_switch)new_led_state |= 0x0080; // light up solar power

    // process auto sequence
    if(automan)
    {
      switch(auto_seq)
      {
        case SEQ_B_STANDBY:
            new_led_state |= 0x0400; // B Standby
            new_led_state |= 0x0008; // DP A OK
            new_led_state |= 0x0800; // DP B OK
            if(countdown > 0)
              countdown--;
            if(countdown == 0)
            {
              auto_seq = SEQ_DP_HIGH_A;
              countdown = waitTime2;
            }
            break;

        case SEQ_DP_HIGH_A:
            new_led_state |= 0x0010; // DP A High
            if(every_other)new_led_state |= 0x0400; // B Standby
            new_led_state |= 0x0040; // Maintenance Required
            new_led_state |= 0x0800; // DP B OK
            new_led_state |= 0x2000; // Changeover Active
            new_led_state |= 0x0020; // Equalisation Valve yellow
            if(countdown > 0)
              countdown--;
            if(countdown == 0)
            {
              auto_seq = SEQ_MOVE_TO_B;
              delay(100);
              openActuator();
            }
            break;
        case SEQ_MOVE_TO_B:
            // opening
            new_led_state |= 0x0010; // DP A High
            new_led_state |= 0x0800; // DP B OK
            new_led_state |= 0x0040; // Maintenance Required
            new_led_state |= 0x0020; // Equalisation Valve yellow
            if(position == 100)
            {
                auto_seq = SEQ_WAIT_AFTER_MOVE_B;
                countdown = waitTime3;
            }            
            break;
        case SEQ_WAIT_AFTER_MOVE_B:
            new_led_state |= 0x0008; // DP A OK
            new_led_state |= 0x0800; // DP B OK
            new_led_state |= 0x0040; // Maintenance Required
            if(countdown > 0)
              countdown--;
            if(countdown == 0)
            {
              auto_seq = SEQ_A_STANDBY;
              countdown = waitTime4;
            }
            break;
        case SEQ_A_STANDBY:
            new_led_state |= 0x0004; // A Standby
            new_led_state |= 0x0008; // DP A OK
            new_led_state |= 0x0800; // DP B OK
            if(countdown > 0)
              countdown--;
            if(countdown == 0)
            {
              auto_seq = SEQ_DP_HIGH_B;
              countdown = waitTime2;
            }
            break;
        case SEQ_DP_HIGH_B:
            new_led_state |= 0x1000; // DP B High
            new_led_state |= 0x0040; // Maintenance Required
            new_led_state |= 0x0008; // DP A OK
            new_led_state |= 0x2000; // Changeover Active
            new_led_state |= 0x0020; // Equalisation Valve yellow
            if(countdown > 0)
              countdown--;
            if(countdown == 0)
            {
              auto_seq = SEQ_MOVE_TO_A;
              delay(100);
              closeActuator();
            }
            break;
        case SEQ_MOVE_TO_A:
            new_led_state |= 0x0008; // DP A OK
            new_led_state |= 0x1000; // DP B High
            new_led_state |= 0x0040; // Maintenance Required
            new_led_state |= 0x0020; // Equalisation Valve yellow
            // closing
            if(position == 0)
            {
                auto_seq = SEQ_B_STANDBY;
                countdown = waitTime4;
            }            
            break;
        case SEQ_WAIT_AFTER_MOVE_A:
            new_led_state |= 0x0008; // DP A OK
            new_led_state |= 0x0800; // DP B OK
            new_led_state |= 0x0040; // Maintenance Required
            if(countdown > 0)
              countdown--;
            if(countdown == 0)
            {
              auto_seq = SEQ_A_STANDBY;
              countdown = waitTime4;
            }
            break;
      }
    }
    else
    {
      // manual mode
        new_led_state |= 0x0008; // DP A OK
        new_led_state |= 0x0800; // DP B OK

        // check AB switch
    }

    SetLEDs(new_led_state);
  }

#ifdef USE_WATCHDOG
   // Reset the watchdog timer
    rtc_wdt_feed();
    #endif
}
