#include <WiFi.h>
#include <WebServer.h>
#include <FS.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <ModbusMaster.h>
#include <HardwareSerial.h>
#include "soc/rtc_wdt.h"


/*
LED codes

0001 - Chamber 'A' On-line
0002 - Chamber 'A' Standby
0004 - Differential Pressure OK 'A'
0008 - Differentail Pressure High 'A'
0010 - Basket Cleaning Required 'A'
0020 - Equalisation Valve/Prime
0040 - Not used ( was maintenance required )
0080 - Solar Power
0100 - Chamber 'B' On-line
0200 - Chamber 'B' Standby
0400 - Differential Pressure OK 'B'
0800 - Differential Pressure High 'B'
1000 - Basket Cleaning  Required 'B'
2000 - Changeover Active
4000 - System Fault
8000 - External Power
*/


//#define USE_WATCHDOG

#define LED1 13 // Chamber 'A' On-line                       Green
#define LED2 12 // Chamber 'A' Standby                       Yellow
#define LED3 14 // Differential Pressure OK 'A'              Green
#define LED4 27 // Differentail Pressure High 'A'            Red
#define LED5 26 // Basket Cleaning Required 'A'              Yellow
#define LED6 25 // Equalisation Valve/Prime                  Yellow
#define LED7 33 // Not used ( was maintenance required ) 
#define LED8 32 // Solar Power                               Green
#define LED9 23 // Chamber 'B' On-line                       Green
#define LED10 22 // Chamber 'B' Standby                      Yellow
#define LED11 15 // Differential Pressure OK 'B'             Green
#define LED12 2 // Differential Pressure High 'B'            Red
#define LED13 5 // Basket Cleaning  Required 'B'             Yellow
#define LED14 18 // Changeover Active                        Yellow
#define LED15 19 // System Fault                             Red
#define LED16 21 // External Power                           Green

#define A_PIN 34
#define B_PIN 35

// connections RS485 module to Arduino
// GND to GND
// VCC to 5V
// RXD to D2 ( set as RX_PIN below )
// TXD to D4 ( set as TX_PIN below )

#define STATUS_OK 0
#define STATUS_MODBUS_ERROR 1

#define SEQ_NOT_SET 0
#define SEQ_B_STANDBY 1
#define SEQ_DP_HIGH_A 2
#define SEQ_PRIME_B 3
#define SEQ_MOVE_TO_B 4
#define SEQ_WAIT_AFTER_MOVE_B 5
#define SEQ_A_STANDBY 6
#define SEQ_DP_HIGH_B 7
#define SEQ_PRIME_A 8
#define SEQ_MOVE_TO_A 9
#define SEQ_WAIT_AFTER_MOVE_A 10

#define MAN_NOT_IN_MANUAL 0
#define MAN_NEUTRAL 1 // AB switch must be in vertical position to start
#define MAN_A_PRIME 2
#define MAN_A_MOVE 3
#define MAN_A_REACHED 4
#define MAN_A_COMPLETE 5
#define MAN_B_PRIME 6
#define MAN_B_MOVE 7
#define MAN_B_REACHED 8
#define MAN_B_COMPLETE 9

// Set your WiFi credentials
const char* ssid = "Paul's Phone";
const char* password = "Mani1234";
uint16_t position = 0;
uint8_t status = STATUS_OK;
uint8_t modbus_error = 0;
unsigned long oldTime;
uint16_t led_value = 0;
bool app_automan = false;
uint8_t auto_seq = SEQ_NOT_SET;
uint8_t countdown = 0;
int waitTime1 = 15; // Standby wait
int waitTime2 = 15; // High wait
int waitTime3 = 10; // Prime wait
int waitTime4 = 600; // after move wait
int waitTime5 = 5; // manual mode dwell
uint8_t remote_auto_manual = 2; // 0 - remote(app), 1 - auto, 2 - manual
uint8_t manual_mode = MAN_NOT_IN_MANUAL;
uint8_t app_A_pressed = 0;
uint8_t app_B_pressed = 0;
uint8_t A_switch = 0;
uint8_t B_switch = 0;
uint8_t powerSource = 0; // 1 for solar, 0 for external

// Create an instance of the web server
WebServer server(80);

HardwareSerial rs485(1); // Use UART1

#define RX_PIN 17
#define TX_PIN 16

// instantiate ModbusMaster object
ModbusMaster node;

char reading_number = 0;
char num[64];
int i = 0;

void openActuator()
{
    // tell actuator to open
    node.writeSingleCoil(0, 1);
}

void closeActuator()
{
    // tell actuator to close
    node.writeSingleCoil(1, 1);
}

void stopActuator()
{
    // tell actuator to stop
    node.writeSingleCoil(3, 1);
}

// Functions triggered by button presses
void handleOpenA() {
    Serial.println("Open A pressed");
    if(remote_auto_manual == 0)
    {
      app_A_pressed = 1;
      app_B_pressed = 0;
    }
}

void handleAuto() {
    Serial.println("Auto pressed");
    if(remote_auto_manual == 0)
    {
      app_automan = !app_automan;
    }
    // Add your logic here
}

void handleOpenB() {
    Serial.println("Open B pressed");
    app_A_pressed = 0;
    app_B_pressed = 1;
}

void handleStop() {
    Serial.println("STOP pressed");
    stopActuator();
    app_A_pressed = 0;
    app_B_pressed = 0;
    app_automan = false;
}

void handleBatteryVoltage() {
    server.send(200, "text/plain", String(14.0));
}

void handlePosition() {
    server.send(200, "text/plain", String(position));
}

void handleSeq() {
  // return sequence string.
  String seqstr;
  if((remote_auto_manual == 1) || ((remote_auto_manual == 0) && app_automan))
  {
    // auto mode
    switch(auto_seq)
    {
      case SEQ_B_STANDBY:
      case SEQ_A_STANDBY:
          seqstr = "DP High in " + String(countdown) + " seconds";
          break;
      case SEQ_DP_HIGH_A:
      case SEQ_DP_HIGH_B:
          seqstr = "Prime in " + String(countdown) + " seconds";
          break;
      case SEQ_PRIME_A:
      case SEQ_PRIME_B:
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
  else
  {
    // manual
    if(countdown > 0)
      seqstr = "Waiting for " + String(countdown) + " seconds";
  }
    server.send(200, "text/plain", seqstr);
}

void handleAutoMan() {
    server.send(200, "text/plain", String(app_automan));
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

void handleGetWaitTime5() {
    server.send(200, "text/plain", String(waitTime5));
}

void handleGetPowerSource() {
    server.send(200, "text/plain", String(powerSource));
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

void handleSetWaitTime5() {
    if (server.hasArg("plain")) {
        StaticJsonDocument<200> doc;
        deserializeJson(doc, server.arg("plain"));
        waitTime5 = doc["waitTime5"];
        server.send(200, "text/plain", "Wait5 time updated");
    } else {
        server.send(400, "text/plain", "Bad Request");
    }
}

void handleSetPowerSource() {
    if (server.hasArg("plain")) {
        StaticJsonDocument<200> doc;
        deserializeJson(doc, server.arg("plain"));
        powerSource = doc["powerSource"];
        server.send(200, "text/plain", "powerSource updated");
    } else {
        server.send(400, "text/plain", "Bad Request");
    }
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

uint16_t getInputRegister(uint16_t u16ReadAddress)
{
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
}

void printDiscreteInput(uint16_t u16ReadAddress, char* str)
{
    uint8_t result = node.readDiscreteInputs(u16ReadAddress, 1);
    
    // do something with data if read is successful
    if (result == node.ku8MBSuccess)
    {
        Serial.print(str);
        Serial.println((int)(node.getResponseBuffer(0) & 0x01) != 0);
  }
  else
  {
    Serial.print("Error code: ");
    Serial.println(result);
  }
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
  pinMode(A_PIN, INPUT);
  pinMode(B_PIN, INPUT);
  pinMode(36, INPUT);  // VP as digital input, pullup not available so add 10k resistor on board
  pinMode(39, INPUT);  // VN as digital input, pullup not available so add 10k resistor on board
  
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
    server.on("/getWaitTime5", HTTP_GET, handleGetWaitTime5);
    server.on("/setWaitTime5", HTTP_POST, handleSetWaitTime5);
    server.on("/getPowerSource", HTTP_GET, handleGetPowerSource);
    server.on("/setPowerSource", HTTP_POST, handleSetPowerSource);
    server.onNotFound(handleFileRequest);  // Serve static files from SPIFFS

    // Start the server
    server.begin();

delay(1000);

    // set these LEDs as output again, after they were messed up by the wifi initialisation

   pinMode(LED12, OUTPUT);
    digitalWrite(LED12, LOW);
    pinMode(LED6, OUTPUT);
    digitalWrite(LED6, LOW);
    
    Serial.println("Server started");
}

bool isDiscreteInputTrue(uint16_t u16ReadAddress)
{
    uint8_t result = node.readDiscreteInputs(u16ReadAddress, 1);
    
    // do something with data if read is successful
    if (result == node.ku8MBSuccess)
    {
        return (int)(node.getResponseBuffer(0) & 0x01) != 0; // Only LSB is valid
    }
  else
  {
    Serial.print("Error code: ");
    Serial.println(result);
  }
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

    led_value = value;
}

bool every_other = false;

void loop() {
  server.handleClient();  // Handle incoming requests

  // set remote/auto/manual from 3 way key switch
  if(!digitalRead(39) /* VN */)remote_auto_manual = 0; // remote ( app )
  else if(!digitalRead(36) /* VP */)remote_auto_manual = 2; // manual
  else remote_auto_manual = 1; // auto








  /* DODGY TEST */
  remote_auto_manual = 0;









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
      }
      else if(position == 100)
      {
        new_led_state |= 0x0100; // B On line
      }
      else
      {
        if(every_other)new_led_state |= 0x2000; // Changeover Active Flashing
      }
    }

    if(powerSource)
      new_led_state |= 0x0080;// Solar
    else
      new_led_state |= 0x8000;// External Power

    // process auto sequence
    if((remote_auto_manual == 1) || ((remote_auto_manual == 0) && app_automan)) 
    {
      switch(auto_seq)
      {
        case SEQ_NOT_SET:
          // auto sequence start
          if(position > 50)
            auto_seq = SEQ_A_STANDBY;
          else
            auto_seq = SEQ_B_STANDBY;
          countdown = waitTime1;
          break;

        case SEQ_B_STANDBY:
            new_led_state |= 0x0200; // B Standby
            new_led_state |= 0x0004; // DP A OK
            if(countdown > 0)
              countdown--;
            if(countdown == 0)
            {
              auto_seq = SEQ_DP_HIGH_A;
              countdown = waitTime2;
            }
            break;

        case SEQ_DP_HIGH_A:
            new_led_state |= 0x0008; // DP A High
            new_led_state |= 0x0200; // B Standby
            if(countdown > 0)
              countdown--;
            if(countdown == 0)
            {
              auto_seq = SEQ_PRIME_B;
              countdown = waitTime3;
            }
            break;

        case SEQ_PRIME_B:
            new_led_state |= 0x0008; // DP A High
            if(every_other)new_led_state |= 0x0020; // Equalisation Valve/Prime flashing
            new_led_state |= 0x0200; // B Standby
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
            new_led_state |= 0x0200; // Chamber 'B' Standby
            if(every_other)new_led_state |= 0x2000; // Changeover Active flashing
            if(!every_other)new_led_state &= 0xfffe; // Chamber 'A' online flashing
            if(position == 100)
            {
                auto_seq = SEQ_WAIT_AFTER_MOVE_B;
                countdown = waitTime4;
            }            
            break;
        case SEQ_WAIT_AFTER_MOVE_B:
            new_led_state |= 0x0400; // DP B OK
            new_led_state |= 0x0010; // Basket Cleaning Required 'A'
            if(countdown > 0)
              countdown--;
            if(countdown == 0)
            {
              auto_seq = SEQ_A_STANDBY;
              countdown = waitTime1;
            }
            break;
        case SEQ_A_STANDBY:
            new_led_state |= 0x0002; // A Standby
            new_led_state |= 0x0400; // DP B OK
            if(countdown > 0)
              countdown--;
            if(countdown == 0)
            {
              auto_seq = SEQ_DP_HIGH_B;
              countdown = waitTime2;
            }
            break;

        case SEQ_DP_HIGH_B:
            new_led_state |= 0x0800; // DP B High
            new_led_state |= 0x0002; // A Standby
            if(countdown > 0)
              countdown--;
            if(countdown == 0)
            {
              auto_seq = SEQ_PRIME_A;
              countdown = waitTime3;
            }
            break;

        case SEQ_PRIME_A:
            new_led_state |= 0x0800; // DP B High
            if(every_other)new_led_state |= 0x0020; // Equalisation Valve/Prime flashing
            new_led_state |= 0x0002; // A Standby
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
            // opening
            new_led_state |= 0x0002; // Chamber 'A' Standby
            if(every_other)new_led_state |= 0x2000; // Changeover Active flashing
            if(!every_other)new_led_state &= 0xfeff; // Chamber 'B' online flashing
            if(position == 0)
            {
                auto_seq = SEQ_WAIT_AFTER_MOVE_A;
                countdown = waitTime4;
            }            
            break;
        case SEQ_WAIT_AFTER_MOVE_A:
            new_led_state |= 0x0004; // DP A OK
            new_led_state |= 0x1000; // Basket Cleaning Required 'B'
            if(countdown > 0)
              countdown--;
            if(countdown == 0)
            {
              auto_seq = SEQ_B_STANDBY;
              countdown = waitTime1;
            }
            break;
      }
      manual_mode = MAN_NOT_IN_MANUAL; // reset manual sequence when in auto mode
    }
    else
    {
      // check state of AB switch
      A_switch = !digitalRead(A_PIN);
      B_switch = !digitalRead(B_PIN);

      uint8_t use_A = (remote_auto_manual == 0) ? app_A_pressed : A_switch;
      uint8_t use_B = (remote_auto_manual == 0) ? app_B_pressed : B_switch;

      // process manual sequence
      switch(manual_mode)
      {
        case MAN_NOT_IN_MANUAL:
          if(!use_A && !use_B)
            manual_mode = MAN_NEUTRAL;
          break;

        case MAN_NEUTRAL:
          if(use_A)
          {
            if(position == 0) // if already at A, jump to next sequence
              manual_mode = MAN_A_REACHED;
            else
              manual_mode = MAN_A_PRIME;
          }
          else if(use_B)
          {
            if(position == 100) // if already at B, jump to next sequence
              manual_mode = MAN_B_REACHED;
            else
              manual_mode = MAN_B_PRIME;
          }
          countdown = waitTime5; // reset dwell time
          break;

        case MAN_B_PRIME:
          if(use_B)
          {
            if(countdown == 0) // dwell time waited
            {
              manual_mode = MAN_B_MOVE;
              openActuator();
            }
            else
            {
              // Prime offline chamber - still waiting for dwell time
              new_led_state |= 0x0004; // Differential Pressure OK 'A'
              if(every_other)new_led_state |= 0x0020; // Equalisation Valve/Prime flashing
              new_led_state |= 0x0200; // B Standby
              countdown--;
            }
          }
          else
            manual_mode = MAN_NEUTRAL; // switched away from B
          break;

        case MAN_B_MOVE:
          if(use_B)
          {
            // Changeover in progress
            if(every_other)new_led_state |= 0x2000; // Changeover Active flashing
            if(!every_other)new_led_state &= 0xfffe; // Chamber 'A' On-Line flashing
            if(every_other)new_led_state |= 0x0200; // B Standby flashing
            if(position == 100) // B position
            {
              manual_mode = MAN_B_REACHED;
              countdown = waitTime5; // reset dwell time
            }
          }
          else
            manual_mode = MAN_NEUTRAL; // switched away from B
          break;

        case MAN_B_REACHED:
          if(use_B)
          {
            if(countdown == 0) // dwell time waited
            {
              manual_mode = MAN_B_COMPLETE;
            }
            else
            {
              // Maintenance required on Chamber A
              new_led_state |= 0x0400; // Differential Pressure OK 'B'
              new_led_state |= 0x0010; // Basket Cleaning Required 'A'
              countdown--;
            }
          }
          else
            manual_mode = MAN_NEUTRAL; // switched away from B
          break;

        case MAN_B_COMPLETE:
          if(use_B)
          {
            // Maintenance complete on Chamber A
            new_led_state |= 0x0400; // Differential Pressure OK 'B'
            new_led_state |= 0x0002; // Chamber 'A' Standby
          }
          else
            manual_mode = MAN_NEUTRAL; // switched away from B
          break;

        case MAN_A_PRIME:
          if(use_A)
          {
            if(countdown == 0) // dwell time waited
            {
              manual_mode = MAN_A_MOVE;
              closeActuator();
            }
            else
            {
              // Prime offline chamber - still waiting for dwell time
              new_led_state |= 0x0400; // Differential Pressure OK 'B'
              if(every_other)new_led_state |= 0x0020; // Equalisation Valve/Prime flashing
              new_led_state |= 0x0002; // A Standby
              countdown--;
            }
          }
          else
            manual_mode = MAN_NEUTRAL; // switched away from A
          break;

        case MAN_A_MOVE:
          if(use_A)
          {
            // Changeover in progress
            if(every_other)new_led_state |= 0x2000; // Changeover Active flashing
            if(!every_other)new_led_state &= 0xfeff; // Chamber 'B' On-Line flashing
            if(every_other)new_led_state |= 0x0002; // A Standby flashing
            if(position == 0) // A position
            {
              manual_mode = MAN_A_REACHED;
              countdown = waitTime5; // reset dwell time
            }
          }
          else
            manual_mode = MAN_NEUTRAL; // switched away from A
          break;

        case MAN_A_REACHED:
          if(use_A)
          {
            if(countdown == 0) // dwell time waited
            {
              manual_mode = MAN_A_COMPLETE;
            }
            else
            {
              // Maintenance required on Chamber B
              new_led_state |= 0x0004; // Differential Pressure OK 'A'
              new_led_state |= 0x1000; // Basket Cleaning Required 'B'
              countdown--;
            }
          }
          else
            manual_mode = MAN_NEUTRAL; // switched away from A
          break;

        case MAN_A_COMPLETE:
          if(use_A)
          {
            // Maintenance complete on Chamber B
            new_led_state |= 0x0004; // Differential Pressure OK 'A'
            new_led_state |= 0x0200; // Chamber 'B' Standby
          }
          else
            manual_mode = MAN_NEUTRAL; // switched away from A
          break;
      }

        auto_seq = SEQ_NOT_SET; // reset auto sequence when in manual mode
    }

    SetLEDs(new_led_state);
  }

#ifdef USE_WATCHDOG
   // Reset the watchdog timer
    rtc_wdt_feed();
    #endif
}
