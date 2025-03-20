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

// connections RS485 module to Arduino
// GND to GND
// VCC to 5V
// RXD to D2 ( set as RX_PIN below )
// TXD to D4 ( set as TX_PIN below )

#define STATUS_OK 0
#define STATUS_MODBUS_ERROR 1

// Set your WiFi credentials
const char* ssid = "BartonFirtop-Control";
const char* password = "123456789";
uint16_t angle = 0;
uint8_t status = STATUS_OK;
uint8_t modbus_error = 0;
unsigned long oldTime;

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

void handleBatteryVoltage() {
    server.send(200, "text/plain", String(14.0));
}

void handleAngle() {
    server.send(200, "text/plain", String(angle));
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

const char* htmlPage = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Barton Firtop Dashboard</title>
    <style>
        /* Add your styles here */
    </style>
</head>
<body>
    <button class="open-a" onclick="sendRequest('openA')">Open A</button>
    <button class="auto" onclick="sendRequest('auto')">Auto</button>
    <button class="open-b" onclick="sendRequest('openB')">Open B</button>
    <button class="stop" onclick="sendRequest('stop')">STOP</button>

    <script>
        function sendRequest(action) {
            fetch('/action', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ action: action })
            })
            .then(response => response.text())
            .then(data => console.log(data))
            .catch(error => console.error('Error:', error));
        }
    </script>
</body>
</html>
)rawliteral";

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
    server.on("/angle", HTTP_GET, handleAngle);
    server.on("/status", HTTP_GET, handleStatus);
    server.onNotFound(handleFileRequest);  // Serve static files from SPIFFS

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

void LEDTest()
{
#ifdef USE_LEDS
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
#endif
}

void loop() {
  server.handleClient();  // Handle incoming requests

  unsigned long newTime = millis();
  if(newTime > oldTime + 2000)
  {
    oldTime = newTime;
    angle = getInputRegister(1);
    if(modbus_error == 0)
    {
      Serial.print("Modbus Success Angle = ");
      Serial.println(angle);
      status = STATUS_OK;
    }  
    else
    {
      status = STATUS_MODBUS_ERROR;
    }

    if(delay_count == 0)
      LEDTest();
    delay_count++;
    if(delay_count >= 30)
      delay_count = 0;
  }

#ifdef USE_WATCHDOG
   // Reset the watchdog timer
    rtc_wdt_feed();
    #endif
}
