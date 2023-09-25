#include <ArduinoJson.h>
#include <EEPROM.h>
#include "painlessMesh.h"
#include <SoftwareSerial.h>
#include <Wire.h>
#include "OLED.h"


// OLED display definitions
#define OLED_SDA D2
#define OLED_SCL D1
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define SSD1306_I2C_ADDRESS 0x3c  // 128x64 OLED Display I2C address

#define eMESH_PREFIX "whatevetytytrYouLike"
#define eMESH_PASSWORD "somethtytytingSneaky"
#define eMESH_PORT 5555
#define MESH_PORT 5555

#define EEPROM_MAX_SIZE 512

/* Indexes of eeprom */
#define QSID_START 0 
#define QSID_END 32
#define QPASS_START 32
#define QPASS_END 64
#define MQTTBROKER_START 64

#define DEBUG_ENABLE //uncomment to disable debugging

#ifdef DEBUG_ENABLE
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

enum NetworkState {
  DISCONNECTED,
  CONNECTED
};
enum State
{
  IDLE,
  RECEIVING,
  COMPLETE
};

NetworkState meshState = DISCONNECTED;
State currentState = IDLE;
SoftwareSerial espSerial(D6, D5);
Scheduler userScheduler; 
painlessMesh mesh;
OLED display(D2,D1);

int skip = 0;
int eeindex = 0;
int configSuccess = 0;
const int BUFFER_SIZE = 300;   // Maximum number of characters that can be received
char inputBuffer[BUFFER_SIZE]; // Buffer to hold incoming data
int bufferIndex = 0;           // Index of next available position in the buffer


void sendMessage(); 

Task taskSendMessage(TASK_SECOND * 1, TASK_FOREVER, &sendMessage);

void sendMessage()
{
  String msg = "{\"nodename\":\"1st floor Temp\",\"value\":25.5,\"unit\":\"Celsius\"}";
  mesh.sendBroadcast(msg);
  taskSendMessage.setInterval(random(TASK_SECOND * 1, TASK_SECOND * 5));
}


void updateOLED(String msg, int x=0 , int y=0){

  display.clear();
  display.print(msg.c_str(),x,y);
}

void receivedCallback(uint32_t from, String &msg)
{
  DEBUG_PRINT("Received from ");
  DEBUG_PRINT(from);
  DEBUG_PRINT(": ");
  DEBUG_PRINTLN(msg.c_str());  

  //send feedback if requested.
  DynamicJsonDocument doc(1024);
  deserializeJson(doc, msg);
  if (doc["pw_type"] == "batt")
  {
    mesh.sendSingle(from, "rcvd");
  }
  //transfer data to gateway serially.
  espSerial.print(msg.c_str());

}

void newConnectionCallback(uint32_t nodeId)
{
  DEBUG_PRINT("--> New Connection, nodeId = ");
  DEBUG_PRINTLN(nodeId);
}

void changedConnectionCallback()
{
  DEBUG_PRINTLN("Changed connections");
}

void nodeTimeAdjustedCallback(int32_t offset)
{
  DEBUG_PRINT("Adjusted time ");
  DEBUG_PRINT(mesh.getNodeTime());
  DEBUG_PRINT(". Offset = ");
  DEBUG_PRINTLN(offset);
}

/**
 * @brief writes string to eeprom but doesn't commit.
 * 
 * @param start start index for eeprom write
 * @param data  const ref to String which is to be written
 */
void writeStringToEEPROM(int start, const String &data) {
  int len = data.length();
  for (int i = 0; i < len; ++i) {
    if (EEPROM.read(start + i) != data[i]) { // Only write if data has changed
      EEPROM.write(start + i, data[i]);
    }
  }
  EEPROM.write(start + len, '\0'); // Null-terminate the string
}

// Function to read from EEPROM
String readFromEEPROM(int start, int end) {
  //check for overflow in reading 
  if(end >= EEPROM_MAX_SIZE)
    end = EEPROM_MAX_SIZE-1;

  String result;
  result.reserve(end - start);
  for (int i = start; i < end; ++i) {
    char c = char(EEPROM.read(i));
    if (c == '\0') {
      eeindex = ++i; // eeindex will store the next char after terminator
      break;
    }
    result += c;
  }
  return result;
}


void readConfig()
{
  // Read qsid from EEPROM
  String qsid = readFromEEPROM(QSID_START,QSID_END);
  
  // Read qpass from EEPROM
  String qpass = readFromEEPROM(QPASS_START,QPASS_END);

  // Read mqttbroker from EEPROM EEPROM_MAX_SIZE indicates
  // read until null terminator is found.
  eeindex = MQTTBROKER_START;
  String mqttbroker = readFromEEPROM(eeindex,EEPROM_MAX_SIZE);

  // Read mqttpass from EEPROM
  String mqttpass = readFromEEPROM(eeindex,EEPROM_MAX_SIZE);

  // Read qos from EEPROM
  String qos = readFromEEPROM(eeindex,EEPROM_MAX_SIZE);

  // Read meshid from EEPROM
  String meshid = readFromEEPROM(eeindex,EEPROM_MAX_SIZE);

  // Read meshpass from EEPROM
  String meshpass = readFromEEPROM(eeindex,EEPROM_MAX_SIZE);

  //Print the retrieved data
  DEBUG_PRINTLN("SSID: " + qsid);
  DEBUG_PRINTLN("Password: " + qpass);
  DEBUG_PRINTLN("MQTT Broker: " + mqttbroker);
  DEBUG_PRINTLN("MQTT Password: " + mqttpass);
  DEBUG_PRINTLN("QoS: " + qos);
  DEBUG_PRINTLN("Mesh ID: " + meshid);
  DEBUG_PRINTLN("Mesh Password: " + meshpass);

  //NOTE: can't we print as done above, why serialize after
  //creating json doc.
  // StaticJsonDocument<256> jsonDocument;
  // jsonDocument["SSID"] = qsid;
  // jsonDocument["Password"] = qpass;
  // jsonDocument["MQTT Broker"] = mqttbroker;
  // jsonDocument["MQTT Password"] = mqttpass;
  // jsonDocument["QoS"] = qos;
  // jsonDocument["Mesh ID"] = meshid;
  // jsonDocument["Mesh Password"] = meshpass;

  // String jsonString;
  // //Serialize the JSON document to a string
  // serializeJson(jsonDocument, jsonString);
  // //Print the JSON string
  // DEBUG_PRINTLN(jsonString);

  configSuccess = 1;

  Serial.println("new mesh connection done");
  meshState = CONNECTED;


  mesh.setDebugMsgTypes(ERROR | STARTUP); // set before init() so that you can see startup messages
  mesh.init(meshid, meshpass, &userScheduler, eMESH_PORT);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
  userScheduler.addTask(taskSendMessage);
  taskSendMessage.enable();
  updateOLED(String(mesh.getNodeId()));
}

void setup()
{
  #ifdef DEBUG_ENABLE
    Serial.begin(9600);
  #endif
  display.begin();
  updateOLED("NeilSoft");
  //readConfig();
  delay(1500);
  EEPROM.begin(EEPROM_MAX_SIZE); // Initializing EEPROM
  espSerial.begin(9600);
  // mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); // all types on
}

void loop()
{

  // Serial.println("main loop");
  if (configSuccess == 1)
  {
    mesh.update();
    meshState = CONNECTED;
    skip = 0;
  }
  else
  {
    Serial.println("send all data via serially, not able to establish communication with mesh");
    StaticJsonDocument<200> doc;
    doc["error"] = "send_mesh_data";
    updateOLED("Mesh Reset");
    String jsonData;
    serializeJson(doc, jsonData);
    espSerial.print(jsonData);
    meshState = DISCONNECTED;
    delay(100); 
  }
  //based on above if-else meshState is decided, and then processed below
  if (meshState == DISCONNECTED)
  {
    while (Serial.available())
    {
      char c = Serial.read();
      DEBUG_PRINT(c);
      if (skip == 2)
      {
        switch (currentState)
        {
        case IDLE:

          if (c == '{')
          {
            inputBuffer[bufferIndex] = c;
            bufferIndex++;
            currentState = RECEIVING;
          }
          break;
        case RECEIVING:
          inputBuffer[bufferIndex] = c;
          bufferIndex++;

          if (c == '}')
          {
            currentState = COMPLETE;
          }

          if (bufferIndex >= BUFFER_SIZE) //NOTE: earlier it was buffer_size - 1 ,which must be wrong
          {
            // Buffer full, discard remaining characters
            bufferIndex = BUFFER_SIZE - 1;
            currentState = IDLE;
          }
          break;

        case COMPLETE:
          // Ignore any additional characters until the buffer is reset
          break;
        }
      }
      //NOTE:
      //following code suggests that we are getting '}' twice
      // and both times it should not be processed and eeprom 
      // is to be cleared. why?
      if (c == '}')
      {
        for (int i = 0; i < EEPROM_MAX_SIZE; ++i)
        {
          EEPROM.write(i, 0);
        }
        EEPROM.commit();
        skip++;
      }
    }
  }

  if (currentState == COMPLETE)
  {
    inputBuffer[bufferIndex] = '\0'; // Terminate the string

    // Parse the JSON string
    DynamicJsonDocument doc(BUFFER_SIZE);
    DeserializationError error = deserializeJson(doc, inputBuffer);

    if (error)
    {
      Serial.print("JSON parsing error: ");
      Serial.println(error.c_str());
    }
    else
    {
      // Read values from JSON and store them in variables
      String qsid = doc["SSID"];
      String qpass = doc["Password"];
      String mqttbroker = doc["MQTT Broker"];
      String mqttpass = doc["MQTT Password"];
      String qos = doc["QoS"];
      String meshid = doc["Mesh ID"];
      String meshpass = doc["Mesh Password"];

      if (qsid.length() > 0 && qpass.length() > 0)
      {
        //DEBUG_PRINTLN("clearing eeprom");

        // // Clear EEPROM
        // for (u32 i = 0; i < EEPROM.length(); ++i)
        // {
        //   EEPROM.write(i, 0);
        // }

        // Write qsid to EEPROM
        writeStringToEEPROM(QSID_START,qsid);

        // Write qpass to EEPROM
        writeStringToEEPROM(QPASS_START,qpass);

        // Write mqttbroker to EEPROM
        eeindex = MQTTBROKER_START;
        writeStringToEEPROM(MQTTBROKER_START,mqttbroker);

        // Write mqttpass to EEPROM
        eeindex += mqttbroker.length() + 1;
        writeStringToEEPROM(eeindex,mqttpass);

        // Write qos to EEPROM
        eeindex += mqttpass.length() + 1;
        writeStringToEEPROM(eeindex,qos);

        // Write meshid to EEPROM
        eeindex += qos.length() + 1;
        writeStringToEEPROM(eeindex,meshid);

        // Write meshpass to EEPROM
        eeindex += meshid.length() + 1;
        writeStringToEEPROM(eeindex,meshpass);

        // Commit changes to EEPROM
        EEPROM.commit();
        Serial.println("Data written to EEPROM");
      }
      else
      {
        Serial.println("Sending 404");
      }
      readConfig();
      // Reset flags and buffer for the next string
      currentState = IDLE;
      bufferIndex = 0;
      memset(inputBuffer, 0, BUFFER_SIZE);
    }
  }
}
