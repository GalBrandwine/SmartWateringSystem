// include library to read and write from flash memory
#include <EEPROM.h>

/* Define the number of bytes you want to access
 * 1 byte for 'runtime', the other for 'frequency'.
 * That means that maximum 'runtime'/'frequency' value will be 255, moreover - I can operate the valve for hole minutes, e.g 0.5 minute wont work - I cant store that in a BYTE
*/
#define EEPROM_SIZE 3


#include <ArduinoOTA.h>

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>

/* These are plugins that needed to be 'installed' in the /path/to/your/Arduino/libraries/
 * When I say 'installed': just download the backage zip from theirs GITHUB, and unsip them 'libraries' dict.
 * Then, restart your Arduino IDE.
 * ESPAsyncWebServer    - https://github.com/me-no-dev/ESPAsyncWebServer
 * SPIFF                - https://randomnerdtutorials.com/install-esp32-filesystem-uploader-arduino-ide/
*/
#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"

#define VALVE_SWITCH_PIN 18
#define MOTOR_PIN 19


// Set these to your desired credentials.
const char *ssid      = "AtzizHaham";
const char *password  = "AtzizHaham";

AsyncWebServer server(80);


SemaphoreHandle_t xMutex;
/* ****************************************************************************
 * ********************************** Some globals ****************************
 * ****************************************************************************
*/
enum VALVE_MODES{
  timer,
  valve_open,
  valve_close
};
VALVE_MODES valveMode = timer;

bool isSystemWatering = false;

#define RUNTIME_OFF_VALUE 255
#define RUNTIME_ON_VALUE 254
int runtime = RUNTIME_OFF_VALUE;

#define FREQUENCY_RESET_VALUE 0
int frequency_hours = FREQUENCY_RESET_VALUE;
int time_left_until_next_watering_min = frequency_hours*60;


//void runtimeTask(void * parameter){
//  int runtime_temp = *((int*)parameter);
//
//  isSystemWatering = true;
//  openForXMinute(runtime_temp);
//  isSystemWatering = false;
//
//  // When you're done, call vTaskDelete. Don't forget this!
//  vTaskDelete(NULL);
//}


void frequencyTask( void* pvParameters )
{
  BaseType_t xReturned;
  // This TaskHandle will allow
  TaskHandle_t xHandle = NULL;
  int seconds = 0;
  int valveTime = 0;
  int hours_passed_from_last_watering = 0; //max watering interval is 168 hours (a week)

  for (;;)
  {
    vTaskDelay( pdMS_TO_TICKS( 1000 ) );
    seconds++;

    if (seconds % 3600 == 0){//10
      hours_passed_from_last_watering++;
    }

//    Serial.print("frequencyTask::isSystemWatering: ");Serial.println(isSystemWatering);
//    Serial.print("frequencyTask::FREQUENCY_RESET_VALUE: ");Serial.println(FREQUENCY_RESET_VALUE);
//    Serial.print("frequencyTask::frequency_hours: ");Serial.println(frequency_hours);
//    Serial.print("frequencyTask::frequency_hours in seconds: ");Serial.println(frequency_hours*3600);
//    Serial.print("frequencyTask::seconds passed from last watering: ");Serial.println(seconds);
//    Serial.print("frequencyTask::hours_passed_from_last_watering: ");Serial.println(hours_passed_from_last_watering);
//    Serial.print("frequencyTask::valveTime: ");Serial.println(valveTime);
//    Serial.println("");



    if (frequency_hours != FREQUENCY_RESET_VALUE){
        seconds                           = seconds % (frequency_hours*3600);       // Frequency_hours in seconds. (If frequency_hours==0) then it throws DivisionByZero!!)
        time_left_until_next_watering_min = frequency_hours*60 - seconds/60;        // hours_passed_from_last_watering;
    }
    else {
        seconds                         = 0;
        valveTime                       = seconds;
        hours_passed_from_last_watering = 0;

        Serial.println("frequencyTask::hours_passed_from_last_watering RESETED ");
    }

    xSemaphoreTake( xMutex, portMAX_DELAY ); //start critical section
    switch (valveMode) {
      case VALVE_MODES::timer:
        Serial.print(seconds - valveTime);Serial.print(" ");Serial.println(runtime*60);
        if (seconds - valveTime>= runtime*60){
          if (closeValve()){
            isSystemWatering    = false;
            valveTime = 0;
          }
        }
        if (hours_passed_from_last_watering >= frequency_hours && frequency_hours != FREQUENCY_RESET_VALUE){ // Time to water the plants.
          Serial.println("frequencyTask::TIME TO WATER THE PLANTS");
          if (!isSystemWatering){
            if (openValve()){
              isSystemWatering    = true;
              hours_passed_from_last_watering = 0;
              valveTime = seconds;
            }
          }
        }
        break;
      case VALVE_MODES::valve_open:
        if (openValve()){
          isSystemWatering    = true;
          valveTime = seconds;
        }
        break;
      case VALVE_MODES::valve_close:
        if (closeValve()){
          isSystemWatering    = false;
        }
        break;
      default:
        // statements
        break;
    }
    xSemaphoreGive( xMutex ); // end start critical section
  }

  vTaskDelete( NULL );
}


bool closeValve() {
  Serial.println("closing");
  Serial.println(digitalRead(VALVE_SWITCH_PIN));
  while (digitalRead(VALVE_SWITCH_PIN) == 1) {
    digitalWrite(MOTOR_PIN, LOW); // OPEN the MOSFET
    delay(100);
  }

  digitalWrite(MOTOR_PIN, HIGH);  // CLOSE the MOSFET
  Serial.println("Valve closed");
  return true;
}


bool openValve() {
  Serial.println("opening");
  Serial.println(digitalRead(VALVE_SWITCH_PIN));
  while (digitalRead(VALVE_SWITCH_PIN) == 0) {
    digitalWrite(MOTOR_PIN, LOW);
    delay(100);
  }

  Serial.println("Valve opened");
  digitalWrite(MOTOR_PIN, HIGH);
  return true;
}


void openForXMinute(int minutes) {

  const TickType_t xDelay = minutes*60000 / portTICK_PERIOD_MS; // X minutes
  Serial.print("openForXMinute::Opening valve for "); Serial.print(minutes); Serial.println(" minutes.");

  openValve();
  vTaskDelay( xDelay);
//  for ( uint32_t tStart = millis();  (millis() - tStart) < period;  ) {} // Simple buisy-wait loop. using milis() instead of delay() wont halt the arduino. watch dog dont like this one.
  closeValve();
}


void processRuntimeRequest(const AsyncWebParameter* p){
    xSemaphoreTake( xMutex, portMAX_DELAY );
    Serial.println();
    Serial.println("processRuntimeRequest::GOT runtine");
    Serial.println(p->name());
    Serial.println(p->value());

    // Check If is is diffirent from previus runtime stored in memory:
    if (p->value().equals("off")) {
      valveMode = VALVE_MODES::valve_close;
      EEPROM.write(2, valveMode);

      runtime = RUNTIME_OFF_VALUE;
      EEPROM.write(0, RUNTIME_OFF_VALUE);
      EEPROM.commit();
    }
    else if (p->value().equals("on")) {
      valveMode = VALVE_MODES::valve_open;
      EEPROM.write(2, valveMode);

      runtime = RUNTIME_ON_VALUE;
      EEPROM.write(0, RUNTIME_ON_VALUE);
      EEPROM.commit();
    }
    else if (p->value().toInt() != runtime){
      valveMode = VALVE_MODES::timer;
      EEPROM.write(2, valveMode);

      runtime = p->value().toInt();
      EEPROM.write(0, p->value().toInt());
      EEPROM.commit();
    }

    Serial.println("Runtime saved in flash memory");
    xSemaphoreGive( xMutex );
}


void processFrequencyRequest(const AsyncWebParameter* p){
    xSemaphoreTake( xMutex, portMAX_DELAY );
    Serial.println();
    Serial.println("processFrequencyRequest::GOT frequency");
    Serial.println(p->name());
    Serial.println(p->value());

    if (p->value().equals("reset")) {
      EEPROM.write(1, FREQUENCY_RESET_VALUE);
      EEPROM.commit();
      Serial.println("processFrequencyRequest::Frequency RESETED");
    }

    // Check If is is different from previous runtime stored in memory:
    if (p->value().toInt() != frequency_hours){
      frequency_hours = p->value().toInt();
      EEPROM.write(1, p->value().toInt());
      EEPROM.commit();
      Serial.println("processFrequencyRequest::Frequency saved in flash memory");
    }
    xSemaphoreGive( xMutex );
}


void init(){
  initPints();
  initEEPROM();

  initValvePosition();
  initAccessPoint();
}


void initPints(){
  Serial.println("initPints::Initiating pins");
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(VALVE_SWITCH_PIN, INPUT);
}


void initEEPROM(){
//  writeToEeeprom();

  // initialize EEPROM with predefined size
  Serial.println();
  Serial.println("initEEPROM::Configuring EEPROM...");
  EEPROM.begin(EEPROM_SIZE);

  // Restore runtime and frequency from memory.
  runtime = EEPROM.read(0);
  frequency_hours = EEPROM.read(1);
  valveMode = (VALVE_MODES)EEPROM.read(2);
  Serial.println("initEEPROM::Restoring from memory:");
  Serial.print("initEEPROM::Runtime:");Serial.println(runtime);
  Serial.print("initEEPROM::Frequency:");Serial.println(frequency_hours);
  Serial.print("initEEPROM::valveMode:");Serial.println(valveMode);
}


void writeToEeeprom(){
  // Debug mode.

  Serial.println("writeToEeeprom::Debug mode");
  EEPROM.write(0, RUNTIME_OFF_VALUE);
  EEPROM.commit();


  EEPROM.write(1, FREQUENCY_RESET_VALUE);
  EEPROM.commit();

  EEPROM.write(2, VALVE_MODES::timer);
  EEPROM.commit();

}

bool initValvePosition(){
  // Resetting valve position
  Serial.println();
  Serial.println("initValvePosition::Reseting valve position...");

//  openValve();
//  closeValve();

  // When you're done, call vTaskDelete. Don't forget this!
  // vTaskDelete(NULL);
  return openValve() && closeValve();
}


void initAccessPoint(){
  Serial.println();
  Serial.println("Configuring access point...");
  // You can remove the password parameter if you want the AP to be open.
  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());
}


bool setUpServer(){
  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/index.html", String(), false);
  });

  // Route to load style.css file
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/style.css", "text/css");
  });

  // This is how the client pass information back to the server.
  server.on("/runtime", HTTP_GET, [](AsyncWebServerRequest * request) {
    // Take 'runtime' parameter from the request object.
    AsyncWebParameter* p = request->getParam(0);
    processRuntimeRequest(p);

    // Send back to client "OK" message, and the desired value (as a confirmation)
    if(runtime == RUNTIME_OFF_VALUE){
      request->send_P(200, "text/plain", "off");
    }
    else{
      request->send_P(200, "text/plain", String(runtime).c_str());
    }

  });

  server.on("/frequency", HTTP_GET, [](AsyncWebServerRequest * request) {
    // Take 'runtime' parameter from the request object.
    AsyncWebParameter* p = request->getParam(0);
    processFrequencyRequest(p);
    request->send_P(200, "text/plain", String(frequency_hours).c_str());
  });

  server.on("/isSystemWatering", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(isSystemWatering).c_str());
  });

  server.on("/getFrequency", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(frequency_hours).c_str());
  });

  server.on("/getRunTime", HTTP_GET, [](AsyncWebServerRequest *request){
    if (runtime == RUNTIME_ON_VALUE){
      request->send_P(200, "text/plain", "on");
    }
    else if (runtime == RUNTIME_OFF_VALUE){
      request->send_P(200, "text/plain", "off");
    }
    else{
      request->send_P(200, "text/plain", String(runtime).c_str());
    }

  });

  server.on("/getTimeLeft", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(time_left_until_next_watering_min).c_str());
  });

  // Start server
  server.begin();

  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return false;
  }

  Serial.println("Server started");
  return true;
}


void setup() {
  Serial.begin(115200);
  xMutex = xSemaphoreCreateMutex();
  init();

  if (!setUpServer()){
    Serial.println("An Error has occurred while setUpServer(). STOPPING.");
    return;
  }

   xTaskCreate(
    frequencyTask,                    // Function that should be called
    "Initiating frequencyTask",       // Name of the task (for debugging)
    1000,                             // Stack size (bytes)
    NULL,            // Parameter to pass
    1,               // Task priority
    NULL             // Task handle
  );
  // ****************************** OTA ************************
  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  })
  .onEnd([]() {
    Serial.println("\nEnd");
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();
}


void loop() {
  ArduinoOTA.handle();
//  Serial.println(digitalRead(VALVE_SWITCH_PIN));
}