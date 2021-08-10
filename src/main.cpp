/**
 *  Birth Alert Router application 
 * 
 *  @author Gilberto Araujo
 * 
 *  References: 
 *    - Cloud Publishing: Rui Santos at https://RandomNerdTutorials.com/esp32-sim800l-publish-data-to-cloud/
 *    - Data Hosting:  https://nothans.com/thingspeak-tutorials/arduino/send-data-to-thingspeak-with-arduino
 */


#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoHttpClient.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_system.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <queue>
#include <sys/time.h>
#include <time.h>
#include "router.h"

using namespace std;

/* If defined, allows terminal debug info */
#define DEBUG
#define DEBUG_REQUEST
// #define DEBUG_EXAMPLE
// #define PUBLISH_RANDOM_DATA

#define AT_CMD_TIMEOUT  1200
#define AT_TIME_LEN     17
#define AT_TIME_BEGIN   10
#define AT_TIME_END     AT_TIME_BEGIN + AT_TIME_LEN

#define QUEUE_MAX_LEN   512

/* GPIO pin to blink (blue LED on LILYGO T-Call SIM800L board) */
#define BLINK_GPIO GPIO_NUM_13

/* Global queues to store sensor samples */
queue<thigh_sensor_data_t> thighSensorQueue;
queue<vulva_sensor_data_t> vulvaSensorQueue;
queue<hygrometer_sensor_data_t> hygroSensorQueue;

/* Mutex to protect the shared queues access */
SemaphoreHandle_t SensorQueueMutex;

/**
 *  BLE definitions
 */

// The remote service we wish to connect to.
static BLEUUID sensorUUID("befab990-ddb3-11eb-ba80-0242ac130004");
static BLEUUID testUUID("c1462ae7-9493-4018-b6d3-dda50387989c");

int scanTime = 1; //In seconds
BLEScan *pBLEScan;

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
#ifdef DEBUG_EXAMPLE
    Serial.printf ("Advertised Device: %s \n", advertisedDevice.toString().c_str());
#endif
  }
};

/**
 *  SIM800L definitions
 */

// Your GPRS credentials
const char apn[] = "zap.vivo.com.br"; // APN
const char gprsUser[] = "vivo";       // GPRS User
const char gprsPass[] = "vivo";       // GPRS Password
const char simPIN[] = "";             // SIM card PIN (leave empty, if not defined)

// Server details
const char server[] = "birthalert.angoeratech.com.br";
const char resource[] = "/api/setSensorCoxa";
const char endpointThighSensor[] = "/api/setSensorCoxa";
const char endpointVulvaSensor[] = "/api/setSensorVulva";
const char endpointHygroSensor[] = "/api/setSensorUmidadeTemperatura";
const char endpointKeepAlive[] = "/api/setKeepAliveRoteador";
const char apiKey[] = "117f08a0a9c5808e93a4c246ec0f2dab";
const int  port = 80;

// NTP server info for time synch
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -3 * 3600;  // Brazilian time offset (GMT-3)
const int   daylightOffset_sec = 3600;

struct tm *timeinfo;
time_t now;

// TTGO T-Call pins
#define MODEM_RST 5
#define MODEM_PWKEY 4
#define MODEM_POWER_ON 23
#define MODEM_TX 27
#define MODEM_RX 26
#define I2C_SDA 21
#define I2C_SCL 22

// Set serial for debug console (to Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands (to SIM800 module)
#define SerialAT Serial1

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800   // Modem is SIM800
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb

// Define the serial console for debug prints, if needed
//#define DUMP_AT_COMMANDS

#include <Wire.h>
#include <TinyGsmClient.h>

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

// I2C for SIM800 (to keep it running when powered from battery)
TwoWire I2CPower = TwoWire(0);

// TinyGSM Client for Internet connection
TinyGsmClient client (modem);

#define IP5306_ADDR 0x75
#define IP5306_REG_SYS_CTL0 0x00

/* Prototypes */
void Sensor_Task(void *pvParameters);
void Cloud_Task(void *pvParameters);
void UI_Task(void *pvParameters);


bool setPowerBoostKeepOn(int en)
{
  I2CPower.beginTransmission(IP5306_ADDR);
  I2CPower.write(IP5306_REG_SYS_CTL0);
  if (en)
  {
    I2CPower.write(0x37); // Set bit1: 1 enable 0 disable boost keep on
  }
  else
  {
    I2CPower.write(0x35); // 0x37 is default reg value
  }
  return I2CPower.endTransmission() == 0;
}

// Function that gets current epoch time
unsigned long getTime()
{
  // time_t now;
  // struct tm timeinfo;

  time(&now);
  timeinfo = localtime(&now);

  return now;
}

// unsigned long getTime()
// {
//   time_t now;
//   struct tm timeinfo;
//   if (!getLocalTime(&timeinfo)) {
//     //Serial.println("Failed to obtain time");
//     return(0);
//   }
//   time(&now);
//   return now;
// }

/**
 *  @brief   Get network time 
 *  @details Get date and time from GSM module (via AT commands).
 *  
 *  @return  Date and time string (YY/MM/DD,hh:mm:ss).
 *
 *  @note Don't call this from tasks (threads).
 */
String getTimeFromGSM ()
{
  String response;
  
  /* Send AT command (Clock)*/
  SerialAT.println ("AT+CCLK?");
  delay(150);

  /* Wait for response */
  for (uint16_t i = 0; i < AT_CMD_TIMEOUT; i++)
  {
    delay(25);
    if (SerialAT.available())
    {
      delay(50);
      response = SerialAT.readString();
      
      /* Get clean date and time string (+CCLK: "21/07/30,14:17:39-12") */
      return response.substring (AT_TIME_BEGIN, AT_TIME_END);
    }
  }
  return "FAIL";  
}

/**
 *  @brief    Set local time from GSM network
 *  @details  Set system clock time and timezone.
 * 
 *  @note No daylight saving (dst) is used. 
 */
void setLocalTime (String dateTime)
{
  tm timeStruct;
  time_t time;
  timeval timeVal = {0, 0};       // {sec, usec}

  timeStruct.tm_year = dateTime.substring(0, 2).toInt() + 100;  // Fixes year value (well known adjustment)
  timeStruct.tm_mon  = dateTime.substring(3, 5).toInt() - 1;    // Fixes month value (well known adjustment)
  timeStruct.tm_mday = dateTime.substring(6, 8).toInt();
  timeStruct.tm_hour = dateTime.substring(9, 11).toInt() + 3;   // Remove GMT-3 offset to transform in UTC time (required for 'settimeofday()')
  timeStruct.tm_min  = dateTime.substring(12, 14).toInt();
  timeStruct.tm_sec  = dateTime.substring(15, 17).toInt();

  // Serial.println (asctime(&timeStruct));

  /* Get UNIX time */
  time = mktime (&timeStruct);
  timeVal.tv_sec = time;

  /* Set system time */
  settimeofday (&timeVal, NULL);
}

void setup()
{

  // initialize serial communication at 115200(?) bits per second:
  Serial.begin(115200);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }
  Serial.println("Scanning...");

  /* Create the queue mutex */
  SensorQueueMutex = xSemaphoreCreateMutex();

  /** 
   * BLE environment setup
   */

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(false); //active scan uses more power, but get results faster
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99); // less or equal setInterval value

  /**
   *  SIM800L environment setup
   */

  // Set serial monitor debugging window baud rate to 115200
  SerialMon.begin(115200);

  // Start I2C communication
  I2CPower.begin(I2C_SDA, I2C_SCL, 400000);

  // Keep power when running from battery
  bool isOk = setPowerBoostKeepOn(1);
  SerialMon.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));

  // Set modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);

  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

  // Restart SIM800 module, it takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();
  // use modem.init() if you don't need the complete restart

  // Unlock your SIM card with a PIN if needed
  if (strlen(simPIN) && modem.getSimStatus() != 3)
  {
    modem.simUnlock(simPIN);
  }

  /* Connect GSM Modem to APN for time synchronization */
  SerialMon.print ("Connecting to Time Server: ");
  SerialMon.print (apn);
  if (!modem.gprsConnect (apn, gprsUser, gprsPass))
  {
    SerialMon.println (" fail");
  }
  else
  {
    /* APN connected */
    SerialMon.println (" OK");

    /* Set system time from network */
    String time = getTimeFromGSM ();
    setLocalTime (time);

    /* Disconnect from Time Server */
    modem.gprsDisconnect();
    SerialMon.println (F("Time Server disconnected"));
  }

  /* RTOS tasks creation to run independently. */

  xTaskCreatePinnedToCore (Sensor_Task, "Sensor Task" // A name just for humans
                          ,8192 // This stack size can be checked & adjusted by reading the Stack Highwater
                          ,NULL //Parameters for the task
                          ,2    // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
                          ,NULL
                          ,0);  //Task Handle

  xTaskCreatePinnedToCore (Cloud_Task, "Cloud Task" // A name just for humans
                          ,8192 // Stack size
                          ,NULL //Parameters for the task
                          ,1    // Priority
                          ,NULL
                          ,1);  //Task Handle

  xTaskCreatePinnedToCore (UI_Task, "UI Task" // A name just for humans
                          ,2048 // Stack size
                          ,NULL //Parameters for the task
                          ,1    // Priority
                          ,NULL
                          ,0);  //Task Handle

  // Now the Task scheduler, which takes over control of scheduling individual Tasks, is automatically started.
}

/**
 *  @brief    Task to handle sensors protocol
 *  @details  Catches sensors advertising data from devices which matches to a specific UUID.
 *            
 *            Sensors protocol data format:
 * 
 *                    **************************************************************************
 *            Thigh:  * Manuf.Code | Sensor Type | Battery | Temperature | Activity | Position *
 *                    *   0x5555   |     0x01    |   0xXX  |    0xXXXX   |  0xXXXX  |   0xXX   *
 *                    **************************************************************************
 *                    ***************************************************************
 *            Vulva:  * Manuf.Code | Sensor Type | Battery |   Dilation  |    Gap   *
 *                    *   0x5555   |     0x02    |   0xXX  |    0xXXXX   |  0xXXXX  *
 *                    ***************************************************************
 *                    ****************************************************************
 *            Hygro:  * Manuf.Code | Sensor Type | Battery |  Humidity | Temperature *
 *                    *   0x5555   |     0x03    |   0xXX  |   0xXXXX  |   0xXXXX    *
 *                    ****************************************************************
 * 
 *  @note   Long data fileds (with two bytes) are stored as 'MSB first' endianess.
 *   
 *  @param [in] pvParameters  Not used.
 */
void Sensor_Task(void *pvParameters __attribute__((unused))) // This is a Task.
{
  thigh_sensor_data_t thighSensor;
  vulva_sensor_data_t vulvaSensor;
  hygrometer_sensor_data_t hygroSensor;
  int type = -1;
  int len = 0;
  char data[32];

  while (1)
  {
    /* Get all available devices */
    BLEScanResults foundDevices = pBLEScan->start(scanTime, false);

    int devicesCount = foundDevices.getCount();
    
#ifdef DEBUG_EXAMPLE
    Serial.print("Devices found: ");
    Serial.println(devicesCount);
    Serial.println("Scan done!");
#endif
    for (int i = 0; i < devicesCount; i++)
    {
      /* Check for known sensors by UUID */
      if (foundDevices.getDevice(i).haveServiceUUID() &&
          foundDevices.getDevice(i).isAdvertisingService(sensorUUID))
      {
#ifdef DEBUG
        Serial.printf("\n");
        Serial.printf("Sensor UUID: %s\n", foundDevices.getDevice(i).getServiceUUID().toString().c_str());
#endif
        /* Check for sensor parameters */
        if (foundDevices.getDevice(i).haveManufacturerData())
        {
          len = foundDevices.getDevice(i).getManufacturerData().length();

          /* Get sensor data */
          memcpy(data, foundDevices.getDevice(i).getManufacturerData().c_str(), len);

#ifdef DEBUG
          /* Debug output */
          Serial.printf("Sensor Dlen: %d\n", len);
          Serial.printf("Sensor Data: ");

          for (int i = 0; i < len; i++)
          {
            Serial.printf("%02x", (int)data[i]);
          }
          Serial.printf("\n");
#endif
          /* get sensor type */
          type = data[SENSOR_TYPE_POS];
#ifdef DEBUG
          Serial.printf("Sensor type: %d\n", type);
#endif
          switch (type)
          {
          case THIGH_SENSOR_TYPE:
            thighSensor.header.name = foundDevices.getDevice(i).getName();
            thighSensor.header.addr = foundDevices.getDevice(i).getAddress().toString();
            thighSensor.header.time = getTime();
            thighSensor.battery = data[THIGH_BATTERY_POS];
            thighSensor.activity = ((data[THIGH_ACTIVITY_POS]) << 8) + data[THIGH_ACTIVITY_POS + 1];
            thighSensor.temperature = ((data[THIGH_TEMPERATURE_POS]) << 8) + data[THIGH_TEMPERATURE_POS + 1];
            thighSensor.position = data[THIGH_POSITION_POS];

            /* Enter critical session to access the queue */
            xSemaphoreTake (SensorQueueMutex, portMAX_DELAY);

            /* Stores sensor sample on queue */
            thighSensorQueue.push (thighSensor);
#ifdef DEBUG
            /* Print latest sample values */
            Serial.printf("Sensor Name: %s\n", thighSensorQueue.back().header.name.c_str());
            Serial.printf("Sensor Addr: %s\n", thighSensorQueue.back().header.addr.c_str());
            Serial.printf("Sensor Time: %d\n", (int) thighSensorQueue.back().header.time);
            Serial.printf("Sensor Batt: %d\n", thighSensorQueue.back().battery);
            Serial.printf("Sensor Act.: %d\n", thighSensorQueue.back().activity);
            Serial.printf("Sensor Temp: %d\n", thighSensorQueue.back().temperature);
            Serial.printf("Sensor Pos : %d\n", thighSensorQueue.back().position);
            Serial.printf("Queue size: %d\n", thighSensorQueue.size());            
#endif
            /* Limits the queue size */
            if (thighSensorQueue.size() > QUEUE_MAX_LEN)
            {
              thighSensorQueue.pop();
            }
            /* Exit critical session */
            xSemaphoreGive (SensorQueueMutex);

            break;

          case VULVA_SENSOR_TYPE:
            vulvaSensor.header.name = foundDevices.getDevice(i).getName();
            vulvaSensor.header.addr = foundDevices.getDevice(i).getAddress().toString();
            vulvaSensor.header.time = getTime();
            vulvaSensor.battery = data[VULVA_BATTERY_POS];
            vulvaSensor.dilation = ((data[VULVA_DILATION_POS]) << 8) + data[VULVA_DILATION_POS + 1];
            vulvaSensor.gap = ((data[VULVA_GAP_POS]) << 8) + data[VULVA_GAP_POS + 1];

            /* Enter critical session to access the queue */
            xSemaphoreTake (SensorQueueMutex, portMAX_DELAY);

            /* Stores sensor sample on queue */
            vulvaSensorQueue.push (vulvaSensor);
#ifdef DEBUG

            /* Print latest sample values */
            Serial.printf("Sensor Name: %s\n", vulvaSensorQueue.back().header.name.c_str());
            Serial.printf("Sensor Addr: %s\n", vulvaSensorQueue.back().header.addr.c_str());
            Serial.printf("Sensor Time: %d\n", (int) vulvaSensorQueue.back().header.time);
            Serial.printf("Sensor Batt: %d\n", vulvaSensorQueue.back().battery);
            Serial.printf("Sensor Dil : %d\n", vulvaSensorQueue.back().dilation);
            Serial.printf("Sensor Gap : %d\n", vulvaSensorQueue.back().gap);
            Serial.printf("Queue size: %d\n", vulvaSensorQueue.size());            
#endif
            /* Limits the queue size */
            if (vulvaSensorQueue.size() > QUEUE_MAX_LEN)
            {
              vulvaSensorQueue.pop();
            }
            /* Exit critical session */
            xSemaphoreGive (SensorQueueMutex);

            break;

          case HYGRO_SENSOR_TYPE:
            hygroSensor.header.name = foundDevices.getDevice(i).getName();
            hygroSensor.header.addr = foundDevices.getDevice(i).getAddress().toString();
            hygroSensor.header.time = getTime();
            hygroSensor.battery = data[HYGRO_BATTERY_POS];
            hygroSensor.humidity = ((data[HYGRO_HUMIDITY_POS]) << 8) + data[HYGRO_HUMIDITY_POS + 1];
            hygroSensor.temperature = ((data[HYGRO_TEMPERATURE_POS]) << 8) + data[HYGRO_TEMPERATURE_POS + 1];

            /* Enter critical session to access the queue */
            xSemaphoreTake (SensorQueueMutex, portMAX_DELAY);

            /* Stores sensor sample on queue */
            hygroSensorQueue.push (hygroSensor);
#ifdef DEBUG
            /* Print latest sample values */
            Serial.printf("Sensor Name: %s\n", hygroSensorQueue.back().header.name.c_str());
            Serial.printf("Sensor Addr: %s\n", hygroSensorQueue.back().header.addr.c_str());
            Serial.printf("Sensor Time: %d\n", (int) hygroSensorQueue.back().header.time);
            Serial.printf("Sensor Batt: %d\n", hygroSensorQueue.back().battery);
            Serial.printf("Sensor Hum.: %d\n", hygroSensorQueue.back().humidity);
            Serial.printf("Sensor Temp: %d\n", hygroSensorQueue.back().temperature);
            Serial.printf("Queue size: %d\n", hygroSensorQueue.size());
#endif
            /* Limits the queue size */
            if (hygroSensorQueue.size() > QUEUE_MAX_LEN)
            {
              hygroSensorQueue.pop();
            }
            /* Exit critical session */
            xSemaphoreGive (SensorQueueMutex);

            break;

          default:
            break;
          }
        }
      }
    }
    // delete results from BLE Scan Buffer to release memory
    pBLEScan->clearResults();

    vTaskDelay(29000 / portTICK_PERIOD_MS);
  }
}

/**
 *  @brief    Task to publish sensor data and keep-alive to cloud
 *  @details  Sends all available samples in sensor queues to cloud once every 10 seconds.
 * 
 *  @param [in] pvParameters  Not used.
 */
void Cloud_Task (void *pvParameters __attribute__((unused))) // This is a Task.
{
  thigh_sensor_data_t thighSensor;
  vulva_sensor_data_t vulvaSensor;
  hygrometer_sensor_data_t hygroSensor;
  String httpRequestBody;
  String response;
  int statusCode = 0;

  /* Creates the HTTP client */
  HttpClient http = HttpClient (client, server, port);

  /* Connect to APN */
  SerialMon.print("Connecting to APN: ");
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass))
  {
    SerialMon.println(" fail");
  }
  else
  {
    /* APN connected */
    SerialMon.println(" OK");

    /* Connect to Server */
    SerialMon.print("Connecting to ");
    SerialMon.print(server);
    if (!client.connect(server, port))
    {
      SerialMon.println(" fail");
    }
    else
    {
      /* Server connected */
      SerialMon.println(" OK");
    }

    while (1)
    {
      /*
       *  Publishes Keep-Alive  
       */
#ifdef DEBUG_REQUEST        
      SerialMon.println("Performing Keep-Alive request...");
#endif        
      /* Get local MacAddress */
      BLEAddress addr = BLEDevice::getAddress();

      /* JSON request data */
      httpRequestBody = "{\"macAddress\":\""     + String (addr.toString().c_str()) + "\","
                         "\"timeStamp\":"        + String (getTime())               + ","
                         "\"sensorsConected\":"  + String (0)                       + ","
                         "\"token\":\""          + String (apiKey)                  + "\"}";

      http.beginRequest();      
      http.post (endpointKeepAlive, "application/json", httpRequestBody);
      http.endRequest();      

#ifdef DEBUG_REQUEST
      SerialMon.println(httpRequestBody);
#endif
      // Read the status code and body of the response
      // statusCode = http.responseStatusCode();
      // response = http.responseBody();

#ifdef DEBUG_REQUEST
      // Serial.print("Status code: ");
      // Serial.println(statusCode);
      // Serial.print("Response: ");
      // Serial.println(response);
#endif
      /**
       *  Publishes available Thigh Sensor data
       */
      if (thighSensorQueue.size() != 0)
      {
        /* Enter critical session to access the queue */
        xSemaphoreTake(SensorQueueMutex, portMAX_DELAY);

        /* Get sensor sample from queue (but don't remove it) */
        thighSensor = thighSensorQueue.front();

        /* Exit critical session */
        xSemaphoreGive(SensorQueueMutex);

        /* Send HTTP Request */
#ifdef DEBUG_REQUEST          
          SerialMon.println("Performing Thigh Sensor request...");
#endif
        /* Converts temperature to floating format */
        float temperature = ((float) thighSensor.temperature) / 10;

        /* JSON request data */
        httpRequestBody = "{\"macAddress\":\""  + String (thighSensor.header.addr.c_str()) + "\","
                           "\"battery\":\""     + String (thighSensor.battery)             + "\","
                           "\"timeStamp\":"     + String (thighSensor.header.time)         + ","
                           "\"temperature\":"   + String (temperature)                     + ","
                           "\"active\":"        + String (thighSensor.activity)            + ","
                           "\"position\":"      + String (thighSensor.position)            + ","
                           "\"token\":\""       + String (apiKey)                          + "\"}";

        http.beginRequest();       
        http.post (endpointThighSensor, "application/json", httpRequestBody);
        http.endRequest();        

#ifdef DEBUG_REQUEST
        SerialMon.println(httpRequestBody);
#endif
        // Read the status code and body of the response
        // statusCode = http.responseStatusCode();
        // response = http.responseBody();

#ifdef DEBUG_REQUEST
        // Serial.print("Status code: ");
        // Serial.println(statusCode);
        // Serial.print("Response: ");
        // Serial.println(response);
#endif
        /* If transaction is successful, remove from queue */
        if (1)
        {
          /* Enter critical session to access the queue */
          xSemaphoreTake(SensorQueueMutex, portMAX_DELAY);

          /* Remove sensor sample from queue */
          thighSensorQueue.pop();

          /* Exit critical session */
          xSemaphoreGive(SensorQueueMutex);
        }
      }

      /*
       *  Publishes available Vulva Sensor data
       */
      if (vulvaSensorQueue.size() != 0)
      {
        /* Enter critical session to access the queue */
        xSemaphoreTake(SensorQueueMutex, portMAX_DELAY);

        /* Get sensor sample from queue (but don't remove it) */
        vulvaSensor = vulvaSensorQueue.front();

        /* Exit critical session */
        xSemaphoreGive(SensorQueueMutex);

        /* Send HTTP Request */
#ifdef DEBUG_REQUEST          
          SerialMon.println("Performing Vulva Sensor request...");
#endif
        /* JSON request data */
        httpRequestBody = "{\"macAddress\":\""  + String (vulvaSensor.header.addr.c_str()) + "\","
                           "\"battery\":\""     + String (vulvaSensor.battery)             + "\","
                           "\"timeStamp\":"     + String (vulvaSensor.header.time)         + ","
                           "\"dilation\":"      + String (vulvaSensor.dilation)            + ","
                           "\"gap\":"           + String (vulvaSensor.gap)                 + ","
                           "\"token\":\""       + String (apiKey)                          + "\"}";

        http.beginRequest();
        http.post (endpointVulvaSensor, "application/json", httpRequestBody);
        http.endRequest();

#ifdef DEBUG_REQUEST
        SerialMon.println(httpRequestBody);
#endif
        // Read the status code and body of the response
        // statusCode = http.responseStatusCode();
        // response = http.responseBody();

#ifdef DEBUG_REQUEST
        // Serial.print("Status code: ");
        // Serial.println(statusCode);
        // Serial.print("Response: ");
        // Serial.println(response);
#endif
        /* If transaction is successful, remove from queue */
        if (1)
        {
          /* Enter critical session to access the queue */
          xSemaphoreTake(SensorQueueMutex, portMAX_DELAY);

          /* Remove sensor sample from queue */
          vulvaSensorQueue.pop();

          /* Exit critical session */
          xSemaphoreGive(SensorQueueMutex);
        }
      }

      /*
       *  Publishes available Hygrometer Sensor data
       */
      if (hygroSensorQueue.size() != 0)
      {
        /* Enter critical session to access the queue */
        xSemaphoreTake(SensorQueueMutex, portMAX_DELAY);

        /* Get sensor sample from queue (but don't remove it) */
        hygroSensor = hygroSensorQueue.front();

        /* Exit critical session */
        xSemaphoreGive(SensorQueueMutex);

        /* Send HTTP Request */
#ifdef DEBUG_REQUEST          
          SerialMon.println("Performing Hygro Sensor request...");
#endif
        /* Converts humidity and temperature to floating format */
        float temperature = ((float) hygroSensor.temperature) / 10;
        float humidity = ((float) hygroSensor.humidity) / 10;

        /* JSON request data */
        httpRequestBody = "{\"macAddress\":\""      + String (hygroSensor.header.addr.c_str()) + "\","
                           "\"battery\":\""         + String (hygroSensor.battery)             + "\","
                           "\"timeStamp\":"         + String (hygroSensor.header.time)         + ","
                           "\"temp_environment\":"  + String (temperature)                     + ","
                           "\"humidity\":"          + String (humidity)                        + ","
                           "\"token\":\""           + String (apiKey)                          + "\"}";

        http.beginRequest();        
        http.post (endpointHygroSensor, "application/json", httpRequestBody);
        http.endRequest();        

#ifdef DEBUG_REQUEST
        SerialMon.println(httpRequestBody);
#endif
        // Read the status code and body of the response
        // statusCode = http.responseStatusCode();
        // response = http.responseBody();

#ifdef DEBUG_REQUEST
        // Serial.print("Status code: ");
        // Serial.println(statusCode);
        // Serial.print("Response: ");
        // Serial.println(response);
#endif
        /* If transaction is successful, remove from queue */
        if (1)
        {
          /* Enter critical session to access the queue */
          xSemaphoreTake(SensorQueueMutex, portMAX_DELAY);

          /* Remove sensor sample from queue */
          hygroSensorQueue.pop();

          /* Exit critical session */
          xSemaphoreGive(SensorQueueMutex);
        }
      }

      /* Reconnect when network is down */
      if (!modem.isNetworkConnected())
      {
        if (!modem.gprsConnect(apn, gprsUser, gprsPass))
        {
          SerialMon.println(" fail");
        }
        else
        {
          /* APN connected */
          SerialMon.println(" OK");

          /* Connect to Server */
          SerialMon.print("Connecting to ");
          SerialMon.print(server);
          if (!client.connect(server, port))
          {
            SerialMon.println(" fail");
          }
          else
          {
            /* Server connected */
            SerialMon.println(" OK");
          }
        }
      }
      vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
  }
}

 /**
 *  @brief    Task intended just to monitoring application behavior
 *  @details  None.
 */
 void UI_Task(void *pvParameters __attribute__((unused))) // This is a Task.
 {
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    gpio_pad_select_gpio(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    while (1)
    {
      /* Blink off (output low) */
      gpio_set_level(BLINK_GPIO, 0);
      vTaskDelay(450 / portTICK_PERIOD_MS);

      /* Blink on (output high) */
      gpio_set_level(BLINK_GPIO, 1);
      vTaskDelay(50 / portTICK_PERIOD_MS);
    }
  }

  void loop()
  {
    // Empty. Things are done in Tasks.
  }