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
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <WiFi.h>
#include <BLEAdvertisedDevice.h>
#include <queue>
#include <time.h>
#include "router.h"

using namespace std;

/* If defined, allows terminal debug info */
// #define DEBUG
// #define DEBUG_EXAMPLE
// #define PUBLISH_RANDOM_DATA

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
static BLEUUID simulationUUID("a995027a-0f33-4d57-8c7b-575a66f812b1");
static BLEUUID simulUUID("1800");

int scanTime = 1; //In seconds
BLEScan *pBLEScan;

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
#ifdef DEBUG_EXAMPLE
    Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
#endif
  }
};

/*
 *  WiFi definitions
 */
WiFiClient client;

// Your WiFi credentials
const char ssid[] = "Wifi_da_Vovo";       // WiFi SSID
const char pass[] = "wifi.gilberto";      // Wifi Password

// Server details
const char server[] = "birthalert.angoeratech.com.br";
const char resource[] = "/api/setSensorCoxa";
const char apiKey[] = "117f08a0a9c5808e93a4c246ec0f2dab";
const int  port = 80;

/* Prototypes */
void Sensor_Task(void *pvParameters);
void Cloud_Task(void *pvParameters);
void UI_Task(void *pvParameters);

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
  char formatted_date[64];
  time_t current_time;
  tm date;

  while (1)
  {
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
          /* Set the timestamp */
          current_time = time(NULL);
          date = *gmtime(&current_time);
          strftime(formatted_date, 64, "%d/%m/%Y %H:%M:%S", &date);

          /* get sensor type */
          type = data[SENSOR_TYPE_POS];
#ifdef DEBUG
          Serial.printf("Sensor type: %d\n", type);
#endif
          switch (type)
          {
          case THIGH_SENSOR_TYPE:
            strcpy(thighSensor.header.name, foundDevices.getDevice(i).getName().c_str());
            strcpy(thighSensor.header.addr, foundDevices.getDevice(i).getAddress().toString().c_str());
            strcpy(thighSensor.header.time, formatted_date);
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
            Serial.printf("Sensor Name: %s\n", thighSensorQueue.back().header.name);
            Serial.printf("Sensor Addr: %s\n", thighSensorQueue.back().header.addr);
            Serial.printf("Sensor Time: %s\n", thighSensorQueue.back().header.time);
            Serial.printf("Sensor Batt: %d\n", thighSensorQueue.back().battery);
            Serial.printf("Sensor Act.: %d\n", thighSensorQueue.back().activity);
            Serial.printf("Sensor Temp: %d\n", thighSensorQueue.back().temperature);
            Serial.printf("Sensor Pos : %d\n", thighSensorQueue.back().position);
#endif
            /* Exit critical session */
            xSemaphoreGive (SensorQueueMutex);

            break;

          case VULVA_SENSOR_TYPE:
            strcpy(vulvaSensor.header.name, foundDevices.getDevice(i).getName().c_str());
            strcpy(vulvaSensor.header.addr, foundDevices.getDevice(i).getAddress().toString().c_str());
            strcpy(vulvaSensor.header.time, formatted_date);
            vulvaSensor.battery = data[VULVA_BATTERY_POS];
            vulvaSensor.dilation = ((data[VULVA_DILATION_POS]) << 8) + data[VULVA_DILATION_POS + 1];
            vulvaSensor.gap = ((data[VULVA_GAP_POS]) << 8) + data[VULVA_GAP_POS + 1];

            /* Enter critical session to access the queue */
            xSemaphoreTake (SensorQueueMutex, portMAX_DELAY);

            /* Stores sensor sample on queue */
            vulvaSensorQueue.push (vulvaSensor);
#ifdef DEBUG
            /* Print latest sample values */
            Serial.printf("Sensor Name: %s\n", vulvaSensorQueue.back().header.name);
            Serial.printf("Sensor Addr: %s\n", vulvaSensorQueue.back().header.addr);
            Serial.printf("Sensor Time: %s\n", vulvaSensorQueue.back().header.time);
            Serial.printf("Sensor Batt: %d\n", vulvaSensorQueue.back().battery);
            Serial.printf("Sensor Dil : %d\n", vulvaSensorQueue.back().dilation);
            Serial.printf("Sensor Gap : %d\n", vulvaSensorQueue.back().gap);
#endif
            /* Exit critical session */
            xSemaphoreGive (SensorQueueMutex);

            break;

          case HYGRO_SENSOR_TYPE:
            strcpy(hygroSensor.header.name, foundDevices.getDevice(i).getName().c_str());
            strcpy(hygroSensor.header.addr, foundDevices.getDevice(i).getAddress().toString().c_str());
            strcpy(hygroSensor.header.time, formatted_date);
            hygroSensor.battery = data[HYGRO_BATTERY_POS];
            hygroSensor.humidity = ((data[HYGRO_HUMIDITY_POS]) << 8) + data[HYGRO_HUMIDITY_POS + 1];
            hygroSensor.temperature = ((data[HYGRO_TEMPERATURE_POS]) << 8) + data[HYGRO_TEMPERATURE_POS + 1];

            /* Enter critical session to access the queue */
            xSemaphoreTake (SensorQueueMutex, portMAX_DELAY);

            /* Stores sensor sample on queue */
            hygroSensorQueue.push (hygroSensor);
#ifdef DEBUG
            /* Print latest sample values */
            Serial.printf("Sensor Name: %s\n", hygroSensorQueue.back().header.name);
            Serial.printf("Sensor Addr: %s\n", hygroSensorQueue.back().header.addr);
            Serial.printf("Sensor Time: %s\n", hygroSensorQueue.back().header.time);
            Serial.printf("Sensor Batt: %d\n", hygroSensorQueue.back().battery);
            Serial.printf("Sensor Hum.: %d\n", hygroSensorQueue.back().humidity);
            Serial.printf("Sensor Temp: %d\n", hygroSensorQueue.back().temperature);
#endif
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

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

/**
 *  @brief    Task to publish sensor data to cloud
 *  @details  Sends all available samples in sensor queues to cloud once every X seconds.
 * 
 *  @param [in] pvParameters  Not used.
 */
void Cloud_Task (void *pvParameters __attribute__((unused))) // This is a Task.
{
  thigh_sensor_data_t thighSensor;
  vulva_sensor_data_t vulvaSensor;
  hygrometer_sensor_data_t hygroSensor;

  while (1)
  {
    // Connect or reconnect to WiFi
    if(WiFi.status() != WL_CONNECTED)
    {
      Serial.print("Attempting to connect to SSID: ");
      Serial.println(ssid);
      while(WiFi.status() != WL_CONNECTED)
      {
        WiFi.begin(ssid, pass);  // Connect to WPA/WPA2 network. Change this line if using open or WEP network
        Serial.print(".");
        delay(5000);     
      } 
      Serial.println("\nConnected.");
    }
    /* Publishes Thigh Sensor available data */
    // while (thighSensorQueue.size() != 0)
    // {
    /* Enter critical session to access the queue */
    xSemaphoreTake(SensorQueueMutex, portMAX_DELAY);

    /* Get and Remove sensor sample from queue */
    thighSensor = thighSensorQueue.front();
    thighSensorQueue.pop();

    /* Exit critical session */
    xSemaphoreGive(SensorQueueMutex);

    /* Send HTTP Request */
    Serial.println("Performing HTTP POST request...");

    /*
          POST /transactions/sensorcoxa HTTP/1.1
          Host: {{ENDPOINT}}
          Content-Type: application/json
          Content-Length: 160
          {
              "macAdrees": "12:09:78:ab:c6:7f"
              "battery": "99"
              "timeStamp": 1623237859,
              "temperature": 36.15,
              "position": 0,
              "active": 13450,
          }
          */

    HttpClient http = HttpClient(client, server, port);

    /* JSON request data */
    String httpRequestBody = "{\"macAddress\":\""  + String (thighSensor.header.addr) + "\","
                              "\"battery\":\""     + String (thighSensor.battery)     + "\","
                              "\"timeStamp\":"     + String (1623237859)              + ","
                              "\"temperature\":"   + String (thighSensor.temperature) + ","
                              "\"active\":"        + String (thighSensor.activity)    + ","
                              "\"position\":"      + String (thighSensor.position)    + ","
                              "\"token\":"         + String (apiKey)                  + "}";
  
    http.post(resource, "Content-Type: application/json", httpRequestBody);

    Serial.println();
    Serial.println(httpRequestBody);
    Serial.println();

    // read the status code and body of the response
    int statusCode = http.responseStatusCode();
    String response = http.responseBody();

    Serial.print("Status code: ");
    Serial.println(statusCode);
    Serial.print("Response: ");
    Serial.println(response);

    // /* JSON request data */
    // String httpRequestData = "{\"macAddress\":\""  + String (thighSensor.header.addr) + "\","
    //                           "\"battery\":\""     + String (thighSensor.battery)     + "\","
    //                           "\"timeStamp\":"     + String (1623237859)              + ","
    //                           "\"temperature\":"   + String (thighSensor.temperature) + ","
    //                           "\"active\":"        + String (thighSensor.activity)    + ","
    //                           "\"position\":"      + String (thighSensor.position)    + ","
    //                           "\"token\":"         + String (apiKey)                  + "}";

    // Create JSON doc and write attributes
    // const size_t capacity = JSON_OBJECT_SIZE(7);
    // DynamicJsonDocument doc(capacity);

    // doc["macAddress"]  = String (thighSensor.header.addr);
    // doc["battery"]     = String (thighSensor.battery) ;
    // doc["timeStamp"]   = 1623237859;
    // doc["temperature"] = thighSensor.temperature;
    // doc["active"]      = thighSensor.activity;
    // doc["position"]    = thighSensor.activity;
    // doc["token"]       = apiKey;

    // client.print (String("POST ") + resource + " HTTP/1.1\r\n");
    // client.print (String("Host: ") + server + "\r\n");
    // client.println ("Content-Type: application/json");
    // // client.println ("Connection: close");
    // client.print ("Content-Length: ");
    // // client.println (httpRequestData.length());
    // // client.println (httpRequestData);
    // client.println(measureJson(doc));

    // Prints doc to client
    // serializeJson (doc, client);

    // SerialMon.println (httpRequestData);

    /* Print response */
    // unsigned long timeout = millis();
    // while (client.connected() && millis() - timeout < 10000L)
    // {
    //   // Print available data (HTTP response from server)
    //   while (client.available())
    //   {
    //     char c = client.read();
    //     SerialMon.print(c);
    //     timeout = millis();
    //   }
    // }
    // SerialMon.println();
    // }

    /* Publishes Vulva Sensor available data */
    while (vulvaSensorQueue.size() != 0)
    {
      /* Enter critical session to access the queue */
      xSemaphoreTake(SensorQueueMutex, portMAX_DELAY);

      /* Get and Remove sensor sample from queue */
      vulvaSensor = vulvaSensorQueue.front();
      vulvaSensorQueue.pop();

      /* Exit critical session */
      xSemaphoreGive(SensorQueueMutex);

      /* Send HTTP Request */
    }

    /* Publishes Hygrometer Sensor available data */
    while (hygroSensorQueue.size() != 0)
    {
      /* Enter critical session to access the queue */
      xSemaphoreTake(SensorQueueMutex, portMAX_DELAY);

      /* Get and Remove sensor sample from queue */
      hygroSensor = hygroSensorQueue.front();
      hygroSensorQueue.pop();

      /* Exit critical session */
      xSemaphoreGive(SensorQueueMutex);

      /* Send HTTP Request */
    }
    vTaskDelay(15000 / portTICK_PERIOD_MS);
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