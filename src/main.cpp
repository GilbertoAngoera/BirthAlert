/**
 *  Birth Alert Router application 
 * 
 *  @author Gilberto Araujo
 * 
 *  References: 
 *    - Cloud Publishing: Rui Santos at https://RandomNerdTutorials.com/esp32-sim800l-publish-data-to-cloud/
 *                        "Permission is hereby granted, free of charge, to any person obtaining a copy
 *                         of this software and associated documentation files.  
 *                         The above copyright notice and this permission notice shall be included in all
 *                         copies or substantial portions of the Software."
 *    - Data Hosting:  https://nothans.com/thingspeak-tutorials/arduino/send-data-to-thingspeak-with-arduino
 */

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <queue>
#include <time.h>
#include "router.h"

using namespace std;

/* If defined, allows terminal debug info */
#define DEBUG
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

/**
 *  SIM800L definitions
 */

// Your GPRS credentials (leave empty, if not needed)
const char apn[] = "zap.vivo.com.br"; // APN
const char gprsUser[] = "vivo";       // GPRS User
const char gprsPass[] = "vivo";       // GPRS Password
const char simPIN[] = "";             // SIM card PIN (leave empty, if not defined)

// Server details
const char server[] = "birthalert.angoeratech.com.br";
const char resource[] = "/api/setSensorCoxa";
const int  port = 80;

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
          foundDevices.getDevice(i).isAdvertisingService(simulationUUID))
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
  int ret = 0;
  thigh_sensor_data_t thighSensor;
  vulva_sensor_data_t vulvaSensor;
  hygrometer_sensor_data_t hygroSensor;

  while (1)
  {
    /* Connect to APN */
    SerialMon.print ("Connecting to APN: ");
    SerialMon.print (apn);
    if (!modem.gprsConnect (apn, gprsUser, gprsPass))
    {
      SerialMon.println (" fail");
    }
    else
    {
      /* APN connected */
      SerialMon.println (" OK");

      /* Connect to Server */
      SerialMon.print("Connecting to ");
      SerialMon.print(server);
      if (!client.connect(server, port)) {
        SerialMon.println(" fail");
      }
      else
      {
        /* Server connected */
        SerialMon.println(" OK");

        /* Publishes Thigh Sensor available data */
        while (thighSensorQueue.size() != 0)
        {
          /* Enter critical session to access the queue */
          xSemaphoreTake (SensorQueueMutex, portMAX_DELAY);

          /* Get and Remove sensor sample from queue */
          thighSensor = thighSensorQueue.front();
          thighSensorQueue.pop();

          /* Exit critical session */
          xSemaphoreGive (SensorQueueMutex);

          /* Send HTTP Request */


        }

        /* Publishes Vulva Sensor available data */
        while (vulvaSensorQueue.size() != 0)
        {
          /* Enter critical session to access the queue */
          xSemaphoreTake (SensorQueueMutex, portMAX_DELAY);

          /* Get and Remove sensor sample from queue */
          vulvaSensor = vulvaSensorQueue.front();
          vulvaSensorQueue.pop();

          /* Exit critical session */
          xSemaphoreGive (SensorQueueMutex);

          /* Send HTTP Request */

          

        }

        /* Publishes Hygrometer Sensor available data */
        while (hygroSensorQueue.size() != 0)
        {
          /* Enter critical session to access the queue */
          xSemaphoreTake (SensorQueueMutex, portMAX_DELAY);
          
          /* Get and Remove sensor sample from queue */
          hygroSensor = hygroSensorQueue.front();
          hygroSensorQueue.pop();

          /* Exit critical session */
          xSemaphoreGive (SensorQueueMutex);

          /* Send HTTP Request */


        }
   
        unsigned long timeout = millis();
        while (client.connected() && millis() - timeout < 10000L)
        {
        
          // Print available data (HTTP response from server)
          while (client.available())
          {
            char c = client.read();
            SerialMon.print(c);
            timeout = millis();
          }
        }      
        SerialMon.println();
    
        // Close client and disconnect from Server
        client.stop();
        SerialMon.println(F("Server disconnected"));
      }

      /* Disconnect from APN */
      modem.gprsDisconnect();
      SerialMon.println (F("GPRS disconnected"));
    }
    vTaskDelay (15000 / portTICK_PERIOD_MS);
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