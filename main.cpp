/**
 *  Birth Alert Router application 
 * 
 *  @author Gilberto Araujo
 */

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include "time.h"
#include "router.h"

// The remote service we wish to connect to.
static BLEUUID sensorUUID ("befab990-ddb3-11eb-ba80-0242ac130004");
static BLEUUID testUUID ("c1462ae7-9493-4018-b6d3-dda50387989c");

/* GPIO pin to blink */
#define BLINK_GPIO		GPIO_NUM_4

void Main_Task( void *pvParameters );
void UI_Task( void *pvParameters );

int scanTime = 5; //In seconds
BLEScan* pBLEScan;

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      // Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
    }
};

void setup()
{
    // initialize serial communication at 115200 bits per second:
    Serial.begin(9600);
    while (!Serial)
    {
      ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
    }
    Serial.println("Scanning...");

    /* Init BLE environment */
    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan (); //create new scan
    pBLEScan->setAdvertisedDeviceCallbacks (new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan (true); //active scan uses more power, but get results faster
    pBLEScan->setInterval (100);
    pBLEScan->setWindow (99);  // less or equal setInterval value

    // Now set up two Tasks to run independently.
    xTaskCreatePinnedToCore (Main_Task
    , "Main Task"  // A name just for humans
    ,  4096 // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL //Parameters for the task
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL
    ,  0 ); //Task Handle

    xTaskCreatePinnedToCore (UI_Task
    ,  "UI Task" // A name just for humans
    ,  2048  // Stack size
    ,  NULL //Parameters for the task
    ,  1  // Priority
    ,  NULL
    ,  0 ); //Task Handle

    // Now the Task scheduler, which takes over control of scheduling individual Tasks, is automatically started.
}

/**
 * 
 */
 void Main_Task ( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
    ThighSensor thighSensor;
    VulvaSensor vulvaSensor;

    while(1)
    {
      BLEScanResults foundDevices = pBLEScan->start (scanTime, false);
      int devicesCount = foundDevices.getCount();

      Serial.print ("Devices found: ");
      Serial.println (devicesCount);
      Serial.println ("Scan done!");
      
      for (int i = 0; i < devicesCount; i++)
      {
        /* Check for known sensors by UUID */
        if (foundDevices.getDevice(i).haveServiceUUID() && foundDevices.getDevice(i).isAdvertisingService(testUUID))
        {
          Serial.printf ("Sensor Name: %s\n", foundDevices.getDevice(i).getName().c_str());
          Serial.printf ("Sensor Addr: %s\n", foundDevices.getDevice(i).getAddress().toString().c_str());
          Serial.printf ("Sensor UUID: %s\n", foundDevices.getDevice(i).getServiceUUID().toString().c_str());

          /* Check for sensor parameters */          
          if (foundDevices.getDevice(i).haveManufacturerData())
          {
            char data[9];            
            int len = foundDevices.getDevice(i).getManufacturerData().length();            
            
            /* Get sensor data */
            strcpy (data, foundDevices.getDevice(i).getManufacturerData().c_str());

            /* Debug output */
            Serial.printf ("Sensor Dlen: %d\n", len);
            Serial.printf ("Sensor data: %s\n", data);
            Serial.printf ("Sensor Data: ");
            for (int i = 0; i < len; i++)
            {              
              Serial.printf ("%02x", (int) data[i]);
            }
            Serial.printf ("\n");

            /* Set the timestamp */
            time_t current_time = time (NULL);
            tm date = *gmtime (&current_time);
            char formatted_date [64];
            strftime (formatted_date, 64, "%d/%m/%Y %H:%M:%S", &date);

            /* get sensor type */
            int type = data [SENSOR_TYPE_POS];

            switch (type)
            {
            case THIGH_SENSOR_TYPE:
              strcpy ((char*) thighSensor.name, (char*) foundDevices.getDevice(i).getName().c_str());
              strcpy (thighSensor.addr, foundDevices.getDevice(i).getAddress().toString().c_str());
              strcpy (thighSensor.time, formatted_date);
              thighSensor.battery = data [THIGH_BATTERY_POS];
              thighSensor.activity = 200;
              thighSensor.temperature = 35;
              thighSensor.position = data [THIGH_POSITION_POS];
              break;
            
            case VULVA_SENSOR_TYPE:
              
              break;
            
            case HYGRO_SENSOR_TYPE:
              
              break;
            
            default:
              break;
            }

            Serial.printf ("Sensor Name: %s\n", thighSensor.name);
            Serial.printf ("Sensor Addr: %s\n", thighSensor.addr);
            Serial.printf ("Sensor Time: %s\n", thighSensor.time);

            // Serial.printf ("Sensor Batt: %d\n", thighSensor.battery);
            // Serial.printf ("Sensor Act : %d\n", thighSensor.activity);
            // Serial.printf ("Sensor Temp: %d\n", thighSensor.temperature);
            // Serial.printf ("Sensor Pos : %d\n", thighSensor.position);
            

          }
        }
      }

      pBLEScan->clearResults ();   // delete results fromBLEScan buffer to release memory
      
      vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

/**
 * 
 */
void UI_Task ( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    gpio_pad_select_gpio (BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    
    while(1)
    {
    	/* Blink off (output low) */
    	gpio_set_level (BLINK_GPIO, 0);
    	vTaskDelay(500 / portTICK_PERIOD_MS);

    	/* Blink on (output high) */
    	gpio_set_level (BLINK_GPIO, 1);
    	vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void loop() 
{
  // Empty. Things are done in Tasks.
}