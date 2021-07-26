/**
 * sensor.h
 *
 *	Birth Alert Router's application
 *
 *  Created on: 07 de jul. de 2021
 *      Author: Gilberto Araujo
 */

#include <Arduino.h>
#include <stdint.h>
#include <queue>

#define HEX_BASE		        16

#define THIGH_SENSOR_TYPE		0x01
#define VULVA_SENSOR_TYPE		0x02
#define HYGRO_SENSOR_TYPE		0x03

#define SENSOR_TYPE_POS         2

/* Thigh sensor fields info */
#define THIGH_BATTERY_POS		3
#define THIGH_TEMPERATURE_POS	4
#define THIGH_ACTIVITY_POS		6
#define THIGH_POSITION_POS		8

#define THIGH_BATTERY_LEN		1
#define THIGH_TEMPERATURE_LEN	2
#define THIGH_ACTIVITY_LEN		2
#define THIGH_POSITION_LEN		1

/* Vulva sensor fields info */
#define VULVA_BATTERY_POS		3
#define VULVA_DILATION_POS		4
#define VULVA_GAP_POS			6

#define VULVA_BATTERY_LEN		1
#define VULVA_DILATION_LEN		2
#define VULVA_GAP_LEN			2

/* Hygro sensor fields info */
#define HYGRO_BATTERY_POS		3
#define HYGRO_HUMIDITY_POS		4
#define HYGRO_TEMPERATURE_POS	6

#define VULVA_BATTERY_LEN		1
#define HYGRO_HUMIDITY_LEN		2
#define HYGRO_TEMPERATURE_LEN	2

/*
 * Types to hold sensors data
 */
typedef struct sensor_data {
	char name [32];
	char addr [18];
	char time [64];
} sensor_data_t;

typedef struct thigh_sensor_data {
	sensor_data_t header;
	uint8_t battery;
	uint16_t temperature;
	uint16_t activity;
	uint8_t position;
} thigh_sensor_data_t;

typedef struct vulva_sensor_data {
	sensor_data_t header;
	uint8_t battery;
	uint16_t dilation;
	uint16_t gap;
} vulva_sensor_data_t;

typedef struct hygrometer_sensor_data {
	sensor_data_t header;
	uint8_t battery;
	uint16_t humidity;
	uint16_t temperature;
} hygrometer_sensor_data_t;

/* Public Prototypes */

class SensorData
{
private:
    /* Sensor attributes */
    String name;
    String address;
    uint8_t battery;
public:
    SensorData(/* args */);
    ~SensorData();
    
    /* Sensor methods */
};

class SensorSample
{
private:
    /* Sensor Sample attributes */
    String timeStamp;
public:
    SensorSample(/* args */);
    ~SensorSample();
    
    /* Sensor Sample methods */
};

class ThighSensorData : public SensorData , public SensorSample
{
private:
    /* Thigh Sensor Data attributes */
    uint16_t temperature;
	uint16_t activity;
	uint8_t position;
public:
    ThighSensorData(/* args */);
    ~ThighSensorData();

    /* Thigh Sensor Data methods */
};

class VulvaSensorData : public SensorData, public SensorSample
{
private:
    /* Vulva Sensor Data attributes */
	uint16_t dilation;
	uint16_t gap;

public:
    VulvaSensorData(/* args */);
    ~VulvaSensorData();

    /* Vulva Sensor Data methods */
};

class HygroSensorData : public SensorData, public SensorSample
{
private:
    /* Hygro Sensor Data attributes */
    uint16_t temperature;
	uint16_t humidity;

public:
    HygroSensorData(/* args */);
    ~HygroSensorData();

    /* Hygro Sensor Data methods */
};

class ThighSensor : public ThighSensorData
{
private:
    /* Thigh Sensor attributes */
    static std::queue<ThighSensorData> queue;
public:
    ThighSensor(/* args */);
    ~ThighSensor();
    
    /* Thigh Sensor methods */
    void setName(String name);
	void setAddress(String addr);
    void setBattery(uint8_t level);
	String getName();
	String getAddress();
    uint8_t getBattery();

    void setTemperature(uint16_t temperature);
    void setActivity(uint16_t activity);
    void setPosition(uint8_t position);
	uint16_t getTemperature();
    uint16_t getActivity();
    uint8_t getPosition();

    uint8_t getBattery();
	void pushToQueue(ThighSensorData sample);
	void removeFromQueue();
    ThighSensorData getFromQueue();
};

class VulvaSensor
{
private:
    /* Vulva Sensor attributes */
    static std::queue<VulvaSensorData> queue;
public:
    VulvaSensor(/* args */);
    ~VulvaSensor();
    
    /* Vulva Sensor methods */
    void setName(String name);
	void setAddress(String addr);
    void setBattery(uint8_t level);
	String getName();
	String getAddress();
    uint8_t getBattery();

    void setDilation(uint16_t dilation);
    void setGap(uint16_t gap);
    uint16_t getDilation();
    uint16_t getGap();

	void pushToQueue(VulvaSensorData sample);
	void removeFromQueue();
    VulvaSensorData getFromQueue();
};

class HygroSensor : public HygroSensorData
{
private:
    /* Hygro Sensor attributes */
    static std::queue<HygroSensorData> queue;
public:
    HygroSensor(/* args */);
    ~HygroSensor();
    
    /* Hygro Sensor methods */
    void setName(String name);
	void setAddress(String addr);
    void setBattery(uint8_t level);
	String getName();
	String getAddress();
    uint8_t getBattery();

    void setTemperature(uint16_t temperature);
    void setHumidity(uint16_t humidity);
    uint16_t getTemperature();
    uint16_t getHumidity();    

	void pushToQueue(HygroSensorData sample);
	void removeFromQueue();
    HygroSensorData getFromQueue();
};
