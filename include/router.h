/**
 * router.h
 *
 *	Birth Alert Router's application
 *
 *  Created on: 07 de jul. de 2021
 *      Author: Gilberto Araujo
 */

#include <stdint.h>

using namespace std;

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
	string name;
	string addr;
	unsigned long time;
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


// class Sensor
// {
// private:
//     /* data */
// public:
//     Sensor(/* args */);
//     ~Sensor();

//     /* Sensor attributes */
//     string name;
//     string addr;
//     string time;
//     uint8_t battery;
    
//     /* Sensor methods */
//     void setBattery();
// };

// class ThighSensor : public Sensor
// {
// private:
//     /* data */
// public:
//     ThighSensor(/* args */);
//     ~ThighSensor();

//     /* Thigh Sensor attributes */
//     uint16_t temperature;
// 	uint16_t activity;
// 	uint8_t position;

//     /* Thigh Sensor methods */
//     void setTemperature();
//     void setactivity();
//     void setPosition();
// };

// class VulvaSensor : public Sensor
// {
// private:
//     /* data */
// public:
//     VulvaSensor(/* args */);
//     ~VulvaSensor();

//     /* Thigh Sensor attributes */
// 	uint16_t dilation;
// 	uint8_t gap;

//     /* Vulva Sensor methods */
//     void setDilation();
//     void setGap();
// };
