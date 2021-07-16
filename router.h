/**
 * router.h
 *
 *	Birth Alert Router's application
 *
 *  Created on: 07 de jul. de 2021
 *      Author: Gilberto Araujo
 */

#include <stdint.h>

#define HYGRO_SENSOR    0
#define THIGH_SENSOR    1
#define VULVA_SENSOR    2

#define THIGH_SENSOR_TYPE		0x03
#define VULVA_SENSOR_TYPE		0x02
#define HYGRO_SENSOR_TYPE		0x01

#define SENSOR_TYPE_POS         2

/* Thigh sensor data fields */
#define THIGH_BATTERY_POS		3
#define THIGH_TEMPERATURE_POS	4
#define THIGH_ACTIVITY_POS		6
#define THIGH_POSITION_POS		8

#define THIGH_BATTERY_LEN		1
#define THIGH_TEMPERATURE_LEN	2
#define THIGH_ACTIVITY_LEN		2
#define THIGH_POSITION_LEN		1

/* Vulva sensor data fields */
#define VULVA_BATTERY_POS		3
#define VULVA_DILATION_POS		4
#define VULVA_GAP_POS			6

#define VULVA_BATTERY_LEN		1
#define VULVA_DILATION_LEN		2
#define VULVA_GAP_LEN			2


class Sensor
{
private:
    /* data */
public:
    Sensor(/* args */);
    ~Sensor();

    /* Sensor attributes */
    char name [32];
    char addr [18];
    char time [32];
};

class ThighSensor : public Sensor
{
private:
    /* data */
public:
    ThighSensor(/* args */);
    ~ThighSensor();

    /* Thigh Sensor attributes */
    uint8_t battery;
    uint16_t temperature;
	uint16_t activity;
	uint8_t position;
};

class VulvaSensor : public Sensor
{
private:
    /* data */
public:
    VulvaSensor(/* args */);
    ~VulvaSensor();

    /* Thigh Sensor attributes */
    uint8_t battery;
    uint16_t temperature;
	uint16_t dilation;
	uint8_t gap;
};
