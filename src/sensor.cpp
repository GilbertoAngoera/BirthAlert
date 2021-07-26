/**
 * sensor.h
 *
 *	Birth Alert Router's application
 *
 *  Created on: 07 de jul. de 2021
 *      Author: Gilberto Araujo
 */

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include <sensor.h>

/* Thigh Sensor methods */
ThighSensor::ThighSensor(/* args */) {}
ThighSensor::~ThighSensor() {}

void ThighSensor::setName(String str)
{
    this->name = str;
}
void ThighSensor::setAddress(String str)
{
    this->address = str;
}
void ThighSensor::setBattery(uint8_t val)
{
    this->battery = val;
}
String ThighSensor::getName()
{
    return this->name;
}
String ThighSensor::getAddress()
{
    return this->address;
}
uint8_t ThighSensor::getBattery()
{
    return this->battery;
}

void ThighSensor::setTemperature(uint16_t val)
{
    this->temperature = val;
}

void ThighSensor::setActivity(uint16_t val)
{
    this->activity = val;
}

void ThighSensor::setPosition(uint8_t val)
{
    this->position = val;
}
uint16_t ThighSensor::getTemperature()
{
    return this->temperature;
}

uint16_t ThighSensor::getActivity()
{
    return this->activity;
}

uint8_t ThighSensor::getPosition()
{
    return this->position;
}

void ThighSensor::setTimeStamp(String str)
{
    this->timeStamp = str;
}

String ThighSensor::getTimeStamp()
{
    return this->timeStamp;
}

void ThighSensor::pushToQueue(ThighSensorData data)
{
    this->queue.push(data);
}

void ThighSensor::removeFromQueue()
{
    this->queue.pop();
}

ThighSensorData ThighSensor::getFromQueue()
{
    return this->queue.front();
}

/* Vulva Sensor methods */
VulvaSensor::VulvaSensor(/* args */) {}
VulvaSensor::~VulvaSensor() {}

HygroSensor::HygroSensor(/* args */) {}
HygroSensor::~HygroSensor() {}
