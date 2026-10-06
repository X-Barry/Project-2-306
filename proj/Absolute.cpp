#include "Absolute.h"

#include <bitset>
#include <Arduino.h>

/* Absolute encoder using signals from LED/PT pairs for position sensing of a rotary motor */

// Global Variables
std::bitset<5> bitPosition;

// Constructor for Absolute class, initializes the current position, threshold, and resolution.
Absolute::Absolute(int threshold, int resolution)
{
    this->threshold = threshold;
    this->resolution = resolution;
    this->currentPos = 0.0;
    this->previousPos = 0.0;
    this->displacement = 0.0;
    this->direction = Direction::NONE;
    update();
}

// Starts the process of reading the analog pins, converting to greycode, then to binary, and finally to an angle.
void Absolute::update()
{
    readAnalogPins();
    greyToBinary();
    binaryToAngle();
}

// Reads the voltage value of the analog pins 0-4, compares it with a threshold and creates a 5 bit greycode number (1 for rach pin above threshhold 0 for each pin below threshold).
void Absolute::readAnalogPins()
{
    for (int i = 0; i < 5; i++)
    {
        int pinValue = analogRead(A0 + i);
        if (pinValue > threshold)
        {
            bitPosition.set(i);
        }
        else
        {
            bitPosition.reset(i);
        }
    }
    return;
}

// Converts greycode number to a 5 bit binary number.
void Absolute::greyToBinary()
{
    std::bitset<5> temp = bitPosition;
    while ((temp >>= 1).any())
    {
        bitPosition ^= temp;
    }
    return;
}

// Converts the 5 bit binary number to a 0-360 degree angle. And compares it with intial to detect CW or CCW rotation.
void Absolute::binaryToAngle()
{
    previousPos = currentPos;
    float angleResolution = 360.0 / 2 ^ resolution; // 32 positions for 5 bit binary number
    currentPos = bitPosition.to_ulong() * angleResolution;

    int diff = (previousPos - currentPos + 360) % 360; // 0–359

    if (diff == 0)
    {
        // no movement
        direction = Direction::NONE;
        displacement = 0.0;
    }
    else if (diff < 180)
    {
        // angle increased by diff degrees (e.g. CW)
        direction = Direction::CW;
        displacement = diff;
    }
    else
    {
        // angle decreased by (360 - diff) degrees (e.g. CCW)
        direction = Direction::CCW;
        displacement = 360 - diff;
    }
    return;
}