#include "Absolute.h"

#include <Arduino.h>

/* Absolute encoder using signals from LED/PT pairs for position sensing of a rotary motor */

// Global Variables
uint8_t bitPosition; // one bit per LED/PT pair, bit 0 = A1 ... bit 4 = A5

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

// Reads the voltage value of the analog pins A1-A5, compares it with a threshold and creates a 5 bit greycode number.
// The wheel (wheel_gray5.dxf) is drawn with the WHITE sectors as logic 1, and white reads BELOW the threshold,
// so a pin below threshold sets the bit. A1 is the LSB, A5 the MSB.
void Absolute::readAnalogPins()
{
    for (int i = 0; i < 5; i++)
    {
        int pinValue = analogRead(A1 + i);
        if (pinValue < threshold)
        {
            bitPosition |= (1 << i);
        }
        else
        {
            bitPosition &= ~(1 << i);
        }
    }
    return;
}

// Converts greycode number to a 5 bit binary number.
void Absolute::greyToBinary()
{
    for (uint8_t temp = bitPosition >> 1; temp; temp >>= 1)
    {
        bitPosition ^= temp;
    }
    return;
}

// Converts the 5 bit binary number to a 0-360 degree angle. And compares it with intial to detect CW or CCW rotation.
void Absolute::binaryToAngle()
{
    previousPos = currentPos;
    float angleResolution = 360.0 / (1 << resolution); // 32 positions for 5 bit binary number
    currentPos = bitPosition * angleResolution;

    float diff = fmod(currentPos - previousPos + 360.0, 360.0); // 0–359.99, keeps the fractional degrees

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