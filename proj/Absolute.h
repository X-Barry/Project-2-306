/*
   Header file for the Absolute class, which provides functionality to read analog pin
   values, convert them to a greycode number, and then convert that number to a binary
   representation and finally to a degree angle.
*/

enum class Direction
{
   CW = 1,
   CCW = -1,
   NONE = 0
};

class Absolute
{
public:
   // Constructor
   Absolute(int threshold, int resolution) {};
   // Getters
   float getAngle()
   {
      return currentPos;
   }
   float getDisplacement()
   {
      return displacement;
   }
   )
   Direction getDirection()
   {
      return direction;
   }

private:
   int threshold;
   int resolution;
   float currentPos;
   float previousPos;
   float displacement;
   Direction direction;
   void readAnalogPins();
   void greyToBinary();
   void binaryToAngle();
}