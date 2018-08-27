#ifndef BELLOWS_H
#define BELLOWS_H

#include <Adafruit_BMP280.h>

class Bellows
{
  public:
  Bellows( int n,  int _muxAddress, int _inflateServo, int _deflateServo );
  void driveServoAngle();
  void driveServoAngle(int servoNum, float openFraction);
  void setDrive( float drive );
  void incrementTarget( float delta );
  void incrementTargetFromPosition( float delta );
  void setup();
  void loop();

  float targetPressure;
  float currentPressure;
  float error; // -1 to 1
  float frustration; // integral of recent error, zeroed when we are on-target

  int n;

  float drive;
  
  private:
  
  int muxAddress;
  int inflateServo;
  int deflateServo;
  Adafruit_BMP280 bmp280;
};
#endif

