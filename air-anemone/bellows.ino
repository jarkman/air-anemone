#include "bellows.h"


// Servo pos for valve open/close
#define SERVO_CLOSE_DEGREES 50.0 //20.0
#define SERVO_OPEN_DEGREES 100.0// 150.0

#define DRIVE_GAIN -1.0

// values tuned for the tentacle valve servos - 130/550
#define SERVOMIN  130 //150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  550 //600 // this is the 'maximum' pulse length count (out of 4096)

BME280 bme280Atmospheric;
BME280 bme280Airbox;

int muxAddressAtmospheric = 4;
int muxAddressAirbox = 3;

void setupFixedPressures()
{
    
  muxSelect(muxAddressAtmospheric);
  bme280Atmospheric.setI2CAddress(0x76);
  if (! bme280Atmospheric.beginI2C() )
  {
    Serial.println("Failed to read BME280 for atmospheric");
  }
  
  muxSelect(muxAddressAirbox);
  bme280Airbox.setI2CAddress(0x76);
  if (! bme280Airbox.beginI2C() )
  {
    Serial.println("Failed to read BME280 for airbox");
  }
}

boolean goodPressure( float pressure )
{
  return pressure > 80000.0; 
}

void readFixedPressures()
{
  muxSelect(muxAddressAtmospheric);
  atmosphericAbsPressure = bme280Atmospheric.readFloatPressure();
  if( tracePressures ) { Serial.print(" atmosphericAbsPressure "); Serial.println(atmosphericAbsPressure);}

  
  muxSelect(muxAddressAirbox);
  airboxAbsPressure = bme280Airbox.readFloatPressure();

  if( tracePressures ) { Serial.print(" airboxAbsPressure "); Serial.println(airboxAbsPressure);}


  if( goodPressure(airboxAbsPressure) && goodPressure(atmosphericAbsPressure))
    baselinePressure = (airboxAbsPressure - atmosphericAbsPressure) * baselinePressureFraction * breatheFraction;
  else
    baselinePressure = 1000.0 * baselinePressureFraction * breatheFraction;

  if( tracePressures ) { Serial.print(" baselinePressure "); Serial.println(baselinePressure);}

}


Bellows::Bellows( int _n, int _muxAddress, int _inflateServo, int _deflateServo )
{
  n = _n;

  muxAddress = _muxAddress;
  inflateServo = _inflateServo;
  deflateServo = _deflateServo;
  error = -2;
  targetPressure = 0.0;
  currentPressure = 0.0;
  frustration = 0;
}

void Bellows::setup()
{
  
  muxSelect(muxAddress);
  bme280.setI2CAddress(0x76);
  if (! bme280.beginI2C() )
  {
    Serial.print("n "); Serial.print(n);
    Serial.println("Failed to read BME280 for bellows");
    delay(1000);
  }

 
/*
  
  delay(300);
  yield();
  setDrive(-1.0);
  delay(300);
  yield();
  setDrive(0);

  */
}

void Bellows::loop()
{

  muxSelect(muxAddress);
  float pressure = bme280.readFloatPressure();

  if( tracePressures ) { Serial.print("n "); Serial.print(n);  Serial.print(" pressure "); Serial.println(pressure);}

  if( ! goodPressure( pressure ) || ! goodPressure( atmosphericAbsPressure ))
  {
    error = targetPressure > baselinePressure/5.0 ? -1.0 : 1.0 ; // crude rule so we still have some motion with no pressure sensors 

    if( trace )
    {
        Serial.print("n "); Serial.print(n);
        Serial.print(" targetPressure "); Serial.println(targetPressure);
        Serial.print(" fallback error "); Serial.println(error);
     }   
  }
  else
  {
    //Serial.println(pressure);
    //Serial.println(atmosphericAbsPressure);
    pressure = pressure - atmosphericAbsPressure;
  
    // a bit of smoothing
    currentPressure = 0.9 * currentPressure + 0.1 * pressure;
    
    error = (targetPressure - currentPressure)/baselinePressure;
  }
  
  if( trace ){Serial.print("targetPressure "); Serial.println(targetPressure);}
  if( trace ){Serial.print("currentPressure "); Serial.println(currentPressure);}
  if( trace ){Serial.print("error fraction "); Serial.println(error);}
  if( trace ){Serial.print("frustration "); Serial.println(frustration);}

  if( fabs(error) < FRUSTRATION_LIMIT )
    frustration = 0;
  else
    frustration += error * loopSeconds;
      
  // simple linear feedback
  setDrive( error * DRIVE_GAIN);
  
}

void Bellows::setDrive( float d ) // -1.0 to deflate, 1.0 to inflate
{
  drive = fconstrain( d, -1.0, 1.0 );
  
  if( drive > 0.0 ) //increase pressure
  {
    driveServoAngle(inflateServo, drive);
    driveServoAngle(deflateServo, 0.0);
  }
  else // decrease pressure
  {
    driveServoAngle(inflateServo, 0.0);
    driveServoAngle(deflateServo, -drive);
  }
  
}



void Bellows::driveServoAngle(int servoNum, float openFraction)
{

  if( trace )
    {Serial.print("openFraction "); Serial.println(openFraction);}

  float servoAngle = fmap(openFraction, 0.0, 1.0, SERVO_CLOSE_DEGREES, SERVO_OPEN_DEGREES);
  
  float pulseLen = fmap( servoAngle, 0, 180, SERVOMIN, SERVOMAX ); // map angle to pulse length in PWM count units

  noMux();
  
  pwm.setPWM(servoNum, 0, pulseLen);
  

}

int printOneBellows( int y, int fh, Bellows*b )
{
  oled.print(twodigits(b->currentPressure/1000.0));

  oled.print(">");
  oled.print(twodigits(b->targetPressure/1000.0));
  
  y += fh; 
  oled.setCursor(0,y); 


  oled.print("dr ");
  oled.print(twodigits( b->drive * 100.0));

  y += fh; 
  oled.setCursor(0,y); 
  
  return y;
}

void  printBellows(  )
{
  int fh = oled.getFontHeight();
  int y = 0;

  oled.setCursor(0,y);

  /*oled.print("Bellows:");
  
  y += fh;
   
  oled.setCursor(0,y); 
  */
  if( enableBellows )
  {
    for( Bellows b : bellows )
      y = printOneBellows( y, fh, &b );
    
  } 
 
}





