#include "bellows.h"


// Servo pos for valve open/close
#define SERVO_CLOSE_DEGREES 50.0 //20.0
#define SERVO_OPEN_DEGREES 130.0// 150.0

#define DRIVE_GAIN -8.0

// values tuned for the tentacle valve servos - 130/550
#define SERVOMIN  130 //150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  550 //600 // this is the 'maximum' pulse length count (out of 4096)

Adafruit_BMP280 bmp280Atmospheric;
Adafruit_BMP280 bmp280Airbox;

int muxAddressAtmospheric = 7;
int muxAddressAirbox = 1;

void startBmp280( Adafruit_BMP280 *b)
{

  if (! b->begin(0x76)) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        
    }
/*
    b->setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,  // temperature
                    Adafruit_BMP280::SAMPLING_X16, // pressure
                    Adafruit_BMP280::SAMPLING_X1,  // humidity
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_0_5 );
                    */



}

void setupFixedPressures()
{
    
  muxSelect(muxAddressAtmospheric);
  startBmp280(&bmp280Atmospheric);
  
  
  muxSelect(muxAddressAirbox);

  startBmp280(&bmp280Airbox);

}

boolean goodPressure( float pressure )
{
  return pressure > 80000.0; 
}

void readFixedPressures()
{
  muxSelect(muxAddressAtmospheric);
  atmosphericAbsPressure = bmp280Atmospheric.readPressure();
  if( tracePressures ) { Serial.print(" atmosphericAbsPressure "); Serial.println(atmosphericAbsPressure);}

  
  muxSelect(muxAddressAirbox);
  airboxAbsPressure = bmp280Airbox.readPressure();

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
  startBmp280(&bmp280);

/*
  Serial.println("Close");
   driveServoAngle(inflateServo, 0.0);
    driveServoAngle(deflateServo, 0.0);

     delay(1000);
    yield();
    delay(1000);
    yield();
    delay(1000);
    yield();

    Serial.println("Open");
  driveServoAngle(inflateServo, 1.0);
    driveServoAngle(deflateServo, 1.0);

 delay(1000);
    yield();
    delay(1000);
    yield();
    delay(1000);
    yield();

  Serial.println("Deflate");
  yield();
  setDrive(-1.0);
    delay(1000);
    yield();
    delay(1000);
    yield();
    delay(1000);
    yield();

  Serial.println("Inflate");

  yield();
  setDrive(-1);
    delay(1000);
        delay(1000);
    yield();
    delay(1000);
    yield();
    delay(1000);
    yield();
    */


}

void Bellows::loop()
{

  muxSelect(muxAddress);
  float pressure = bmp280.readPressure();

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
    
    error = (currentPressure - targetPressure)/baselinePressure;
  }

  drive = error * DRIVE_GAIN;
  
  if( traceBellows ){Serial.print("targetPressure "); Serial.println(targetPressure);}
  if( traceBellows ){Serial.print("currentPressure "); Serial.println(currentPressure);}
  if( traceBellows ){Serial.print("error fraction "); Serial.println(error);}
  if( traceBellows ){Serial.print("drive "); Serial.println(drive);}

  if( fabs(error) < FRUSTRATION_LIMIT )
    frustration = 0;
  else
    frustration += error * loopSeconds;
      
  // simple linear feedback
  setDrive( drive);
  
}

void Bellows::setDrive( float d ) // -1.0 to deflate, 1.0 to inflate
{
  drive = fconstrain( d, -1.0, 1.0 );
  float inflateDrive = drive; //fconstrain( d, 0.60, 1.0 ); // never allow inflate servo to close, it stalls
  float deflateDrive = - drive;
  /*
  if( drive > 0.0 )
    deflateDrive = 0.0;
  else
    deflateDrive = - drive;
   */ 

  if( drive > 0.0 ) //increase pressure
  {
    driveServoAngle(inflateServo, inflateDrive);
    driveServoAngle(deflateServo, 0);
  }
  else // decrease pressure
  {
    driveServoAngle(inflateServo, 0);
    driveServoAngle(deflateServo, deflateDrive);
  }
  
}



void Bellows::driveServoAngle(int servoNum, float openFraction)
{

  openFraction = fconstrain( openFraction,0.0, 1.0 );

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





