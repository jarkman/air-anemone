#include <NintendoExtensionCtrl.h>


// This runs the air anemone with 2 servo-controlled air valves,
// taking user input from a wii nunchuck,
// with pressure feeback from BME280 pressure sensors in each chamber
// a tca9548a i2c multiplexer on the Adafruit breakout
// a MCP23017 GPIO to read PIR signals
// and a Adafruit PWMServoDriver board 

// We use Polulu libraries for the LSM303 because they use a lot less memory than the Adafruit libraries

// Code lives in https://github.com/jarkman/air-anemone

// It runs on a Wemos D1 mini with an oled screen


// Wiring:

// "Boot from Flash" requires some pin state to be right:

// D3/GPIO0 = 1 (high)
// D4/GPIO2 = 1 (high)
// D8/GPIO15 = 0 (low)

// so we'd better use these as outputs

// Pins are
//  I2C
//    D1 SCL
//    D2 SDA
//      wired to OLED and to mux and to servo board
// PS2 uses D3 D4 D6 D8
// Neopixels are on D6
// Encoder is on D5 & D7, switch on D0, pulling to ground

// or, in order:
// D0 Encoder switch
// D1 SCL
// D2 SDA
// D3 Neopixels 0  via CD40109BE A p3 -> E p4   http://www.ti.com/lit/ds/symlink/cd40109b.pdf
// D4 Neopixels 1                B p6 -> F p5
// D5 Encoder A    
// D6 Neopixels 2                C  p10 -> G p11
// D7 Encoder B
// D8 - 


// Each node has one compass sensor and either two or zero rangers

// i2c addresses:
// Oled   : 0x3C
// Adafruit_PWMServoDriver: 0x40
// wii nunchuck: 0x52
// MCP23017 GPIO: 0x20                   // http://ww1.microchip.com/downloads/en/DeviceDoc/20001952C.pdf

// Mux    : 0x70, 0x71
// and beyond the mux:
//  LSM303 : 0x19 & 0x1E
//  BMP280: 0x76


// Mux channels:
// 0: bellows 0  BMP280 and low LSM303
// 1: bellows 1  BMP280 and high LSM303
// 2: bellows 2  BMP280
// 3: airbox pressure BMP280
// 4: atmospheric pressure BMP280 (on control board)



#include "Node.h"
#include <Wire.h>
//#include <Servo.h>
#include <SFE_MicroOLED.h>  // Include the SFE_MicroOLED library
#include <Adafruit_PWMServoDriver.h>

#define FRUSTRATION_LIMIT 0.01 // error value above which frustration accumulates


#include "bellows.h"

boolean trace = false;          // activity tracing for finding crashes - leave off to get better response times
boolean traceBehaviour = true;
boolean traceNodes = false;
boolean tracePressures = false;
boolean tracePirs = false;

boolean enableBellows = true;  // turn on/off bellows code
boolean enableBehaviour = true;
boolean calibrateCompasses = false; // turn on then rotate each compass smoothly about all axes to get the individual compass min/max values for compass setup



void setupNodes();
void loopNodes();
void setupEncoder();



#define PIN_RESET 255  //
#define DC_JUMPER 0  // I2C Addres: 0 - 0x3C, 1 - 0x3D

MicroOLED oled(PIN_RESET, DC_JUMPER);  // I2C Example

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();



#define BELLOWS 3
// servo numbers on the PWM servo driver
Bellows bellows[] = {Bellows(0,                    0, 0, 1)};



float attentionAngle = 0.0; // net attention direction, degrees clockwise from bellows 0
float attentionAmount = 0.0; // 0.0 to 1.1


// UI screens accessible via encoder
#define UI_STATES 3
long uiState = 1;

char report[80];

float bendTargetX = 0.0; // -1.0 to 1.0
float bendTargetY = 0.0;

boolean gotNunchuck = false;

float joyX = 0.0; // -1.0 to 1.0
float joyY = 0.0;
float baselinePressureFraction = 1.0;
float breatheFraction = 1.0;

long breatheStartT = 0;
float breathePeriod = 20000.0; // in millis
float breatheAmplitude = 0.2;

float airboxAbsPressure = 0.0; //101000.0 + 1500.0; // Pa, about what we expect from our blower
float atmosphericAbsPressure = 0.0; //101000.0; // Pa
float baselinePressure = 1000.0;

float wavePeriod = 10000.0; // in millis
float waveFraction = 0.5; // 0 to 1.0

float waveAmplitude = 1.0; // 0 to 1.0
long waveStartT = 0;


float loopSeconds = 0.1; // duration of our loop() in seconds, used for normalising assorted constants

// pose targets for a boot-time wriggle selftest
#define SELFTEST_MILLIS 10000 //10000
#define NUM_SELFTEST 5
// target pressure ratios for chambers
float selftest[NUM_SELFTEST][3] = {{1.0,1.0,1.0},{1.0,0.0,0.0},{0.0,1.0,0.0},{0.0,0.0,1.0},{1.0,1.0,1.0}};
int nextSelftest = -1;
long selftestStartMillis = -1;

void setupI2C()
{
  Wire.begin();
  Wire.setClock(10000);  // 10k for a 10m wire length limit
}



void setup() {
  setupDisplay();
  
  delay(2000);
  Serial.begin(115200);


  if( trace ) Serial.println("---Setup---");
  if( trace ) Serial.println("..i2c");
  setupI2C();

  setupPir();


  setupFixedPressures();
  
  setupServoDriver();

  setupEncoder();

  setupLeds();

  setupNunchuck();

  setupWifi();
  
  //baseServo.attach(D7); 
  //tipServo.attach(D8); 

  for( int b = 0; b < BELLOWS; b ++ )
  {
    Bellows *bellow = &(bellows[b]);
    bellow->setup();
  }
  waveStartT = millis();
  breatheStartT = millis();

  
}

void setupServoDriver()
{
  noMux();
  
  pwm.begin();
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
}


void loopBreathe()
{
  // modulate overall pressure up & down a bit ihn a slow sine
  float theta = 2 * PI * (millis() - breatheStartT)/breathePeriod;

   
  breatheFraction = 1.0 - breatheAmplitude * 0.5 * sin(theta);
 
}

float wave(float phaseOffset) // -1.0 to 1.0
{
  float theta = 2 * PI * (millis() - waveStartT)/wavePeriod;

   
  return waveAmplitude * sin(theta + phaseOffset);
  
}

boolean loopSelftest()
{
  if( nextSelftest >= NUM_SELFTEST )
    return false; // selftest done


  float error = 0;
  for( int b = 0; b < BELLOWS; b ++ )
  {
    Bellows *bellow = &(bellows[b]);
  
    error += fabs( bellow->error);
  }
  
  if( nextSelftest < 0 ||  // first time round the loop
    error < 0.1 ||          // arrived at target pose
    (selftestStartMillis > 0 && millis() - selftestStartMillis > SELFTEST_MILLIS) ) // time has been too long, must be broken
  {
    //move on to next pose
    Serial.print("Starting selftest pose ");
    Serial.println(nextSelftest);
    nextSelftest ++;
    selftestStartMillis = millis();
    
    if( nextSelftest >= NUM_SELFTEST )
      return false; // selftest done

    int i = 0;
    for( int b = 0; b < BELLOWS; b ++ )
    {
      Bellows *bellow = &(bellows[b]);
      bellow->targetPressure = baselinePressure * selftest[nextSelftest][i++];  
    }
    
  }
  return true;
}

boolean loopManual()
{
  if( ! gotNunchuck )
    return false;

  if( nunchuckIdle())
    return false;
  
  // TODO - return false when nunchuck idle
  setInflateAmount(1.0+0.5*joyX);

  return true;
}


void setInflateAmount(float x)
{
  // at 0,0, set all three to nominal pressure
  for( int b = 0; b < BELLOWS; b ++ )
  {
    Bellows *bellow = &(bellows[b]);

    bellow->targetPressure = baselinePressure * (0.2 + 0.8*x);
  }


}

void loopWave()
{
   /*
  //float controlVal = analogRead(controlPotPin);   

  // waveFraction sets the amount of autowave we are going to add to the stick motion
  //float waveFraction = fmap(controlVal, 0.0,905.0, 0.0, 1.0); // magic numbers for control pot

  float effectiveWaveFraction = waveFraction;

  long stickDelay = millis() - lastStickMoveMillis; // how long since we last wiggled the stick ?
  if( stickDelay > 0 && stickDelay < 20000 )
    effectiveWaveFraction *= (float) (stickDelay + 2000) / (20000.0 + 2000.0); // fade back in over 20 secs, but always keep 10%

  //Serial.println(effectiveWaveFraction);
  
  float tipPhaseDelay = - fmap( effectiveWaveFraction, 0.0, 1.0, - PI / 2.0, 0.3 ); // 90 degrees delay at small amplitude, for a wiggle, no delay at large amplitude, for a whole-tentacle curl

  //wavePeriod = fmap( waveFraction, 0.0, 1.0, 10000.0, 20000.0 );


  float tipDrive = stickLeftX * (1.5 * stickLeftY - 0.5) + stickRightX;
  
  baseBellows.target(stickLeftX +  effectiveWaveFraction * wave(0));
  tipBellows.target(tipDrive +  effectiveWaveFraction * wave(tipPhaseDelay));
  */

}
void loop() {

  long start = millis();

  loopBreathe();
  loopWifi();
  loopPir();
  
  loopEncoder();
  loopDisplay();
  loopLeds();

  loopNunchuck();
 

  //delay(100);

  //return;

  readFixedPressures();
  
  
  if( trace ) Serial.println("---Loop---");
  if( trace ) Serial.println("selftest/wave"); 
  if( ! loopManual())
    if( ! loopSelftest())
    
      if( ! loopBehaviour())
        loopWave();


  for( int b = 0; b < BELLOWS; b ++ )
    {
      Bellows *bellow = &(bellows[b]);
      bellow->loop();
    }

  long end = millis();
  if( trace ){ Serial.print("loop took ") ; Serial.println( end-start );}

  loopSeconds = 0.001 * (float)( end-start );
  

   delay(50);
}



float fmapConstrained(float x, float in_min, float in_max, float out_min, float out_max)
{
  float f = fmap( x,  in_min, in_max, out_min, out_max);

  if( f < out_min )
    f = out_min;

  if( f > out_max )
    f = out_max;

  return f;
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


float fconstrain(float f, float out_min, float out_max)
{
  if( f < out_min )
    f = out_min;

  if( f > out_max )
    f = out_max;

  return f;
}

// https://learn.adafruit.com/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout/wiring-and-test
#define TCAADDR0 0x70
#define NO_MUX 16

 
void muxSelect(uint8_t i) {
  uint8_t m0;
  
  if( i== NO_MUX )
  {
    // disable both
    m0 = 0;
    
  }
  else
  {
    
    m0 = 1 << i;
 
  }
 
 
  Wire.beginTransmission(TCAADDR0);
  Wire.write(m0);
  Wire.endTransmission(); 
  
 
}

// disable mux 
void noMux()
{
  muxSelect( NO_MUX );
}



