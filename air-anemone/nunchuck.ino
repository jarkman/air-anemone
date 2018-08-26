#include <NintendoExtensionCtrl.h>

Nunchuk nchuk;

long lastNunchuckTime = 0;

void setupNunchuck() {

    noMux();
  nchuk.begin();

  if (!nchuk.connect()) {
    Serial.println("Nunchuk not detected!");
    
  }
  else
  {
    gotNunchuck = true;
    
  }
}

boolean nunchuckIdle()
{
  return millis() - lastNunchuckTime > 20000;
}

void loopNunchuck() {

  if( ! gotNunchuck )
  {
    //Serial.println("no nunchuck");
    return;
  } 
    noMux();
  boolean success = nchuk.update();  // Get new data from the controller

  if (success == true) {  // We've got data!
    nchuk.printDebug();  // Print all of the values!
    joyX = fmap(nchuk.joyX(),0.0,255.0,-1.0,1.0);
    joyY = fmap(nchuk.joyY(),0.0,255.0,-1.0,1.0);
    if( fabs( joyX ) > 0.1 || fabs( joyY ) > 0.1 )
      lastNunchuckTime = millis();
      
    baselinePressureFraction = fmap(nchuk.pitchAngle(), -180.0, 180.0, 0.0, 1.0 );
  }
  else {  // Data is bad :(
    Serial.println("Controller Disconnected!");
    nchuk.reconnect();
    joyX = 0.0;
    joyY = 0.0;
  }
}
