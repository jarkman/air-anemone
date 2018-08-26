
#include "Adafruit_MCP23017.h"

Adafruit_MCP23017 mcp;

#define PIRS 3
boolean pir[PIRS];
float pirActivity[] = {0.0, 0.0, 0.0}; // 0.0 to 1.0
float pirAngle[] = { 60, 180, 300 }; // degrees clockwise from bellows 0


void setupPir()
{
  noMux();
  mcp.begin();      // use default address 0
  for( int p = 0; p < PIRS; p++)
  {
    mcp.pinMode(p, INPUT);
    //mcp.pinMode(p, OUTPUT);
    // mcp.pullUp(p, LOW);
  }
}

boolean s = false;
void loopPir()
{

  s = ! s;

  if( tracePirs ) Serial.print("PIRs");
  
  noMux();
  for( int p = 0; p < PIRS; p++)
  {
    // mcp.digitalWrite(p, s);
     
    pir[p] = mcp.digitalRead(p);

    if( tracePirs )
    {
    Serial.print(p);
    Serial.print(":");
    Serial.print(pir[p]);
    Serial.print(" ");
    }
    
    if( pir[p] )
      pirActivity[p]=1.0; // instant response
    else
      pirActivity[p] = 0.9*pirActivity[p]; // slow decay

   
   
  }

  if( tracePirs )
    Serial.println("");
   
  float totalActivity = 0.0;
  float maxActivity = 0.0;
  
  for( int p = 0; p < PIRS; p++)
  {
    totalActivity += pirActivity[p];
    if( pirActivity[p] > maxActivity )
      maxActivity = pirActivity[p];
  }  

  for( int p = 0; p < PIRS; p++)
    moveAttention( pirAngle[p], pirActivity[p], totalActivity );

  attentionAmount = attentionAmount * 0.9 + maxActivity * 0.1; 

  
}

float normaliseAngle(float a)
{
  while( a > 360.0 )
    a -= 360.0;
   while( a < 0.0 )
    a += 360.0;

   return a;
}

float angleDelta( float from, float to )
{
  // return clockwise angle from from to to, in the range -180 to 180
  float d = to - from ;
  while( d > 180.0 )
    d -= 180.0;
   while( d < -180.0 )
    d += 180.0;

   return d;
}

void moveAttention( float toAngle, float byActivity, float totalActivity )
{
  // adjust attention direction weighted by activity
  attentionAngle = attentionAngle + byActivity * angleDelta(attentionAngle, toAngle);
  attentionAngle = normaliseAngle( attentionAngle ); // convert to range 0->360

}

void  printPirs(  )
{
  int fh = oled.getFontHeight();
  int y = 0;

  oled.setCursor(0,y);

  oled.print("PIRs:");
  
  y += fh;
   
  oled.setCursor(0,y); 

  for( int p = 0; p < PIRS; p++)
  {
    oled.print(p);
    
    if( pir[p] )
      oled.print(" * ");
    else
      oled.print(" - ");

    oled.print(twodigits( pirActivity[p] * 100.0));  
    oled.print("%");
    
    y += fh;
   
    oled.setCursor(0,y); 
  }

  oled.print(threedigits(attentionAngle));

  oled.print("d ");

  oled.print(twodigits( attentionAmount * 100.0));  
  oled.print("%");
 
}

