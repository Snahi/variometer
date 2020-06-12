#include "Wire.h"
#include "I2Cdev.h"
#include "Seeed_BMP280.h" // barometer



////////////////////////////////////////////////////////////////////////////////////////////////////
// global variables
////////////////////////////////////////////////////////////////////////////////////////////////////
#define DATA_GATHERING_PERIOD 128.0

// sound
#define SPEAKER_PIN 3

// tones
#define MAX_SINK_TONE 200     // tone played when the maximum considered threshold is played
#define BASE_SINK_TONE 400    // tone played when the sink threshold is reached
#define MIN_SNIFF_TONE 600    // tone played when threshold of sniffing is reached
#define MIN_CLIMB_TONE 700    // tone played when the minimum climb rate threshold is reached
#define MAX_CLIMB_TONE 1200   // tone played when the maximym climb rate threshold is reached

// climb thresholds
#define MIN_SINK -2.0
#define MIN_CLIMB 0.1
#define MAX_CLIMB 4.0

// tones durations (in percentages of sound playing cycle)
#define TONE_PERIOD 400
#define CLIMB_TONE_DURATION 300

// climb states
#define CLIMB_STATE_SINK -1
#define CLIMB_STATE_NEUTRAL 0
#define CLIMB_STATE_CLIMB 1

////////////////////////////////////////////////////////////////////////////////////////////////////
// global variables
////////////////////////////////////////////////////////////////////////////////////////////////////
// devices
I2Cdev   I2C_M;
BMP280 bmp280;              // barometer

// height & speed
float heightSum;            // during data gathering period height is summed to calculate avg later
int numOfMeasures;          // how many measures were taken during the data gathering period
long lastDataGatherTime;    // time at which the last data gather took place 
float speedV;               // current vertical speed
float lastHeight;           // avg height from the previous data gathering
bool lastClimbState;

// sound
float toneTimeStamp;



////////////////////////////////////////////////////////////////////////////////////////////////////
// setup
////////////////////////////////////////////////////////////////////////////////////////////////////



void setup() 
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // initialize serial communication
  Serial.begin(38400);
  delay(1000);

  // barometer
  if (bmp280.init()) 
    Serial.println("Barometer ok");
  else
    Serial.println("Barometer fail");

  // height & speed
  lastDataGatherTime = millis();
  lastHeight = bmp280.calcAltitude(bmp280.getPressure());
  speedV = 0;
  heightSum = 0;
  numOfMeasures = 0;

  // sound
  toneTimeStamp = millis();
  lastClimbState = CLIMB_STATE_NEUTRAL;
}



////////////////////////////////////////////////////////////////////////////////////////////////////
// main
////////////////////////////////////////////////////////////////////////////////////////////////////



void loop()
{
  updateVerticalSpeedFromBaro();
  playBeep();
  Serial.println(floor(speedV * 10) / 10.0);
}



void updateVerticalSpeedFromBaro()
{
  float height = bmp280.calcAltitude(bmp280.getPressure());
  heightSum += height;
  numOfMeasures += 1;
  long diff = millis() - lastDataGatherTime;
  if (diff >= DATA_GATHERING_PERIOD)
  {
    float newHeight = heightSum / numOfMeasures;
    speedV = (newHeight - lastHeight) / (diff / 1000.0);
    numOfMeasures = 0;
    heightSum = 0;
    lastHeight = newHeight;
    lastDataGatherTime = millis();
  }
}



void playBeep()
{
  int currClimbState = obtainClimbState();
  
  if (currClimbState != lastClimbState || millis() - toneTimeStamp >= TONE_PERIOD)
  {
    if (speedV >= MIN_CLIMB)  // if climbs
    {
      float climbRangeRatio = min(speedV, MAX_CLIMB) / MAX_CLIMB;
      int toneRange = MAX_CLIMB_TONE - MIN_CLIMB_TONE;
      int toneFreq = MIN_CLIMB_TONE + (climbRangeRatio * (float) toneRange);

      tone(SPEAKER_PIN, toneFreq, CLIMB_TONE_DURATION);
      lastClimbState = currClimbState;
    }
    
    toneTimeStamp = millis();
  }
}



int obtainClimbState()
{
  if (speedV <= MIN_SINK)
    return CLIMB_STATE_SINK;
  else if (speedV >= MIN_CLIMB)
    return CLIMB_STATE_NEUTRAL;
  else
    return CLIMB_STATE_NEUTRAL;
}
