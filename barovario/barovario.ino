#include "Wire.h"
#include "I2Cdev.h"
#include "Seeed_BMP280.h" // barometer
#include "SeeedOLED.h"    // display



////////////////////////////////////////////////////////////////////////////////////////////////////
// constants
////////////////////////////////////////////////////////////////////////////////////////////////////

// vertical speed & height
#define HEIGHT_READ_PERIOD 200  // period between two height reads
#define HEIGHT_HIST_SIZE 5
#define SPEED_CALC_T ((HEIGHT_READ_PERIOD * (HEIGHT_HIST_SIZE - 1)) / 1000.0)

// sound
#define SPEAKER_PIN 11

// tones
#define MIN_SINK_TONE 300     // tone played when the minimum threshold for sink is reached
#define MAX_SINK_TONE 100     // tone played when the maximum considered sink threshold is reached
#define SINK_TONE_RANGE (MIN_SINK_TONE - MAX_SINK_TONE)
#define MIN_CLIMB_TONE 400    // tone played when the minimum climb rate threshold is reached
#define MAX_CLIMB_TONE 1500   // tone played when the maximum climb rate threshold is reached
#define CLIMB_TONE_RANGE (MAX_CLIMB_TONE - MIN_CLIMB_TONE)

// tones periods
#define MIN_CLIMB_TONE_PERIOD 40        // period of playing a tone at the maximum considered climb rate
#define MAX_CLIMB_TONE_PERIOD 400       // period of playing a tone at the minimum considered climb rate
#define CLIMB_TONE_PERIOD_RANGE (MAX_CLIMB_TONE_PERIOD - MIN_CLIMB_TONE_PERIOD)
#define SINK_PERIOD 1000                 // period of playing a sink tone (it's a continuous sound, so it just can't be too short) 
#define MAX_PERIOD_BETWEEN_BEEPS 360
#define MIN_PERIOD_BETWEEN_BEEPS 60
#define PERIOD_BETWEEN_BEEPS_RANGE (MAX_PERIOD_BETWEEN_BEEPS - MIN_PERIOD_BETWEEN_BEEPS)  

// climb thresholds
#define MIN_SINK -2.0   // climb rate at which it starts being considered as a sink
#define MAX_SINK -3.5   // below that climb rate sink is not distinguished anymore, because it doesn't matter
#define MIN_CLIMB 0.1   // minimum climb rate which is considered as a climb
#define MAX_CLIMB 4.0   // above that climb rate climb is not distinguisehd anymore

// display
#define DIGIT_ROWS 7
#define DIGIT_COLUMNS 4
#define DISP_CLIMB_X 5          // x postion at which the current climb rate first digit will be displayed
#define DISP_CLIMB_Y 0          // y position at which the current climb rate will be displayed
#define DISP_MINUS_X 1          // if sink
#define DISP_MINUS_Y 4          // if sink
#define HEIGHT_DISP_PERIOD 500
#define DISP_ASL_ROW 0          // row starting at which the ASL altitude will be displayed
#define DISP_ASL_COL 0          // column starting at which the ASL altitude will be displayed
#define AVG_CLIMB_PERIOD 300    // how often avg climb is updated (just on display, not for beep)



////////////////////////////////////////////////////////////////////////////////////////////////////
// display digits
////////////////////////////////////////////////////////////////////////////////////////////////////
/*
  The following arrays represent big digits on the display
 */



const unsigned char zero[DIGIT_ROWS * DIGIT_COLUMNS] = {
  1, 1, 1, 1,
  1, 0, 0, 1,
  1, 0, 0, 1,
  1, 0, 0, 1,
  1, 0, 0, 1,
  1, 0, 0, 1,
  1, 1, 1, 1
};

const unsigned char one[DIGIT_ROWS * DIGIT_COLUMNS] = {
  0, 0, 1, 0,
  0, 1, 1, 0,
  0, 0, 1, 0,
  0, 0, 1, 0,
  0, 0, 1, 0,
  0, 0, 1, 0,
  0, 0, 1, 0
};

const unsigned char two[DIGIT_ROWS * DIGIT_COLUMNS] = {
  1, 1, 1, 1,
  0, 0, 0, 1,
  0, 0, 0, 1,
  1, 1, 1, 1,
  1, 0, 0, 0,
  1, 0, 0, 0,
  1, 1, 1, 1
};

const unsigned char three[DIGIT_ROWS * DIGIT_COLUMNS] = {
  1, 1, 1, 1,
  0, 0, 0, 1,
  0, 0, 0, 1,
  1, 1, 1, 1,
  0, 0, 0, 1,
  0, 0, 0, 1,
  1, 1, 1, 1
};

const unsigned char four[DIGIT_ROWS * DIGIT_COLUMNS] = {
  1, 0, 0, 1,
  1, 0, 0, 1,
  1, 0, 0, 1,
  1, 1, 1, 1,
  0, 0, 0, 1,
  0, 0, 0, 1,
  0, 0, 0, 1
};

const unsigned char five[DIGIT_ROWS * DIGIT_COLUMNS] = {
  1, 1, 1, 1,
  1, 0, 0, 0,
  1, 0, 0, 0,
  1, 1, 1, 1,
  0, 0, 0, 1,
  0, 0, 0, 1,
  1, 1, 1, 1
};

const unsigned char six[DIGIT_ROWS * DIGIT_COLUMNS] = {
  1, 1, 1, 1,
  1, 0, 0, 0,
  1, 0, 0, 0,
  1, 1, 1, 1,
  1, 0, 0, 1,
  1, 0, 0, 1,
  1, 1, 1, 1
};

const unsigned char seven[DIGIT_ROWS * DIGIT_COLUMNS] = {
  1, 1, 1, 1,
  0, 0, 0, 1,
  0, 0, 0, 1,
  0, 0, 0, 1,
  0, 0, 0, 1,
  0, 0, 0, 1,
  0, 0, 0, 1
};

const unsigned char eight[DIGIT_ROWS * DIGIT_COLUMNS] = {
  1, 1, 1, 1,
  1, 0, 0, 1,
  1, 0, 0, 1,
  1, 1, 1, 1,
  1, 0, 0, 1,
  1, 0, 0, 1,
  1, 1, 1, 1
};

const unsigned char nine[DIGIT_ROWS * DIGIT_COLUMNS] = {
  1, 1, 1, 1,
  1, 0, 0, 1,
  1, 0, 0, 1,
  1, 1, 1, 1,
  0, 0, 0, 1,
  0, 0, 0, 1,
  1, 1, 1, 1
};



////////////////////////////////////////////////////////////////////////////////////////////////////
// global variables
////////////////////////////////////////////////////////////////////////////////////////////////////



// devices
I2Cdev   I2C_M;
BMP280 bmp280;  // barometer

// height
float currHeight;
long heightUpdTs;                   // time stamp of the most recent height update
float heightHist[HEIGHT_HIST_SIZE]; // contains 5 last measured heights
int heightHistInsIdx;               // index at which the next history will be inserted

// speed
float speedV; // current vertical speed (climb rate)

// sound
long currToneFinishTs; // timestamp at which the currently played tone will be finished

// display
float lastDisplayedClimb; // time stamp when the most recent climb was displayed
long lastHeightDisp;      // time stamp when the most recent height was displayed



////////////////////////////////////////////////////////////////////////////////////////////////////
// setup
////////////////////////////////////////////////////////////////////////////////////////////////////



void setup() 
{
  Serial.begin(38400);
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  delay(1000);

  // display
  SeeedOled.init();               //initialze SEEED OLED display
  SeeedOled.clearDisplay();       //clear the screen and set start position to top left corner
  SeeedOled.setNormalDisplay();   //Set display to normal mode (i.e non-inverse mode)
  SeeedOled.setPageMode();        //Set addressing mode to Page Mode
  SeeedOled.setBrightness(255);
  
  lastDisplayedClimb = 0.0;
  lastHeightDisp = 0.0;

  // barometer
  if (bmp280.init()) 
    SeeedOled.putString("Barometer OK");
  else
    SeeedOled.putString("Barometer FAIL");

  // height & speed
  currHeight = bmp280.calcAltitude(bmp280.getPressure());
  heightUpdTs = millis();
  for (int i = 0; i < HEIGHT_HIST_SIZE; ++i)
  {
    heightHist[i] = currHeight;
  }
  heightHistInsIdx = 0;
  speedV = 0;

  // sound
  currToneFinishTs = 0;

  delay(2000);
  SeeedOled.clearDisplay();

  displayClimbRate(0.0);
}



////////////////////////////////////////////////////////////////////////////////////////////////////
// loop
////////////////////////////////////////////////////////////////////////////////////////////////////



void loop()
{
  updateHeight();
  updateVerticalSpeedFromBaro();
  playBeep();
  displayCurrClimbRate();
  displayCurrHeight();
}



////////////////////////////////////////////////////////////////////////////////////////////////////
// height
////////////////////////////////////////////////////////////////////////////////////////////////////



void updateHeight()
{
  long diff = millis() - heightUpdTs;
  if (diff >= HEIGHT_READ_PERIOD)
  {
    currHeight = bmp280.calcAltitude(bmp280.getPressure());
    heightHist[heightHistInsIdx] = currHeight;
    heightHistInsIdx = (heightHistInsIdx + 1) % HEIGHT_HIST_SIZE;
    heightUpdTs = millis();
  }
}



/*
 * Returns height regarding which the current vertical speed should be calculated (calculated as 
 * current height minus the result of this function). 
 * This function must be called AFTER update height, because it uses heightInsIdx and at the moment
 * of calling this function heightInsIdx must point to the position on which the NEXT height WILL be
 * inserted.
 */
float getPreviousHeight()
{
  /* 
   *  Example:
   *  Lets assume, that we have a history of size 5 and height was just inserted at index 3, so 
   *  heightHistInsIdx is equalt to 4 now. We are interested in getting the oldest historical value, 
   *  which is stored at index 4, so it is the same as the current value of heightHistInsIdx. 
   *  HeightInsIdx is modulo incremented, so even if the most recent height insert was at the last
   *  position (4), then heightHistInsIdx points to 0. All we have to do is return the historical
   *  value at heightHistInsIdx
   *  
   */
   return heightHist[heightHistInsIdx];
}



////////////////////////////////////////////////////////////////////////////////////////////////////
// vertical speed
////////////////////////////////////////////////////////////////////////////////////////////////////



void updateVerticalSpeedFromBaro()
{
    float heightDiff = currHeight - getPreviousHeight();
    speedV = heightDiff / SPEED_CALC_T; // v = s/t
}



////////////////////////////////////////////////////////////////////////////////////////////////////
// sound
////////////////////////////////////////////////////////////////////////////////////////////////////



void playBeep()
{
  if (millis() > currToneFinishTs)
  {
    int newTone = obtainTone();
    int periodBetweenBeeps; // how much time should pass after the newTone will be played so that the next tone can be played
    int newTonePeriod = obtainTonePeriod(&periodBetweenBeeps);
    playTone(newTone, newTonePeriod);
    currToneFinishTs = millis() + newTonePeriod + periodBetweenBeeps;
  }
}



void playTone(int freq, int period)
{
  tone(SPEAKER_PIN, freq, period);
}



int obtainTonePeriod(int* pPeriodBetweenBeeps)
{
  int period = MAX_CLIMB_TONE_PERIOD;
  *pPeriodBetweenBeeps = 0;

  if (speedV >= MIN_CLIMB)
  {
    float maxClimbCoeff = 1.0 - min(speedV / MAX_CLIMB, 1.0); // the bigger the climb rate, the smaller the coefficient
    period = MIN_CLIMB_TONE_PERIOD + (maxClimbCoeff * (float) CLIMB_TONE_PERIOD_RANGE);
    *pPeriodBetweenBeeps = MIN_PERIOD_BETWEEN_BEEPS + maxClimbCoeff * (float) PERIOD_BETWEEN_BEEPS_RANGE;
  }
  else if (speedV <= MIN_SINK)
  {
    period = SINK_PERIOD;
  }

  return period;
}



int obtainTone()
{
  int toneFreq = 0;

  if (speedV >= MIN_CLIMB)  // sink
  {
    float maxClimbCoeff = min(speedV / MAX_CLIMB, 1.0); // the bigger the climb rate, the bigger the coefficient
    toneFreq = MIN_CLIMB_TONE + (maxClimbCoeff * (float) CLIMB_TONE_RANGE);
  }
  else if (speedV <= MIN_SINK)  // climb
  {
    float maxSinkCoeff = min((MIN_SINK - speedV) / (MIN_SINK - MAX_SINK), 1.0);  // after the max sink is reached it is 1.0, before <0, 1)
    toneFreq = MIN_SINK_TONE - (maxSinkCoeff * SINK_TONE_RANGE);
  }

  return toneFreq;
}



////////////////////////////////////////////////////////////////////////////////////////////////////
// display
////////////////////////////////////////////////////////////////////////////////////////////////////



void printSquare(int fill, int row, int col)
{
  SeeedOled.setTextXY(row, col);
  for (int i = 0; i < 8; i++)
  {
    SeeedOled.sendData(fill);
  }
}



////////////////////////////////////////////////////////////////////////////////////////////////////
// display climb rate
////////////////////////////////////////////////////////////////////////////////////////////////////



void displayCurrClimbRate()
{
   float currClimb = round(speedV * 10) / 10.0;
   if (currClimb != lastDisplayedClimb)
   {
      displayClimbRate(speedV);
      lastDisplayedClimb = currClimb;
   }
}



void displayClimbRate(float climbRate)
{
    int climbIntegerPart = abs((climbRate * 10) / 10);
    int climbFractionPart = abs(((int) (climbRate * 10)) % 10);

    printMinus(climbRate);
    printBigDigit(climbIntegerPart, DISP_CLIMB_X, DISP_CLIMB_Y);
    
    int dotX = DISP_CLIMB_X + DIGIT_COLUMNS + 1;
    int dotY = DISP_CLIMB_Y + DIGIT_ROWS - 1; 
    printSquare(0xFF, dotY, dotX);
    
    int fractionPartX = dotX + 2;
    printBigDigit(climbFractionPart, fractionPartX, DISP_CLIMB_Y);
}



void printMinus(float climbRate)
{
  if (climbRate < 0.0)
  {
    printSquare(0xFF, DISP_MINUS_Y, DISP_MINUS_X);
    printSquare(0xFF, DISP_MINUS_Y, DISP_MINUS_X + 1);
  }
  else
  {
    printSquare(0x0, DISP_MINUS_Y, DISP_MINUS_X);
    printSquare(0x0, DISP_MINUS_Y, DISP_MINUS_X + 1);
  }
}



void printBigDigit(unsigned char digit, int x, int y)
{
  unsigned char* digitArr;

  switch (digit)
  {
    case 0 : digitArr = zero; break;
    case 1 : digitArr = one; break;
    case 2 : digitArr = two; break;
    case 3 : digitArr = three; break;
    case 4 : digitArr = four; break;
    case 5 : digitArr = five; break;
    case 6 : digitArr = six; break;
    case 7 : digitArr = seven; break;
    case 8 : digitArr = eight; break;
    default : digitArr = nine; break; // no more than 9 can be displayed
  }
  
  for (int row = 0; row < DIGIT_ROWS; row++)
  {
    for (int col = 0; col < DIGIT_COLUMNS; col++)
    {
      if (digitArr[row * DIGIT_COLUMNS + col] == 1)
        printSquare(0xFF, y + row, x + col);
      else
        printSquare(0, y + row, x + col);
    }
  }
}



////////////////////////////////////////////////////////////////////////////////////////////////////
// display height
////////////////////////////////////////////////////////////////////////////////////////////////////



void displayCurrHeight()
{
  if (millis() - lastHeightDisp > HEIGHT_DISP_PERIOD)
  {
    SeeedOled.setTextXY(DISP_ASL_ROW, DISP_ASL_COL);
    SeeedOled.putString("ASL: ");
    SeeedOled.setTextXY(DISP_ASL_ROW + 1, DISP_ASL_COL);
    SeeedOled.putNumber(currHeight);
    
    lastHeightDisp = millis();
  }
}
