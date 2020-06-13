#include "Wire.h"
#include "I2Cdev.h"
#include "Seeed_BMP280.h" // barometer
#include "SeeedOLED.h"    // display



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

// display
#define DIGIT_ROWS 7
#define DIGIT_COLUMNS 4
#define DISP_CLIMB_X 5          // x postion at which the current climb rate first digit will be displayed
#define DISP_CLIMB_Y 0          // y position at which the current climb rate will be displayed
#define DISP_MINUS_X 1          // if sink
#define DISP_MINUS_Y 4          // if sink
#define HEIGHT_DISP_PERIOD 1000
#define DISP_ASL_ROW 0
#define DISP_ASL_COL 0
#define AVG_CLIMB_PERIOD 300    // how often avg climb is updated



////////////////////////////////////////////////////////////////////////////////////////////////////
// display digits
////////////////////////////////////////////////////////////////////////////////////////////////////



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
BMP280 bmp280;              // barometer

// height & speed
float heightSum;            // during data gathering period height is summed to calculate avg later
int numOfMeasures;          // how many measures were taken during the data gathering period
long lastDataGatherTime;    // time at which the last data gather took place 
float speedV;               // current vertical speed
float lastHeight;           // avg height from the previous data gathering
bool lastClimbState;
// avg climb for display
float avgClimb;
float climbSum;
int numOfClimbMeasures;
long lastAvgClimbUpdate;

// sound
float toneTimeStamp;

// display
float lastDisplayedClimb;
long lastHeightDisp;        // time stamp when height was updated


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
  climbSum = 0.0;
  avgClimb = 0.0;
  numOfClimbMeasures = 0;
  lastAvgClimbUpdate = millis();

  // barometer
  if (bmp280.init()) 
    SeeedOled.putString("Barometer OK");
  else
    SeeedOled.putString("Barometer FAIL");

  // height & speed
  lastDataGatherTime = millis();
  lastHeight = bmp280.calcAltitude(bmp280.getPressure());
  speedV = 0;
  heightSum = 0;
  numOfMeasures = 0;

  // sound
  toneTimeStamp = millis();
  lastClimbState = CLIMB_STATE_NEUTRAL;

  delay(2000);
  SeeedOled.clearDisplay();

  displayClimbRate(0.0);
}



////////////////////////////////////////////////////////////////////////////////////////////////////
// main
////////////////////////////////////////////////////////////////////////////////////////////////////



void loop()
{
  updateVerticalSpeedFromBaro();
  updateAvgClimb();
  playBeep();
  displayCurrClimbRate();
  displayCurrHeight();
}



////////////////////////////////////////////////////////////////////////////////////////////////////
// vertical speed
////////////////////////////////////////////////////////////////////////////////////////////////////



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



////////////////////////////////////////////////////////////////////////////////////////////////////
// avg climb
////////////////////////////////////////////////////////////////////////////////////////////////////



void updateAvgClimb()
{
  climbSum += speedV;
  numOfClimbMeasures++;
  
  if (millis() - lastAvgClimbUpdate >= AVG_CLIMB_PERIOD)
  {
    avgClimb = climbSum / (float) numOfClimbMeasures;
    climbSum = 0.0;
    numOfClimbMeasures = 0;
    lastAvgClimbUpdate = millis();
  }
}



////////////////////////////////////////////////////////////////////////////////////////////////////
// sound
////////////////////////////////////////////////////////////////////////////////////////////////////



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
   float currClimb = round(avgClimb * 10) / 10.0;
   Serial.println(currClimb);
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
    SeeedOled.putNumber(lastHeight);
    
    lastHeightDisp = millis();
  }
}
