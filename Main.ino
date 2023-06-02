#include <cgAnem.h>
#include <Arduino.h>//base arduino library (probably not needed)
#include <Wire.h>//i2c transmission library
#include <LiquidCrystal.h>//display
#include <HX711.h>//load cell
#include <Adafruit_Sensor.h>//adafruit library, common for all unified sensors
#include <Adafruit_BMP085_U.h>//barometer
#include <EEPROM.h> //this is used to store values when running load cell calibration
#include <Servo.h> //for the motor, need to use this for PWM output

#define ENC_A 3 //encoder A, Green, pin 3
#define ENC_B 2 //encoder B, white, pin 2
#define ENC_SW 4 //encoder button

#define TCAADDR 0x70 //address of the first multiplexer. there is also one at 0x71 but there is a function to scan multiple multiplexers, it just needs a starting point

//LoadCell pins have to be pwm to fake i2c like the hx711 boards do
#define Load1_DAT 6
#define Load1_CLK 7
#define Load2_DAT 8
#define Load2_CLK 9

//i2c sensor definitions
ClimateGuard_Anem cgAnem(ANEM_I2C_ADDR); //Anemometer Definition
Adafruit_BMP085_Unified bmp1 = Adafruit_BMP085_Unified(1); //all the barometers
Adafruit_BMP085_Unified bmp2 = Adafruit_BMP085_Unified(2);
Adafruit_BMP085_Unified bmp3 = Adafruit_BMP085_Unified(3);
Adafruit_BMP085_Unified bmp4 = Adafruit_BMP085_Unified(4);
Adafruit_BMP085_Unified bmp5 = Adafruit_BMP085_Unified(5);
Adafruit_BMP085_Unified bmp6 = Adafruit_BMP085_Unified(6);
Adafruit_BMP085_Unified bmp7 = Adafruit_BMP085_Unified(7);

//motor setup, motor controller is dumb and doesn't need to be powered to init the arduino side of things.
byte motorPin = 5; // signal pin for the ESC.
Servo motor;
//int for motorGo;
int motorCommand;
int motorTarget = 0; //needed for motor governor, used in ramp function, target is desired final speed
int motorCurrent = 0; //lags target by ramp speed* loops needed to equal target
bool motorInitComplete = 0; //locks out motorInit after it's been run.

//barometer readings need to be initalized, as well as whatever averaging we do, as well as barometer select, because that needs to be specified before you request sensor data
int baroAvg = 0; //average of actual sensor data
int baro0 = 0; //actual sensor data
int baro1 = 0;
int baro2 = 0;
int baro3 = 0;
int baro4 = 0;
int baro5 = 0;
int baro6 = 0;
uint8_t baroSelect = 0; //select from 0-6 to pick a barometer. always do this first before calling baroRead

//Load Cell Amplifiers
HX711 LoadCell_1;
HX711 LoadCell_2;

float calibration_factor_1 = -1200; //these should be overwritten with eeprom reads but in case they arent these are close for 1kg load cells
float calibration_factor_2 = -1200;
long loadCellGo1Reading = 0; //this is the actual reading of the load cell used to drive the mass toward user-entered mass to calibrate load cells
long loadCellGo2Reading = 0; //same for 2
long loadCell1Reading = 0; //actual reading, note the force vector points opposite the load cell arrow
long loadCell2Reading = 0;
uint8_t loadcellprecision1 = 0; //this is the speed of adjustment modifier. goes 0 for inc. 1000, 1 = 100, 2 = 10, 3 = satified
uint8_t loadcellprecision2 = 0;

//encoder
bool encoder_CCW = 0; //this flag gets raised when the rotation changes, and when checked, should be reset if it's 1.
bool encoder_CW = 0; //this flag gets raised when the rotation changes, and when checked, should be reset if it's 1.
bool encoder_pulse = 0; //this is used in interactive menus to tell if the encoder has pulsed either direction to trigger a display update if true
static uint8_t counter = 1;      //this variable will be changed by encoder input
static uint8_t oldcounter = 1;  //for comaprison of counter to check if it's increasing or decreasing. used in read_encoder_rot();

const int rs = 53, en = 51, d4 = 49, d5 = 47, d6 = 45, d7 = 43;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7); //for the *lcd.xyz();* commands later

uint8_t addr = 0; //this is the address of the i2c address we're polling. This is used in startup error handler

//failure flags block begin, default to failed unless a subroutine clears the fail
bool i2c_1_missing = 1; //is the first multiplexer missing
//bool i2c_2_missing = 0; //we didn't end up needing two multiplexers
bool barofail_1 = 1;
bool barofail_2 = 1;
bool barofail_3 = 1;
bool barofail_4 = 1;
bool barofail_5 = 1;
bool barofail_6 = 1;
bool barofail_7 = 1;
bool anemfail_notfound = 1;
//load cells can't be checked. the good ol "look at the screen and poke them a bit" method is going to have to suffice. in addition, they should drift a few grams with any slight temp change. anything pinned at 0 for more than 5 minutes or so is disconnected
bool state_trouble = 0; //this is the summary flag. this gets set if any sensor errors are found on bootup. it can be cleared but it will have to be cleared by clearing the above flags to 0

//fake sensor values here. these are needed to give something for the displays to display when there are issues reading sensors. Allows for disabling of sensor libraries as well
int fakebaro1 = -1;
int fakebaro2 = -1;
int fakebaro3 = -1;
int fakebaro4 = -1;
int fakebaro5 = -1;
int fakebaro6 = -1;
int fakebaro7 = -1;
int fakeanemometer = -1;
int fakeLoadCell_1 = -1;
int fakeLoadCell_2 = -1;

bool ignoreLoadCell_1 = 0;//for debug
bool ignoreLoadCell_2 = 0;//for debug
//failure flags block ends

//flow control flags, determines which displayUpdate we do
bool startupComplete = 0; //if 0, check for missing sensors and show error flags if they're not there
bool main_interactive = 0;
bool run_interactive = 0;
bool serial_interactive = 0;
bool sensorstats_interactive = 0;
bool calibrate_interactive = 0;
bool baud_interactive = 0;
bool barostats_interactive = 0;
bool anemstats_interactive = 0;
bool loadstats_interactive = 0;
bool liveadjustv_interactive = 0; //only triggered by run menu
bool aboutMenu = 0;
bool active = 0; //this flag gets raised by pressing start. Will kick off the dominoes, figuratively. This will start the speed governor, data dumper, and the run menu display mode.

//flow control, main menu flags for mainMenuInteractive()
bool mainInteractiveFirstRun = 0; //this is the flag that tells us whether this loop is the first time you've been in main interactive. allows us to do some setup one time
bool main_hover_1 = 0; //in the actual menu one of these will always be 1, but we'll handle that in the actual logic
bool main_hover_2 = 0;
bool main_hover_3 = 0;
bool main_hover_4 = 0;
bool main_hover_5 = 0;
bool main_hover_6 = 0;
bool main_hover_7 = 0;

//visible flags are display windows. we can only display 4 lines at a time so this is the logic flag to dodge around which line range we can see
bool main_visible_1to4 = 0; //similar to the hover, one of these will always be 1, but we're doing that logic in the function.
bool main_visible_2to5 = 0;
bool main_visible_3to6 = 0;
bool main_visible_4to7 = 0;

//flow control, run menu flags for runMenuInteractive()
bool runInteractiveFirstRun = 0; //this is the flag that tells us whether this loop is the first time you've been in main interactive. allows us to do some setup one time
bool run_hover_1 = 0; //in the actual menu one of these will always be 1, but we'll handle that in the actual logic
bool run_hover_2 = 0;
bool run_hover_3 = 0;
bool run_hover_4 = 0;
/** //we dont need the rest of the bools because at the moment the run menu doesnt need to scroll. Only 4 options.
  bool run_hover_5 = 0;
  bool run_hover_6 = 0;

  //visible flags are display windows. we can only display 4 lines at a time so this is the logic flag to dodge around which line range we can see
  bool run_visible_1to4 = 0; //similar to the hover, one of these will always be 1, but we're doing that logic in the function.
  bool run_visible_2to5 = 0;
  bool run_visible_3to6 = 0;
*/
//run data entry, we ask user for input when they first set active == 1
bool runDataEntryComplete = 0; // this checks time and velo flags, only 1 if both are 1
bool timeDataEntryComplete = 0;
bool veloDataEntryComplete = 0;
bool trialDataEntryComplete = 0; //this flag checks if the user has completed entering data for the number of trials
bool warmupComplete = 0; //this checks to see that the warmup function completed
//run data entry display velo and time
int velocityTarget = 0; // target velocity for run routine
int currentVelocity = 0; //this is the current velocity anemometer reading without decimals.
int runTime = 0; //target runtime for run routine
int originalRunTime = 0; //stores data entry in non changing variable to reset runTime when
int numTrials = 1; //how many trials do we want to run? Minimum 1, this counts down from user value
int numTrialsObserved = 1; //how many trials have we run so far? This value simply increments up when runTime increments down.
int runTimeDisplaySeconds = 0; //is modulo of runTime, IE how many seconds will be left over if you subtract all whole minutes from it
int runTimeDisplayMinutes = 0; //using left over seconds above, number of whole minutes to display as an input time
int trialTimeEstimate = 0;
int trialTimeDisplayHours = 0;
int trialTimeDisplayMinutes = 0;

//flow control, menu flags for serial interface submenu
bool serialInteractiveFirstRun = 0;
bool serial_hover_1 = 0;
bool serial_hover_2 = 0;
bool serial_hover_3 = 0;
bool serial_hover_4 = 0;

//flow control, menu for sensor statistics
bool sensorStatsInteractiveFirstRun = 0;
bool sensorstats_hover_1 = 0;
bool sensorstats_hover_2 = 0;
bool sensorstats_hover_3 = 0;
bool sensorstats_hover_4 = 0;

//flow control, for viewing barostats (sub of sensorstats)
bool baroStatsInteractiveFirstRun = 0;
bool barostats_hover_1 = 0;
bool barostats_hover_2 = 0;
bool barostats_hover_3 = 0;
bool barostats_hover_4 = 0;
bool barostats_hover_5 = 0;
bool barostats_hover_6 = 0;
bool barostats_hover_7 = 0;
bool barostats_hover_8 = 0;
bool barostats_visible_1to4 = 0;
bool barostats_visible_2to5 = 0;
bool barostats_visible_3to6 = 0;
bool barostats_visible_4to7 = 0;
bool barostats_visible_5to8 = 0;

bool anemStatsInteractiveFirstRun = 0; //only one line for anemometer
bool loadStatsInteractiveFirstRun = 0; //and one for load, same code structure

//flow control for calibrate menu
bool calibrateInteractiveFirstRun = 0;
bool calibrate_hover_1 = 0;
bool calibrate_hover_2 = 0;
bool calibrate_hover_3 = 0;
//no 4th, we dont have that many things in need of calibrate routines
//and for loadCellDataEntry
bool loadCellDataEntryComplete = 0;
bool firstDigitComplete = 0;
bool secondDigitComplete = 0;
bool thirdDigitComplete = 0;
bool loadCellCalibrate1_firstrun = 0; //this is a convenient place to put firstrun flags
bool loadCellCalibrate2_firstrun = 0;
bool loadCellGo1Active = 0; //shortcut flag to load cell driving routine
bool loadCellGo2Active = 0;

//for handling cases that switch if the current trial is the first trial, must have some flags for various subsystems
bool firstTrialComplete = 0;
bool dataDumperFirstRun = 1; //checks to see if this is the first measurement (aka first row containing values) and if so, the corresponding function will print out a trial counter, otherwise will do nothing

//flow control, menu flags for serial interface change baud rate menu
bool baudInteractiveFirstRun = 0;
bool baud_hover_1 = 0;
bool baud_hover_2 = 0;
bool baud_hover_3 = 0;
bool baud_hover_4 = 0;
bool baud_hover_5 = 0;
bool baud_hover_6 = 0;
bool baud_hover_7 = 0;
bool baud_hover_8 = 0;
bool baud_hover_9 = 0;
bool baud_hover_10 = 0;
bool baud_hover_11 = 0;
bool baud_hover_12 = 0;
bool baud_hover_13 = 0; //yup this leads to an absolutely enormous menu system
bool baud_visible_1to4 = 0;
bool baud_visible_2to5 = 0;
bool baud_visible_3to6 = 0;
bool baud_visible_4to7 = 0;
bool baud_visible_5to8 = 0;
bool baud_visible_6to9 = 0;
bool baud_visible_7to10 = 0;
bool baud_visible_8to11 = 0;
bool baud_visible_9to12 = 0;
bool baud_visible_10to13 = 0;

//sensor select flow control
bool sensorSelectBaro = 0;
bool sensorSelectBaroBegin = 0;
bool sensorSelectLoadCell = 0;
bool sensorSelectLoadCellBegin = 0;
bool sensorSelectComplete = 0;
bool baroyes = 1; //by defualt, barometer is selected. modified by sensor select
bool loadcellyes = 1; //by default, load cell is selected

//delays and other timing params
int clickDelay = 225; //delay after click to next screen in ms, this is a real delay because user needs time to have their thumb bounce off the switch
long updateDelay = 1000; //main menu or run menu update pulse delay. since wind tunnel is multitasking, this is a state machine span parameter
long updateDelayRun = 2000; //slower update loop for run menu
long updateDelaySensorStats = 300; //update rate for sensorstats, 300 is flickering a bit, 400 is clear.
long runTimeTicker = 1000; //every 1000 ms, subtract 1 second from the screen counter
long motorRampSlope = 500; //if this value is small, the motor ramp is fast, if it is large it is slow. a gentlr speed seems to be about 500 ms. used in state machine
unsigned long previousTime = 0; //state machine time storage variable, unsigned because it lags millis(); by 1000 ms steps
unsigned long previousTime2 = 0; //needed another one
unsigned long previousTime3 = 0;
unsigned long runTimeTickerPrevious = 0; //this is for the countdown state machine for the runTime clock when showing the run menu
unsigned long motorRampSlopePrevious = 0; //tracked in the state machine to determine when the next motor ramp step should be
unsigned long runStartMillis = 0; //this tracks the milli time at sensor dump init. this is needed for offset for accurate time in sensordumper
unsigned long sensorDumperMillis = 0; //this is needed to be able to "pause" millis while in submenus, so as to hide the time discrepancy from the csv log
unsigned long sensorResumeMillis = 0; //this is the time discrepancy between the sensor dumper time and the real time, will be corrected when exiting the interactive menu
unsigned long sensorDumperDelay = 1000; //this delays the sensor dumper loop to allow the microcontroller not to choke

//Sensor Timings
//Anemometer Timings
long updateDelayAnem = 1000; //main menu display update delay
unsigned long previousTimeAnem = 0;
long loadCellTestTime = 0; //this is used to check if a load cell is lagging when trying to read. Taking advantage of the fact that the library tries to raise a load cell for 3 seconds and then drops it.

//serial baud rate flag, 115200 by default
long serialBaudRate = 115200; //serial.begin doesn't like getting fed a long, so this variable is just displaying the baud rate. it's changed by the baud sub of serial sub of main or run

//load cell calibration
long loadcellcalibratedmass = 0;

//limits
//these are designed to be changed by an override by using the wimbdy easter egg, wimbdy is a debug mode and the values set there might be a bit excessive for normal operation. Change these at your peril
int motorpwmlimit = 1700; //upper limit motor pwm value
int timelimitrun = 300; //upper time limit (in seconds)
int velocityTargetLimit = 9;//upper speed limit for velocityTarget
bool wimbdyDebug = 0; //debug flag, to be used to bring us out of research mode. enables diagnostic serial prints and other stuff

void setup()
{
  motor.write(0); //keep the motor stopped DO NOT DELETE EVER
  lcd.begin(20, 4);
  Serial.begin(115200);

  //lcd.print("Hello World!!!!!!!!!abcdefghijklmnop----@@@@@@@@@@@@@@#######");

  //sensor init block
  //anemometer init
  //Serial.print("Anemometer Setup");
  //Serial.println();
  if (cgAnem.init()) //try to init the sensor module
  {
    //Serial.println("Sensor successfully found");
  }
  uint8_t sensorChipId = cgAnem.getChipId(); /*Returns chip id, default value: 0x11.*/
  //Serial.print("Chip id: 0x");
  //Serial.println(sensorChipId, HEX);

  uint8_t firmWareVer = cgAnem.getFirmwareVersion(); /*Returns firmware version.*/
  //Serial.print("Firmware version: ");
  //Serial.println(firmWareVer);
  //Serial.println("-------------------------------------");
  //Serial.println("try to set the duct area for volumetric flow calculations");
  cgAnem.set_duct_area(100); //set duct area for volumetric flow calculation in cm^2. If duct area not set cgAnem.airConsumption will be -255 (default value)
  //Serial.println("Duct area set as " + String(cgAnem.ductArea) + " cm^2");


  //anemometer init finished
  //load cell init
  EEPROM.get(0, calibration_factor_1); //get eeprom values for calibration factor out of storage
  EEPROM.get(8, calibration_factor_2);

  LoadCell_1.begin(Load1_DAT, Load1_CLK); //if there are short wires but no actual load cell amps connected this has lagged the program before as it tries to establish library comms with the devices
  LoadCell_1.set_scale(calibration_factor_1);
  LoadCell_1.tare(); //Reset the LoadCell_1 to 0
  LoadCell_2.begin(Load2_DAT, Load2_CLK);
  LoadCell_2.set_scale(calibration_factor_2);
  LoadCell_2.tare(); //Reset the LoadCell_1 to 0

  long zero_factor_1 = LoadCell_1.read_average(); //Get a baseline reading
  //Serial.print("Zero factor_1: "); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
  //Serial.println(zero_factor_1);

  long zero_factor_2 = LoadCell_2.read_average(); //Get a baseline reading
  //Serial.print("Zero factor_2: "); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
  //Serial.println(zero_factor_2);

  //Barometer init, will freeze program if missing
  Wire.begin();

  /* Initialise the 1st sensor */
  tcaselect(0); //if you're here because tcaselect was not declared in this scope theres a curlybracket problem somewhere... godspeed
  if (!bmp1.begin())
  {
    /* There was a problem detecting the bmp1085 ... check your connections */
    //Serial.print("Ooops, no bmp085 detected ... Check your wiring or I2C ADDR!");
    //while (1);
  }

  /* Initialise the 2nd sensor */
  tcaselect(1);
  if (!bmp2.begin())
  {
    /* There was a problem detecting the bmp1085 ... check your connections */
    //Serial.print("Ooops, no bmp085 detected ... Check your wiring or I2C ADDR!");
    //while (1);
  }

  /* Initialise the 3rd sensor */
  tcaselect(2);
  if (!bmp3.begin())
  {
    /* There was a problem detecting the bmp1085 ... check your connections */
    //Serial.print("Ooops, no bmp085 detected ... Check your wiring or I2C ADDR!");
    //while (1);
  }
  /* Initialise the 4th sensor */
  tcaselect(3);
  if (!bmp4.begin())
  {
    /* There was a problem detecting the bmp1085 ... check your connections */
    //Serial.print("Ooops, no bmp085 detected ... Check your wiring or I2C ADDR!");
    //while (1);
  }
  /* Initialise the 5th sensor */
  tcaselect(4);
  if (!bmp5.begin())
  {
    /* There was a problem detecting the bmp1085 ... check your connections */
    //Serial.print("Ooops, no bmp085 detected ... Check your wiring or I2C ADDR!");
    //while (1);
  }
  /* Initialise the 6th sensor */
  tcaselect(5);
  if (!bmp6.begin())
  {
    /* There was a problem detecting the bmp1085 ... check your connections */
    //Serial.print("Ooops, no bmp085 detected ... Check your wiring or I2C ADDR!");
    //while (1);
  }
  /* Initialise the 7th sensor */
  tcaselect(6);
  if (!bmp7.begin())
  {
    /* There was a problem detecting the bmp1085 ... check your connections */
    //Serial.print("Ooops, no bmp085 detected ... Check your wiring or I2C ADDR!");
    //while (1);
  }
  //Baro init end

  //splash screen display routine

  lcd.setCursor(0, 0);
  lcd.print("                    "); //print 20 spaces, effectively clears the line
  lcd.setCursor(0, 1);
  lcd.print(" WindOS Version 1.2 ");
  lcd.setCursor(0, 2);
  lcd.print(" By Kameron Markham ");
  lcd.setCursor(0, 3);
  //lcd.print("                    "); //print 20 spaces, effectively clears the line

  motor.attach(motorPin); //setup the motor pin ahead of time. Doesn't matter if the ESC isn't active yet, and will start motor with motorInit()



  /* Setup encoder pins as inputs */
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  //Serial.println("Start"); //all prints have been disabled if theyre not part of the data dumper to keep it simple for people using this on multiple platforms
  delay(100);
  //easter egg, if you hold the switch on startup you get an extra line. This is also a way to get into debug mode
  if ((digitalRead(ENC_SW) == LOW)) {
    lcd.print(" [it heckin WIMBDY] ");
    //debug mode flag overrides
    motorpwmlimit = 1900; //upper limit motor pwm value, doesnt do anything over 1900
    timelimitrun = 7200; //upper time limit (in seconds)
    velocityTargetLimit = 30;//upper speed limit for velocityTarget, unattainable if above 24.5 by sensor. realistically, unknown limit at this time with the titan 5000 series
    //motorRampSlope = 500; //if this value is small, the motor ramp is fast, if it is large it is slow. a gentlr speed seems to be about 50 ms. used in state machine
    //end debug overrides
    wimbdyDebug = 1;
    delay(2000);
  }

  //static uint8_t oldcounter = 0; //defining this for the encoder direction detection function
  delay(2000);
}

void(* resetFunc) (void) = 0; //declare reset function @ address 0, this will be called by running resetFunc();

void loop()
{
  //delay(1000); //enable to make it think slower for sane debugging at loop speed
  //Serial.println(millis()); //this will allow us to calculate the loop speed, but makes the thing horrible to use and lags everything out
  //end debug
  if (anemfail_notfound == 0) {
    if (cgAnem.data_update()) // the prints don't have to happen but the function needs to run or the anemometer doesn't pass data
    {
      //Serial.println("Air flow rate: " + String(cgAnem.airflowRate) + " m/s");
      //Serial.println("Current temperature: " + String(cgAnem.temperature) + " C");
      //Serial.println("Volumetric Flow Rate:" + String(cgAnem.airConsumption) + " m^3/hour");
      if (cgAnem.unsteadyProcess) {
        //Serial.println("the sensor is in unstable process and not stabilised");
      }
      else {
        //Serial.println("the sensor is stabilised");
        //do nothing
      }
    }
    else
      checkCgAnem();
  }
  //encoder block start
  int8_t tmpdata;
  /**/
  tmpdata = read_encoder();
  if ( tmpdata ) {
    //Serial.print("Counter value: ");
    //Serial.println(counter, DEC);
    counter += tmpdata;
    read_encoder_rot(); //check rotation only if a new counter value is available
  }
  //end encoder block

  if (active == 0) {//this is all the main stuff. run stuff needs this flag set to 1

    //if button is pressed while on the main menu, flip interactive variable to true
    if ((main_interactive == 0) && (startupComplete == 1) && (digitalRead(ENC_SW) == LOW)) { //the main menu display update conditions
      main_interactive = 1;
      //encoder_CW = 0; // no input yet so if set earlier reset it
      //encoder_CCW = 0;
      delay(clickDelay);
    }
    //main interactive and submenu control logic, important to lock this out if the submenus are active otherwise can't move the cursor
    if (main_interactive == 1 && (serial_interactive == 0) && (calibrate_interactive == 0) && sensorstats_interactive == 0) {
      mainMenuUserInput(); //menu logic (main)
    }
    if ((serial_interactive == 1) && (baud_interactive == 0)) {
      serialMenuUserInput(); //serial menu logic
    }
    if (sensorstats_interactive == 1 && barostats_interactive == 0 && anemstats_interactive == 0 && loadstats_interactive == 0) { //multiple lockouts for the submenus
      sensorStatsMenuUserInput();
    }
    if (barostats_interactive == 1) {
      if (millis() - previousTime2 >= updateDelaySensorStats) {
        previousTime2 = millis(); // first thing to do is updating the snapshot of time
        // time for timed action
        baroStatsMenuUserInput();
      }
    }
    if (anemstats_interactive == 1) {
      if (millis() - previousTime2 >= updateDelaySensorStats) {
        previousTime2 = millis(); // first thing to do is updating the snapshot of time
        // time for timed action
        anemStatsDisplay();
      }
    }
    if (loadstats_interactive == 1) {
      if (millis() - previousTime2 >= updateDelaySensorStats) {
        previousTime2 = millis(); // first thing to do is updating the snapshot of time
        // time for timed action
        loadStatsDisplay();
      }
    }


    if (baud_interactive == 1) {
      baudMenuUserInput();
    }
    if (calibrate_interactive == 1) {
      calibrateMenuUserInput();
    }
    if (main_interactive == 1) {
      previousTime = millis(); //locks the update loop out to stop menu flashing in submenus
    }
    //update screen by updateDelay if we're not in main_interactive OR RUN_INTERACTIVE in future
    if (millis() - previousTime >= updateDelay) {
      previousTime = millis(); // first thing to do is updating the snapshot of time
      // time for timed action
      displayUpdate(); //updates display
    }
  } //end of inactive section
  //begin run logic
  if (active == 1) {
    if ((trialDataEntryComplete == 1) && (runDataEntryComplete == 1) && (motorInitComplete == 1)) { //after data entry and data entry kicking off motor init, then run:
      //will always run as long as motorInit is done, data entry is done, and tunnel is active.
      motorGovernor();

      //if button is pressed while on the run menu, flip interactive variable to true
      if ((run_interactive == 0) && (digitalRead(ENC_SW) == LOW)) { //the run menu display update conditions
        run_interactive = 1;
        //encoder_CW = 0; // no input yet so if set earlier reset it
        //encoder_CCW = 0;
        delay(clickDelay);
      }
      //run interactive and submenu control logic, important to lock this out if the submenus are active otherwise can't move the cursor
      if (run_interactive == 1 && (liveadjustv_interactive == 0) && (sensorstats_interactive == 0)) {
        runMenuUserInput(); //menu logic (run)
      }
      if (sensorstats_interactive == 1 && barostats_interactive == 0 && anemstats_interactive == 0 && loadstats_interactive == 0) { //multiple lockouts for the submenus
        sensorStatsMenuUserInput();
      }
      if (barostats_interactive == 1) {
        if (millis() - previousTime2 >= updateDelaySensorStats) {
          previousTime2 = millis(); // first thing to do is updating the snapshot of time
          // time for timed action
          baroStatsMenuUserInput();
        }
      }
      if (anemstats_interactive == 1) {
        if (millis() - previousTime2 >= updateDelaySensorStats) {
          previousTime2 = millis(); // first thing to do is updating the snapshot of time
          // time for timed action
          anemStatsDisplay();
        }
      }
      if (loadstats_interactive == 1) {
        if (millis() - previousTime2 >= updateDelaySensorStats) {
          previousTime2 = millis(); // first thing to do is updating the snapshot of time
          // time for timed action
          loadStatsDisplay();
        }
      }
      if (liveadjustv_interactive == 1) {
        liveAdjustV();
      }

      if (run_interactive == 1) {
        previousTime = millis(); //locks the update loop out to stop menu flashing in submenus
      }
      //update screen by updateDelay if we're not in run_interactive
      if (millis() - previousTime >= updateDelayRun) { //different update delay flag for run loop, adjust for performance
        previousTime = millis(); // first thing to do is updating the snapshot of time
        // time for timed action
        displayUpdate(); //updates display
      }
      /**
        if (warmupComplete == 1) { //raised at start of warmup
        motorGovernor(); //loop to drive motor

        }
      */
    }//end of if rundataentry == 1
    if (runDataEntryComplete == 0) { //shortcut to get back to the data entry section
      runDataEntry();
    }
    if ((trialDataEntryComplete == 0) && (runDataEntryComplete == 1)) {//shortcut to get back to the data entry section
      trialDataEntry();
    }
    if ((trialDataEntryComplete == 1) && (runDataEntryComplete == 1) && (run_interactive == 0)) { //only tick the clock down or run sensor dumper if user is not in submenus

      if (millis() - previousTime3 >= sensorDumperDelay) {
        previousTime3 = millis(); // first thing to do is updating the snapshot of time
        // time for timed action
        sensorDumperMillis = millis() - sensorResumeMillis; //only update after correcting for time discrepancy from being in submenu
        sensorDumper(); //this writes each time div (of sensorDumperDelay) of sensor values to the serial port
      }
      //sensor dumper state machine




      if ((runTime > 0) && ((millis() - runTimeTickerPrevious >= runTimeTicker))) { //counts millis to determine whole number seconds
        runTime = (runTime - 1); //subtract 1 second from runtime, tick down the clock
        runTimeTickerPrevious = millis();
      }
      if (runTime <= 0) { // tunnel has finished running
        motor.writeMicroseconds(0); //stop
        active = 0; //set tunnel active to 0 to stop the loop from restarting the motor in motorGovernor
        motorTarget = 1200; //so the ramp and governor knows where to start as opposed to going immediately to whatever speed the target is
        motorCurrent = 1200; //correspondingly, don't make motorgovernor try to ramp to a different speed until program instructs in the usual place
        numTrialsObserved++; //increment to make trial counter count up
        numTrials--;

        /**
                //begin diagnostic prints. must determine machine state to reset for new trial
                //menu flow control
                Serial.println(); //blank line to avoid the sensor dumper
                Serial.print("startupComplete=");
                Serial.println(startupComplete); //if 0, check for missing sensors and show error flags if they're not there
                Serial.print("main_interactive=");
                Serial.println(main_interactive);
                Serial.print("run_interactive=");
                Serial.println(run_interactive);
                Serial.print("serial_interactive=");
                Serial.println(serial_interactive);
                Serial.print("sensorstats_interactive=");
                Serial.println(sensorstats_interactive);
                Serial.print("calibrate_interactive=");
                Serial.println(calibrate_interactive);
                Serial.print("baud_interactive=");
                Serial.println(baud_interactive);
                Serial.print("barostats_interactive=");
                Serial.println(barostats_interactive);
                Serial.print("anemstats_interactive=");
                Serial.println(anemstats_interactive);
                Serial.print("loadstats_interactive=");
                Serial.println(loadstats_interactive);
                Serial.print("liveadjustv_interactive=");
                Serial.println(liveadjustv_interactive);
                Serial.print("aboutMenu=");
                Serial.println(aboutMenu);
                Serial.print("active=");
                Serial.println(active);

                //velocity and runtime
                Serial.print("velocityTarget=");
                Serial.println(velocityTarget);
                Serial.print("currentVelocity=");
                Serial.println(currentVelocity);
                Serial.print("runTime=");
                Serial.println(runTime);

                //
                Serial.print("runTimeTickerPrevious=");
                Serial.println(runTimeTickerPrevious);
                Serial.print("runStartMillis=");
                Serial.println(runStartMillis);
                Serial.print("sensorDumperMillis=");
                Serial.println(sensorDumperMillis);
                Serial.print("sensorResumeMillis=");
                Serial.println(sensorResumeMillis);

        */
        //Serial.print("baroyes");
        //Serial.println(baroyes);
        //Serial.print("loadcellyes");
        //Serial.println(loadcellyes);

        //end diagnostic prints
        if (numTrials > 0) {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(" Trial(s) complete. ");
          lcd.setCursor(0, 1);
          lcd.print("WindOS will run a   "); //data goes at pos 6
          lcd.setCursor(0, 2);
          lcd.print("new trial in:       "); //pos 6
          lcd.setCursor(0, 3);
          lcd.print("10 seconds.          ");
          lcd.setCursor(10, 3); //begin overwriting the " " on last line from left to right with "." every second (makes it look slick without much effort, hey?)
          delay (1000);
          lcd.print(".");
          delay (1000);
          lcd.print(".");
          delay (1000);
          lcd.print(".");
          delay (1000);
          lcd.print(".");
          delay(1000);
          lcd.print(".");
          delay (1000);
          lcd.print(".");
          delay (1000);
          lcd.print(".");
          delay (1000);
          lcd.print(".");
          delay(1000);
          lcd.print(".");
        }

        //delay(100000); // debug delay to stop it from resetting, normally comment out
        if (numTrials > 0) { //negative is undefined for numTrials
          //rerun, we're going again!
          active = 1;
          runTime = originalRunTime; //reset runtime to rewind the clock
          dataDumperFirstRun = 1; //reset this to print the first measurement note window again
          sensorDumperInit();
        }
        if (numTrials == 0) {
          lcd.setCursor(0, 0);
          lcd.print(" Trial(s) completed.");
          lcd.setCursor(0, 1);
          lcd.print("WindOS will reset in"); //data goes at pos 6
          lcd.setCursor(0, 2);
          lcd.print("5 seconds           "); //pos 6
          lcd.setCursor(0, 3);
          lcd.print("                    ");
          delay(5000);
          resetFunc();
        }
      }
    }
  } //end active section, nothing in here will run if not activated yet
} //end of void loop

void checkCgAnem()
{

  //if (cgAnem.overVcc)
  //Serial.println("the input voltage is too high and Anemometer module is in self-protect mode");
  //else //uncomment if you use the if else
  //Serial.println("Anemometer wiring error");
}

/* returns change in encoder state (-1,0,1) */
int8_t read_encoder()
{
  int8_t enc_states[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
  static uint8_t old_AB = 0;
  /**/
  old_AB <<= 2;                   //remember previous state
  //old_AB |= ( ENC_PORT & 0x03 );  //add current state
  old_AB |= ((digitalRead (ENC_B) << 1) | digitalRead (ENC_A));//add current state
  return ( enc_states[( old_AB & 0x0f )]);
}

void read_encoder_rot() {
  if (counter != oldcounter) {
    if (counter < oldcounter) {
      encoder_CCW = 1;
      //Serial.print("encoder_CCW=");
      //Serial.print(encoder_CCW);
      //Serial.println();
      //Serial.print("CCW");
      //Serial.println();
    }
    if (counter > oldcounter) {
      encoder_CW = 1;
      //Serial.print("encoder_CW=");
      //Serial.print(encoder_CW);
      //Serial.println();
      //Serial.print("CW");
      //Serial.println();
    }
    oldcounter = counter;
  }
}
void warmup () { //switch into active mode as commanded my main_interactive
  active = 1;
  runStartInit ();
  //motorInit(); //actually start the motor, shouldn't do it here except for debugging
  warmupComplete = 1;
}

//below contains main menu functions
void mainMenuUserInput() {
  if (mainInteractiveFirstRun == 1) { //we always start at 1, 1-4
    main_hover_1 = 1;
    main_visible_1to4 = 1;
    mainInteractiveFirstRun = 0; //no longer the first run, so set this flag to not run this loop again until you reset it by navigating to the non-interactive main menu
    //Serial.print("first run!!");
    displayUpdate(); //let the user see the submenu faster
  } // end MainInteractiveFirstRun == 1 if loop
  //debug block flags
  /**
    Serial.print("mainMenuUserInput();");
    Serial.println();
    Serial.print("mainInteractiveFirstRun=");
    Serial.print(mainInteractiveFirstRun); //this is the flag that tells us whether this loop is the first time you've been in main interactive. allows us to do some setup one time
    Serial.println();
    Serial.print("main_hover_1=");
    Serial.print(main_hover_1); //in the actual menu one of these will always be 1, but we'll handle that in the actual logic
    Serial.println();
    Serial.print("main_hover_2=");
    Serial.print(main_hover_2);
    Serial.println();
    Serial.print("main_hover_3=");
    Serial.print(main_hover_3);
    Serial.println();
    Serial.print("main_hover_4=");
    Serial.print(main_hover_4);
    Serial.println();
    Serial.print("main_hover_5=");
    Serial.print(main_hover_5);
    Serial.println();
    Serial.print("main_hover_6=");
    Serial.print(main_hover_6);
    Serial.println();

    //visible flags are display windows. we can only display 4 lines at a time so this is the logic flag to dodge around which line range we can see
    Serial.print("main_visible_1to4=");
    Serial.print(main_visible_1to4); //similar to the hover, one of these will always be 1, but we're doing that logic in the function.
    Serial.println();
    Serial.print("main_visible_2to5=");
    Serial.print(main_visible_2to5);
    Serial.println();
    Serial.print("main_visible_3to6=");
    Serial.print(main_visible_3to6);
    Serial.println();
    Serial.print("encoder_CW=");
    Serial.print(encoder_CW);
    Serial.println();
    Serial.print("encoder_CCW=");
    Serial.print(encoder_CCW);
    Serial.println();
  */
  //end debug block

  if (mainInteractiveFirstRun == 0) { //once we are no longer in first run (skip the logic code the first loop)
    if ((encoder_CW == 1) || (encoder_CCW == 1)) {
      encoder_pulse = 1; //raise the pulse flag
    }
    if (encoder_pulse == 1) {
      //encoder logic block
      if ((encoder_CW == 1) && (main_hover_1 == 1)) {
        main_hover_1 = 0;
        main_hover_2 = 1;
        main_hover_3 = 0;
        main_hover_4 = 0;
        main_hover_5 = 0;
        main_hover_6 = 0;
        main_hover_7 = 0;
        encoder_CW = 0;
      }
      if ((encoder_CW == 1) && (main_hover_2 == 1)) {
        main_hover_1 = 0;
        main_hover_2 = 0;
        main_hover_3 = 1;
        main_hover_4 = 0;
        main_hover_5 = 0;
        main_hover_6 = 0;
        main_hover_7 = 0;
        encoder_CW = 0;
      }
      if ((encoder_CW == 1) && (main_hover_3 == 1)) {
        main_hover_1 = 0;
        main_hover_2 = 0;
        main_hover_3 = 0;
        main_hover_4 = 1;
        main_hover_5 = 0;
        main_hover_6 = 0;
        main_hover_7 = 0;
        encoder_CW = 0;
      }
      if ((encoder_CW == 1) && (main_hover_4 == 1)) {
        main_hover_1 = 0;
        main_hover_2 = 0;
        main_hover_3 = 0;
        main_hover_4 = 0;
        main_hover_5 = 1;
        main_hover_6 = 0;
        main_hover_7 = 0;
        encoder_CW = 0;
      }
      if ((encoder_CW == 1) && (main_hover_5 == 1)) {
        main_hover_1 = 0;
        main_hover_2 = 0;
        main_hover_3 = 0;
        main_hover_4 = 0;
        main_hover_5 = 0;
        main_hover_6 = 1;
        main_hover_7 = 0;
        encoder_CW = 0;
      }
      if ((encoder_CW == 1) && (main_hover_6 == 1)) {
        main_hover_1 = 0;
        main_hover_2 = 0;
        main_hover_3 = 0;
        main_hover_4 = 0;
        main_hover_5 = 0;
        main_hover_6 = 0;
        main_hover_7 = 1;
        encoder_CW = 0;
      }
      if ((encoder_CCW == 1) && (main_hover_7 == 1)) {
        main_hover_1 = 0;
        main_hover_2 = 0;
        main_hover_3 = 0;
        main_hover_4 = 0;
        main_hover_5 = 0;
        main_hover_6 = 1;
        main_hover_7 = 0;
        encoder_CCW = 0;
      }
      if ((encoder_CCW == 1) && (main_hover_6 == 1)) {
        main_hover_1 = 0;
        main_hover_2 = 0;
        main_hover_3 = 0;
        main_hover_4 = 0;
        main_hover_5 = 1;
        main_hover_6 = 0;
        main_hover_7 = 0;
        encoder_CCW = 0;
      }
      if ((encoder_CCW == 1) && (main_hover_5 == 1)) {
        main_hover_1 = 0;
        main_hover_2 = 0;
        main_hover_3 = 0;
        main_hover_4 = 1;
        main_hover_5 = 0;
        main_hover_6 = 0;
        main_hover_7 = 0;
        encoder_CCW = 0;
      }
      if ((encoder_CCW == 1) && (main_hover_4 == 1)) {
        main_hover_1 = 0;
        main_hover_2 = 0;
        main_hover_3 = 1;
        main_hover_4 = 0;
        main_hover_5 = 0;
        main_hover_6 = 0;
        main_hover_7 = 0;
        encoder_CCW = 0;
      }
      if ((encoder_CCW == 1) && (main_hover_3 == 1)) {
        main_hover_1 = 0;
        main_hover_2 = 1;
        main_hover_3 = 0;
        main_hover_4 = 0;
        main_hover_5 = 0;
        main_hover_6 = 0;
        main_hover_7 = 0;
        encoder_CCW = 0;
      }
      if ((encoder_CCW == 1) && (main_hover_2 == 1)) {
        main_hover_1 = 1;
        main_hover_2 = 0;
        main_hover_3 = 0;
        main_hover_4 = 0;
        main_hover_5 = 0;
        main_hover_6 = 0;
        main_hover_7 = 0;
        encoder_CCW = 0;
      }
      if (main_visible_1to4 == 1) {
        if (main_hover_5 == 1) {
          main_visible_2to5 = 1;
          main_visible_1to4 = 0;
          //Serial.print("main_visible_2to5 = 1");
          //Serial.println();
        }
        if (main_hover_6 == 1) {
          main_visible_3to6 = 1;
          main_visible_1to4 = 0;
          //Serial.print("main_visible_3to6 = 1");
          //Serial.println();
        }
        if (main_hover_7 == 1) {
          main_visible_4to7 = 1;
          main_visible_1to4 = 0;
          //Serial.print("main_visible_3to6 = 1");
          //Serial.println();
        }
      }

      if (main_visible_2to5 == 1) {
        if (main_hover_1 == 1) {
          main_visible_1to4 = 1;
          main_visible_2to5 = 0;
          //Serial.print("main_visible_1to4 = 1");
          //Serial.println();
        }
        if (main_hover_6 == 1) {
          main_visible_3to6 = 1;
          main_visible_2to5 = 0;
          //Serial.print("main_visible_3to6 = 1");
          //Serial.println();
        }
        if (main_hover_7 == 1) {
          main_visible_4to7 = 1;
          main_visible_2to5 = 0;
          //Serial.print("main_visible_3to6 = 1");
          //Serial.println();
        }
      }
      if (main_visible_3to6 == 1) {
        if (main_hover_2 == 1) {
          main_visible_2to5 = 1;
          main_visible_3to6 = 0;
          //Serial.print("main_visible_2to5 = 1");
          //Serial.println();
        }
        if (main_hover_1 == 1) {
          main_visible_1to4 = 1;
          main_visible_3to6 = 0;
          //Serial.print("main_visible_1to4 = 1");
          //Serial.println();
        }
        if (main_hover_7 == 1) {
          main_visible_3to6 = 0;
          main_visible_4to7 = 1;
          //Serial.print("main_visible_3to6 = 1");
          //Serial.println();
        }
      }
      if (main_visible_4to7 == 1) {
        if (main_hover_2 == 1) {
          main_visible_2to5 = 1;
          main_visible_4to7 = 0;
          //Serial.print("main_visible_2to5 = 1");
          //Serial.println();
        }
        if (main_hover_1 == 1) {
          main_visible_1to4 = 1;
          main_visible_4to7 = 0;
          //Serial.print("main_visible_1to4 = 1");
          //Serial.println();
        }
        if (main_hover_3 == 1) {
          main_visible_3to6 = 1;
          main_visible_4to7 = 0;
          //Serial.print("main_visible_3to6 = 1");
          //Serial.println();
        }
      }
      encoder_pulse = 0; //reset the pulse flag
      encoder_CW = 0; //if you put in an extra rotate, this will zero it if the move was invalid (out of range), otherwise this results in refreshing the display at speed of loop
      encoder_CCW = 0; //same for other direction
      displayUpdate(); //update the display if and only if the user made a change, and do it after the values have been modified
    } //end if encoder pulse
    //submenu flags, these trigger overrides to get you into deeper menus
    if ((digitalRead(ENC_SW) == LOW)) {
      delay(clickDelay); //allow time for thumb to come off button, important because user will have pressed the button to get into the menu, don't wanna instantly cycle
      //Serial.print("you pressed the button!");
      if (main_hover_1 == 1) {
        //Serial.print("return to main menu from main_interactive");
        //Serial.println();
        main_interactive = 0; //return to the main, labeled "Info Screen"
        displayUpdate(); //does one update, lets the user see their menu faster. makes it _feel_ faster
      }
      if (main_hover_2 == 1) {
        //start routine call here
        warmup(); //will do jumps to a function to start things happening
      }
      if (main_hover_3 == 1) {
        //Sensor Stats page call here
        sensorStatsInteractiveFirstRun = 1;
        sensorstats_interactive = 1; //oh yeah. It's all coming together
      }
      if ((main_hover_4 == 1) && (ignoreLoadCell_1 == 0) && (ignoreLoadCell_2 == 0)) {
        //Calibrate menu page here
        calibrate_interactive = 1;
        calibrateInteractiveFirstRun = 1;
        displayUpdate();
      }
      if (main_hover_5 == 1) {
        //Serial Interface page
        serial_interactive = 1; //go into serial_interface submenu
        serialInteractiveFirstRun = 1; //if you don't set this the menu locks up
        displayUpdate();
      }
      if (main_hover_6 == 1) {
        aboutMenu = 1; //go into the about menu
        displayUpdate(); //make it snappy
      }
      if (main_hover_7 == 1) {
        resetFunc(); //reset us now, user commanded reset
      }
    } //end digitalRead==LOW
  } // end "no longer the first run"
} //end mainMenuUserInput

//begin runMenuUserInput
void runMenuUserInput() {
  if (runInteractiveFirstRun == 1) { //we always start at 1, 1-4
    run_hover_1 = 1;
    //run_visible_1to4 = 1;
    runInteractiveFirstRun = 0; //no longer the first run, so set this flag to not run this loop again until you reset it by navigating to the non-interactive run menu
    //Serial.print("first run!!");
    displayUpdate(); //let the user see the submenu faster
  } // end runInteractiveFirstRun == 1 if loop
  //debug block flags
  /**
    Serial.print("runMenuUserInput();");
    Serial.println();
    Serial.print("runInteractiveFirstRun=");
    Serial.print(runInteractiveFirstRun); //this is the flag that tells us whether this loop is the first time you've been in run interactive. allows us to do some setup one time
    Serial.println();
    Serial.print("run_hover_1=");
    Serial.print(run_hover_1); //in the actual menu one of these will always be 1, but we'll handle that in the actual logic
    Serial.println();
    Serial.print("run_hover_2=");
    Serial.print(run_hover_2);
    Serial.println();
    Serial.print("run_hover_3=");
    Serial.print(run_hover_3);
    Serial.println();
    Serial.print("run_hover_4=");
    Serial.print(run_hover_4);
    Serial.println();
    Serial.print("run_hover_5=");
    Serial.print(run_hover_5);
    Serial.println();
    Serial.print("run_hover_6=");
    Serial.print(run_hover_6);
    Serial.println();

    //visible flags are display windows. we can only display 4 lines at a time so this is the logic flag to dodge around which line range we can see
    Serial.print("run_visible_1to4=");
    Serial.print(run_visible_1to4); //similar to the hover, one of these will always be 1, but we're doing that logic in the function.
    Serial.println();
    Serial.print("run_visible_2to5=");
    Serial.print(run_visible_2to5);
    Serial.println();
    Serial.print("run_visible_3to6=");
    Serial.print(run_visible_3to6);
    Serial.println();
    Serial.print("encoder_CW=");
    Serial.print(encoder_CW);
    Serial.println();
    Serial.print("encoder_CCW=");
    Serial.print(encoder_CCW);
    Serial.println();
  */
  //end debug block

  if (runInteractiveFirstRun == 0) { //once we are no longer in first run (skip the logic code the first loop)
    if ((encoder_CW == 1) || (encoder_CCW == 1)) {
      encoder_pulse = 1; //raise the pulse flag
    }
    if (encoder_pulse == 1) {
      //encoder logic block
      if ((encoder_CW == 1) && (run_hover_1 == 1)) {
        run_hover_1 = 0;
        run_hover_2 = 1;
        run_hover_3 = 0;
        run_hover_4 = 0;
        //run_hover_5 = 0;
        //run_hover_6 = 0;
        encoder_CW = 0;
      }
      if ((encoder_CW == 1) && (run_hover_2 == 1)) {
        run_hover_1 = 0;
        run_hover_2 = 0;
        run_hover_3 = 1;
        run_hover_4 = 0;
        //run_hover_5 = 0;
        //run_hover_6 = 0;
        encoder_CW = 0;
      }
      if ((encoder_CW == 1) && (run_hover_3 == 1)) {
        run_hover_1 = 0;
        run_hover_2 = 0;
        run_hover_3 = 0;
        run_hover_4 = 1;
        //run_hover_5 = 0;
        //run_hover_6 = 0;
        encoder_CW = 0;
      }
      /**
        if ((encoder_CW == 1) && (run_hover_4 == 1)) {
        run_hover_1 = 0;
        run_hover_2 = 0;
        run_hover_3 = 0;
        run_hover_4 = 0;
        run_hover_5 = 1;
        run_hover_6 = 0;
        encoder_CW = 0;
        }
        if ((encoder_CW == 1) && (run_hover_5 == 1)) {
        run_hover_1 = 0;
        run_hover_2 = 0;
        run_hover_3 = 0;
        run_hover_4 = 0;
        run_hover_5 = 0;
        run_hover_6 = 1;
        encoder_CW = 0;
        }
        if ((encoder_CCW == 1) && (run_hover_6 == 1)) {
        run_hover_1 = 0;
        run_hover_2 = 0;
        run_hover_3 = 0;
        run_hover_4 = 0;
        run_hover_5 = 1;
        run_hover_6 = 0;
        encoder_CCW = 0;
        }
        if ((encoder_CCW == 1) && (run_hover_5 == 1)) {
        run_hover_1 = 0;
        run_hover_2 = 0;
        run_hover_3 = 0;
        run_hover_4 = 1;
        run_hover_5 = 0;
        run_hover_6 = 0;
        encoder_CCW = 0;
        }
      */
      if ((encoder_CCW == 1) && (run_hover_4 == 1)) {
        run_hover_1 = 0;
        run_hover_2 = 0;
        run_hover_3 = 1;
        run_hover_4 = 0;
        //run_hover_5 = 0;
        //run_hover_6 = 0;
        encoder_CCW = 0;
      }
      if ((encoder_CCW == 1) && (run_hover_3 == 1)) {
        run_hover_1 = 0;
        run_hover_2 = 1;
        run_hover_3 = 0;
        run_hover_4 = 0;
        //run_hover_5 = 0;
        //run_hover_6 = 0;
        encoder_CCW = 0;
      }
      if ((encoder_CCW == 1) && (run_hover_2 == 1)) {
        run_hover_1 = 1;
        run_hover_2 = 0;
        run_hover_3 = 0;
        run_hover_4 = 0;
        //run_hover_5 = 0;
        //run_hover_6 = 0;
        encoder_CCW = 0;
      }
      /**
        if (run_visible_1to4 == 1) {
        if (run_hover_5 == 1) {
          run_visible_2to5 = 1;
          run_visible_1to4 = 0;
          Serial.print("run_visible_2to5 = 1");
          Serial.println();
        }
        if (run_hover_6 == 1) {
          run_visible_3to6 = 1;
          run_visible_1to4 = 0;
          Serial.print("run_visible_3to6 = 1");
          Serial.println();
        }
        }

        if (run_visible_2to5 == 1) {
        if (run_hover_1 == 1) {
          run_visible_1to4 = 1;
          run_visible_2to5 = 0;
          Serial.print("run_visible_1to4 = 1");
          Serial.println();
        }
        if (run_hover_6 == 1) {
          run_visible_3to6 = 1;
          run_visible_2to5 = 0;
          Serial.print("run_visible_3to6 = 1");
          Serial.println();
        }
        }
        if (run_visible_3to6 == 1) {
        if (run_hover_2 == 1) {
          run_visible_2to5 = 1;
          run_visible_3to6 = 0;
          Serial.print("run_visible_2to5 = 1");
          Serial.println();
        }
        if (run_hover_1 == 1) {
          run_visible_1to4 = 1;
          run_visible_3to6 = 0;
          Serial.print("run_visible_1to4 = 1");
          Serial.println();
        }
        }
      */
      encoder_pulse = 0; //reset the pulse flag
      encoder_CW = 0; //if you put in an extra rotate, this will zero it if the move was invalid (out of range), otherwise this results in refreshing the display at speed of loop
      encoder_CCW = 0; //same for other direction
      displayUpdate(); //update the display if and only if the user made a change, and do it after the values have been modified
    } //end if encoder pulse
    //submenu flags, these trigger overrides to get you into deeper menus
    if ((digitalRead(ENC_SW) == LOW)) {
      delay(clickDelay); //allow time for thumb to come off button, important because user will have pressed the button to get into the menu, don't wanna instantly cycle
      //Serial.print("you pressed the button!");
      if (run_hover_1 == 1) {
        //Serial.print("return to run menu from run_interactive");
        //Serial.println();
        run_interactive = 0; //return to the run, labeled "Info Screen"
        //perform time correction
        sensorResumeMillis = millis() - sensorDumperMillis; //time discrepancy value should be current value of millis minus old value of millis
        displayUpdate(); //does one update, lets the user see their menu faster. makes it _feel_ faster
      }
      if (run_hover_2 == 1) {
        //Abort routine call here
        Serial.println("User commanded hard reset!!!! Program resetting...");
        //motor.detach(); //Emergency stop
        motor.writeMicroseconds(0); //stop
        /**
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("E-STOP!!! Halted!   ");
          lcd.setCursor(0, 1);
          lcd.print("Remove power from   "); //data goes at pos 6
          lcd.setCursor(0, 2);liveadjustv
          lcd.print("the motor before    "); //pos 6
          lcd.setCursor(0, 3);
          lcd.print("cycling power.      ");
        */
        delay (100);
        //while (1) {
        //do nothing forever
        //}
        resetFunc(); //will hard reset but will also make motor throttle up to 100% after a few seconds, so we don't use it for E-stops
      }
      if (run_hover_3 == 1) {
        //Sensor Stats page call here
        sensorStatsInteractiveFirstRun = 1;
        sensorstats_interactive = 1; //oh yeah. It's all coming together
      }

      if (run_hover_4 == 1) {
        //Live Adjust Set Velocity Routine here
        liveAdjustV_init();
        liveadjustv_interactive = 1;
      }
      /** //we don't need to do more than this while the tunnel is running. if we ever do make sure to uncomment the above window changes too
        if (run_hover_5 == 1) {
        //Serial Interface page
        serial_interactive = 1; //go into serial_interface submenu
        serialInteractiveFirstRun = 1; //if you don't set this the menu locks up
        displayUpdate();
        }
        if (run_hover_6 == 1) {
        aboutMenu = 1; //go into the about menu
        displayUpdate(); //make it snappy
        }
      */
    } //end digitalRead==LOW
  } // end "no longer the first run"
} //end runMenuUserInput

void motorInit() { //should run once, triggered after trialDataEntry. will start motor at idle speed
  motorTarget = 1200; //so the ramp and governor knows where to start
  motorCurrent = 1200;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Motor Init");
  lcd.setCursor(0, 1);
  lcd.print("Please wait...");
  motorInitComplete = 1;
  motor.writeMicroseconds(700); // send "stop" signal to ESC. Also necessary to arm the ESC.
  delay(7000); // delay to allow the ESC to recognize the stopped signal.
  motor.writeMicroseconds(1200); //bring motor to idle speed
  firstTrialComplete = 1;
}

void runStartInit () { //prints out the static elements of the run data entry screen
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Input desired V(tgt)");
  lcd.setCursor(0, 1);
  lcd.print("and test duration...");
  lcd.setCursor(4, 2); //leave 4 characters open on 2nd line
  lcd.print("m/s");
  lcd.setCursor(12, 2); //space one space after m/s, 5 more spaces past the time, plus 1 for padding
  lcd.print("(MM:SS)"); //should finish on spot 19
  runDataEntry(); //have user input velocity and Runtime, v in whole m/s and time in 15 second increments
}

void trialStartInit () { //print static elements of trial selector screen
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Input desired trial ");
  lcd.setCursor(0, 1);
  lcd.print("number (# of cycles)");
  lcd.setCursor(0, 2);
  lcd.print("#t:");
  lcd.setCursor(0, 3);
  lcd.print("T+ done: ");
  lcd.setCursor(15, 3); //move to end of line to print time definition
  lcd.print("hh:mm");
  trialDataEntry();
}

void liveAdjustV_init() {//kicked by menu selection, loop will then run dynamic section
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Live Adjust V");
  lcd.setCursor(0, 1);
  lcd.print("Original V(tgt):");
  lcd.print(velocityTarget);
  lcd.print(".5");
  lcd.setCursor(0, 2);
  lcd.print("New V(tgt): "); //next print at 12,2
  lcd.setCursor(0, 3);
  lcd.print("Click to exit.");
}

void liveAdjustV() {//dynamic live adjust v
  if ((encoder_CW == 1) || (encoder_CCW == 1)) {
    encoder_pulse = 1; //raise the pulse flag
  }
  if (encoder_pulse == 1) {
    //encoder logic block
    if (velocityTarget < velocityTargetLimit) { //limited at 10 by current motor, 24 by anemometer
      if (encoder_CW == 1) { //increase digit

        velocityTarget = (velocityTarget + 1);
        lcd.setCursor(12, 2);
        lcd.print("  "); //blank the line to avoid stale digits
        lcd.setCursor(12, 2);
        lcd.print(velocityTarget, DEC);
        lcd.print(".5");
        encoder_CW = 0;
      }
    }
    if (velocityTarget > 0) { //cant be negative so don't let it decrease if it's 0
      if (encoder_CCW == 1) { //decrease digit, must set up so users can't input negative numbers! thats what the extra conditon is for
        velocityTarget = (velocityTarget - 1);
        lcd.setCursor(12, 2);
        lcd.print("  "); //blank the line to avoid stale digits
        lcd.setCursor(12, 2);
        lcd.print((velocityTarget), DEC);
        lcd.print(".5"); //to make the data match the serial output, because of the way velocity data is governed
        encoder_CCW = 0;
      }
    }
    encoder_pulse = 0; //reset the pulse flag
    encoder_CW = 0; //if you put in an extra rotate, this will zero it if the move was invalid (out of range), otherwise this results in refreshing the display at speed of loop
    encoder_CCW = 0; //same for other direction

    //print velo to screen

  } //end if encoder pulse
  //submenu flags, these trigger overrides to get you into deeper menus
  if (digitalRead(ENC_SW) == LOW) {
    liveadjustv_interactive = 0;
  }//end digitalRead==LOW
  delay(clickDelay);
  displayUpdate();
}//velo data entry


void runDataEntry() { //needs to be shortcut into the loop
  delay(50); //makes the user input run better. not sure why but if the main loop runs too fast it skips valid pulses.
  if (veloDataEntryComplete == 0) {
    if ((encoder_CW == 1) || (encoder_CCW == 1)) {
      encoder_pulse = 1; //raise the pulse flag
    }
    if (encoder_pulse == 1) {
      //encoder logic block
      if (velocityTarget < velocityTargetLimit) { //limited at 10 by current motor
        if (encoder_CW == 1) { //increase digit

          velocityTarget = (velocityTarget + 1);
          lcd.setCursor(0, 2);
          lcd.print("  "); //blank the line to avoid stale digits
          lcd.setCursor(0, 2);
          lcd.print(velocityTarget, DEC);
          lcd.print(".5");
          encoder_CW = 0;
        }
      }
      if (velocityTarget > 0) { //cant be negative so don't let it decrease if it's 0
        if (encoder_CCW == 1) { //decrease digit, must set up so users can't input negative numbers! thats what the extra conditon is for
          velocityTarget = (velocityTarget - 1);
          lcd.setCursor(0, 2);
          lcd.print("  "); //blank the line to avoid stale digits
          lcd.setCursor(0, 2);
          lcd.print((velocityTarget), DEC);
          lcd.print(".5"); //to make the data match the serial output, because of the way velocity data is governed
          encoder_CCW = 0;
        }
      }
      encoder_pulse = 0; //reset the pulse flag
      encoder_CW = 0; //if you put in an extra rotate, this will zero it if the move was invalid (out of range), otherwise this results in refreshing the display at speed of loop
      encoder_CCW = 0; //same for other direction

      //print velo to screen

    } //end if encoder pulse
    //submenu flags, these trigger overrides to get you into deeper menus
    if (digitalRead(ENC_SW) == LOW) {
      veloDataEntryComplete = 1;
      lcd.setCursor(9, 2);
      lcd.print(":"); // put the colon in to make it more obvious that time is the thing required now
    }//end digitalRead==LOW
    delay(clickDelay);
  }//velo data entry

  //time data entry

  if (timeDataEntryComplete == 0 && veloDataEntryComplete == 1) {
    if ((encoder_CW == 1) || (encoder_CCW == 1)) {
      encoder_pulse = 1; //raise the pulse flag
    }
    if (encoder_pulse == 1) {
      //encoder logic block
      if (runTime < timelimitrun) { //5 minutes is the max, don't allow increases if already at 5 minutes, remember runtime is in seconds
        if (encoder_CW == 1) { //increase digit

          runTime = (runTime + 15); //15 seocnd increments, runTime is in seconds, parsed to minutes and seconds
          //beign parse math to get runtime in minutes and seconds
          if (runTime >= 60) {
            runTimeDisplaySeconds = (runTime % 60); //gets modulo of runtime/60 which will tell you how many seconds remain after dividing by 60 to get minutes.
            runTimeDisplayMinutes = (runTime - runTimeDisplaySeconds); //will first make runTimeMinutes equal whole number seconds
            runTimeDisplayMinutes = (runTimeDisplayMinutes / 60); //...and then divide by 60 to get whole number minutes
          }
          if (runTime < 60) { //if it isnt a full minute...
            runTimeDisplaySeconds = runTime; //we only have seconds and can skip the modulo math
            runTimeDisplayMinutes = 0; //and we have 0 minutes
          }
          encoder_CW = 0;
        }
      }
      if (runTime > 0) { //runTime can't be negative
        if (encoder_CCW == 1) { //decrease digit, must set up so users can't input negative numbers! thats what the extra conditon is for
          runTime = (runTime - 15); //15 seocnd increments, runTime is in seconds, parsed to minutes and seconds
          //beign parse math to get runtime in minutes and seconds
          if (runTime >= 60) {
            runTimeDisplaySeconds = (runTime % 60); //gets modulo of runtime/60 which will tell you how many seconds remain after dividing by 60 to get minutes.
            runTimeDisplayMinutes = (runTime - runTimeDisplaySeconds); //will first make runTimeMinutes equal whole number seconds
            runTimeDisplayMinutes = (runTimeDisplayMinutes / 60); //...and then divide by 60 to get whole number minutes
          }
          if (runTime < 60) { //if it isnt a full minute...
            runTimeDisplaySeconds = runTime; //we only have seconds and can skip the modulo math
            runTimeDisplayMinutes = 0; //and we have 0 minutes
          }
          encoder_CCW = 0;
        }
      }
      encoder_pulse = 0; //reset the pulse flag
      encoder_CW = 0; //if you put in an extra rotate, this will zero it if the move was invalid (out of range), otherwise this results in refreshing the display at speed of loop
      encoder_CCW = 0; //same for other direction

      //time display update
      lcd.setCursor(8, 2); //prepare to print minutes (back it up to 7 instead of 8 if minutes will ever be greater than 10, right now limited to 5
      lcd.print(" "); //blank the line to avoid stale digits
      lcd.setCursor(8, 2);
      lcd.print(runTimeDisplayMinutes, DEC);
      lcd.setCursor(10, 2); //prepare to print seconds
      lcd.print(" 0"); //blank the line to avoid stale digits but add a stale 0 just in case, to make the clock look right
      lcd.setCursor(10, 2);
      lcd.print(runTimeDisplaySeconds, DEC);

    } //end if encoder pulse
    //submenu flags, these trigger overrides to get you into deeper menus
    if (digitalRead(ENC_SW) == LOW) {
      timeDataEntryComplete = 1;
      originalRunTime = runTime;
      delay(clickDelay); //delay to put the other screen up so as to not skip the baro or load cell sensor select
    }//end digitalRead==LOW
  }//time data entry

  //sensor select
  if ((veloDataEntryComplete == 1) && (timeDataEntryComplete == 1)) {
    if ((encoder_CW == 1) || (encoder_CCW == 1)) {
      encoder_pulse = 1; //raise the pulse flag
      //Serial.println("encoderpulse");
    }
    if (sensorSelectBaroBegin == 0) {
      //set up lcd for baro
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Use Barometer Data? ");
      lcd.setCursor(0, 1);
      lcd.print(">Yes");
      lcd.setCursor(1, 2);
      lcd.print("No");


      sensorSelectBaroBegin = 1;
    }
    if ((sensorSelectBaro == 0) && (encoder_pulse == 1)) {
      if ((baroyes == 0) && (encoder_pulse == 1)) {
        lcd.setCursor(0, 2);
        lcd.print(">");
        lcd.setCursor(0, 1);
        lcd.print(" ");
        baroyes ^= 1; // flip the value of baroyes
        //Serial.println(baroyes);
        //Serial.println("baro enabled");
        encoder_pulse = 0; //reset the pulse flag
        encoder_CW = 0; //if you put in an extra rotate, this will zero it if the move was invalid (out of range), otherwise this results in refreshing the display at speed of loop
        encoder_CCW = 0; //same for other direction
      }
      if ((baroyes == 1) && (encoder_pulse == 1)) {
        lcd.setCursor(0, 2);
        lcd.print(" ");
        lcd.setCursor(0, 1);
        lcd.print(">");
        baroyes ^= 1; // flip the value of baroyes
        //Serial.println(baroyes);
        //Serial.println("baro disabled");
        encoder_pulse = 0; //reset the pulse flag
        encoder_CW = 0; //if you put in an extra rotate, this will zero it if the move was invalid (out of range), otherwise this results in refreshing the display at speed of loop
        encoder_CCW = 0; //same for other direction
      }
    }
    //click the button to exit
    while (digitalRead(ENC_SW) == LOW && (sensorSelectBaro == 0)) {//this is not as nice ui wise but it prevents stalling the whole program
      sensorSelectBaro = 1;
      //Serial.println("barodone");
      if (((barofail_1 == 1) || (barofail_2 == 1) || (barofail_3 == 1) || (barofail_4 == 1) || (barofail_5 == 1) || (barofail_6 == 1) || (barofail_7 == 1))) {
        baroyes = 0; //override it if the user is being obtuse and asking for baro data when one is missing, prevents reads later
      }
      delay(clickDelay);
    }
    //end baro sensor select
    if ((sensorSelectLoadCellBegin == 0) && (sensorSelectBaro == 1)) {
      //set up lcd for load cells
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Use Load Cell Data? ");
      lcd.setCursor(0, 1);
      lcd.print(">Yes");
      lcd.setCursor(1, 2);
      lcd.print("No");
      sensorSelectLoadCellBegin = 1;
    }
    if ((sensorSelectLoadCell == 0) && (sensorSelectBaro == 1)) {
      if ((loadcellyes == 0) && (encoder_pulse == 1)) {
        lcd.setCursor(0, 2);
        lcd.print(">");
        lcd.setCursor(0, 1);
        lcd.print(" ");
        loadcellyes = 1;
        //Serial.println("load cell enabled");
        encoder_pulse = 0; //reset the pulse flag
        encoder_CW = 0; //if you put in an extra rotate, this will zero it if the move was invalid (out of range), otherwise this results in refreshing the display at speed of loop
        encoder_CCW = 0; //same for other direction
      }
      if ((loadcellyes == 1) && (encoder_pulse == 1)) {
        lcd.setCursor(0, 2);
        lcd.print(" ");
        lcd.setCursor(0, 1);
        lcd.print(">");
        loadcellyes = 0;
        //Serial.println("load cell disabled");
        encoder_pulse = 0; //reset the pulse flag
        encoder_CW = 0; //if you put in an extra rotate, this will zero it if the move was invalid (out of range), otherwise this results in refreshing the display at speed of loop
        encoder_CCW = 0; //same for other direction
      }

      //click the button to exit
      while (digitalRead(ENC_SW) == LOW) {//this is not as nice ui wise but it prevents stalling the whole program
        sensorSelectLoadCell = 1;
        //Serial.println("LCdone");
        if ((ignoreLoadCell_1 == 1) || (ignoreLoadCell_2 == 1)) {
          loadcellyes = 0; //similarly override the loadcell output if this is missing
        }
        delay(clickDelay);
      }
      //Serial.println("loop");


    } //end load cell sensor select
    if ((sensorSelectBaro == 1) && (sensorSelectLoadCell == 1)) { //if all sensor selects are done, exit and begin.
      sensorSelectComplete = 1;
    }
  }// end sensorSelect

  //final start kickoff:
  if ((veloDataEntryComplete == 1) && (timeDataEntryComplete == 1) && (sensorSelectComplete == 1)) {
    runDataEntryComplete = 1;
    //albatross
    trialStartInit();

    //below 2 lines will jump over trialDataEntry
    //sensorDumperInit(); //prints the first CSV line for the sensor dumper
    //motorInit();
  }
} // end run data entry

void trialDataEntry() { //needs to be shortcut into the loop
  delay(50); //makes the user input run better. not sure why but if the main loop runs too fast it skips valid pulses.
  //trial data entry

  if (trialDataEntryComplete == 0) {
    if ((encoder_CW == 1) || (encoder_CCW == 1)) {
      encoder_pulse = 1; //raise the pulse flag
    }
    if (encoder_pulse == 1) {
      //encoder logic block
      if (numTrials < 100) { //100 trials is the max, don't allow increases if already at 300 trials
        if (encoder_CW == 1) { //increase digit

          numTrials = (numTrials + 1); //15 seocnd increments, runTime is in seconds, parsed to minutes and seconds
          encoder_CW = 0;
        }
      }
      if (numTrials > 1) { //numTrials can't be less than 1, must have one trial at least
        if (encoder_CCW == 1) { //decrease digit, must set up so users can't input numbers fewer than 1! thats what the extra conditon is for
          numTrials = (numTrials - 1);
          encoder_CCW = 0;
        }
      }
      encoder_pulse = 0; //reset the pulse flag
      encoder_CW = 0; //if you put in an extra rotate, this will zero it if the move was invalid (out of range), otherwise this results in refreshing the display at speed of loop
      encoder_CCW = 0; //same for other direction

      //trials display update
      lcd.setCursor(5, 2); //prepare to print trials
      lcd.print("   "); //blank the line to avoid stale digits, will never be more than 3 digits, where should this be
      lcd.setCursor(5, 2); //rewind the cursor
      lcd.print(numTrials, DEC); //blank the line to avoid stale digits

      // calculate estimated time
      trialTimeEstimate = ((numTrials * runTime) + (5 * numTrials));

      // calculate hours and minutes
      trialTimeDisplayHours = trialTimeEstimate / 3600;
      trialTimeDisplayMinutes = (trialTimeEstimate % 3600) / 60;

      // display estimated total trial Time
      lcd.setCursor(9, 3);
      lcd.print(trialTimeDisplayHours);
      lcd.print(":");
      if (trialTimeDisplayMinutes < 10) {
        lcd.print("0");
      }
      lcd.print(trialTimeDisplayMinutes);

    } //end if encoder pulse
    //submenu flags, these trigger overrides to get you into deeper menus
    if (digitalRead(ENC_SW) == LOW) {
      trialDataEntryComplete = 1;
      delay(clickDelay); //delay to put the other screen up so as to not skip the baro or load cell sensor select
    }//end digitalRead==LOW
  }//time data entry
  //final start kickoff:
  if (trialDataEntryComplete == 1) {
    //albatross_bird
    //we now have numTrials
    sensorDumperInit(); //prints the first CSV line for the sensor dumper (will carry through the info for numTrials)
    if (firstTrialComplete == 0) {
      motorInit(); //not needed after first run because we manually reset the motor, don't want to add delays to new trials
    }
  }
} // end trial data entry

//begin motor governor functions
void motorGovernor() {
  currentVelocity = floor(int(cgAnem.airflowRate)); //get current velocity and throw out the decimals. Real speed will be logged but the governor will tend to keep things at or above target speed by 1 m/s, so -0.00,+0.99m/s

  if (millis() - motorRampSlopePrevious >= motorRampSlope) { //state machine to control the motor, will only update motor if enough time has elapsed since the last adjustment. This governs the governor
    motorRampSlopePrevious = millis(); // first thing to do is updating the snapshot of time
    // time for timed action

    if ((currentVelocity < velocityTarget) && (motorTarget < motorpwmlimit)) {
      motorTarget = motorTarget + 10; // where 10 is the constant, or P term. these are in pulses, not motor percents, because theyre smaller.
      //motorRamp(); //execute the change
      //motorGo(motorTarget); //testing to see if this is faster than ramping, IE no lag
    }
    if (currentVelocity > velocityTarget) {
      motorTarget = motorTarget - 10;
      //motorRamp(); //execute the change
      
    }
    motorGo(motorTarget);
  }
}

//two governors??

/**
void motorRamp() { //sends motor from current speed to target speed at ramp constant speed, adjust ramp constant to increase or decrease motor slope, will need state machines to fully optimize. Laggy?
  if (motorTarget > motorCurrent) {
    //Serial.println("Ramping UP");
    //for (int i = motorCurrent; i < motorpwmlimit; i++) {//ramp up
    if (millis() - motorRampSlopePrevious >= motorRampSlope) { //state machine to control the motor, will only update motor if enough time has elapsed since the last adjustment. This governs the governor
      motorRampSlopePrevious = millis(); // first thing to do is updating the snapshot of time
      // time for timed action
      if (motorTarget < motorpwmlimit) { //must add this extra check or ESC gets unhappy with out of range values and shuts down
      motorGo(motorTarget);
      }
      else {
        motorTarget = motorpwmlimit;
      }
      
      //Serial.print(i);
      //motorCurrent = i;
    }
    //if (motorCurrent == motorTarget) {
    // break;
    //}
    //}
  }
  if (motorTarget < motorCurrent) {
    //Serial.println("Ramping DOWN");
    //for (int i = motorCurrent; i > 1150; i--) { //ramp down
    if (millis() - motorRampSlopePrevious >= motorRampSlope) { //state machine to control the motor, will only update motor if enough time has elapsed since the last adjustment. This governs the governor
      motorRampSlopePrevious = millis(); // first thing to do is updating the snapshot of time
      // time for timed action
      motorGo(motorTarget);
      //Serial.print(i);
      //motorCurrent = i;
    }
    //if (motorCurrent == motorTarget) {
    //break;
    //}




    // time for timed action
    //delay(motorCurveGain);
    //motorGo(i);
    //Serial.print(i);
    //motorCurrent = i;
    //}
  }
} //end motorRamp
*/

int motorGo(int motorCommand) { //spin motor at motorCommand pulse rate
  motor.writeMicroseconds(motorCommand);
}

void sensorDumperInit() { //this is the first line, after this will be called by the loop
  runStartMillis = millis(); //stores value of millis at time of sensorDumperInit execution
  Serial.println("WindOS Sensor Dump,Elapsed Time (s),Barometer 1 (mbar),Barometer 2 (mbar),Barometer 3 (mbar),Barometer 4 (mbar),Barometer 5 (mbar),Barometer 6 (mbar),Barometer 7 (mbar),Velocity (m/s),Velocity Target (m/s),Temperature (Celcius),Load Cell 1 (g),Load Cell 2 (g),Commanded Motor Power (%)");
}

void sensorDumper() {
  if (baroyes == 1) { //if user has asked for baro data...
    //get new sensor values
    baroSelect = 0; //baro 0
    baroRead();
    baroSelect = 1; //baro 1
    baroRead();
    baroSelect = 2; //baro 2
    baroRead();
    baroSelect = 3; //baro 3
    baroRead();
    baroSelect = 4; //baro 4
    baroRead();
    baroSelect = 5; //baro 5
    baroRead();
    baroSelect = 6; //baro 6
    baroRead();
  }

  //now baro0 - baro7 should have values
  if (loadcellyes == 1) { //if user has asked for load cell data...
    loadCell1Reading = LoadCell_1.get_units(); //unlike loadCellGo, we use this to change the direction of the vector to have positive force be up instead of down
    loadCell2Reading = LoadCell_2.get_units(); //unlike loadCellGo, we use this to change the direction of the vector to have positive force be up instead of down
  }

  if (baroyes == 0) { //if user has asked for no baro data...
    baro0 = 0;
    baro1 = 0;
    baro2 = 0;
    baro3 = 0;
    baro4 = 0;
    baro5 = 0;
    baro6 = 0;
  }
  if (loadcellyes == 0) { //if user has asked for no load cell data
    loadCell1Reading = 0;
    loadCell2Reading = 0;
  }

  //String(cgAnem.airflowRate) //we can use these directly in the prints
  //String(cgAnem.temperature)

  //echo values to serial port
  if (dataDumperFirstRun == 1) {
    Serial.print("Trial Number: ");
    Serial.print(numTrialsObserved); //this is a variable which will only get ++numTrialsObserved incremented by 1, this is a seperate counter to numTrials which actually controlls the behavior at the end of the program
    dataDumperFirstRun = 0;
  }
  Serial.print(""); //message window, can switch this with an if for several flags but for now no flags are used so leave the space
  Serial.print(","); //next value
  Serial.print((float(sensorDumperMillis - runStartMillis)) / 1000); //prints elapsed test time in seconds for plotting test results over time
  Serial.print(","); //next value
  Serial.print(baro0);
  Serial.print(","); //next value
  Serial.print(baro1);
  Serial.print(","); //next value
  Serial.print(baro2);
  Serial.print(","); //next value
  Serial.print(baro3);
  Serial.print(","); //next value
  Serial.print(baro4);
  Serial.print(","); //next value
  Serial.print(baro5);
  Serial.print(","); //next value
  Serial.print(baro6);
  Serial.print(","); //next value
  Serial.print(String(cgAnem.airflowRate));
  Serial.print(","); //next value
  Serial.print(float(float(velocityTarget) + 0.5)); //add 0.5 to make range center correctly
  Serial.print(",");
  Serial.print(String(cgAnem.temperature));
  Serial.print(",");
  Serial.print(loadCell1Reading);
  Serial.print(",");
  Serial.print(loadCell2Reading);
  Serial.print(",");
  Serial.print(float(((float(motorTarget) - 1200) / 700) * 100)); //prints motor percentage (converts from PWM)
  Serial.println();//line feed because we're done printing values
}




void anemStatsDisplay() {
  if (anemStatsInteractiveFirstRun == 1) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Anemometer Readout: ");
    lcd.setCursor(0, 1);
    lcd.print("Temp:"); //data goes at pos 6
    lcd.setCursor(0, 2);
    lcd.print("Velo:"); //pos 6
    lcd.setCursor(0, 3);
    lcd.print("             >Back< ");
    anemStatsInteractiveFirstRun = 0;
  }
  if (anemfail_notfound == 0) { //if we print bad values it blows everything up with -255 values. clearly wrong but not displaying anything is better.
    //custom clear, only targets dynamic rows, will cause some flicker
    lcd.setCursor(6, 1);
    lcd.print("              "); //cut 6 spaces off the normal 20 to not overflow spaces
    lcd.setCursor(6, 2);
    lcd.print("              ");
    //write dynamic info
    lcd.setCursor(6, 1);
    lcd.print(String(cgAnem.temperature));
    lcd.print("C");
    lcd.setCursor(6, 2);
    lcd.print(String(cgAnem.airflowRate));
    lcd.print("m/s");
  }
  if (anemfail_notfound == 1) { //if we print bad values it blows everything up with -255 values. clearly wrong but not displaying anything is better.
    lcd.setCursor(0, 1);
    lcd.print("Not Found!");
  }
  while (digitalRead(ENC_SW) == LOW) { //exit trap
    //cant use a delay here because it will upset the timing of data collection in the run loop
    anemstats_interactive = 0;
    displayUpdate(); //show the menu
  }
}

void loadStatsDisplay() {
  if (loadStatsInteractiveFirstRun == 1) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Load Cell Readouts: ");
    lcd.setCursor(0, 1);
    lcd.print("LoadCell 1:"); //data goes at pos 12
    lcd.setCursor(0, 2);
    lcd.print("LoadCell 2:"); //pos 12
    lcd.setCursor(0, 3);
    lcd.print("             >Back< ");
    loadStatsInteractiveFirstRun = 0;
  }

  //nonzeroed load cell values
  loadCell1Reading = LoadCell_1.get_units(); //unlike loadCellGo, we use this to change the direction of the vector to have positive force be up instead of down
  loadCell2Reading = LoadCell_2.get_units(); //unlike loadCellGo, we use this to change the direction of the vector to have positive force be up instead of down

  //custom clear, only targets dynamic rows
  lcd.setCursor(12, 1);
  lcd.print("        "); //cut 12 spaces off the normal 20 to not overflow spaces
  lcd.setCursor(12, 2);
  lcd.print("        ");

  //print load cell 1 to screen
  lcd.setCursor(12, 1);
  lcd.print(loadCell1Reading);
  lcd.print("g"); //equivalent to mbar but also 1 letter shorter, more space on the menu. I would have otherwise set this to say mbar
  lcd.setCursor(12, 2);
  lcd.print(loadCell2Reading);
  lcd.print("g"); //equivalent to mbar but also 1 letter shorter, more space on the menu. I would have otherwise set this to say mbar

  while (digitalRead(ENC_SW) == LOW) { //exit trap
    //cant use a delay here because it will upset the timing of data collection in the run loop
    loadstats_interactive = 0;
    displayUpdate(); //show the menu
  }
}

void loadcellCalibrate1() {
  if (loadCellCalibrate1_firstrun == 1) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(" Remove any specimen");
    lcd.setCursor(0, 1);
    lcd.print("from Load Cell 1.");
    lcd.setCursor(0, 2);
    lcd.print("Click when done:");
    lcd.setCursor(0, 3);
    lcd.print("              >Done< ");
    while (digitalRead(ENC_SW) == HIGH) { // this block pauses all code execution while user reads and acknowledges the display. 38 is the button pin
      // Do nothing
    }
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Do not touch!"); //print 20 spaces, effectively clears the line
    lcd.setCursor(0, 1);
    lcd.print("Zeroing Scale...");
    //lcd.setCursor(0, 2);
    //lcd.print("");
    //lcd.setCursor(0, 3);
    //lcd.print("");
    delay(5000); //delay longer than standard because this is the "no touch" section
    LoadCell_1.tare(); //Reset the LoadCell_1 to 0
    LoadCell_1.set_scale(calibration_factor_1); //Adjust to this calibration factor
    lcd.setCursor(0, 0);
    lcd.print("Current Cal. Factor:");
    lcd.setCursor(0, 1);
    lcd.print("                    "); //blank the line before we print something arbitrary
    lcd.setCursor(0, 1);
    lcd.print(calibration_factor_1);
    lcd.setCursor(0, 3);
    lcd.print("               >OK< ");
    while (digitalRead(ENC_SW) == HIGH) { // this block pauses all code execution while user reads and acknowledges the display. 38 is the button pin
      // Do nothing
    }
    lcd.clear(); //clear because of arbitrary writing for dynamic loadcell known mass entry (next block, donw below a bit)
    delay(clickDelay);
    loadCellCalibrate1_firstrun = 0;

    //if some mass exists already for the loadcellcalibrationmass, then display it real quick
    if (loadcellcalibratedmass != 0) {
      lcd.setCursor(9, 2);
      lcd.print(loadcellcalibratedmass, DEC);
    }
    lcd.setCursor(0, 0);
    lcd.print("Hang known mass from"); //print 20 spaces, effectively clears the line
    lcd.setCursor(0, 1);
    lcd.print("Load cell.");
    lcd.setCursor(0, 2);
    lcd.print("Enter:"); //print target is (0, 9) for the number entry thing
    lcd.setCursor(18, 2); //dodge the number entry area
    lcd.print("g");
  } //end loadcell firstrun
  if (loadCellCalibrate1_firstrun == 0) {
    if ((firstDigitComplete == 0) && (secondDigitComplete == 0) && (thirdDigitComplete == 0) && (loadcellcalibratedmass >= 100)) { //hundreds
      lcd.setCursor(9, 3);
      lcd.print("^  "); //no other case needed because this will add or subtract 100
    }
    if ((firstDigitComplete == 1) && (secondDigitComplete == 0) && (thirdDigitComplete == 0) && (loadcellcalibratedmass >= 100)) { //tens
      lcd.setCursor(9, 3);
      lcd.print(" ^ ");
    }
    if ((firstDigitComplete == 1) && (secondDigitComplete == 0) && (thirdDigitComplete == 0) && (loadcellcalibratedmass < 100)) { //tens
      lcd.setCursor(9, 3);
      lcd.print("^  "); //alternate case to cover when the stored number was/is still less than at least 100, will shift if user adds so many 10s that it becomes greater than 100 to keep "^" pointing at correct digit
    }
    if ((firstDigitComplete == 1) && (secondDigitComplete == 1) && (thirdDigitComplete == 0) && (loadcellcalibratedmass >= 100)) { //ones
      lcd.setCursor(9, 3);
      lcd.print("  ^");
    }
    if ((firstDigitComplete == 1) && (secondDigitComplete == 1) && (thirdDigitComplete == 0) && (loadcellcalibratedmass < 100) && (loadcellcalibratedmass >= 10)) { //ones
      lcd.setCursor(9, 3);
      lcd.print(" ^ "); //ones when number is a 2 digit number, so between 10 and 99 inclusive
    }
    if ((firstDigitComplete == 1) && (secondDigitComplete == 1) && (thirdDigitComplete == 0) && (loadcellcalibratedmass < 10)) { //ones when less than 10
      lcd.setCursor(9, 3);
      lcd.print("^  ");
    }

    if (loadCellDataEntryComplete == 0) {
      loadCellDataEntry();
    }
    if (loadCellDataEntryComplete == 1 && loadCellGo1Active == 0 && loadCellGo2Active == 0) {
      firstDigitComplete = 0;
      secondDigitComplete = 0;
      thirdDigitComplete = 0;
      //Serial.print("Load Cell Calibrate Action!"); //do auto-calibration
      //Serial.println();
      //Serial.print("loadCellCalibratedMass=");
      //Serial.print(loadcellcalibratedmass);
      //Serial.println();

      calibration_factor_1 = -99999; //unlikely to match the mass, use as a starting point to drive the calibration
      loadCellGo1Active = 1; //shortcut flag
      //Serial.print("loadCellDataEntryExit1");
      //Serial.println();
      loadcellprecision1 = 0; //reset precision for next run if user asks again
      lcd.setCursor(0, 0);
      lcd.print("Mass Reading:       "); //start prints at 14
      lcd.setCursor(0, 1);
      lcd.print("Cal. Fact:          "); //11
      lcd.setCursor(0, 2);
      lcd.print("Target:"); //just overwrite the Enter: prompt
      loadCellGo1(); //this is the loop to actually calibrate the load cell. it'll need to be shortcut to because it needs to loop


      //done
      //the below are needed to exit only if loadCellGo doesn't exit
      //calibrate_interactive = 0; //exit calibrate menu
      //displayUpdate(); //to print the applicable menu
    }
  }
}

void loadcellCalibrate2() {
  if (loadCellCalibrate2_firstrun == 1) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(" Remove any specimen"); //print 20 spaces, effectively clears the line
    lcd.setCursor(0, 1);
    lcd.print("from Load Cell 2.");
    lcd.setCursor(0, 2);
    lcd.print("Click when done:");
    lcd.setCursor(0, 3);
    lcd.print("              >Done< ");
    while (digitalRead(ENC_SW) == HIGH) { // this block pauses all code execution while user reads and acknowledges the display. 38 is the button pin
      // Do nothing
    }
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Do not touch!"); //print 20 spaces, effectively clears the line
    lcd.setCursor(0, 1);
    lcd.print("Zeroing Scale...");
    //lcd.setCursor(0, 2);
    //lcd.print("");
    //lcd.setCursor(0, 3);
    //lcd.print("");
    delay(5000); //delay longer than standard because this is the "no touch" section
    LoadCell_2.tare(); //Reset the LoadCell_1 to 0



    LoadCell_2.set_scale(calibration_factor_2); //Adjust to this calibration factor
    lcd.setCursor(0, 0);
    lcd.print("Current Cal. Factor:");
    lcd.setCursor(0, 1);
    lcd.print("                    "); //blank the line before we print something arbitrary
    lcd.setCursor(0, 1);
    lcd.print(calibration_factor_2);
    lcd.setCursor(0, 3);
    lcd.print("               >OK< ");
    while (digitalRead(ENC_SW) == HIGH) { // this block pauses all code execution while user reads and acknowledges the display. 38 is the button pin
      // Do nothing
    }
    lcd.clear(); //clear because of arbitrary writing for dynamic loadcell known mass entry (next block, donw below a bit)
    delay(clickDelay);
    loadCellCalibrate2_firstrun = 0;

    if (loadcellcalibratedmass != 0) {
      lcd.setCursor(9, 2);
      lcd.print(loadcellcalibratedmass, DEC);
    }
    lcd.setCursor(0, 0);
    lcd.print("Hang known mass from"); //print 20 spaces, effectively clears the line
    lcd.setCursor(0, 1);
    lcd.print("Load cell.");
    lcd.setCursor(0, 2);
    lcd.print("Enter:"); //print target is (0, 9) for the number entry thing
    lcd.setCursor(18, 2); //dodge the number entry area
    lcd.print("g");
  } //end loadcell firstrun
  if (loadCellCalibrate2_firstrun == 0) {
    if ((firstDigitComplete == 0) && (secondDigitComplete == 0) && (thirdDigitComplete == 0) && (loadcellcalibratedmass >= 100)) { //hundreds
      lcd.setCursor(9, 3);
      lcd.print("^  "); //no other case needed because this will add or subtract 100
    }
    if ((firstDigitComplete == 1) && (secondDigitComplete == 0) && (thirdDigitComplete == 0) && (loadcellcalibratedmass >= 100)) { //tens
      lcd.setCursor(9, 3);
      lcd.print(" ^ ");
    }
    if ((firstDigitComplete == 1) && (secondDigitComplete == 0) && (thirdDigitComplete == 0) && (loadcellcalibratedmass < 100)) { //tens
      lcd.setCursor(9, 3);
      lcd.print("^  "); //alternate case to cover when the stored number was/is still less than at least 100, will shift if user adds so many 10s that it becomes greater than 100 to keep "^" pointing at correct digit
    }
    if ((firstDigitComplete == 1) && (secondDigitComplete == 1) && (thirdDigitComplete == 0) && (loadcellcalibratedmass >= 100)) { //ones
      lcd.setCursor(9, 3);
      lcd.print("  ^");
    }
    if ((firstDigitComplete == 1) && (secondDigitComplete == 1) && (thirdDigitComplete == 0) && (loadcellcalibratedmass < 100) && (loadcellcalibratedmass >= 10)) { //ones
      lcd.setCursor(9, 3);
      lcd.print(" ^ "); //ones when number is a 2 digit number, so between 10 and 99 inclusive
    }
    if ((firstDigitComplete == 1) && (secondDigitComplete == 1) && (thirdDigitComplete == 0) && (loadcellcalibratedmass < 10)) { //ones when less than 10
      lcd.setCursor(9, 3);
      lcd.print("^  ");
    }


    if (loadCellDataEntryComplete == 0) {
      loadCellDataEntry();
    }
    if (loadCellDataEntryComplete == 1 && loadCellGo1Active == 0 && loadCellGo2Active == 0) { //only if data entry is done and lock it out if the load cell routine is up
      firstDigitComplete = 0;
      secondDigitComplete = 0;
      thirdDigitComplete = 0;
      //Serial.print("Load Cell Calibrate Action 2!"); //do auto-calibration for load cell 2
      //Serial.println();
      //Serial.print("loadCellCalibratedMass=");
      //Serial.print(loadcellcalibratedmass);
      //Serial.println();
      calibration_factor_2 = -99999;
      loadCellGo2Active = 1; //shortcut flag
      //Serial.print("loadCellDataEntryExit2");
      //Serial.println();
      loadcellprecision2 = 0; //reset precision for next run if user asks again
      lcd.setCursor(0, 0);
      lcd.print("Mass Reading:       "); //start prints at 14
      lcd.setCursor(0, 1);
      lcd.print("Cal. Fact:          "); //11
      lcd.setCursor(0, 2);
      lcd.print("Target:"); //just overwrite the Enter: prompt
      loadCellGo2(); //same as 1 for 2


      //exits used to be here but since we're actually doing something, look in that function
    }
  }
}

void loadCellGo1() { //calibrate first load cell. This is called after the user input and also is shortcut to based on flag, same way load cell data entry is
  //loadcellcalibratedmass is the user input mass
  //calibration factor will be reset to a definitely-unsolved factor for driving to user mass
  LoadCell_1.set_scale(calibration_factor_1);
  loadCellGo1Reading = (-1 * LoadCell_1.get_units());

  lcd.setCursor(14, 0);
  lcd.print(loadCellGo1Reading, DEC); //start prints at 14
  lcd.setCursor(11, 1);
  lcd.print(calibration_factor_1); //11


  if (loadcellprecision1 == 3) { // you've satisfied 1000, 100, and 10 adjustment speeds
    //exit, lower loadCellGo flag
    loadCellGo1Active = 0; //deactivate the running routine
    calibrate_interactive = 0; //exit calibrate menu
    //Serial.print("loadCellGo1 Exited!");
    //Serial.println();
    //Serial.print("mass reading EXIT:");
    //Serial.print(loadCellGo1Reading, DEC);
    //Serial.println();
    //Serial.print("calibration_factor_1 EXIT=");
    //Serial.print(calibration_factor_1, DEC);
    //Serial.println();
    EEPROM.put(0, calibration_factor_1);
    //Serial.print("calibration_factor_1 stored to EEPROM!");
    //Serial.println();
    delay(3000); //delay to allow user to see that the calibration is done
    displayUpdate(); //to print the applicable menu

  }
  loadCellGo1Reading = (-1 * LoadCell_1.get_units()); //must update before we check or it could be outdated for single-loop corrections
  if ((loadcellcalibratedmass > loadCellGo1Reading) && (loadcellprecision1 == 0) && (loadcellprecision1 != 3)) { //super coarse adjustment
    calibration_factor_1 += 1000; //large negative number will get closer to 0

    LoadCell_1.set_scale(calibration_factor_1); //apply the new calibration factor to the load cell
    loadCellGo1Reading = (-1 * LoadCell_1.get_units()); //also must update afterwards

    if (loadCellGo1Reading < 0) { //divide by zero? produces large funky negative valuers, like underflow.
      if (loadcellprecision1 == 0) {
        calibration_factor_1 -= 1001;
        //Serial.print("Negative safety tripped!");
        //Serial.println();
      }
      if (loadcellprecision1 == 1) {
        calibration_factor_1 -= 101;
        //Serial.print("Negative safety tripped!");
        //Serial.println();
      }
      if (loadcellprecision1 == 2) {
        calibration_factor_1 -= 11;
        //Serial.print("Negative safety tripped!");
        //Serial.println();
      }
      LoadCell_1.set_scale(calibration_factor_1); //apply the new calibration factor to the load cell
      loadCellGo1Reading = (-1 * LoadCell_1.get_units()); //also must update afterwards
    }
    //Serial.print("mass reading:");
    //Serial.print(loadCellGo1Reading, DEC);
    //Serial.println();
    //Serial.print("calibration_factor_1=");
    //Serial.print(calibration_factor_1, DEC);
    //Serial.println();


  }

  if ((loadcellcalibratedmass > loadCellGo1Reading) && (loadcellprecision1 == 1) && (loadcellprecision1 != 3)) { //coarse adjustment
    calibration_factor_1 += 100; //large negative number will get closer to 0

    LoadCell_1.set_scale(calibration_factor_1); //apply the new calibration factor to the load cell
    loadCellGo1Reading = (-1 * LoadCell_1.get_units());

    if (loadCellGo1Reading < 0) { //divide by zero? produces large funky negatiive valuers, like underflow.
      if (loadcellprecision1 == 0) {
        calibration_factor_1 -= 1001;
        //Serial.print("Negative safety tripped!");
        //Serial.println();
      }
      if (loadcellprecision1 == 1) {
        calibration_factor_1 -= 101;
        //Serial.print("Negative safety tripped!");
        //Serial.println();
      }
      if (loadcellprecision1 == 2) {
        calibration_factor_1 -= 11;
        //Serial.print("Negative safety tripped!");
        //Serial.println();
      }
      LoadCell_1.set_scale(calibration_factor_1); //apply the new calibration factor to the load cell
      loadCellGo1Reading = (-1 * LoadCell_1.get_units()); //also must update afterwards
    }

    //Serial.print("mass reading:");
    //Serial.print(loadCellGo1Reading, DEC);
    //Serial.println();
    //Serial.print("calibration_factor_1=");
    //Serial.print(calibration_factor_1, DEC);
    //Serial.println();


  }
  if ((loadcellcalibratedmass > loadCellGo1Reading) && (loadcellprecision1 == 2) && (loadcellprecision1 != 3)) { //fine adjustment
    calibration_factor_1 += 10; //large negative number will get closer to 0

    LoadCell_1.set_scale(calibration_factor_1); //apply the new calibration factor to the load cell
    loadCellGo1Reading = (-1 * LoadCell_1.get_units());

    if (loadCellGo1Reading < 0) { //divide by zero? produces large funky negative valuers, like underflow.
      if (loadcellprecision1 == 0) {
        calibration_factor_1 -= 1001;
        //Serial.print("Negative safety tripped!");
        //Serial.println();
      }
      if (loadcellprecision1 == 1) {
        calibration_factor_1 -= 101;
        //Serial.print("Negative safety tripped!");
        //Serial.println();
      }
      if (loadcellprecision1 == 2) {
        calibration_factor_1 -= 11;
        //Serial.print("Negative safety tripped!");
        //Serial.println();
      }
      LoadCell_1.set_scale(calibration_factor_1); //apply the new calibration factor to the load cell
      loadCellGo1Reading = (-1 * LoadCell_1.get_units()); //also must update afterwards
    }

    //Serial.print("mass reading:");
    //Serial.print(loadCellGo1Reading, DEC);
    //Serial.println();
    //Serial.print("calibration_factor_1=");
    //Serial.print(calibration_factor_1, DEC);
    //Serial.println();
  }

  if (loadcellcalibratedmass <= loadCellGo1Reading) { //we check here but we also MUST check in the loop because otherwise the calibratedmass vs reading wont get checked
    //Serial.print("loadcellcalibratedmass <= loadCellGo1Reading!!");
    //Serial.println();
    if (loadcellprecision1 == 0 && loadcellprecision1 != 1 && loadcellprecision1 != 2 && (loadcellprecision1 != 3)) { //adds back 1000 to get closer without undershooting. makes the code easier
      calibration_factor_1 -= 1000;//I say "adds back" but calibration factor starts out very negative
      LoadCell_1.set_scale(calibration_factor_1);
      loadCellGo1Reading = (-1 * LoadCell_1.get_units()); //just modified the cal factor, make scale correct
      loadcellprecision1 = 1; //takes us to precision level 1
      //Serial.print("precision level 1");
      //Serial.println();
      //Serial.print("calibration factor:");
      //Serial.print(calibration_factor_1, DEC);
      //Serial.println();
      //Serial.print("mass reading:");
      //Serial.print(loadCellGo1Reading, DEC);
      //Serial.println();
    }
    if ((loadcellcalibratedmass <= loadCellGo1Reading) && loadcellprecision1 == 1 && loadcellprecision1 != 0 && loadcellprecision1 != 2 && (loadcellprecision1 != 3)) {
      calibration_factor_1 -= 100;
      LoadCell_1.set_scale(calibration_factor_1);
      loadCellGo1Reading = (-1 * LoadCell_1.get_units());
      loadcellprecision1 = 2; //takes us to precision level 2
      //Serial.print("precision level 2");
      //Serial.println();
      //Serial.print("calibration factor:");
      //Serial.print(calibration_factor_1, DEC);
      //Serial.println();
      //Serial.print("mass reading:");
      //Serial.print(loadCellGo1Reading, DEC);
      //Serial.println();
    }
    if ((loadcellcalibratedmass <= loadCellGo1Reading) && loadcellprecision1 == 2 && loadcellprecision1 != 0 && loadcellprecision1 != 1 && (loadcellprecision1 != 3)) {
      calibration_factor_1 -= 10;
      LoadCell_1.set_scale(calibration_factor_1);
      loadCellGo1Reading = (-1 * LoadCell_1.get_units());
      loadcellprecision1 = 3; //takes us to precision level 3 and trips the exit routine
      //Serial.print("precision level 3");
      //Serial.println();
      //Serial.print("calibration factor:");
      //Serial.print(calibration_factor_1, DEC);
      //Serial.println();
      //Serial.print("mass reading:");
      //Serial.print(loadCellGo1Reading, DEC);
      //Serial.println();
    }
  }
}



void loadCellGo2() { //calibrate first load cell. This is called after the user input and also is shortcut to based on flag, same way load cell data entry is
  //loadcellcalibratedmass is the user input mass
  //calibration factor will be reset to a definitely-unsolved factor for driving to user mass
  LoadCell_2.set_scale(calibration_factor_2);
  loadCellGo2Reading = (-1 * LoadCell_2.get_units());

  lcd.setCursor(14, 0);
  lcd.print(loadCellGo2Reading, DEC); //start prints at 14
  lcd.setCursor(11, 1);
  lcd.print(calibration_factor_2); //11

  if (loadcellprecision2 == 3) { // you've satisfied 1000, 100, and 10 adjustment speeds
    //exit, lower loadCellGo flag
    loadCellGo2Active = 0; //deactivate the running routine
    calibrate_interactive = 0; //exit calibrate menu
    //Serial.print("loadCellGo2 Exited!");
    //Serial.println();
    //Serial.print("mass reading EXIT:");
    //Serial.print(loadCellGo2Reading, DEC);
    //Serial.println();
    //Serial.print("calibration_factor_2 EXIT=");
    //Serial.print(calibration_factor_2, DEC);
    //Serial.println();
    EEPROM.put(8, calibration_factor_2);
    //Serial.print("calibration_factor_1 stored to EEPROM!");
    //Serial.println();
    delay(3000); //delay to allow user to see that the calibration is done
    displayUpdate(); //to print the applicable menu

  }
  loadCellGo2Reading = (-1 * LoadCell_2.get_units()); //must update before we check or it could be outdated for single-loop corrections
  if ((loadcellcalibratedmass > loadCellGo2Reading) && (loadcellprecision2 == 0) && (loadcellprecision2 != 3)) { //super coarse adjustment
    calibration_factor_2 += 1000; //large negative number will get closer to 0

    LoadCell_2.set_scale(calibration_factor_2); //apply the new calibration factor to the load cell
    loadCellGo2Reading = (-1 * LoadCell_2.get_units()); //also must update afterwards

    if (loadCellGo2Reading < 0) { //divide by zero? produces large funky negatiive valuers, like underflow.
      if (loadcellprecision2 == 0) {
        calibration_factor_2 -= 1001;
        //Serial.print("Negative safety tripped!");
        //Serial.println();
      }
      if (loadcellprecision2 == 1) {
        calibration_factor_2 -= 101;
        //Serial.print("Negative safety tripped!");
        //Serial.println();
      }
      if (loadcellprecision2 == 2) {
        calibration_factor_2 -= 11;
        //Serial.print("Negative safety tripped!");
        //Serial.println();
      }
      LoadCell_2.set_scale(calibration_factor_2); //apply the new calibration factor to the load cell
      loadCellGo2Reading = (-1 * LoadCell_2.get_units()); //also must update afterwards
    }
    //Serial.print("mass reading:");
    //Serial.print(loadCellGo2Reading, DEC);
    //Serial.println();
    //Serial.print("calibration_factor_2=");
    //Serial.print(calibration_factor_2, DEC);
    //Serial.println();
    //CRASH or fails to hit another loop

  }

  if ((loadcellcalibratedmass > loadCellGo2Reading) && (loadcellprecision2 == 1) && (loadcellprecision2 != 3)) { //coarse adjustment
    calibration_factor_2 += 100; //large negative number will get closer to 0

    LoadCell_2.set_scale(calibration_factor_2); //apply the new calibration factor to the load cell
    loadCellGo2Reading = (-1 * LoadCell_2.get_units());

    if (loadCellGo2Reading < 0) { //divide by zero? produces large funky negatiive valuers, like underflow.
      if (loadcellprecision2 == 0) {
        calibration_factor_2 -= 1001;
        //Serial.print("Negative safety tripped!");
        //Serial.println();
      }
      if (loadcellprecision2 == 1) {
        calibration_factor_2 -= 101;
        //Serial.print("Negative safety tripped!");
        //Serial.println();
      }
      if (loadcellprecision2 == 2) {
        calibration_factor_2 -= 11;
        //Serial.print("Negative safety tripped!");
        //Serial.println();
      }
      LoadCell_2.set_scale(calibration_factor_2); //apply the new calibration factor to the load cell
      loadCellGo2Reading = (-1 * LoadCell_2.get_units()); //also must update afterwards
    }

    //Serial.print("mass reading:");
    //Serial.print(loadCellGo2Reading, DEC);
    //Serial.println();
    //Serial.print("calibration_factor_2=");
    //Serial.print(calibration_factor_2, DEC);
    //Serial.println();


  }
  if ((loadcellcalibratedmass > loadCellGo2Reading) && (loadcellprecision2 == 2) && (loadcellprecision2 != 3)) { //fine adjustment
    calibration_factor_2 += 10; //large negative number will get closer to 0

    LoadCell_2.set_scale(calibration_factor_2); //apply the new calibration factor to the load cell
    loadCellGo2Reading = (-1 * LoadCell_2.get_units());

    if (loadCellGo2Reading < 0) { //divide by zero? produces large funky negatiive valuers, like underflow.
      if (loadcellprecision2 == 0) {
        calibration_factor_2 -= 1001;
        //Serial.print("Negative safety tripped!");
        //Serial.println();
      }
      if (loadcellprecision2 == 1) {
        calibration_factor_2 -= 101;
        //Serial.print("Negative safety tripped!");
        //Serial.println();
      }
      if (loadcellprecision2 == 2) {
        calibration_factor_2 -= 11;
        //Serial.print("Negative safety tripped!");
        //Serial.println();
      }
      LoadCell_2.set_scale(calibration_factor_2); //apply the new calibration factor to the load cell
      loadCellGo2Reading = (-1 * LoadCell_2.get_units()); //also must update afterwards
    }

    //Serial.print("mass reading:");
    //Serial.print(loadCellGo2Reading, DEC);
    //Serial.println();
    //Serial.print("calibration_factor_2=");
    //Serial.print(calibration_factor_2, DEC);
    //Serial.println();
  }

  if (loadcellcalibratedmass <= loadCellGo2Reading) { //we check here but we also MUST check in the loop because otherwise the calibratedmass vs reading wont get checked
    //Serial.print("loadcellcalibratedmass <= loadCellGo2Reading!!");
    //Serial.println();
    if (loadcellprecision2 == 0 && loadcellprecision2 != 1 && loadcellprecision2 != 2 && (loadcellprecision2 != 3)) { //adds back 1000 to get closer without undershooting. makes the code easier
      calibration_factor_2 -= 1000;//I say "adds back" but calibration factor starts out very negative
      LoadCell_2.set_scale(calibration_factor_2);
      loadCellGo2Reading = (-1 * LoadCell_2.get_units()); //just modified the cal factor, make scale correct
      loadcellprecision2 = 1; //takes us to precision level 1
      //Serial.print("precision level 1");
      //Serial.println();
      //Serial.print("calibration factor:");
      //Serial.print(calibration_factor_2, DEC);
      //Serial.println();
      //Serial.print("mass reading:");
      //Serial.print(loadCellGo2Reading, DEC);
      //Serial.println();
    }
    if ((loadcellcalibratedmass <= loadCellGo2Reading) && loadcellprecision2 == 1 && loadcellprecision2 != 0 && loadcellprecision2 != 2 && (loadcellprecision2 != 3)) {
      calibration_factor_2 -= 100;
      LoadCell_2.set_scale(calibration_factor_2);
      loadCellGo2Reading = (-1 * LoadCell_2.get_units());
      loadcellprecision2 = 2; //takes us to precision level 2
      //Serial.print("precision level 2");
      //Serial.println();
      //Serial.print("calibration factor:");
      //Serial.print(calibration_factor_2, DEC);
      //Serial.println();
      //Serial.print("mass reading:");
      //Serial.print(loadCellGo2Reading, DEC);
      //Serial.println();
    }
    if ((loadcellcalibratedmass <= loadCellGo2Reading) && loadcellprecision2 == 2 && loadcellprecision2 != 0 && loadcellprecision2 != 1 && (loadcellprecision2 != 3)) {
      calibration_factor_2 -= 10;
      LoadCell_2.set_scale(calibration_factor_2);
      loadCellGo2Reading = (-1 * LoadCell_2.get_units());
      loadcellprecision2 = 3; //takes us to precision level 3 and trips the exit routine
      //Serial.print("precision level 3");
      //Serial.println();
      //Serial.print("calibration factor:");
      //Serial.print(calibration_factor_2, DEC);
      //Serial.println();
      //Serial.print("mass reading:");
      //Serial.print(loadCellGo2Reading, DEC);
      //Serial.println();
    }
  }
}

void loadCellDataEntry() {

  if ((encoder_CW == 1) || (encoder_CCW == 1)) {
    encoder_pulse = 1; //raise the pulse flag
  }
  if (encoder_pulse == 1) {
    //encoder logic block
    if (encoder_CW == 1) { //increase digit
      if ((firstDigitComplete == 0) && (secondDigitComplete == 0) && (thirdDigitComplete == 0)) {
        //operate on hundreds place
        loadcellcalibratedmass = (loadcellcalibratedmass + 100);
        lcd.setCursor(9, 2);
        lcd.print("   "); //blank the line to avoid stale digits
        lcd.setCursor(9, 2);
        lcd.print(loadcellcalibratedmass, DEC);
      }
      if ((firstDigitComplete == 1) && (secondDigitComplete == 0) && (thirdDigitComplete == 0)) {
        //operate on tens place
        loadcellcalibratedmass = (loadcellcalibratedmass + 10);
        lcd.setCursor(9, 2);
        lcd.print("   ");
        lcd.setCursor(9, 2);
        lcd.print(loadcellcalibratedmass, DEC);
      }
      if ((firstDigitComplete == 1) && (secondDigitComplete == 1) && (thirdDigitComplete == 0)) {
        //operate on ones place
        loadcellcalibratedmass = (loadcellcalibratedmass + 1);
        lcd.setCursor(9, 2);
        lcd.print("   ");
        lcd.setCursor(9, 2);
        lcd.print(loadcellcalibratedmass, DEC);
      }
      encoder_CW = 0;
    }
    if (encoder_CCW == 1) { //decrease digit, must set up so users can't input negative numbers! thats what the extra conditon is for
      if ((firstDigitComplete == 0) && (secondDigitComplete == 0) && (thirdDigitComplete == 0) && (loadcellcalibratedmass >= 0)) {
        //operate on hundreds place
        loadcellcalibratedmass = (loadcellcalibratedmass - 100);
        if (loadcellcalibratedmass <= 0) { //no, user, you cannot set it less than 0, not even a bit.
          loadcellcalibratedmass = 0;
          lcd.setCursor(9, 2);
          lcd.print("000"); //this way the user can see it zeroed on them, even if using an older value from a different calibration
        }
        lcd.setCursor(9, 2);
        lcd.print("   "); //blank the line to avoid stale digits
        lcd.setCursor(9, 2);
        lcd.print(loadcellcalibratedmass, DEC);
      }
      if ((firstDigitComplete == 1) && (secondDigitComplete == 0) && (thirdDigitComplete == 0) && (loadcellcalibratedmass >= 0)) {
        //operate on tens place
        loadcellcalibratedmass = (loadcellcalibratedmass - 10);
        if (loadcellcalibratedmass <= 0) { //no, user, you cannot set it less than 0, not even a bit.
          loadcellcalibratedmass = 0;
          lcd.setCursor(9, 2);
          lcd.print("000");
        }
        lcd.setCursor(9, 2);
        lcd.print("   "); //blank the line to avoid stale digits
        lcd.setCursor(9, 2);
        lcd.print(loadcellcalibratedmass, DEC);
      }
      if ((firstDigitComplete == 1) && (secondDigitComplete == 1) && (thirdDigitComplete == 0) && (loadcellcalibratedmass >= 0)) {
        //operate on ones place
        loadcellcalibratedmass = (loadcellcalibratedmass - 1);
        if (loadcellcalibratedmass <= 0) { //no, user, you cannot set it less than 0, not even a bit.
          loadcellcalibratedmass = 0;
          lcd.setCursor(9, 2);
          lcd.print("000");
        }
        lcd.setCursor(9, 2);
        lcd.print("   "); //blank the line to avoid stale digits
        lcd.setCursor(9, 2);
        lcd.print(loadcellcalibratedmass, DEC);
      }
      encoder_CCW = 0;
    }
    encoder_pulse = 0; //reset the pulse flag
    encoder_CW = 0; //if you put in an extra rotate, this will zero it if the move was invalid (out of range), otherwise this results in refreshing the display at speed of loop
    encoder_CCW = 0; //same for other direction
    //print current value of loadcellcalibratedmass to screen
  } //end if encoder pulse
  //submenu flags, these trigger overrides to get you into deeper menus
  if ((digitalRead(ENC_SW) == LOW) && (firstDigitComplete == 0) && (secondDigitComplete == 0) && (thirdDigitComplete == 0)) {
    delay(clickDelay); //allow time for thumb to come off button, important because user will have pressed the button to get into the menu, don't wanna instantly cycle
    firstDigitComplete = 1;
  } //end digitalRead==LOW
  if ((digitalRead(ENC_SW) == LOW) && (firstDigitComplete == 1) && (secondDigitComplete == 0) && (thirdDigitComplete == 0)) {
    delay(clickDelay); //allow time for thumb to come off button, important because user will have pressed the button to get into the menu, don't wanna instantly cycle
    secondDigitComplete = 1;
  }//end digitalRead==LOW
  if ((digitalRead(ENC_SW) == LOW) && (firstDigitComplete == 1) && (secondDigitComplete == 1) && (thirdDigitComplete == 0)) {
    delay(clickDelay); //allow time for thumb to come off button, important because user will have pressed the button to get into the menu, don't wanna instantly cycle
    thirdDigitComplete = 1;
  }//end digitalRead==LOW
  if ((firstDigitComplete == 1) && (secondDigitComplete == 1) && (thirdDigitComplete == 1)) {
    loadCellDataEntryComplete = 1;
  }
} //end loadCellDataEntry

void calibrateMenuUserInput() {
  if (calibrateInteractiveFirstRun == 1) { //we always start at cursor position 1. need to set this in the menu you're coming from, otherwise wont hover anything
    calibrate_hover_1 = 1;
    calibrateInteractiveFirstRun = 0; //no longer the first run, so set this flag to not run this loop again until you reset it
    displayUpdate(); //let the user see the submenu faster
  }// end calibrateInteractiveFirstRun
  if (loadCellCalibrate1_firstrun == 0) { //shortcut, gets us back in loadcellcalibrate1 and data entry without navigating through encoder_pulse
    loadcellCalibrate1();
  }
  if (loadCellGo1Active == 1) { //similar shortcut, this shorts the data entry thing and lets us get into the actual driving section
    loadCellGo1();
  }
  if (loadCellGo2Active == 1) { //similar shortcut, this shorts the data entry thing and lets us get into the actual driving section
    loadCellGo2();
  }
  if (loadCellCalibrate2_firstrun == 0) { //have to do the same thing with 2, or it won't take input
    loadcellCalibrate2();
  }
  if (calibrateInteractiveFirstRun == 0 && loadCellCalibrate1_firstrun == 1 && loadCellCalibrate2_firstrun == 1) {//once we are no longer in first run (skip the logic code the first loop), also check to see that we're not in loadcell menu underneath

    if ((encoder_CW == 1) || (encoder_CCW == 1)) {
      encoder_pulse = 1; //raise the pulse flag
    }
    if (encoder_pulse == 1) {
      //encoder logic block
      if ((encoder_CW == 1) && (calibrate_hover_1 == 1)) {
        calibrate_hover_1 = 0;
        calibrate_hover_2 = 1;
        calibrate_hover_3 = 0;
        encoder_CW = 0;
      }
      if ((encoder_CW == 1) && (calibrate_hover_2 == 1)) {
        calibrate_hover_1 = 0;
        calibrate_hover_2 = 0;
        calibrate_hover_3 = 1;
        encoder_CW = 0;
      }

      if ((encoder_CCW == 1) && (calibrate_hover_3 == 1)) {
        calibrate_hover_1 = 0;
        calibrate_hover_2 = 1;
        calibrate_hover_3 = 0;
        encoder_CCW = 0;
      }
      if ((encoder_CCW == 1) && (calibrate_hover_2 == 1)) {
        calibrate_hover_1 = 1;
        calibrate_hover_2 = 0;
        calibrate_hover_3 = 0;
        encoder_CCW = 0;
      }
      encoder_pulse = 0; //reset the pulse flag
      encoder_CW = 0; //if you put in an extra rotate, this will zero it if the move was invalid (out of range), otherwise this results in refreshing the display at speed of loop
      encoder_CCW = 0; //same for other direction
      displayUpdate(); //update the display if and only if the user made a change, and do it after the values have been modified
    } //end if encoder pulse
    //submenu flags, these trigger overrides to get you into deeper menus
    if ((digitalRead(ENC_SW) == LOW)) {
      delay(clickDelay); //allow time for thumb to come off button, important because user will have pressed the button to get into the menu, don't wanna instantly cycle
      //Serial.print("you pressed the button!");
      if (calibrate_hover_1 == 1) {
        calibrate_hover_1 = 0;
        calibrate_interactive = 0; //return to the main interactive submenu
        displayUpdate(); //does one update, lets the user see their menu faster. makes it _feel_ faster
      }
      if (calibrate_hover_2 == 1) {
        calibrate_hover_2 = 0;
        loadCellCalibrate1_firstrun = 1;
        firstDigitComplete = 0;
        secondDigitComplete = 0;
        thirdDigitComplete = 0;
        loadCellDataEntryComplete = 0; //gotta reset al lthis so you can calibrate again if you want. if you don't, it's hard to do  common operation of calibrate one first, then other one
        loadcellCalibrate1();
      }
      if (calibrate_hover_3 == 1) {
        calibrate_hover_3 = 0;
        loadCellCalibrate2_firstrun = 1;
        firstDigitComplete = 0;
        secondDigitComplete = 0;
        thirdDigitComplete = 0;
        loadCellDataEntryComplete = 0;
        loadcellCalibrate2();
      }
    } //end digitalRead==LOW
  } // end "no longer the first run"
} //end calibrateMenuUserInput


void serialMenuUserInput() {
  if (serialInteractiveFirstRun == 1) { //we always start at cursor position 1. need to set this in the menu you're coming from, otherwise wont hover anything
    serial_hover_1 = 1;
    serialInteractiveFirstRun = 0; //no longer the first run, so set this flag to not run this loop again until you reset it by navigating to the non-interactive main menu
    displayUpdate(); //let the user see the submenu faster
  }// end serialInteractiveFirstRun

  if (serialInteractiveFirstRun == 0) { //once we are no longer in first run (skip the logic code the first loop)
    if ((encoder_CW == 1) || (encoder_CCW == 1)) {
      encoder_pulse = 1; //raise the pulse flag
    }
    if (encoder_pulse == 1) {
      //encoder logic block
      if ((encoder_CW == 1) && (serial_hover_1 == 1)) {
        serial_hover_1 = 0;
        serial_hover_2 = 1;
        serial_hover_3 = 0;
        serial_hover_4 = 0;
        encoder_CW = 0;
      }
      if ((encoder_CW == 1) && (serial_hover_2 == 1)) {
        serial_hover_1 = 0;
        serial_hover_2 = 0;
        serial_hover_3 = 1;
        serial_hover_4 = 0;
        encoder_CW = 0;
      }
      if ((encoder_CW == 1) && (serial_hover_3 == 1)) {
        serial_hover_1 = 0;
        serial_hover_2 = 0;
        serial_hover_3 = 0;
        serial_hover_4 = 1;
        encoder_CW = 0;
      }
      if ((encoder_CCW == 1) && (serial_hover_4 == 1)) {
        serial_hover_1 = 0;
        serial_hover_2 = 0;
        serial_hover_3 = 1;
        serial_hover_4 = 0;
        encoder_CCW = 0;
      }
      if ((encoder_CCW == 1) && (serial_hover_3 == 1)) {
        serial_hover_1 = 0;
        serial_hover_2 = 1;
        serial_hover_3 = 0;
        serial_hover_4 = 0;
        encoder_CCW = 0;
      }
      if ((encoder_CCW == 1) && (serial_hover_2 == 1)) {
        serial_hover_1 = 1;
        serial_hover_2 = 0;
        serial_hover_3 = 0;
        serial_hover_4 = 0;
        encoder_CCW = 0;
      }
      encoder_pulse = 0; //reset the pulse flag
      encoder_CW = 0; //if you put in an extra rotate, this will zero it if the move was invalid (out of range), otherwise this results in refreshing the display at speed of loop
      encoder_CCW = 0; //same for other direction
      displayUpdate(); //update the display if and only if the user made a change, and do it after the values have been modified
    } //end if encoder pulse
    //submenu flags, these trigger overrides to get you into deeper menus
    if ((digitalRead(ENC_SW) == LOW)) {
      delay(clickDelay); //allow time for thumb to come off button, important because user will have pressed the button to get into the menu, don't wanna instantly cycle
      //Serial.print("you pressed the button!");
      if (serial_hover_1 == 1) {
        serial_interactive = 0; //return to the main interactive submenu
        displayUpdate(); //does one update, lets the user see their menu faster. makes it _feel_ faster
      }
      if (serial_hover_2 == 1) {
        current_serial_info(); //calls the display directly to save lots of coding
      }
      if (serial_hover_3 == 1) {
        //change baud rate
        baudInteractiveFirstRun = 1; //or baud menu won't work
        baud_interactive = 1;
      }
      if (serial_hover_4 == 1) {
        //Reset serial port to default settings (can actually put the action here since it doesn't display anything)
        serialBaudRate = 115200;
        Serial.flush(); // wait for last transmitted data to be sent
        Serial.begin(115200);//this is default anyway so no need to use this unless you changed it and are now changing it back
        while (Serial.available()) Serial.read(); // empty  out possible garbage from input buffer
        // if the device was sending data while you changed the baud rate, the info in the input buffer
        // is corrupted.
        serial_interactive = 0; //return to the main interactive menu
        main_interactive = 0; //no, you know what? we're going straight to the main menu

        //fully reset everything so it's clean for the first runs when the menus get re-entered
        serialInteractiveFirstRun = 1;
        serial_hover_1 = 0;
        serial_hover_2 = 0;
        serial_hover_3 = 0;
        serial_hover_4 = 0;
        mainInteractiveFirstRun = 1; //pretty sure this isnt needed because main menu does this every display loop but just so we know we did it here already...
        main_hover_1 = 0;
        main_hover_2 = 0;
        main_hover_3 = 0;
        main_hover_4 = 0;
        main_hover_5 = 0;
        main_hover_6 = 0;
        main_visible_1to4 = 0;
        main_visible_2to5 = 0;
        main_visible_3to6 = 0;
      }
    } //end digitalRead==LOW
  } // end "no longer the first run"
} //end serialMenuUserInput

void sensorStatsMenuUserInput() {
  if (sensorStatsInteractiveFirstRun == 1) { //we always start at cursor position 1. need to set this in the menu you're coming from, otherwise wont hover anything
    sensorstats_hover_1 = 1;
    sensorStatsInteractiveFirstRun = 0; //no longer the first run, so set this flag to not run this loop again until you reset it by navigating to the non-interactive main menu
    displayUpdate(); //let the user see the submenu faster
  }// end serialInteractiveFirstRun

  if (sensorStatsInteractiveFirstRun == 0) { //once we are no longer in first run (skip the logic code the first loop)
    if ((encoder_CW == 1) || (encoder_CCW == 1)) {
      encoder_pulse = 1; //raise the pulse flag
    }
    if (encoder_pulse == 1) {
      //encoder logic block
      if ((encoder_CW == 1) && (sensorstats_hover_1 == 1)) {
        sensorstats_hover_1 = 0;
        sensorstats_hover_2 = 1;
        sensorstats_hover_3 = 0;
        sensorstats_hover_4 = 0;
        encoder_CW = 0;
      }
      if ((encoder_CW == 1) && (sensorstats_hover_2 == 1)) {
        sensorstats_hover_1 = 0;
        sensorstats_hover_2 = 0;
        sensorstats_hover_3 = 1;
        sensorstats_hover_4 = 0;
        encoder_CW = 0;
      }
      if ((encoder_CW == 1) && (sensorstats_hover_3 == 1)) {
        sensorstats_hover_1 = 0;
        sensorstats_hover_2 = 0;
        sensorstats_hover_3 = 0;
        sensorstats_hover_4 = 1;
        encoder_CW = 0;
      }
      if ((encoder_CCW == 1) && (sensorstats_hover_4 == 1)) {
        sensorstats_hover_1 = 0;
        sensorstats_hover_2 = 0;
        sensorstats_hover_3 = 1;
        sensorstats_hover_4 = 0;
        encoder_CCW = 0;
      }
      if ((encoder_CCW == 1) && (sensorstats_hover_3 == 1)) {
        sensorstats_hover_1 = 0;
        sensorstats_hover_2 = 1;
        sensorstats_hover_3 = 0;
        sensorstats_hover_4 = 0;
        encoder_CCW = 0;
      }
      if ((encoder_CCW == 1) && (sensorstats_hover_2 == 1)) {
        sensorstats_hover_1 = 1;
        sensorstats_hover_2 = 0;
        sensorstats_hover_3 = 0;
        sensorstats_hover_4 = 0;
        encoder_CCW = 0;
      }
      encoder_pulse = 0; //reset the pulse flag
      encoder_CW = 0; //if you put in an extra rotate, this will zero it if the move was invalid (out of range), otherwise this results in refreshing the display at speed of loop
      encoder_CCW = 0; //same for other direction
      displayUpdate(); //update the display if and only if the user made a change, and do it after the values have been modified
    } //end if encoder pulse
    //submenu flags, these trigger overrides to get you into deeper menus
    if ((digitalRead(ENC_SW) == LOW)) {
      delay(clickDelay); //allow time for thumb to come off button, important because user will have pressed the button to get into the menu, don't wanna instantly cycle
      //Serial.print("you pressed the button!");
      if (sensorstats_hover_1 == 1) {
        sensorstats_interactive = 0; //return to the main interactive submenu
        displayUpdate(); //does one update, lets the user see their menu faster. makes it _feel_ faster
      }
      if (sensorstats_hover_2 == 1) { //barometer page
        if ((barofail_1 == 0) && (barofail_2 == 0) && (barofail_3 == 0) && (barofail_4 == 0) && (barofail_5 == 0) && (barofail_6 == 0) && (barofail_7 == 0)) {
          baroStatsInteractiveFirstRun = 1; //this logic is copied from the baud menu because of how long it will be
          lcd.clear(); //removes the old cursor, this is needed for optimized display of fast objects.
          barostats_interactive = 1;
        }

      }
      if (sensorstats_hover_3 == 1) { //anemometer page
        if (anemfail_notfound == 0) {
          //direct display flag raise, gets called directly from loop, not displayUpdate
          lcd.clear(); //same idea as the barometer one, but we can control the whole screen without scroll, this is the only time it gets cleared
          anemStatsInteractiveFirstRun = 1;
          anemstats_interactive = 1; //not really interactive but the naming convention is kinda there?
        }
      }
      if (sensorstats_hover_4 == 1) {
        if ((ignoreLoadCell_1 == 0) && (ignoreLoadCell_2 == 0)) {
          //direct display flag raise, gets called directly from loop, not displayUpdate
          lcd.clear(); //same idea as the barometer one, but we can control the whole screen without scroll, this is the only time it gets cleared
          loadStatsInteractiveFirstRun = 1;
          loadstats_interactive = 1; //not really interactive but the naming convention is kinda there?
        }
      }
    } //end digitalRead==LOW
  } // end "no longer the first run"
} //end sensorStatsMenuUserInput


void current_serial_info() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Baud:");
  lcd.setCursor(7, 0);
  lcd.print(serialBaudRate, DEC);
  //Serial.print("serialBaudRate=");
  //Serial.print(serialBaudRate);
  //Serial.println();
  lcd.setCursor(0, 1);
  lcd.print("DBits: 8 SBits: 1");
  lcd.setCursor(0, 2);
  lcd.print("Parity: No");
  lcd.setCursor(0, 3);
  lcd.print("Flow Ctr:XON/XOFF");
  while (digitalRead(ENC_SW) == HIGH) { // this block pauses all code execution while user reads and acknowledges the display. ENC_SW is the button pin
    // Do nothing
  }
  delay(clickDelay);
  displayUpdate(); //no longer ticking display automatically so this needs to call to take this display down
}

void baudMenuUserInput() { //suuuuuper long
  if (baudInteractiveFirstRun == 1) { //we always start at 1, 1-4
    baud_hover_1 = 1;
    baud_visible_1to4 = 1;
    baudInteractiveFirstRun = 0; //no longer the first run, so set this flag to not run this loop again until you reset it by navigating to the non-interactive baud menu
    displayUpdate(); //let the user see the submenu faster
  } // end baudInteractiveFirstRun == 1 if loop

  if (baudInteractiveFirstRun == 0) { //once we are no longer in first run (skip the logic code the first loop)
    if ((encoder_CW == 1) || (encoder_CCW == 1)) {
      encoder_pulse = 1; //raise the pulse flag
    }
    if (encoder_pulse == 1) {
      //encoder logic block
      if ((encoder_CW == 1) && (baud_hover_1 == 1)) {
        baud_hover_1 = 0;
        baud_hover_2 = 1;
        baud_hover_3 = 0;
        baud_hover_4 = 0;
        baud_hover_5 = 0;
        baud_hover_6 = 0;
        baud_hover_7 = 0;
        baud_hover_8 = 0;
        baud_hover_9 = 0;
        baud_hover_10 = 0;
        baud_hover_11 = 0;
        baud_hover_12 = 0;
        baud_hover_13 = 0;
        encoder_CW = 0;
      }
      if ((encoder_CW == 1) && (baud_hover_2 == 1)) {
        baud_hover_1 = 0;
        baud_hover_2 = 0;
        baud_hover_3 = 1;
        baud_hover_4 = 0;
        baud_hover_5 = 0;
        baud_hover_6 = 0;
        baud_hover_7 = 0;
        baud_hover_8 = 0;
        baud_hover_9 = 0;
        baud_hover_10 = 0;
        baud_hover_11 = 0;
        baud_hover_12 = 0;
        baud_hover_13 = 0;
        encoder_CW = 0;
      }
      if ((encoder_CW == 1) && (baud_hover_3 == 1)) {
        baud_hover_1 = 0;
        baud_hover_2 = 0;
        baud_hover_3 = 0;
        baud_hover_4 = 1;
        baud_hover_5 = 0;
        baud_hover_6 = 0;
        baud_hover_7 = 0;
        baud_hover_8 = 0;
        baud_hover_9 = 0;
        baud_hover_10 = 0;
        baud_hover_11 = 0;
        baud_hover_12 = 0;
        baud_hover_13 = 0;
        encoder_CW = 0;
      }
      if ((encoder_CW == 1) && (baud_hover_4 == 1)) {
        baud_hover_1 = 0;
        baud_hover_2 = 0;
        baud_hover_3 = 0;
        baud_hover_4 = 0;
        baud_hover_5 = 1;
        baud_hover_6 = 0;
        baud_hover_7 = 0;
        baud_hover_8 = 0;
        baud_hover_9 = 0;
        baud_hover_10 = 0;
        baud_hover_11 = 0;
        baud_hover_12 = 0;
        baud_hover_13 = 0;
        encoder_CW = 0;
      }
      if ((encoder_CW == 1) && (baud_hover_5 == 1)) {
        baud_hover_1 = 0;
        baud_hover_2 = 0;
        baud_hover_3 = 0;
        baud_hover_4 = 0;
        baud_hover_5 = 0;
        baud_hover_6 = 1;
        baud_hover_7 = 0;
        baud_hover_8 = 0;
        baud_hover_9 = 0;
        baud_hover_10 = 0;
        baud_hover_11 = 0;
        baud_hover_12 = 0;
        baud_hover_13 = 0;
        encoder_CW = 0;
      }
      if ((encoder_CW == 1) && (baud_hover_6 == 1)) {
        baud_hover_1 = 0;
        baud_hover_2 = 0;
        baud_hover_3 = 0;
        baud_hover_4 = 0;
        baud_hover_5 = 0;
        baud_hover_6 = 0;
        baud_hover_7 = 1;
        baud_hover_8 = 0;
        baud_hover_9 = 0;
        baud_hover_10 = 0;
        baud_hover_11 = 0;
        baud_hover_12 = 0;
        baud_hover_13 = 0;
        encoder_CW = 0;
      }
      if ((encoder_CW == 1) && (baud_hover_7 == 1)) {
        baud_hover_1 = 0;
        baud_hover_2 = 0;
        baud_hover_3 = 0;
        baud_hover_4 = 0;
        baud_hover_5 = 0;
        baud_hover_6 = 0;
        baud_hover_7 = 0;
        baud_hover_8 = 1;
        baud_hover_9 = 0;
        baud_hover_10 = 0;
        baud_hover_11 = 0;
        baud_hover_12 = 0;
        baud_hover_13 = 0;
        encoder_CW = 0;
      }
      if ((encoder_CW == 1) && (baud_hover_8 == 1)) {
        baud_hover_1 = 0;
        baud_hover_2 = 0;
        baud_hover_3 = 0;
        baud_hover_4 = 0;
        baud_hover_5 = 0;
        baud_hover_6 = 0;
        baud_hover_7 = 0;
        baud_hover_8 = 0;
        baud_hover_9 = 1;
        baud_hover_10 = 0;
        baud_hover_11 = 0;
        baud_hover_12 = 0;
        baud_hover_13 = 0;
        encoder_CW = 0;
      }
      if ((encoder_CW == 1) && (baud_hover_9 == 1)) {
        baud_hover_1 = 0;
        baud_hover_2 = 0;
        baud_hover_3 = 0;
        baud_hover_4 = 0;
        baud_hover_5 = 0;
        baud_hover_6 = 0;
        baud_hover_7 = 0;
        baud_hover_8 = 0;
        baud_hover_9 = 0;
        baud_hover_10 = 1;
        baud_hover_11 = 0;
        baud_hover_12 = 0;
        baud_hover_13 = 0;
        encoder_CW = 0;
      }
      if ((encoder_CW == 1) && (baud_hover_10 == 1)) {
        baud_hover_1 = 0;
        baud_hover_2 = 0;
        baud_hover_3 = 0;
        baud_hover_4 = 0;
        baud_hover_5 = 0;
        baud_hover_6 = 0;
        baud_hover_7 = 0;
        baud_hover_8 = 0;
        baud_hover_9 = 0;
        baud_hover_10 = 0;
        baud_hover_11 = 1;
        baud_hover_12 = 0;
        baud_hover_13 = 0;
        encoder_CW = 0;
      }
      if ((encoder_CW == 1) && (baud_hover_11 == 1)) {
        baud_hover_1 = 0;
        baud_hover_2 = 0;
        baud_hover_3 = 0;
        baud_hover_4 = 0;
        baud_hover_5 = 0;
        baud_hover_6 = 0;
        baud_hover_7 = 0;
        baud_hover_8 = 0;
        baud_hover_9 = 0;
        baud_hover_10 = 0;
        baud_hover_11 = 0;
        baud_hover_12 = 1;
        baud_hover_13 = 0;
        encoder_CW = 0;
      }
      if ((encoder_CW == 1) && (baud_hover_12 == 1)) {
        baud_hover_1 = 0;
        baud_hover_2 = 0;
        baud_hover_3 = 0;
        baud_hover_4 = 0;
        baud_hover_5 = 0;
        baud_hover_6 = 0;
        baud_hover_7 = 0;
        baud_hover_8 = 0;
        baud_hover_9 = 0;
        baud_hover_10 = 0;
        baud_hover_11 = 0;
        baud_hover_12 = 0;
        baud_hover_13 = 1;
        encoder_CW = 0;
      }
      if ((encoder_CCW == 1) && (baud_hover_13 == 1)) {
        baud_hover_1 = 0;
        baud_hover_2 = 0;
        baud_hover_3 = 0;
        baud_hover_4 = 0;
        baud_hover_5 = 0;
        baud_hover_6 = 0;
        baud_hover_7 = 0;
        baud_hover_8 = 0;
        baud_hover_9 = 0;
        baud_hover_10 = 0;
        baud_hover_11 = 0;
        baud_hover_12 = 1;
        baud_hover_13 = 0;
        encoder_CCW = 0;
      }
      if ((encoder_CCW == 1) && (baud_hover_12 == 1)) {
        baud_hover_1 = 0;
        baud_hover_2 = 0;
        baud_hover_3 = 0;
        baud_hover_4 = 0;
        baud_hover_5 = 0;
        baud_hover_6 = 0;
        baud_hover_7 = 0;
        baud_hover_8 = 0;
        baud_hover_9 = 0;
        baud_hover_10 = 0;
        baud_hover_11 = 1;
        baud_hover_12 = 0;
        baud_hover_13 = 0;
        encoder_CCW = 0;
      }
      if ((encoder_CCW == 1) && (baud_hover_11 == 1)) {
        baud_hover_1 = 0;
        baud_hover_2 = 0;
        baud_hover_3 = 0;
        baud_hover_4 = 0;
        baud_hover_5 = 0;
        baud_hover_6 = 0;
        baud_hover_7 = 0;
        baud_hover_8 = 0;
        baud_hover_9 = 0;
        baud_hover_10 = 1;
        baud_hover_11 = 0;
        baud_hover_12 = 0;
        baud_hover_13 = 0;
        encoder_CCW = 0;
      }
      if ((encoder_CCW == 1) && (baud_hover_10 == 1)) {
        baud_hover_1 = 0;
        baud_hover_2 = 0;
        baud_hover_3 = 0;
        baud_hover_4 = 0;
        baud_hover_5 = 0;
        baud_hover_6 = 0;
        baud_hover_7 = 0;
        baud_hover_8 = 0;
        baud_hover_9 = 1;
        baud_hover_10 = 0;
        baud_hover_11 = 0;
        baud_hover_12 = 0;
        baud_hover_13 = 0;
        encoder_CCW = 0;
      }
      if ((encoder_CCW == 1) && (baud_hover_9 == 1)) {
        baud_hover_1 = 0;
        baud_hover_2 = 0;
        baud_hover_3 = 0;
        baud_hover_4 = 0;
        baud_hover_5 = 0;
        baud_hover_6 = 0;
        baud_hover_7 = 0;
        baud_hover_8 = 1;
        baud_hover_9 = 0;
        baud_hover_10 = 0;
        baud_hover_11 = 0;
        baud_hover_12 = 0;
        baud_hover_13 = 0;
        encoder_CCW = 0;
      }
      if ((encoder_CCW == 1) && (baud_hover_8 == 1)) {
        baud_hover_1 = 0;
        baud_hover_2 = 0;
        baud_hover_3 = 0;
        baud_hover_4 = 0;
        baud_hover_5 = 0;
        baud_hover_6 = 0;
        baud_hover_7 = 1;
        baud_hover_8 = 0;
        baud_hover_9 = 0;
        baud_hover_10 = 0;
        baud_hover_11 = 0;
        baud_hover_12 = 0;
        baud_hover_13 = 0;
        encoder_CCW = 0;
      }
      if ((encoder_CCW == 1) && (baud_hover_7 == 1)) {
        baud_hover_1 = 0;
        baud_hover_2 = 0;
        baud_hover_3 = 0;
        baud_hover_4 = 0;
        baud_hover_5 = 0;
        baud_hover_6 = 1;
        baud_hover_7 = 0;
        baud_hover_8 = 0;
        baud_hover_9 = 0;
        baud_hover_10 = 0;
        baud_hover_11 = 0;
        baud_hover_12 = 0;
        baud_hover_13 = 0;
        encoder_CCW = 0;
      }
      if ((encoder_CCW == 1) && (baud_hover_6 == 1)) {
        baud_hover_1 = 0;
        baud_hover_2 = 0;
        baud_hover_3 = 0;
        baud_hover_4 = 0;
        baud_hover_5 = 1;
        baud_hover_6 = 0;
        baud_hover_7 = 0;
        baud_hover_8 = 0;
        baud_hover_9 = 0;
        baud_hover_10 = 0;
        baud_hover_11 = 0;
        baud_hover_12 = 0;
        baud_hover_13 = 0;
        encoder_CCW = 0;
      }
      if ((encoder_CCW == 1) && (baud_hover_5 == 1)) {
        baud_hover_1 = 0;
        baud_hover_2 = 0;
        baud_hover_3 = 0;
        baud_hover_4 = 1;
        baud_hover_5 = 0;
        baud_hover_6 = 0;
        baud_hover_7 = 0;
        baud_hover_8 = 0;
        baud_hover_9 = 0;
        baud_hover_10 = 0;
        baud_hover_11 = 0;
        baud_hover_12 = 0;
        baud_hover_13 = 0;
        encoder_CCW = 0;
      }
      if ((encoder_CCW == 1) && (baud_hover_4 == 1)) {
        baud_hover_1 = 0;
        baud_hover_2 = 0;
        baud_hover_3 = 1;
        baud_hover_4 = 0;
        baud_hover_5 = 0;
        baud_hover_6 = 0;
        baud_hover_7 = 0;
        baud_hover_8 = 0;
        baud_hover_9 = 0;
        baud_hover_10 = 0;
        baud_hover_11 = 0;
        baud_hover_12 = 0;
        baud_hover_13 = 0;
        encoder_CCW = 0;
      }
      if ((encoder_CCW == 1) && (baud_hover_3 == 1)) {
        baud_hover_1 = 0;
        baud_hover_2 = 1;
        baud_hover_3 = 0;
        baud_hover_4 = 0;
        baud_hover_5 = 0;
        baud_hover_6 = 0;
        baud_hover_7 = 0;
        baud_hover_8 = 0;
        baud_hover_9 = 0;
        baud_hover_10 = 0;
        baud_hover_11 = 0;
        baud_hover_12 = 0;
        baud_hover_13 = 0;
        encoder_CCW = 0;
      }
      if ((encoder_CCW == 1) && (baud_hover_2 == 1)) {
        baud_hover_1 = 1;
        baud_hover_2 = 0;
        baud_hover_3 = 0;
        baud_hover_4 = 0;
        baud_hover_5 = 0;
        baud_hover_6 = 0;
        baud_hover_7 = 0;
        baud_hover_8 = 0;
        baud_hover_9 = 0;
        baud_hover_10 = 0;
        baud_hover_11 = 0;
        baud_hover_12 = 0;
        baud_hover_13 = 0;
        encoder_CCW = 0;
      }
      if (baud_visible_1to4 == 1) {
        if (baud_hover_5 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 1;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_6 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 1;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_7 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 1;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_8 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 1;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_9 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 1;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_10 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 1;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_11 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 1;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_12 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 1;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_13 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 1;
        }
      }
      if (baud_visible_2to5 == 1) {
        if (baud_hover_1 == 1) { //scroll up
          baud_visible_1to4 = 1;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_6 == 1) { //scroll down
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 1;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_7 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 1;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_8 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 1;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_9 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 1;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_10 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 1;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_11 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 1;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_12 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 1;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_13 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 1;
        }
      }
      if (baud_visible_3to6 == 1) {
        if (baud_hover_1 == 1) { //scroll up
          baud_visible_1to4 = 1;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_2 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 1;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_7 == 1) {//scroll down
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 1;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_8 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 1;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_9 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 1;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_10 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 1;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_11 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 1;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_12 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 1;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_13 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 1;
        }
      }
      if (baud_visible_4to7 == 1) {
        if (baud_hover_1 == 1) { //scroll up
          baud_visible_1to4 = 1;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_2 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 1;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_3 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 1;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_8 == 1) {//scroll down
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 1;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_9 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 1;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_10 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 1;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_11 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 1;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_12 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 1;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_13 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 1;
        }
      }
      if (baud_visible_5to8 == 1) {
        if (baud_hover_1 == 1) { //scroll up
          baud_visible_1to4 = 1;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_2 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 1;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_3 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 1;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_4 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 1;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_9 == 1) {//scroll down
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 1;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_10 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 1;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_11 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 1;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_12 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 1;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_13 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 1;
        }
      }
      if (baud_visible_6to9 == 1) {
        if (baud_hover_1 == 1) { //scroll up
          baud_visible_1to4 = 1;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_2 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 1;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_3 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 1;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_4 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 1;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_5 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 1;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_10 == 1) {//scroll down
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 1;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_11 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 1;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_12 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 1;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_13 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 1;
        }
      }
      if (baud_visible_7to10 == 1) {
        if (baud_hover_1 == 1) { //scroll up
          baud_visible_1to4 = 1;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_2 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 1;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_3 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 1;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_4 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 1;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_5 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 1;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_6 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 1;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_11 == 1) {//scroll down
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 1;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_12 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 1;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_13 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 1;
        }
      }
      if (baud_visible_8to11 == 1) {
        if (baud_hover_1 == 1) { //scroll up
          baud_visible_1to4 = 1;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_2 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 1;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_3 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 1;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_4 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 1;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_5 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 1;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_6 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 1;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_7 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 1;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_12 == 1) {//scroll down
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 1;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_13 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 1;
        }
      }
      if (baud_visible_9to12 == 1) {
        if (baud_hover_1 == 1) { //scroll up
          baud_visible_1to4 = 1;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_2 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 1;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_3 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 1;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_4 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 1;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_5 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 1;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_6 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 1;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_7 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 1;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_8 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 1;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_13 == 1) {//scroll down
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 1;
        }
      }
      if (baud_visible_10to13 == 1) {
        if (baud_hover_1 == 1) { //scroll up
          baud_visible_1to4 = 1;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_2 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 1;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_3 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 1;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_4 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 1;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_5 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 1;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_6 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 1;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_7 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 1;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_8 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 1;
          baud_visible_9to12 = 0;
          baud_visible_10to13 = 0;
        }
        if (baud_hover_9 == 1) {
          baud_visible_1to4 = 0;
          baud_visible_2to5 = 0;
          baud_visible_3to6 = 0;
          baud_visible_4to7 = 0;
          baud_visible_5to8 = 0;
          baud_visible_6to9 = 0;
          baud_visible_7to10 = 0;
          baud_visible_8to11 = 0;
          baud_visible_9to12 = 1;
          baud_visible_10to13 = 0;
        }
      }
      encoder_pulse = 0; //reset the pulse flag
      encoder_CW = 0; //if you put in an extra rotate, this will zero it if the move was invalid (out of range), otherwise this results in refreshing the display at speed of loop
      encoder_CCW = 0; //same for other direction
      displayUpdate(); //update the display if and only if the user made a change, and do it after the flags have been modified
    } //end if encoder pulse
    //selection flags, will change baud to what you need it to be
    if ((digitalRead(ENC_SW) == LOW)) {
      delay(clickDelay); //allow time for thumb to come off button, important because user will have pressed the button, don't wanna instantly cycle
      //Serial.print("you pressed the button!");
      if (baud_hover_1 == 1) {
        baud_interactive = 0; //return to the serial menu
        displayUpdate(); //does one update, lets the user see their menu faster. makes it _feel_ faster
      }
      if (baud_hover_2 == 1) {
        serialBaudRate = 300;
        Serial.flush(); // wait for last transmitted data to be sent
        Serial.begin(300);
        while (Serial.available()) Serial.read(); // empty  out possible garbage from input buffer
        // if the device was sending data while you changed the baud rate, the info in the input buffer
        // is corrupted.
        baud_interactive = 0; //return to the serial menu
        displayUpdate();
      }
      if (baud_hover_3 == 1) {
        serialBaudRate = 600;
        Serial.flush(); // wait for last transmitted data to be sent
        Serial.begin(600);
        while (Serial.available()) Serial.read(); // empty  out possible garbage from input buffer
        // if the device was sending data while you changed the baud rate, the info in the input buffer
        // is corrupted.
        baud_interactive = 0; //return to the serial menu
        displayUpdate();
      }
      if (baud_hover_4 == 1) {
        serialBaudRate = 1200;
        Serial.flush(); // wait for last transmitted data to be sent
        Serial.begin(1200);
        while (Serial.available()) Serial.read(); // empty  out possible garbage from input buffer
        // if the device was sending data while you changed the baud rate, the info in the input buffer
        // is corrupted.
        baud_interactive = 0; //return to the serial menu
        displayUpdate();
      }
      if (baud_hover_5 == 1) {
        serialBaudRate = 2400;
        Serial.flush(); // wait for last transmitted data to be sent
        Serial.begin(2400);
        while (Serial.available()) Serial.read(); // empty  out possible garbage from input buffer
        // if the device was sending data while you changed the baud rate, the info in the input buffer
        // is corrupted.
        baud_interactive = 0; //return to the serial menu
        displayUpdate();
      }
      if (baud_hover_6 == 1) {
        serialBaudRate = 4800;
        Serial.flush(); // wait for last transmitted data to be sent
        Serial.begin(4800);
        while (Serial.available()) Serial.read(); // empty  out possible garbage from input buffer
        // if the device was sending data while you changed the baud rate, the info in the input buffer
        // is corrupted.
        baud_interactive = 0; //return to the serial menu
        displayUpdate();
      }
      if (baud_hover_7 == 1) {
        serialBaudRate = 9600;
        Serial.flush(); // wait for last transmitted data to be sent
        Serial.begin(9600);
        while (Serial.available()) Serial.read(); // empty  out possible garbage from input buffer
        // if the device was sending data while you changed the baud rate, the info in the input buffer
        // is corrupted.
        baud_interactive = 0; //return to the serial menu
        displayUpdate();
      }
      if (baud_hover_8 == 1) {
        serialBaudRate = 14400;
        Serial.flush(); // wait for last transmitted data to be sent
        Serial.begin(14400);
        while (Serial.available()) Serial.read(); // empty  out possible garbage from input buffer
        // if the device was sending data while you changed the baud rate, the info in the input buffer
        // is corrupted.
        baud_interactive = 0; //return to the serial menu
        displayUpdate();
      }
      if (baud_hover_9 == 1) {
        serialBaudRate = 19200;
        Serial.flush(); // wait for last transmitted data to be sent
        Serial.begin(19200);
        while (Serial.available()) Serial.read(); // empty  out possible garbage from input buffer
        // if the device was sending data while you changed the baud rate, the info in the input buffer
        // is corrupted.
        baud_interactive = 0; //return to the serial menu
        displayUpdate();
      }
      if (baud_hover_10 == 1) {
        serialBaudRate = 28800;
        Serial.flush(); // wait for last transmitted data to be sent
        Serial.begin(28800);
        while (Serial.available()) Serial.read(); // empty  out possible garbage from input buffer
        // if the device was sending data while you changed the baud rate, the info in the input buffer
        // is corrupted.
        baud_interactive = 0; //return to the serial menu
        displayUpdate();
      }
      if (baud_hover_11 == 1) {
        serialBaudRate = 38400;
        Serial.flush(); // wait for last transmitted data to be sent
        Serial.begin(38400);
        while (Serial.available()) Serial.read(); // empty  out possible garbage from input buffer
        // if the device was sending data while you changed the baud rate, the info in the input buffer
        // is corrupted.
        baud_interactive = 0; //return to the serial menu
        displayUpdate();
      }
      if (baud_hover_12 == 1) {
        serialBaudRate = 57600;
        Serial.flush(); // wait for last transmitted data to be sent
        Serial.begin(57600);
        while (Serial.available()) Serial.read(); // empty  out possible garbage from input buffer
        // if the device was sending data while you changed the baud rate, the info in the input buffer
        // is corrupted.
        baud_interactive = 0; //return to the serial menu
        displayUpdate();
      }
      if (baud_hover_13 == 1) {
        serialBaudRate = 115200;
        Serial.flush(); // wait for last transmitted data to be sent
        Serial.begin(115200);//this is default anyway so no need to use this unless you changed it and are now changing it back
        while (Serial.available()) Serial.read(); // empty  out possible garbage from input buffer
        // if the device was sending data while you changed the baud rate, the info in the input buffer
        // is corrupted.
        baud_interactive = 0; //return to the serial menu
        displayUpdate();
      }
    } //end digitalRead==LOW
  } // end "no longer the first run"
} //end baudInput

void baroStatsMenuUserInput() { //suuuuuper long
  if (baroStatsInteractiveFirstRun == 1) { //we always start at 1, 1-4
    barostats_hover_1 = 1;
    barostats_visible_1to4 = 1;
    baroStatsInteractiveFirstRun = 0; //no longer the first run, so set this flag to not run this loop again
    displayUpdate(); //let the user see the submenu faster
  } // end baroStatsInteractiveFirstRun == 1 if loop

  if (baroStatsInteractiveFirstRun == 0) { //once we are no longer in first run (skip the logic code the first loop)
    if ((encoder_CW == 1) || (encoder_CCW == 1)) {
      encoder_pulse = 1; //raise the pulse flag
    }
    if (encoder_pulse == 1) {
      //encoder logic block
      if ((encoder_CW == 1) && (barostats_hover_1 == 1)) {
        barostats_hover_1 = 0;
        barostats_hover_2 = 1;
        barostats_hover_3 = 0;
        barostats_hover_4 = 0;
        barostats_hover_5 = 0;
        barostats_hover_6 = 0;
        barostats_hover_7 = 0;
        barostats_hover_8 = 0;
        encoder_CW = 0;
      }
      if ((encoder_CW == 1) && (barostats_hover_2 == 1)) {
        barostats_hover_1 = 0;
        barostats_hover_2 = 0;
        barostats_hover_3 = 1;
        barostats_hover_4 = 0;
        barostats_hover_5 = 0;
        barostats_hover_6 = 0;
        barostats_hover_7 = 0;
        barostats_hover_8 = 0;
        encoder_CW = 0;
      }
      if ((encoder_CW == 1) && (barostats_hover_3 == 1)) {
        barostats_hover_1 = 0;
        barostats_hover_2 = 0;
        barostats_hover_3 = 0;
        barostats_hover_4 = 1;
        barostats_hover_5 = 0;
        barostats_hover_6 = 0;
        barostats_hover_7 = 0;
        barostats_hover_8 = 0;
        encoder_CW = 0;
      }
      if ((encoder_CW == 1) && (barostats_hover_4 == 1)) {
        barostats_hover_1 = 0;
        barostats_hover_2 = 0;
        barostats_hover_3 = 0;
        barostats_hover_4 = 0;
        barostats_hover_5 = 1;
        barostats_hover_6 = 0;
        barostats_hover_7 = 0;
        barostats_hover_8 = 0;
        encoder_CW = 0;
      }
      if ((encoder_CW == 1) && (barostats_hover_5 == 1)) {
        barostats_hover_1 = 0;
        barostats_hover_2 = 0;
        barostats_hover_3 = 0;
        barostats_hover_4 = 0;
        barostats_hover_5 = 0;
        barostats_hover_6 = 1;
        barostats_hover_7 = 0;
        barostats_hover_8 = 0;
        encoder_CW = 0;
      }
      if ((encoder_CW == 1) && (barostats_hover_6 == 1)) {
        barostats_hover_1 = 0;
        barostats_hover_2 = 0;
        barostats_hover_3 = 0;
        barostats_hover_4 = 0;
        barostats_hover_5 = 0;
        barostats_hover_6 = 0;
        barostats_hover_7 = 1;
        barostats_hover_8 = 0;
        encoder_CW = 0;
      }
      if ((encoder_CW == 1) && (barostats_hover_7 == 1)) {
        barostats_hover_1 = 0;
        barostats_hover_2 = 0;
        barostats_hover_3 = 0;
        barostats_hover_4 = 0;
        barostats_hover_5 = 0;
        barostats_hover_6 = 0;
        barostats_hover_7 = 0;
        barostats_hover_8 = 1;
        encoder_CW = 0;
      }
      if ((encoder_CCW == 1) && (barostats_hover_8 == 1)) {
        barostats_hover_1 = 0;
        barostats_hover_2 = 0;
        barostats_hover_3 = 0;
        barostats_hover_4 = 0;
        barostats_hover_5 = 0;
        barostats_hover_6 = 0;
        barostats_hover_7 = 1;
        barostats_hover_8 = 0;
        encoder_CCW = 0;
      }
      if ((encoder_CCW == 1) && (barostats_hover_7 == 1)) {
        barostats_hover_1 = 0;
        barostats_hover_2 = 0;
        barostats_hover_3 = 0;
        barostats_hover_4 = 0;
        barostats_hover_5 = 0;
        barostats_hover_6 = 1;
        barostats_hover_7 = 0;
        barostats_hover_8 = 0;
        encoder_CCW = 0;
      }
      if ((encoder_CCW == 1) && (barostats_hover_6 == 1)) {
        barostats_hover_1 = 0;
        barostats_hover_2 = 0;
        barostats_hover_3 = 0;
        barostats_hover_4 = 0;
        barostats_hover_5 = 1;
        barostats_hover_6 = 0;
        barostats_hover_7 = 0;
        barostats_hover_8 = 0;
        encoder_CCW = 0;
      }
      if ((encoder_CCW == 1) && (barostats_hover_5 == 1)) {
        barostats_hover_1 = 0;
        barostats_hover_2 = 0;
        barostats_hover_3 = 0;
        barostats_hover_4 = 1;
        barostats_hover_5 = 0;
        barostats_hover_6 = 0;
        barostats_hover_7 = 0;
        barostats_hover_8 = 0;
        encoder_CCW = 0;
      }
      if ((encoder_CCW == 1) && (barostats_hover_4 == 1)) {
        barostats_hover_1 = 0;
        barostats_hover_2 = 0;
        barostats_hover_3 = 1;
        barostats_hover_4 = 0;
        barostats_hover_5 = 0;
        barostats_hover_6 = 0;
        barostats_hover_7 = 0;
        barostats_hover_8 = 0;
        encoder_CCW = 0;
      }
      if ((encoder_CCW == 1) && (barostats_hover_3 == 1)) {
        barostats_hover_1 = 0;
        barostats_hover_2 = 1;
        barostats_hover_3 = 0;
        barostats_hover_4 = 0;
        barostats_hover_5 = 0;
        barostats_hover_6 = 0;
        barostats_hover_7 = 0;
        barostats_hover_8 = 0;
        encoder_CCW = 0;
      }
      if ((encoder_CCW == 1) && (barostats_hover_2 == 1)) {
        barostats_hover_1 = 1;
        barostats_hover_2 = 0;
        barostats_hover_3 = 0;
        barostats_hover_4 = 0;
        barostats_hover_5 = 0;
        barostats_hover_6 = 0;
        barostats_hover_7 = 0;
        barostats_hover_8 = 0;
        encoder_CCW = 0;
      }
      //end cursor move
      //begin window moving
      if (barostats_visible_1to4 == 1) { //can only go down, top of range
        if (barostats_hover_5 == 1) {
          barostats_visible_1to4 = 0;
          barostats_visible_2to5 = 1;
          barostats_visible_3to6 = 0;
          barostats_visible_4to7 = 0;
          barostats_visible_5to8 = 0;
        }
        if (barostats_hover_6 == 1) {
          barostats_visible_1to4 = 0;
          barostats_visible_2to5 = 0;
          barostats_visible_3to6 = 1;
          barostats_visible_4to7 = 0;
          barostats_visible_5to8 = 0;
        }
        if (barostats_hover_7 == 1) {
          barostats_visible_1to4 = 0;
          barostats_visible_2to5 = 0;
          barostats_visible_3to6 = 0;
          barostats_visible_4to7 = 1;
          barostats_visible_5to8 = 0;
        }
        if (barostats_hover_8 == 1) {
          barostats_visible_1to4 = 0;
          barostats_visible_2to5 = 0;
          barostats_visible_3to6 = 0;
          barostats_visible_4to7 = 0;
          barostats_visible_5to8 = 1;
        }
      } //end baro 1to4
      if (barostats_visible_2to5 == 1) {
        if (barostats_hover_1 == 1) { //scroll up
          barostats_visible_1to4 = 1;
          barostats_visible_2to5 = 0;
          barostats_visible_3to6 = 0;
          barostats_visible_4to7 = 0;
          barostats_visible_5to8 = 0;
        }
        if (barostats_hover_6 == 1) { //scroll down
          barostats_visible_1to4 = 0;
          barostats_visible_2to5 = 0;
          barostats_visible_3to6 = 1;
          barostats_visible_4to7 = 0;
          barostats_visible_5to8 = 0;
        }
        if (barostats_hover_7 == 1) {
          barostats_visible_1to4 = 0;
          barostats_visible_2to5 = 0;
          barostats_visible_3to6 = 0;
          barostats_visible_4to7 = 1;
          barostats_visible_5to8 = 0;
        }
        if (barostats_hover_8 == 1) {
          barostats_visible_1to4 = 0;
          barostats_visible_2to5 = 0;
          barostats_visible_3to6 = 0;
          barostats_visible_4to7 = 0;
          barostats_visible_5to8 = 1;
        }
      }
      if (barostats_visible_3to6 == 1) {
        if (barostats_hover_1 == 1) { //scroll up
          barostats_visible_1to4 = 1;
          barostats_visible_2to5 = 0;
          barostats_visible_3to6 = 0;
          barostats_visible_4to7 = 0;
          barostats_visible_5to8 = 0;
        }
        if (barostats_hover_2 == 1) {
          barostats_visible_1to4 = 0;
          barostats_visible_2to5 = 1;
          barostats_visible_3to6 = 0;
          barostats_visible_4to7 = 0;
          barostats_visible_5to8 = 0;
        }
        if (barostats_hover_7 == 1) {//scroll down
          barostats_visible_1to4 = 0;
          barostats_visible_2to5 = 0;
          barostats_visible_3to6 = 0;
          barostats_visible_4to7 = 1;
          barostats_visible_5to8 = 0;
        }
        if (barostats_hover_8 == 1) {
          barostats_visible_1to4 = 0;
          barostats_visible_2to5 = 0;
          barostats_visible_3to6 = 0;
          barostats_visible_4to7 = 0;
          barostats_visible_5to8 = 1;
        }
      }
      if (barostats_visible_4to7 == 1) {
        if (barostats_hover_1 == 1) { //scroll up
          barostats_visible_1to4 = 1;
          barostats_visible_2to5 = 0;
          barostats_visible_3to6 = 0;
          barostats_visible_4to7 = 0;
          barostats_visible_5to8 = 0;
        }
        if (barostats_hover_2 == 1) {
          barostats_visible_1to4 = 0;
          barostats_visible_2to5 = 1;
          barostats_visible_3to6 = 0;
          barostats_visible_4to7 = 0;
          barostats_visible_5to8 = 0;
        }
        if (barostats_hover_3 == 1) {
          barostats_visible_1to4 = 0;
          barostats_visible_2to5 = 0;
          barostats_visible_3to6 = 1;
          barostats_visible_4to7 = 0;
          barostats_visible_5to8 = 0;
        }
        if (barostats_hover_8 == 1) {//scroll down
          barostats_visible_1to4 = 0;
          barostats_visible_2to5 = 0;
          barostats_visible_3to6 = 0;
          barostats_visible_4to7 = 0;
          barostats_visible_5to8 = 1;
        }
      }
      if (barostats_visible_5to8 == 1) {
        if (barostats_hover_1 == 1) { //scroll up
          barostats_visible_1to4 = 1;
          barostats_visible_2to5 = 0;
          barostats_visible_3to6 = 0;
          barostats_visible_4to7 = 0;
          barostats_visible_5to8 = 0;
        }
        if (barostats_hover_2 == 1) {
          barostats_visible_1to4 = 0;
          barostats_visible_2to5 = 1;
          barostats_visible_3to6 = 0;
          barostats_visible_4to7 = 0;
          barostats_visible_5to8 = 0;
        }
        if (barostats_hover_3 == 1) {
          barostats_visible_1to4 = 0;
          barostats_visible_2to5 = 0;
          barostats_visible_3to6 = 1;
          barostats_visible_4to7 = 0;
          barostats_visible_5to8 = 0;
        }
        if (barostats_hover_4 == 1) {
          barostats_visible_1to4 = 0;
          barostats_visible_2to5 = 0;
          barostats_visible_3to6 = 0;
          barostats_visible_4to7 = 1;
          barostats_visible_5to8 = 0;
        }
      }
      lcd.clear(); //only clear screen if we pulse, means we refresh more quickly. requires more handling in the display method
    }//end encoder_pulse
    encoder_pulse = 0; //reset the pulse flag
    encoder_CW = 0; //if you put in an extra rotate, this will zero it if the move was invalid (out of range), otherwise this results in refreshing the display at speed of loop
    encoder_CCW = 0; //same for other direction
    displayUpdate(); //update the display if and only if the user made a change, and do it after the flags have been modified
  } //end if encoder pulse
  //selection flags, will change barostats to what you need it to be
  if ((digitalRead(ENC_SW) == LOW)) {
    delay(clickDelay); //allow time for thumb to come off button, important because user will have pressed the button, don't wanna instantly cycle
    //Serial.print("you pressed the button!");
    if (barostats_hover_1 == 1) { //technically this check is redundant, becuase there's only one thing to do but keeping it for structure
      barostats_interactive = 0; //return to the sensor stats menu
      displayUpdate(); //does one update, lets the user see their menu faster. makes it _feel_ faster
    }
    /** //we have the capability of making the below things do things when clicked, but no actual real need since the barometer info is displayed next to the menu in display loop. heres a framework anyway.
      if (barostats_hover_2 == 1) {
      //no action
      }
      if (barostats_hover_3 == 1) {
      //no action
      }
      if (barostats_hover_4 == 1) {
      //no action
      }
      if (barostats_hover_5 == 1) {
      //no action
      }
      if (barostats_hover_6 == 1) {
      //no action
      }
      if (barostats_hover_7 == 1) {
      //no action
      }
      if (barostats_hover_8 == 1) {
      //no action
      }
      if (barostats_hover_9 == 1) {
      //no action
      }
    */

  } //end digitalRead==LOW
} //end barostatInput

void displayUpdate() { //this is a very very long function, it is responsible for displaying the entire UI and every submenu though so it is understandable. The pre-main errors are handled seperately once per boot
  
  /** here follows a standard display write. this is how we update the display. remember, you get 20 characters per line
    lcd.setCursor(0, 0);
    lcd.print("");
    lcd.setCursor(0, 1);
    lcd.print("");
    lcd.setCursor(0, 2);
    lcd.print("");
    lcd.setCursor(0, 3);
    lcd.print("");
  */
  if (startupComplete == 0) { //this checks to see if you have arrived at main menu yet (after initialization, resets on arduino reset)
    startupSensorErrorHandler();
  }//end startup

  if (active == 0) {
    if ((main_interactive == 0) && (startupComplete == 1)) { //the main menu display update conditions
      lcd.clear();
      mainMenu();
    }//end main menu display
    if ((main_interactive == 1) && (startupComplete == 1) && (aboutMenu == 0) && (serial_interactive == 0) && (calibrate_interactive == 0) && (sensorstats_interactive == 0)) { //switch into main_interactive only if none of the submenus of main_interactive are up
      mainMenuInteractive();
    }//end main_interactive

    //submenus get brought up starting here. These need to also be checked in the above loop to avoid drawing the main menu interactive over the submenus
    if ((serial_interactive == 1) && (baud_interactive == 0)) {//this one needs to check two flags because there's a deeper submenu under it
      serial_interface_interactive();
    }
    if (baud_interactive == 1) {
      baudMenuInteractive();
    }
    if (calibrate_interactive == 1) {
      calibrate_display_interactive();
    }
    if ((sensorstats_interactive == 1) && (barostats_interactive == 0)) {//multiple checks here because multiple submenus can be triggered
      sensorstats_display_interactive();
    }
    if (barostats_interactive == 1) {
      barostatsMenuInteractive();
    }
    if (aboutMenu == 1) {
      aboutDisplay();
    }
  }

  if (active == 1) {
    if (run_interactive == 0) { //the run menu display update conditions
      lcd.clear(); //this was previosuly commented out but if it isnt in, the screen will aquire stale characters
      runMenu();
    }//end run menu display
    if ((run_interactive == 1) && (sensorstats_interactive == 0) && (liveadjustv_interactive == 0)) { //switch into run_interactive only if none of the submenus of run_interactive are up
      runMenuInteractive();
    }//end run_interactive

    //submenus get brought up starting here. These need to also be checked in the above loop to avoid drawing the run menu interactive over the submenus
    if ((sensorstats_interactive == 1) && (barostats_interactive == 0)) {//multiple checks here because multiple submenus can be triggered
      sensorstats_display_interactive();
    }
    if (barostats_interactive == 1) {
      barostatsMenuInteractive();
    }
    //if (liveAdjustV_interactive == 1) {
    //liveAdjustVMenuInteractive(); //this needs to be reactivated later
    //} //saffron
  }
} //end displayUpdate()

void calibrate_display_interactive() {
  /** //display SOMETHING
    lcd.setCursor(0, 0);
    lcd.print("SUCCESS             ");
    lcd.setCursor(0, 1);
    lcd.print("12345678901234567890");
    lcd.setCursor(0, 2);
    lcd.print("abcdefghijklmnopqrst");
    lcd.setCursor(0, 3);
    lcd.print("09876543210987654321");
  */
  //draw 1 to 4 menu options without the hover character, shifted right one to make room for the hover character
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Go Back");
  lcd.setCursor(1, 1);
  lcd.print("Cal LoadCell 1");
  lcd.setCursor(1, 2);
  lcd.print("Cal LoadCell 2");
  //lcd.setCursor(1, 3); //we don't need these lines
  //lcd.print("");

  if (calibrate_hover_1 == 1) { // draw hover character on first row
    lcd.setCursor(0, 0);
    lcd.print(">");
    lcd.setCursor(0, 1);
    lcd.print(" ");
    lcd.setCursor(0, 2);
    lcd.print(" ");
    lcd.setCursor(0, 3);
    lcd.print(" ");
  }
  if (calibrate_hover_2 == 1) { //2nd
    lcd.setCursor(0, 0);
    lcd.print(" ");
    lcd.setCursor(0, 1);
    lcd.print(">");
    lcd.setCursor(0, 2);
    lcd.print(" ");
    lcd.setCursor(0, 3);
    lcd.print(" ");
  }
  if (calibrate_hover_3 == 1) { //etc
    lcd.setCursor(0, 0);
    lcd.print(" ");
    lcd.setCursor(0, 1);
    lcd.print(" ");
    lcd.setCursor(0, 2);
    lcd.print(">");
    lcd.setCursor(0, 3);
    lcd.print(" ");
  }
  /** //don't need the 4th row
    if (serial_hover_4 == 1) {
    lcd.setCursor(0, 0);
    lcd.print(" ");
    lcd.setCursor(0, 1);
    lcd.print(" ");
    lcd.setCursor(0, 2);
    lcd.print(" ");
    lcd.setCursor(0, 3);
    lcd.print(">");
    }
  */
}

void sensorstats_display_interactive() { //this is one of the only shared functions that run and main share. So it needs to be tolerant of both and NOT printing any serial data. We could put something here to print a warning about data timing to the info column of the data dump?
  /** //display SOMETHING
    lcd.setCursor(0, 0);
    lcd.print("SUCCESS             ");
    lcd.setCursor(0, 1);
    lcd.print("12345678901234567890");
    lcd.setCursor(0, 2);
    lcd.print("abcdefghijklmnopqrst");
    lcd.setCursor(0, 3);
    lcd.print("09876543210987654321");
  */
  //draw 1 to 4 menu options without the hover character, shifted right one to make room for the hover character
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Go Back");
  lcd.setCursor(1, 1);
  if ((barofail_1 == 0) && (barofail_2 == 0) && (barofail_3 == 0) && (barofail_4 == 0) && (barofail_5 == 0) && (barofail_6 == 0) && (barofail_7 == 0)) {
    lcd.print("Barometers");
  } else {
    lcd.print("Baro ERR!");
  }
  lcd.setCursor(1, 2);
  if (anemfail_notfound == 0) {
    lcd.print("Anemometer");
  } else {
    lcd.print("Anem ERR!");
  }
  lcd.setCursor(1, 3);
  if ((ignoreLoadCell_1 == 0) && (ignoreLoadCell_2 == 0)) {
    lcd.print("Load Cells");
  } else {
    lcd.print("LoadCellERR!");
  }


  if (sensorstats_hover_1 == 1) { // draw hover character on first row
    lcd.setCursor(0, 0);
    lcd.print(">");
    lcd.setCursor(0, 1);
    lcd.print(" ");
    lcd.setCursor(0, 2);
    lcd.print(" ");
    lcd.setCursor(0, 3);
    lcd.print(" ");
  }
  if (sensorstats_hover_2 == 1) { //2nd
    lcd.setCursor(0, 0);
    lcd.print(" ");
    lcd.setCursor(0, 1);
    lcd.print(">");
    lcd.setCursor(0, 2);
    lcd.print(" ");
    lcd.setCursor(0, 3);
    lcd.print(" ");
  }
  if (sensorstats_hover_3 == 1) { //etc
    lcd.setCursor(0, 0);
    lcd.print(" ");
    lcd.setCursor(0, 1);
    lcd.print(" ");
    lcd.setCursor(0, 2);
    lcd.print(">");
    lcd.setCursor(0, 3);
    lcd.print(" ");
  }
  if (sensorstats_hover_4 == 1) {
    lcd.setCursor(0, 0);
    lcd.print(" ");
    lcd.setCursor(0, 1);
    lcd.print(" ");
    lcd.setCursor(0, 2);
    lcd.print(" ");
    lcd.setCursor(0, 3);
    lcd.print(">");
  }
}

void serial_interface_interactive() {
  /** //display SOMETHING
    lcd.setCursor(0, 0);
    lcd.print("SUCCESS             ");
    lcd.setCursor(0, 1);
    lcd.print("12345678901234567890");
    lcd.setCursor(0, 2);
    lcd.print("abcdefghijklmnopqrst");
    lcd.setCursor(0, 3);
    lcd.print("09876543210987654321");
  */
  //draw 1 to 4 menu options without the hover character, shifted right one to make room for the hover character
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Go Back");
  lcd.setCursor(1, 1);
  lcd.print("Current Serial Info");
  lcd.setCursor(1, 2);
  lcd.print("Change Baud Rate");
  lcd.setCursor(1, 3);
  lcd.print("Reset Serial Port");

  if (serial_hover_1 == 1) { // draw hover character on first row
    lcd.setCursor(0, 0);
    lcd.print(">");
    lcd.setCursor(0, 1);
    lcd.print(" ");
    lcd.setCursor(0, 2);
    lcd.print(" ");
    lcd.setCursor(0, 3);
    lcd.print(" ");
  }
  if (serial_hover_2 == 1) { //2nd
    lcd.setCursor(0, 0);
    lcd.print(" ");
    lcd.setCursor(0, 1);
    lcd.print(">");
    lcd.setCursor(0, 2);
    lcd.print(" ");
    lcd.setCursor(0, 3);
    lcd.print(" ");
  }
  if (serial_hover_3 == 1) { //etc
    lcd.setCursor(0, 0);
    lcd.print(" ");
    lcd.setCursor(0, 1);
    lcd.print(" ");
    lcd.setCursor(0, 2);
    lcd.print(">");
    lcd.setCursor(0, 3);
    lcd.print(" ");
  }
  if (serial_hover_4 == 1) {
    lcd.setCursor(0, 0);
    lcd.print(" ");
    lcd.setCursor(0, 1);
    lcd.print(" ");
    lcd.setCursor(0, 2);
    lcd.print(" ");
    lcd.setCursor(0, 3);
    lcd.print(">");
  }
}

void baudMenuInteractive() { //this is a display cycle. Also good god it's long
  /** //display SOMETHING
    lcd.setCursor(0, 0);
    lcd.print("SUCCESS             ");
    lcd.setCursor(0, 1);
    lcd.print("12345678901234567890");
    lcd.setCursor(0, 2);
    lcd.print("abcdefghijklmnopqrst");
    lcd.setCursor(0, 3);
    lcd.print("09876543210987654321");
  */

  if (baud_visible_1to4 == 1) {
    //draw 1 to 4 menu options without the hover character, shifted right one to make room for the hover character
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Go Back");
    lcd.setCursor(1, 1);
    lcd.print("300");
    lcd.setCursor(1, 2);
    lcd.print("600");
    lcd.setCursor(1, 3);
    lcd.print("1200");

    if (baud_hover_1 == 1) { // draw hover character on first row
      lcd.setCursor(0, 0);
      lcd.print(">");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (baud_hover_2 == 1) { //2nd
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(">");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (baud_hover_3 == 1) { //etc
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(">");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (baud_hover_4 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(">");
    }
  }//end main_visible_1to4
  if (baud_visible_2to5 == 1) {

    //draw 2 - 5 menu options without the hover character
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("300");
    lcd.setCursor(1, 1);
    lcd.print("600");
    lcd.setCursor(1, 2);
    lcd.print("1200");
    lcd.setCursor(1, 3);
    lcd.print("2400");

    if (baud_hover_2 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(">");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (baud_hover_3 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(">");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (baud_hover_4 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(">");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (baud_hover_5 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(">");
    }
  }

  if (baud_visible_3to6 == 1) {

    //draw 3 - 6 menu options without the hover character
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("600");
    lcd.setCursor(1, 1);
    lcd.print("1200");
    lcd.setCursor(1, 2);
    lcd.print("2400");
    lcd.setCursor(1, 3);
    lcd.print("4800");

    if (baud_hover_3 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(">");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (baud_hover_4 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(">");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (baud_hover_5 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(">");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (baud_hover_6 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(">");
    }
  }
  if (baud_visible_4to7 == 1) {

    //draw 4 - 7 menu options without the hover character
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("1200");
    lcd.setCursor(1, 1);
    lcd.print("2400");
    lcd.setCursor(1, 2);
    lcd.print("4800");
    lcd.setCursor(1, 3);
    lcd.print("9600");

    if (baud_hover_4 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(">");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (baud_hover_5 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(">");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (baud_hover_6 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(">");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (baud_hover_7 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(">");
    }
  }
  if (baud_visible_5to8 == 1) {

    //draw 5 - 8 menu options without the hover character
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("2400");
    lcd.setCursor(1, 1);
    lcd.print("4800");
    lcd.setCursor(1, 2);
    lcd.print("9600");
    lcd.setCursor(1, 3);
    lcd.print("14400");

    if (baud_hover_5 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(">");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (baud_hover_6 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(">");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (baud_hover_7 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(">");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (baud_hover_8 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(">");
    }
  }
  if (baud_visible_6to9 == 1) { //nice

    //draw 6 - 9 menu options without the hover character
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("4800");
    lcd.setCursor(1, 1);
    lcd.print("9600");
    lcd.setCursor(1, 2);
    lcd.print("14400");
    lcd.setCursor(1, 3);
    lcd.print("19200");

    if (baud_hover_6 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(">");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (baud_hover_7 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(">");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (baud_hover_8 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(">");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (baud_hover_9 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(">");
    }
  }
  if (baud_visible_7to10 == 1) {

    //draw 7 - 10 menu options without the hover character
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("9600");
    lcd.setCursor(1, 1);
    lcd.print("14400");
    lcd.setCursor(1, 2);
    lcd.print("19200");
    lcd.setCursor(1, 3);
    lcd.print("28800");

    if (baud_hover_7 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(">");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (baud_hover_8 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(">");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (baud_hover_9 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(">");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (baud_hover_10 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(">");
    }
  }
  if (baud_visible_8to11 == 1) {

    //draw 8 - 11 menu options without the hover character
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("14400");
    lcd.setCursor(1, 1);
    lcd.print("19200");
    lcd.setCursor(1, 2);
    lcd.print("28800");
    lcd.setCursor(1, 3);
    lcd.print("38.4K");

    if (baud_hover_8 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(">");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (baud_hover_9 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(">");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (baud_hover_10 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(">");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (baud_hover_11 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(">");
    }
  }
  if (baud_visible_9to12 == 1) {

    //draw 9 - 12 menu options without the hover character
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("19200");
    lcd.setCursor(1, 1);
    lcd.print("28800");
    lcd.setCursor(1, 2);
    lcd.print("38.4K");
    lcd.setCursor(1, 3);
    lcd.print("57.6K");

    if (baud_hover_9 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(">");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (baud_hover_10 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(">");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (baud_hover_11 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(">");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (baud_hover_12 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(">");
    }
  }
  if (baud_visible_10to13 == 1) {

    //draw 10 - 13 menu options without the hover character
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("28800");
    lcd.setCursor(1, 1);
    lcd.print("38.4K");
    lcd.setCursor(1, 2);
    lcd.print("57.6K");
    lcd.setCursor(1, 3);
    lcd.print("115.2K"); //we could probably go faster but I'm sick of drawing menus and this is probably fast enough

    if (baud_hover_10 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(">");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (baud_hover_11 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(">");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (baud_hover_12 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(">");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (baud_hover_13 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(">");
    }
  }
}

void barostatsMenuInteractive() { //this is a display cycle. Another long cycle
  /** //display SOMETHING
    lcd.setCursor(0, 0);
    lcd.print("SUCCESS             ");
    lcd.setCursor(0, 1);
    lcd.print("12345678901234567890");
    lcd.setCursor(0, 2);
    lcd.print("abcdefghijklmnopqrst");
    lcd.setCursor(0, 3);
    lcd.print("09876543210987654321");
  */

  if (barostats_visible_1to4 == 1) {
    //draw 1 to 4 menu options without the hover character, shifted right one to make room for the hover character
    //lcd.clear(); trying something new, only clear on the pulse
    lcd.setCursor(1, 0);
    lcd.print("Go Back");
    lcd.setCursor(1, 1);
    lcd.print("Baro 1:     "); //print over (under?) the data area to clear old data efficiently
    baroSelect = 0; //baro 0 select
    baroRead();//update baro reading
    lcd.setCursor(9, 1); //reset cursor to draw over the spaces given earlier
    lcd.print(baro0);
    lcd.setCursor(1, 2);
    lcd.print("Baro 2:     ");
    baroSelect = 1;
    baroRead();
    lcd.setCursor(9, 2);
    lcd.print(baro1);
    lcd.setCursor(1, 3);
    lcd.print("Baro 3:     ");
    baroSelect = 2;
    baroRead();
    lcd.setCursor(9, 3);
    lcd.print(baro2);
    //below has been modified to be as barebones as possible to speed up the draw loop
    if (barostats_hover_1 == 1) { // draw hover character on first row
      lcd.setCursor(0, 0);
      lcd.print(">");
    }
    if (barostats_hover_2 == 1) { //2nd
      lcd.setCursor(0, 1);
      lcd.print(">");
    }
    if (barostats_hover_3 == 1) { //etc
      lcd.setCursor(0, 2);
      lcd.print(">");
    }
    if (barostats_hover_4 == 1) {
      lcd.setCursor(0, 3);
      lcd.print(">");
    }
  }//end main_visible_1to4
  if (barostats_visible_2to5 == 1) {

    //draw 2 - 5 menu options without the hover character
    lcd.setCursor(1, 0);
    lcd.print("Baro 1:     ");
    baroSelect = 0;
    baroRead();
    lcd.setCursor(9, 0);
    lcd.print(baro0);
    lcd.setCursor(1, 1);
    lcd.print("Baro 2:     ");
    baroSelect = 1;
    baroRead();
    lcd.setCursor(9, 1);
    lcd.print(baro1);
    lcd.setCursor(1, 2);
    lcd.print("Baro 3:     ");
    baroSelect = 2;
    baroRead();
    lcd.setCursor(9, 2);
    lcd.print(baro2);
    lcd.setCursor(1, 3);
    lcd.print("Baro 4:     ");
    baroSelect = 3;
    baroRead();
    lcd.setCursor(9, 3);
    lcd.print(baro3);

    if (barostats_hover_2 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(">");
    }
    if (barostats_hover_3 == 1) {
      lcd.setCursor(0, 1);
      lcd.print(">");
    }
    if (barostats_hover_4 == 1) {
      lcd.setCursor(0, 2);
      lcd.print(">");
    }
    if (barostats_hover_5 == 1) {
      lcd.setCursor(0, 3);
      lcd.print(">");
    }
  }

  if (barostats_visible_3to6 == 1) {

    //draw 3 - 6 menu options without the hover character
    lcd.setCursor(1, 0);
    lcd.print("Baro 2:     ");
    baroSelect = 1;
    baroRead();
    lcd.setCursor(9, 0);
    lcd.print(baro1);
    lcd.setCursor(1, 1);
    lcd.print("Baro 3:     ");
    baroSelect = 2;
    baroRead();
    lcd.setCursor(9, 1);
    lcd.print(baro2);
    lcd.setCursor(1, 2);
    lcd.print("Baro 4:     ");
    baroSelect = 3;
    baroRead();
    lcd.setCursor(9, 2);
    lcd.print(baro3);
    lcd.setCursor(1, 3);
    lcd.print("Baro 5:     ");
    baroSelect = 4;
    baroRead();
    lcd.setCursor(9, 3);
    lcd.print(baro4);

    if (barostats_hover_3 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(">");
    }
    if (barostats_hover_4 == 1) {
      lcd.setCursor(0, 1);
      lcd.print(">");
    }
    if (barostats_hover_5 == 1) {
      lcd.setCursor(0, 2);
      lcd.print(">");
    }
    if (barostats_hover_6 == 1) {
      lcd.setCursor(0, 3);
      lcd.print(">");
    }
  }
  if (barostats_visible_4to7 == 1) {

    //draw 4 - 7 menu options without the hover character
    lcd.setCursor(1, 0);
    lcd.print("Baro 3:     ");
    baroSelect = 2;
    baroRead();
    lcd.setCursor(9, 0);
    lcd.print(baro2);
    lcd.setCursor(1, 1);
    lcd.print("Baro 4:     ");
    baroSelect = 3;
    baroRead();
    lcd.setCursor(9, 1);
    lcd.print(baro3);
    lcd.setCursor(1, 2);
    lcd.print("Baro 5:     ");
    baroSelect = 4;
    baroRead();
    lcd.setCursor(9, 2);
    lcd.print(baro4);
    lcd.setCursor(1, 3);
    lcd.print("Baro 6:     ");
    baroSelect = 5;
    baroRead();
    lcd.setCursor(9, 3);
    lcd.print(baro5);

    if (barostats_hover_4 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(">");
    }
    if (barostats_hover_5 == 1) {
      lcd.setCursor(0, 1);
      lcd.print(">");
    }
    if (barostats_hover_6 == 1) {
      lcd.setCursor(0, 2);
      lcd.print(">");
    }
    if (barostats_hover_7 == 1) {
      lcd.setCursor(0, 3);
      lcd.print(">");
    }
  }
  if (barostats_visible_5to8 == 1) {

    //draw 5 - 8 menu options without the hover character
    lcd.setCursor(1, 0);
    lcd.print("Baro 4:     ");
    baroSelect = 3;
    baroRead();
    lcd.setCursor(9, 0);
    lcd.print(baro3);
    lcd.setCursor(1, 1);
    lcd.print("Baro 5:     ");
    baroSelect = 4;
    baroRead();
    lcd.setCursor(9, 1);
    lcd.print(baro4);
    lcd.setCursor(1, 2);
    lcd.print("Baro 6:     ");
    baroSelect = 5;
    baroRead();
    lcd.setCursor(9, 2);
    lcd.print(baro5);
    lcd.setCursor(1, 3);
    lcd.print("Baro 7:     ");
    baroSelect = 6;
    baroRead();
    lcd.setCursor(9, 3);
    lcd.print(baro6);

    if (barostats_hover_5 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(">");
    }
    if (barostats_hover_6 == 1) {
      lcd.setCursor(0, 1);
      lcd.print(">");
    }
    if (barostats_hover_7 == 1) {
      lcd.setCursor(0, 2);
      lcd.print(">");
    }
    if (barostats_hover_8 == 1) {
      lcd.setCursor(0, 3);
      lcd.print(">");
    }
  }
} //end barostatsMenuInteractive

void mainMenuInteractive() {
  /** //display SOMETHING
    lcd.setCursor(0, 0);
    lcd.print("SUCCESS             ");
    lcd.setCursor(0, 1);
    lcd.print("12345678901234567890");
    lcd.setCursor(0, 2);
    lcd.print("abcdefghijklmnopqrst");
    lcd.setCursor(0, 3);
    lcd.print("09876543210987654321");
  */
  //reset flags upon exiting the submenus
  loadCellCalibrate1_firstrun = 1; //this is a convenient place to put firstrun flags
  loadCellCalibrate2_firstrun = 1;


  if (main_visible_1to4 == 1) {
    //draw 1 to 4 menu options without the hover character, shifted right one to make room for the hover character
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Info Screen");
    lcd.setCursor(1, 1);
    lcd.print("Start");
    lcd.setCursor(1, 2);
    lcd.print("Sensor Statistics");
    lcd.setCursor(1, 3);
    if ((ignoreLoadCell_1 == 0) && (ignoreLoadCell_2 == 0)) {
      lcd.print("Calibrate");
    } else {
      lcd.print("Cal Locked!");
    }

    if (main_hover_1 == 1) { // draw hover character on first row
      lcd.setCursor(0, 0);
      lcd.print(">");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (main_hover_2 == 1) { //2nd
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(">");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (main_hover_3 == 1) { //etc
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(">");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (main_hover_4 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(">");
    }
  }//end main_visible_1to4
  if (main_visible_2to5 == 1) {

    //draw 2 - 5 menu options without the hover character
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Start");
    lcd.setCursor(1, 1);
    lcd.print("Sensor Statistics");
    lcd.setCursor(1, 2);
    if ((ignoreLoadCell_1 == 0) && (ignoreLoadCell_2 == 0)) {
      lcd.print("Calibrate");
    } else {
      lcd.print("Cal Locked!");
    }
    lcd.setCursor(1, 3);
    lcd.print("Serial Interface");

    if (main_hover_2 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(">");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (main_hover_3 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(">");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (main_hover_4 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(">");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (main_hover_5 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(">");
    }
  }

  if (main_visible_3to6 == 1) {

    //draw 3 - 6 menu options without the hover character
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Sensor Statistics");
    lcd.setCursor(1, 1);
    if ((ignoreLoadCell_1 == 0) && (ignoreLoadCell_2 == 0)) {
      lcd.print("Calibrate");
    } else {
      lcd.print("Cal Locked!");
    }
    lcd.setCursor(1, 2);
    lcd.print("Serial Interfaces");
    lcd.setCursor(1, 3);
    lcd.print("About");

    if (main_hover_3 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(">");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (main_hover_4 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(">");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (main_hover_5 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(">");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (main_hover_6 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(">");
    }
  }
  if (main_visible_4to7 == 1) {
    //draw 1 to 4 menu options without the hover character, shifted right one to make room for the hover character
    lcd.clear();
    lcd.setCursor(1, 0);
    if ((ignoreLoadCell_1 == 0) && (ignoreLoadCell_2 == 0)) {
      lcd.print("Calibrate");
    } else {
      lcd.print("Cal Locked!");
    }
    lcd.setCursor(1, 1);
    lcd.print("Serial Interfaces");
    lcd.setCursor(1, 2);
    lcd.print("About");
    lcd.setCursor(1, 3);
    lcd.print("Reset");

    if (main_hover_4 == 1) { // draw hover character on first row
      lcd.setCursor(0, 0);
      lcd.print(">");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (main_hover_5 == 1) { //2nd
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(">");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (main_hover_6 == 1) { //etc
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(">");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (main_hover_7 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(">");
    }
  }//end main_visible_1to4
}

//same as above, for run menu

void runMenuInteractive() {
  /** //display SOMETHING
    lcd.setCursor(0, 0);
    lcd.print("SUCCESS             ");
    lcd.setCursor(0, 1);
    lcd.print("12345678901234567890");
    lcd.setCursor(0, 2);
    lcd.print("abcdefghijklmnopqrst");
    lcd.setCursor(0, 3);
    lcd.print("09876543210987654321");
  */

  //if (run_visible_1to4 == 1) {
  //draw 1 to 4 menu options without the hover character, shifted right one to make room for the hover character
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Return");
  lcd.setCursor(1, 1);
  lcd.print("ABORT (Reset)");
  lcd.setCursor(1, 2);
  lcd.print("Sensor Statistics");
  lcd.setCursor(1, 3);
  lcd.print("Live Adjust V(tgt)");

  if (run_hover_1 == 1) { // draw hover character on first row
    lcd.setCursor(0, 0);
    lcd.print(">");
    lcd.setCursor(0, 1);
    lcd.print(" ");
    lcd.setCursor(0, 2);
    lcd.print(" ");
    lcd.setCursor(0, 3);
    lcd.print(" ");
  }
  if (run_hover_2 == 1) { //2nd
    lcd.setCursor(0, 0);
    lcd.print(" ");
    lcd.setCursor(0, 1);
    lcd.print(">");
    lcd.setCursor(0, 2);
    lcd.print(" ");
    lcd.setCursor(0, 3);
    lcd.print(" ");
  }
  if (run_hover_3 == 1) { //etc
    lcd.setCursor(0, 0);
    lcd.print(" ");
    lcd.setCursor(0, 1);
    lcd.print(" ");
    lcd.setCursor(0, 2);
    lcd.print(">");
    lcd.setCursor(0, 3);
    lcd.print(" ");
  }
  if (run_hover_4 == 1) {
    lcd.setCursor(0, 0);
    lcd.print(" ");
    lcd.setCursor(0, 1);
    lcd.print(" ");
    lcd.setCursor(0, 2);
    lcd.print(" ");
    lcd.setCursor(0, 3);
    lcd.print(">");
  }
  //}//end run_visible_1to4

  /** //disabled because currently not using the extra display windows
    if (run_visible_2to5 == 1) {

    //draw 2 - 5 menu options without the hover character
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Start");
    lcd.setCursor(1, 1);
    lcd.print("Sensor Statistics");
    lcd.setCursor(1, 2);
    lcd.print("Calibrate");
    lcd.setCursor(1, 3);
    lcd.print("Serial Interface");

    if (run_hover_2 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(">");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (run_hover_3 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(">");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (run_hover_4 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(">");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (run_hover_5 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(">");
    }
    }

    if (run_visible_3to6 == 1) {

    //draw 3 - 6 menu options without the hover character
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Sensor Statistics");
    lcd.setCursor(1, 1);
    lcd.print("Calibrate");
    lcd.setCursor(1, 2);
    lcd.print("Serial Interfaces");
    lcd.setCursor(1, 3);
    lcd.print("About");

    if (run_hover_3 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(">");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (run_hover_4 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(">");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (run_hover_5 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(">");
      lcd.setCursor(0, 3);
      lcd.print(" ");
    }
    if (run_hover_6 == 1) {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(">");
    }
    }
  */
} //end runMenuInteractive

void mainMenu() {
  //Serial.print("mainMenu()");
  //Serial.println();

  mainInteractiveFirstRun = 1; //this is for the navigable menu system, not the reset flag for this display mode
  encoder_CW = 0;
  encoder_CCW = 0;
  lcd.setCursor(0, 0);
  lcd.print("State:"); //characters 1-6
  lcd.setCursor(10, 0);
  lcd.print("V:");  //characters 11-12
  lcd.setCursor(0, 1); //change to line 2
  lcd.print("T:"); //characters 1-2
  lcd.setCursor(10, 1);
  lcd.print("P:"); //characters 11-12
  lcd.setCursor(0, 2); //change to line 3
  lcd.print("L1:"); //characters from 1-3
  lcd.setCursor(10, 2);
  lcd.print("L2:"); //characters 11-13
  //thats the static elements on the screen. now we'll pass over it again and poke in the dynamic data. easiest first, state flag controls status OK/ERR and the entirety of line 4
  if (state_trouble == 1) {
    lcd.setCursor(0, 3);
    lcd.print("WindOS not ready!"); //since the display got cleared, we don't need to write the trailing spaces
    lcd.setCursor(6, 0);
    lcd.print("ERR"); //if our aim is good, this plops right next to the state flag on the screen
  }
  if (state_trouble == 0) {
    lcd.setCursor(0, 3);
    lcd.print("WindOS ready.       ");
    lcd.setCursor(6, 0);
    lcd.print("OK "); //if our aim is good, this plops right next to the state flag on the screen
  }
  if (anemfail_notfound == 0) { //if we print bad values it blows everything up with -255 values. clearly wrong but not displaying anything is better.
    lcd.setCursor(2, 1);
    lcd.print(String(cgAnem.temperature));
    lcd.print("C");
    lcd.setCursor(12, 0);
    lcd.print(String(cgAnem.airflowRate));
    lcd.print("m/s");
  }
  if (anemfail_notfound == 1) { //if we print bad values it blows everything up with -255 values. clearly wrong but not displaying anything is better.
    lcd.setCursor(3, 1);
    lcd.print("NO ANM");
    lcd.setCursor(12, 0);
    lcd.print("NO ANM");
  }

  //print old values, baro and load cell take some time to compute, so this removes the flashing problem
  lcd.setCursor(12, 1); //aim for spot next to "P:"
  if ((barofail_1 == 0) || (barofail_2 == 0) || (barofail_3 == 0) || (barofail_4 == 0) || (barofail_5 == 0) || (barofail_6 == 0) || (barofail_7 == 0)) {
    lcd.print(baroAvg); //print the old value
    lcd.print("hPa"); //equivalent to mbar but also 1 letter shorter, more space on the menu. I would have otherwise set this to say mbar
  }
  if ((barofail_1 == 1) || (barofail_2 == 1) || (barofail_3 == 1) || (barofail_4 == 1) || (barofail_5 == 1) || (barofail_6 == 1) || (barofail_7 == 1)) {
    lcd.print("BARO ERR");
  }
  if (ignoreLoadCell_1 == 0) {
    //print load cell 1 to screen
    lcd.setCursor(3, 2); //aim for spot next to "L1:"
    lcd.print(loadCell1Reading);
    lcd.print("g");
  }
  if (ignoreLoadCell_1 == 1) {
    //print load cell 1 to screen
    lcd.setCursor(3, 2); //aim for spot next to "L1:"
    lcd.print("ERR");
  }
  if (ignoreLoadCell_2 == 0) {
    //print load cell 2 to screen
    lcd.setCursor(13, 2); //aim for spot next to "L2:"
    lcd.print(loadCell2Reading);
    lcd.print("g");
  }
  if (ignoreLoadCell_2 == 1) {
    //print load cell 2 to screen
    lcd.setCursor(13, 2); //aim for spot next to "L2:"
    lcd.print("ERR");
  }


  //similar logic to the anemometer, no value is the bad sign
  if (barofail_1 == 0 && barofail_2 == 0 && barofail_3 == 0 && barofail_4 == 0 && barofail_5 == 0 && barofail_6 == 0 && barofail_7 == 0) {//this loop is slow
    baroSelect = 0; //baro 0
    baroRead();
    baroSelect = 1; //baro 1
    baroRead();
    baroSelect = 2; //baro 2
    baroRead();
    baroSelect = 3; //baro 3
    baroRead();
    baroSelect = 4; //baro 4
    baroRead();
    baroSelect = 5; //baro 5
    baroRead();
    baroSelect = 6; //baro 6
    baroRead();
    //now baro0 - baro6 should have values
    baroAvg = ((baro0 + baro1 + baro2 + baro3 + baro4 + baro5 + baro6) / 7); //average the readings together (accuracy)
    lcd.setCursor(12, 1); //aim for spot next to "P:"
    lcd.print(baroAvg); //print the new value
    lcd.print("hPa"); //equivalent to mbar but also 1 letter shorter, more space on the menu. I would have otherwise set this to say mbar
  }
  //nonzeroed load cell values
  if (ignoreLoadCell_1 == 0) {
    loadCell1Reading = LoadCell_1.get_units(); //unlike loadCellGo, we use this to change the direction of the vector to have positive force be up instead of down
  }

  if (ignoreLoadCell_1 == 0) {
    loadCell2Reading = LoadCell_2.get_units(); //unlike loadCellGo, we use this to change the direction of the vector to have positive force be up instead of down
  }
}//end mainMenu()

//same as above, for run menu
void runMenu() {
  //Serial.print("runMenu()"); //this should absolutely be suppressed when we're running for real.
  //Serial.println(); //this too

  runInteractiveFirstRun = 1; //this is for the navigable menu system, not the reset flag for this display mode
  encoder_CW = 0;
  encoder_CCW = 0;
  lcd.setCursor(0, 0);
  lcd.print("State:"); //characters 1-6
  lcd.setCursor(10, 0);
  lcd.print("V:");  //characters 11-12
  lcd.setCursor(0, 1); //change to line 2
  lcd.print("T:"); //characters 1-2
  lcd.setCursor(10, 1);
  lcd.print("P:"); //characters 11-12
  lcd.setCursor(0, 2); //change to line 3
  lcd.print("L1:"); //characters from 1-3
  lcd.setCursor(10, 2);
  lcd.print("L2:"); //characters 11-13
  //thats the static elements on the screen. now we'll pass over it again and poke in the dynamic data. easiest first, state flag controls status OK/RUN


  lcd.setCursor(0, 3); //4th line pos 0
  lcd.print("V(tgt):"); //target velocity, IE what user has input (always whole numbers, no decimal, needs padding of 6 spaces, + 7 = next static print on line 13
  lcd.print(velocityTarget);
  lcd.print(".5"); //because of the way the velocity governor works, this will display speed target accurately
  lcd.print("m/s");
  lcd.setCursor(14, 3); //hopefully line 15 of last row
  lcd.print("T-");//T minus, as in time remaining until tunnel shutdown, have 6 spaces left, perfect for "00:00"
  //print time remaining
  //convert time remaining from seconds to minutes and seconds
  if (runTime >= 60) {
    runTimeDisplaySeconds = (runTime % 60); //gets modulo of runtime/60 which will tell you how many seconds remain after dividing by 60 to get minutes.
    runTimeDisplayMinutes = (runTime - runTimeDisplaySeconds); //will first make runTimeMinutes equal whole number seconds
    runTimeDisplayMinutes = (runTimeDisplayMinutes / 60); //...and then divide by 60 to get whole number minutes
  }
  if (runTime < 60) { //if it isnt a full minute...
    runTimeDisplaySeconds = runTime; //we only have seconds and can skip the modulo math
    runTimeDisplayMinutes = 0; //and we have 0 minutes
  }
  //print minutes first
  lcd.print(runTimeDisplayMinutes);
  lcd.print(":"); // need that clock icon yo
  lcd.print(runTimeDisplaySeconds);

  lcd.setCursor(6, 0);
  lcd.print("RUN"); //if our aim is good, this plops right next to the state flag on the screen

  if (anemfail_notfound == 0) { //if we print bad values it blows everything up with -255 values. clearly wrong but not displaying anything is better.
    lcd.setCursor(3, 1);
    lcd.print(String(cgAnem.temperature));
    lcd.print("C");
    lcd.setCursor(12, 0);
    lcd.print(String(cgAnem.airflowRate));
    lcd.print("m/s");
  }
  if (anemfail_notfound == 1) { //if we print bad values it blows everything up with -255 values. clearly wrong but not displaying anything is better.
    lcd.setCursor(3, 1);
    lcd.print("NO ANM");
    lcd.setCursor(12, 0);
    lcd.print("NO ANM");
  }

  if (barofail_7 == 0) {
    //print old values, baro and load cell take some time to compute, so this removes the flashing problem
    lcd.setCursor(12, 1); //aim for spot next to "P:"
    lcd.print(baro6); //print external barometer value
    lcd.print("hPa"); //equivalent to mbar but also 1 letter shorter, more space on the menu. I would have otherwise set this to say mbar
    //print load cell 1 to screen
    lcd.setCursor(3, 2); //aim for spot next to "L1:"
    lcd.print(loadCell1Reading);
    lcd.print("g"); //equivalent to mbar but also 1 letter shorter, more space on the menu. I would have otherwise set this to say mbar
    //print load cell 2 to screen
    lcd.setCursor(13, 2); //aim for spot next to "L2:"
    lcd.print(loadCell2Reading);
    lcd.print("g"); //equivalent to mbar but also 1 letter shorter, more space on the menu. I would have otherwise set this to say mbar

    //similar logic to the anemometer, no value is the bad sign
    if (barofail_7 == 0) {//the averaging value is too slow, and possibly inaccurate. must display only external barometer per display update, costs ~50ms
      baroSelect = 6; //baro "7"
      baroRead();
      //now baro6 has a new value
      lcd.setCursor(12, 1); //aim for spot next to "P:"
      lcd.print(baro6); //print the new value
      lcd.print("hPa"); //equivalent to mbar but also 1 letter shorter, more space on the menu. I would have otherwise set this to say mbar
    }
    //nonzeroed load cell values
    if (barofail_7 == 1) {
      lcd.setCursor(12, 1); //aim for spot next to "P:"
      lcd.print("BARO_ERR"); //print the new value
    }

    if (ignoreLoadCell_1 == 0 ) {
      loadCell1Reading = LoadCell_1.get_units(); //unlike loadCellGo, we use this to change the direction of the vector to have positive force be up instead of down
    }

    if (ignoreLoadCell_2 == 0) {
      loadCell2Reading = LoadCell_2.get_units(); //unlike loadCellGo, we use this to change the direction of the vector to have positive force be up instead of down
    }

    if (ignoreLoadCell_1 == 1 ) {
      lcd.setCursor(3, 2); //aim for spot next to "L1:"
      lcd.print("ERR"); //equivalent to mbar but also 1 letter shorter, more space on the menu. I would have otherwise set this to say mbar
    }
    if (ignoreLoadCell_1 == 1 ) {
      lcd.setCursor(13, 2); //aim for spot next to "L1:"
      lcd.print("ERR"); //equivalent to mbar but also 1 letter shorter, more space on the menu. I would have otherwise set this to say mbar
    }


  } //end runMenu()
}


void aboutDisplay() { //should be handled by the displayUpdate state machine so no need to worry here
  lcd.setCursor(0, 0);
  lcd.print("WindOS V1.2         ");
  lcd.setCursor(0, 1);
  lcd.print("Last Update: 5/26/23");
  lcd.setCursor(0, 2);
  lcd.print("By Kameron Markham  ");
  lcd.setCursor(0, 3);
  lcd.print("bit.ly/Gale-WindOS  "); //normally the >clickme< selector would be dynamic but here we're basically faking it to make it easier to program
  //while (digitalRead(ENC_SW) == LOW) {//this is not as nice ui wise but it prevents stalling the whole program
    delay(7500); //auto-exit after 7.5 seconds. for some reason, exiting on button press is sketchy
    aboutMenu = 0;
    delay(clickDelay);
    displayUpdate();
  //}
}


void tcaselect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

//this function reads the i2c multiplexers sequentially from 0x70-0x77 (if applicable) and per i2c multiplexer reads from channel 0-7
//disabled for testing since it's functions arent needed
/**
  void tcaselect(uint8_t i) {
  Serial.println("tcaselect");
  byte address = TCAADDR;
  static byte previousAddress = 0x99;

  while (i > 7) {
    Serial.println("tcawhile");
    i -= 8;
    address++;
  }

  if (address != previousAddress) {
    Serial.println("address != previousAddress");
    if (previousAddress != 0x99) {
      Serial.println("previousAddress != 0x99");
      Wire.beginTransmission(previousAddress);
      Serial.println("Wire.beginTransmission(previousAddress)");
      Wire.write(0);
      Serial.println("Wire.beginTransmission(previousAddress)");
      Wire.endTransmission();
      Serial.println("Wire.endTransmission();");
    }
    previousAddress = address;
    Serial.println("previousAddress = address;");

  }

  Wire.beginTransmission(address);
  Serial.println("Wire.beginTransmission(address);");
  Wire.write(1 << i);
  Serial.println("Wire.write(1 << i);");
  Wire.endTransmission();
  Serial.println("Wire.endTransmission();");

  }
*/

void baroRead() {//this will hide the actual read code and set a value for pressure given a sensor. that way all you have to do to read a sensor is ask after setting baroSelect. starts at 0
  /* Get a new sensor event */
  //Serial.print("baroRead!");
  sensors_event_t event;
  if (baroSelect == 0) { //sensor number that was picked
    tcaselect(0); //set the channel
    bmp1.getEvent(&event); //lags loop by 35 ms per sensor requested
    if (event.pressure) {
      baro0 = event.pressure;
    }
  } //end BaroSelect == 0

  if (baroSelect == 1) { //sensor number that was picked
    tcaselect(1); //set the channel
    bmp2.getEvent(&event);
    if (event.pressure) {
      baro1 = event.pressure;
    }
  } //end BaroSelect == 1

  if (baroSelect == 2) { //sensor number that was picked
    tcaselect(2); //set the channel
    bmp3.getEvent(&event);
    if (event.pressure) {
      baro2 = event.pressure;
    }
  } //end BaroSelect == 2

  if (baroSelect == 3) { //sensor number that was picked
    tcaselect(3); //set the channel
    bmp4.getEvent(&event);
    if (event.pressure) {
      baro3 = event.pressure;
    }
  } //end BaroSelect == 3

  if (baroSelect == 4) { //sensor number that was picked
    tcaselect(4); //set the channel
    bmp5.getEvent(&event);
    if (event.pressure) {
      baro4 = event.pressure;
    }
  } //end BaroSelect == 4

  if (baroSelect == 5) { //sensor number that was picked
    tcaselect(5); //set the channel
    bmp6.getEvent(&event);
    if (event.pressure) {
      baro5 = event.pressure;
    }
  } //end BaroSelect == 5

  if (baroSelect == 6) { //sensor number that was picked
    tcaselect(6); //set the channel
    bmp7.getEvent(&event);
    if (event.pressure) {
      baro6 = event.pressure;
    }
  } //end BaroSelect == 6
}//end baroRead()

void startupSensorErrorHandler() {
  //begin checking for the two i2c boards here
  //i2c 1 + attached devices
  for (uint8_t t = 0; t < 8; t++) { //this function can handle multiple TCA boards but we're just using one. final digit matches how many channels you have total, starting at 1. Multiplexers should go in sequential order, 0x70, 0x71 to 0x77
    tcaselect(t);
    //Serial.print("TCA Port #"); Serial.println(t); //showing which TCA channel we're on

    for (uint8_t addr = 0; addr <= 127; addr++) {
      if (addr == TCAADDR) continue; //skip looking for the first TCA multiplexer
      if (addr == (TCAADDR + 1)) continue; //skip looking for the 2nd TCA multiplexer, 1 up from the first. add more offsets for multiple boards
      if (addr == (ANEM_I2C_ADDR)) continue; //skip the anemometer, we know it's there (or not) through other checks

      Wire.beginTransmission(addr);
      if (!Wire.endTransmission()) {
        //Serial.print("Found I2C 0x");  Serial.println(addr, HEX); //polling for i2c addresses
        //begin barometer checks
        if ((t == 0 ) && (addr == 77, HEX)) {
          barofail_1 = 0;
        }
        if ((t == 1 ) && (addr == 77, HEX)) {
          barofail_2 = 0;
        }
        if ((t == 2 ) && (addr == 77, HEX)) {
          barofail_3 = 0;
        }
        if ((t == 3 ) && (addr == 77, HEX)) {
          barofail_4 = 0;
        }
        if ((t == 4 ) && (addr == 77, HEX)) {
          barofail_5 = 0;
        }
        if ((t == 5 ) && (addr == 77, HEX)) {
          barofail_6 = 0;
        }
        if ((t == 6 ) && (addr == 77, HEX)) {
          barofail_7 = 0;
        }
        //end baro checks
      }//end !Wire.endTransmission
    }//end i2c address scan
  }//end i2c barometer scan
  //begin i2c anemometer check
  addr = ANEM_I2C_ADDR; //set addr to check for the anemometer at whatever the library says the address should be
  Wire.beginTransmission(addr);
  if (!Wire.endTransmission()) {
    //Serial.print("Found I2C 0x");  Serial.println(addr, HEX); //polling for anemometer
    anemfail_notfound = 0; //clear the trouble code
  }//end !Wire.endTransmission

  //check for i2c multiplexer 1 missing
  addr = TCAADDR; //set addr to check for the multiplexer
  Wire.beginTransmission(addr);
  if (!Wire.endTransmission()) {
    //Serial.print("Found I2C 0x");  Serial.println(addr, HEX); //polling for multiplexer, expected at 0x70
    i2c_1_missing = 0; //clear the trouble code
  }//end !Wire.endTransmission


  if (i2c_1_missing == 1) {
    lcd.setCursor(0, 0);
    lcd.print("I2C Multiplexer 1   ");
    lcd.setCursor(0, 1);
    lcd.print("missing! Expected at");
    lcd.setCursor(0, 2);
    lcd.print("I2C address: 0x70   ");
    lcd.setCursor(0, 3);
    lcd.print("             >Next< "); //normally the >clickme< selector would be dynamic but here we're basically faking it to make it easier to program
    while (digitalRead(ENC_SW) == HIGH) { // this block pauses all code execution while user reads and acknowledges the display. 38 is the button pin
      // Do nothing
    }
    delay(clickDelay);
  }//end i2c_1_missing

  //begin display of calibrating sensors
  lcd.setCursor(0, 0);
  lcd.print("                    ");
  lcd.setCursor(0, 1);
  lcd.print("     Checking       ");
  lcd.setCursor(0, 2);
  lcd.print("     Sensors....    ");
  lcd.setCursor(0, 3);
  lcd.print("                    ");
  delay(2000); //delay so user can see the message. this used to be to allow the anemometer to warm up but since we don't check it for reasonability (it does that itself) we can boot faster

  //begin checking for trouble flags
  if ((barofail_1 == 1) || (barofail_2 == 1) || (barofail_3 == 1) || (barofail_4 == 1) || (barofail_5 == 1) || (barofail_6 == 1) || (barofail_7 == 1) || (anemfail_notfound == 1) && (i2c_1_missing == 0)) {
    //tell user something has an error, begin error checking screens
    state_trouble = 1;//we're also setting a general flag that says something is a 1, comes into play later
    lcd.setCursor(0, 0);
    lcd.print("                    ");
    lcd.setCursor(0, 1);
    lcd.print("Failure to Calibrate");
    lcd.setCursor(0, 2);
    lcd.print("                    ");
    lcd.setCursor(0, 3);
    lcd.print("    >Check sensors< ");

    while (digitalRead(ENC_SW) == HIGH) { // this block pauses all code execution while user reads and acknowledges the display. 38 is the button pin
      // Do nothing
    }
    delay(clickDelay);
    //begin barofail block
    if (barofail_1 == 1) {
      lcd.setCursor(0, 0);
      lcd.print("Failing Barometer:  ");
      lcd.setCursor(0, 1);
      lcd.print(" Baro 1: Not Found! ");
      lcd.setCursor(0, 2);
      lcd.print("                    ");
      lcd.setCursor(0, 3);
      lcd.print("             >Next< ");
      while (digitalRead(ENC_SW) == HIGH) { // this block pauses all code execution while user reads and acknowledges the display. ENC_SW is the button pin
        // Do nothing
      }
      delay(clickDelay);
    }//end barofail_1
    if (barofail_2 == 1) {
      lcd.setCursor(0, 0);
      lcd.print("Failing Barometer:  ");
      lcd.setCursor(0, 1);
      lcd.print(" Baro 2: Not Found! ");
      lcd.setCursor(0, 2);
      lcd.print("                    ");
      lcd.setCursor(0, 3);
      lcd.print("             >Next< ");
      while (digitalRead(ENC_SW) == HIGH) { // this block pauses all code execution while user reads and acknowledges the display. ENC_SW is the button pin
        // Do nothing
      }
      delay(clickDelay);
    }//end barofail_2
    if (barofail_3 == 1) {
      lcd.setCursor(0, 0);
      lcd.print("Failing Barometer:  ");
      lcd.setCursor(0, 1);
      lcd.print(" Baro 3: Not Found! ");
      lcd.setCursor(0, 2);
      lcd.print("                    ");
      lcd.setCursor(0, 3);
      lcd.print("             >Next< ");
      while (digitalRead(ENC_SW) == HIGH) { // this block pauses all code execution while user reads and acknowledges the display. ENC_SW is the button pin
        // Do nothing
      }
      delay(clickDelay);
    }//end barofail_3
    if (barofail_4 == 1) {
      lcd.setCursor(0, 0);
      lcd.print("Failing Barometer:  ");
      lcd.setCursor(0, 1);
      lcd.print(" Baro 4: Not Found! ");
      lcd.setCursor(0, 2);
      lcd.print("                    ");
      lcd.setCursor(0, 3);
      lcd.print("             >Next< ");
      while (digitalRead(ENC_SW) == HIGH) { // this block pauses all code execution while user reads and acknowledges the display. ENC_SW is the button pin
        // Do nothing
      }
      delay(clickDelay);
    }//end barofail_4
    if (barofail_5 == 1) {
      lcd.setCursor(0, 0);
      lcd.print("Failing Barometer:  ");
      lcd.setCursor(0, 1);
      lcd.print(" Baro 5: Not Found! ");
      lcd.setCursor(0, 2);
      lcd.print("                    ");
      lcd.setCursor(0, 3);
      lcd.print("             >Next< ");
      while (digitalRead(ENC_SW) == HIGH) { // this block pauses all code execution while user reads and acknowledges the display. ENC_SW is the button pin
        // Do nothing
      }
      delay(clickDelay);
    }//end barofail_5
    if (barofail_6 == 1) {
      lcd.setCursor(0, 0);
      lcd.print("Failing Barometer:  ");
      lcd.setCursor(0, 1);
      lcd.print(" Baro 6: Not Found! ");
      lcd.setCursor(0, 2);
      lcd.print("                    ");
      lcd.setCursor(0, 3);
      lcd.print("             >Next< ");
      while (digitalRead(ENC_SW) == HIGH) { // this block pauses all code execution while user reads and acknowledges the display. ENC_SW is the button pin
        // Do nothing
      }
      delay(clickDelay);
    }//end barofail_6
    if (barofail_7 == 1) {
      lcd.setCursor(0, 0);
      lcd.print("Failing Barometer:  ");
      lcd.setCursor(0, 1);
      lcd.print(" Baro 7: Not Found! ");
      lcd.setCursor(0, 2);
      lcd.print("                    ");
      lcd.setCursor(0, 3);
      lcd.print("             >Next< ");
      while (digitalRead(ENC_SW) == HIGH) { // this block pauses all code execution while user reads and acknowledges the display. ENC_SW is the button pin
        // Do nothing
      }
      delay(clickDelay);
    }//end barofail_7
    //end barofail block
    if (anemfail_notfound == 1) {
      lcd.setCursor(0, 0);
      lcd.print("Failing Anemometer: ");
      lcd.setCursor(0, 1);
      lcd.print("    Not Found!      ");
      lcd.setCursor(0, 2);
      lcd.print("                    ");
      lcd.setCursor(0, 3);
      lcd.print("             >Next< ");
      while (digitalRead(ENC_SW) == HIGH) { // this block pauses all code execution while user reads and acknowledges the display. ENC_SW is the button pin
        // Do nothing
      }
      delay(clickDelay);
    }//anemometer_notfound end
  }//end calibration failed

  startupComplete = 1;
  //Serial.print("Startup Complete");
  //Serial.println();
}// end startupSensorErrorHandler();

/**
  ░░░░░░░░░░░░░░░░░░░░
  ░▄▀▄▀▀▀▀▄▀▄░░░░░░░░░
  ░█░░░░░░░░▀▄░░░░░░▄░
  █░░▀░░▀░░░░░▀▄▄░░█░█
  █░▄░█▀░▄░░░░░░░▀▀░░█
  █░░▀▀▀▀░░░░░░░░░░░░█
  █░░░░░░░░░░░░░░░░░░█
  █░░░░░░░░░░░░░░░░░░█
  ░█░░▄▄░░▄▄▄▄░░▄▄░░█░
  ░█░▄▀█░▄▀░░█░▄▀█░▄▀░
  ░░▀░░░▀░░░░░▀░░░▀░░░
*/
