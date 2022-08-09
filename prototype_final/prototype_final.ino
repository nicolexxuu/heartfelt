#define BLYNK_TEMPLATE_ID "TMPLVmzj5gZK"
#define BLYNK_DEVICE_NAME "heartfelt"
#define BLYNK_AUTH_TOKEN "FWIkCMu5cn0eFZv1WdfAS4YkYyayr81F"

// Blynk libraries
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>

char auth[] = BLYNK_AUTH_TOKEN;

// WiFi credentials
char ssid[] = "AirPennNet-Device";
char pass[] = "penn1740wifi";


// SparkFun load cell libraries
#include <Wire.h>
#include <EEPROM.h> //Needed to record user settings

#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h" // click here to get the library: http://librarymanager/All#SparkFun_NAU8702

NAU7802 myScale; // create instance of the NAU7802 class

// EEPROM locations to store 4-byte variables
#define LOCATION_CALIBRATION_FACTOR 0 // float, requires 4 bytes of EEPROM
#define LOCATION_ZERO_OFFSET 10 // must be more than 4 away from previous spot. Long, requires 4 bytes of EEPROM

bool settingsDetected = false; // used to prompt user to calibrate their scale

// take average of weights and compression times to smooth at jitter
#define AVG_SIZE 5
float avgWeights[AVG_SIZE], avgTimes[AVG_SIZE];
int timeIdx = 0, weightIdx = 0;

double startPressTime, endPressTime, currTime; // used to calculate compression speed
// tracks whether user releases chest fully between compressions
bool returnedBetweenPresses = true;

const int MIN_CMP_PER_MIN = 100, MAX_CMP_PER_MIN = 120;
const int COMP_THRESHOLD = 200, RELEASED = 50;
const double NUM_SPRINGS = 5;
const long ZERO_OFFSET = 34127;
const float CALIB_FACTOR = -500;

void setup() {
  Serial.begin(9600);
  startPressTime = endPressTime = millis();	

  Blynk.begin(auth, ssid, pass);
  Blynk.virtualWrite(V3, "");

  Wire.begin();
  Wire.setClock(400000); // Qwiic Scale is capable of running at 400kHz if desired

  if (myScale.begin() == false)
  {
    Serial.println("Scale not detected. Please check wiring. Freezing...");
    while (1);
  }
  Serial.println("Scale detected!");

  readSystemSettings(); // load zeroOffset and calibrationFactor from EEPROM

  myScale.setSampleRate(NAU7802_SPS_320); // increase to max sample rate
  myScale.calibrateAFE(); // re-cal analog front end when we change gain, sample rate, or channel 

  myScale.setCalibrationFactor(CALIB_FACTOR);
  myScale.setZeroOffset(ZERO_OFFSET);
  recordSystemSettings();

  Serial.print("Zero offset: ");
  Serial.println(myScale.getZeroOffset());
  Serial.print("Calibration factor: ");
  Serial.println(myScale.getCalibrationFactor());
}

void loop()
{
  if(!settingsDetected) Serial.print("\tScale not calibrated. Press 'c'.");

  // calculate average weight
  float currentWeight = myScale.getWeight();
  avgWeights[weightIdx] = currentWeight;
  weightIdx = (weightIdx + 1) % AVG_SIZE;

  double avgWeight = 0;
  for (int x = 0 ; x < AVG_SIZE ; x++)
    avgWeight += avgWeights[x];
  avgWeight /= AVG_SIZE;

  if(avgWeight < 0) avgWeight *= -1; // oops
  

  // 100-125 lbs is optimal compression force
  if(avgWeight > COMP_THRESHOLD) { 
    if(endPressTime != -1) { // pushing downwards; crossed threshold
      currTime = millis();
      double time = calcAvgTime(currTime - startPressTime);
      startPressTime = millis();
      
      // Serial.print("time in between compressions: ");
      // Serial.println(time);

      if(time < 60000/MAX_CMP_PER_MIN) {
        Serial.println("Too fast!");
        Blynk.virtualWrite(V2, "Too fast!");
      } else if(time > 60000/MIN_CMP_PER_MIN) {
        Blynk.virtualWrite(V2, "Too slow!");
        Serial.println("Too slow!");
      } else {
        Blynk.virtualWrite(V2, "Good speed!");
        Serial.println("Good speed!");
      }

      if(!returnedBetweenPresses) {
        Serial.println("Make sure to let the chest recover fully between compressions!");
        Blynk.virtualWrite(V3, "Let the chest recover between compressions!");
      } else {
        Blynk.virtualWrite(V3, "");
      }

      endPressTime = -1;
      returnedBetweenPresses = false;
      delay(100);
    }
  } else {
    if(avgWeight < RELEASED) returnedBetweenPresses = true;

    if(endPressTime == -1) { // releasing upwards; crossed threshold (again)
      endPressTime = millis();
      delay(100);
    }
  }


  if (Serial.available()) {
    byte incoming = Serial.read();

    if (incoming == 't') myScale.calculateZeroOffset();
    else if (incoming == 'c')  calibrateScale();
  }
}

double calcAvgTime(double idleTime) {
    avgTimes[timeIdx] = idleTime;
    timeIdx = (timeIdx + 1) % AVG_SIZE;

    double sum = 0;
    for(double d : avgTimes) sum += d;
    return sum / AVG_SIZE;
}

// gives user the ability to set a known weight on the scale and calculate a calibration factor
void calibrateScale(void) {
  Serial.println();
  Serial.println();
  Serial.println(F("Scale calibration"));

  Serial.println(F("Setup scale with no weight on it. Press a key when ready."));
  while (Serial.available()) Serial.read(); // clear anything in RX buffer
  while (Serial.available() == 0) delay(10); // wait for user to press key

  myScale.calculateZeroOffset(64); // zero or tare the scale. Average over 64 readings.
  Serial.print(F("New zero offset: "));
  Serial.println(myScale.getZeroOffset());

  Serial.println(F("Place known weight on scale. Press a key when weight is in place and stable."));
  while (Serial.available()) Serial.read(); // clear anything in RX buffer
  while (Serial.available() == 0) delay(10); // wait for user to press key

  Serial.print(F("Please enter the weight, without units, currently sitting on the scale (for example '4.25'): "));
  while (Serial.available()) Serial.read(); // clear anything in RX buffer
  while (Serial.available() == 0) delay(10); // wait for user to press key

  // read user input
  float weightOnScale = Serial.parseFloat();
  Serial.println();

  myScale.calculateCalibrationFactor(weightOnScale, 64); // tTell the library how much weight is currently on it
  Serial.print(F("New cal factor: ")); 
  Serial.println(myScale.getCalibrationFactor(), 2);

  Serial.print(F("New Scale Reading: "));
  Serial.println(myScale.getWeight(), 2);

  recordSystemSettings(); // commit these values to EEPROM
}

// record the current system settings to EEPROM
void recordSystemSettings(void) {
  // get various values from the library and commit them to NVM
  EEPROM.put(LOCATION_CALIBRATION_FACTOR, myScale.getCalibrationFactor());
  EEPROM.put(LOCATION_ZERO_OFFSET, myScale.getZeroOffset());
}

// reads the current system settings from EEPROM
// if anything looks weird, reset setting to default value
void readSystemSettings(void) {
  float settingCalibrationFactor; // value used to convert the load cell reading to lbs or kg
  long settingZeroOffset; // zero value that is found when scale is tared

  // look up the calibration factor
  EEPROM.get(LOCATION_CALIBRATION_FACTOR, settingCalibrationFactor);
  if (settingCalibrationFactor == 0xFFFFFFFF)
  {
    settingCalibrationFactor = 0; // default to 0
    EEPROM.put(LOCATION_CALIBRATION_FACTOR, settingCalibrationFactor);
  }
  
  // look up the zero tare point
  EEPROM.get(LOCATION_ZERO_OFFSET, settingZeroOffset);
  if (settingZeroOffset == 0xFFFFFFFF)
  {
    settingZeroOffset = 1000L; // default to 1000 so we don't get inf
    EEPROM.put(LOCATION_ZERO_OFFSET, settingZeroOffset);
  }

  // pass these values to the library
  myScale.setCalibrationFactor(settingCalibrationFactor);
  myScale.setZeroOffset(settingZeroOffset);

  settingsDetected = true; // assume for the moment that there are good cal values
  if (settingCalibrationFactor < 0.1 || settingZeroOffset == 1000)
    settingsDetected = false; // Defaults detected. Prompt user to cal scale.
}
