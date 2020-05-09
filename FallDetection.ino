#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

/**

   Code Written By: Tendai Makumire
   Last Updated: 09-May-2020

  Code Description:
  Code for wearable(on the waist) device designed to detect falls. Uses the ADXL345
  to detect orientation and measure the horizontal vector magnitude (HZM). The ADXl345
  is oriented so that the X axis is downwards and the Y axis points to the right. 
  Thresholds are extracted from SisFall Dataset. 

  TODO:
  - Use proper MAX values for checks
  - Automatic Calibration using Magnetometer 
  - Improve Orientation estimate with gyroscope
  - Develop interrupt based I2C driver for sensor
  - Reduce memory usage

**/

/************************* Defines *******************************************************/
#define CONSTANT_G 9.81
#define ACCEL_THRESHOLD  (1.82 * CONSTANT_G)
#define SETTLE_THRESHOLD (0.30 * CONSTANT_G)
#define ANGLE_THRESHOLD 40.0
#define BUFFER_LEN 127
#define SAMPLE_TIME_MS 40

#define MAX_UINT16 1000
#define MAX_INT16 1000
#define MAX_FLOAT 1000.0

#define OFFSET_X 0.32;
#define OFFSET_Y 0.56;
#define OFFSET_Z 1.06;

/************************* Structs ****************************************************************/
typedef struct {
  float roll;
  float pitch;

} Imu_rotation;

typedef struct {
  float x;
  float y;
  float z;

} Imu_accel;

/************************* Enums ****************************************************************/
enum States {
  stateMeasureAccel,
  stateSetTriggers,
  stateResetTriggers,
  stateCheckSignal,
  stateCheckOrientation,
  stateFallDetected

};

/************************* Function Prototypes ****************************************************/
Imu_accel ReadAccel();
Imu_rotation CalcTilt(Imu_accel* accel);
Imu_rotation AverageRotation(uint16_t start_index, uint8_t samples);

float CalcRoll(Imu_accel* accel);
float CalcPitch(Imu_accel* accel);
float CalcHoriz_Mag(Imu_accel* accel);
float CalcPkPk();
float FilterLowPass(float new_input, float old_output);
uint16_t GetBufferPosition(uint16_t current_pos, int8_t samples);

/************************* Global Variables ***********************************************/
States mState;
Imu_rotation mTilt[BUFFER_LEN];
Imu_rotation mCurrent_orientation;

float mVector_mag[10] = {0.0};
uint8_t mMag_index = 0;

uint16_t mBuffer_index = 0;
uint16_t mAccel_trigger = MAX_UINT16;
int16_t mTilt_trigger = MAX_INT16;

float mY_rotF = 0.0;
float mZ_rotF = 0.0;

unsigned long mCurrent_time;
Adafruit_ADXL345_Unified adxl = Adafruit_ADXL345_Unified(1);

/************************* Main ****************************************************/
void setup() {

  Serial.begin(115200);

  // Disable pullups
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);

  // Enable LED
  pinMode(LED_BUILTIN, OUTPUT);

  /* Initialise the ADXL345 */
  while (!adxl.begin()) {
    /* Flash the LED a couple of times times then try again */
    blinkLED();
  }

  // Initialise ADXL345 parameters
  adxl.setRange(ADXL345_RANGE_16_G);
  adxl.setDataRate(ADXL345_DATARATE_25_HZ);

  // Set initial values for orientation
  Imu_accel initAccel = ReadAccel();
  mTilt[mBuffer_index] = CalcTilt(&initAccel);
  mBuffer_index++;

  mCurrent_time = millis();

  // Start measuring accel 
  mState = stateMeasureAccel;

}


void loop() {
  
  switch(mState) {

  case stateMeasureAccel:
  {
    // Wait 40ms until next sample
    while((unsigned long)(millis() - mCurrent_time) <= SAMPLE_TIME_MS);
    Imu_accel accel = ReadAccel();
    mCurrent_time = millis();

    // Calculate Horizontal acceleration vector magnitude
    float accel_mag = CalcHoriz_Mag(&accel);

    if (mBuffer_index > mAccel_trigger + 50) {
        mVector_mag[mMag_index] = accel_mag;
        mMag_index++;
    }

    // Get Current roll/pitch angles
    Imu_rotation filteredTilt;
    Imu_rotation currentTilt = CalcTilt(&accel);

    // Apply Low Pass IIR Filter to roll/pitch angles
    mY_rotF = FilterLowPass(currentTilt.pitch, mY_rotF);
    mZ_rotF = FilterLowPass(currentTilt.roll, mZ_rotF);
    filteredTilt.roll = mZ_rotF;
    filteredTilt.pitch = mY_rotF;
    mTilt[mBuffer_index] = filteredTilt;

    /* ------------ DEBUG Roll and Pitch  ------------*/
    Serial.print(mTilt[mBuffer_index].pitch); Serial.print(" ");
    Serial.println(mTilt[mBuffer_index].roll);

    // Check if acceleration above the threshold
    if (accel_mag > ACCEL_THRESHOLD) {
      mState = stateSetTriggers;
    }

    // Check if enough time has passed after the fall trigger
    if (mBuffer_index == mTilt_trigger) {
      mState = stateCheckSignal;
    }
    break;
  }

  case stateSetTriggers:
  {
    mMag_index = 0;
    mAccel_trigger = mBuffer_index;
    mTilt_trigger = GetBufferPosition(mAccel_trigger, 60);

    // Get Orientation 2.4s before acceleration trigger and set as current Orientation
    // Take an average of the samples before from -2.4s to -2.0s
    uint16_t index = GetBufferPosition(mAccel_trigger, -60);
    mCurrent_orientation = AverageRotation(index, 10);

    /* ------------ DEBUG Roll and Pitch after High Accel  ------------*/
    Serial.print("Current Orientation is (Forward/Backward | Left/Right: ");
    Serial.print(mCurrent_orientation.pitch); Serial.print(" ");
    Serial.println(mCurrent_orientation.roll);

    // Go back to measuring the accel
    mState = stateMeasureAccel;
    break;
  }

  case stateCheckSignal:
  {
    if (CalcPkPk() < SETTLE_THRESHOLD) {
      /* ------------------------ DEBUG   ------------------------*/
      Serial.println("Signal has settled, check change in orientation");

      mState = stateCheckOrientation;
    }
    else {
      // No Fall Detected so reset the trigger points
      mState = stateResetTriggers;
    }
    break;
  }

  case stateResetTriggers:
  {
    mMag_index = 0;
    mTilt_trigger = MAX_INT16;
    mAccel_trigger = MAX_UINT16;

    // Continue measuring accel
    mState = stateMeasureAccel;
    break;
  }

  case stateCheckOrientation:
  {
    uint16_t index = GetBufferPosition(mBuffer_index, -10);
    Imu_rotation new_orientation = AverageRotation(index, 10);

    /* ------------ DEBUG Roll and Pitch after High Accel  ------------*/
    Serial.print("New Orientation is (Forward/Backward | Left/Right: ");
    Serial.print(new_orientation.pitch); Serial.print(" ");
    Serial.println(new_orientation.roll);

    float orientation_change; 
    float roll_change = abs(new_orientation.roll - mCurrent_orientation.roll);
    float pitch_change = abs(new_orientation.pitch - mCurrent_orientation.pitch);
    
    if (pitch_change > roll_change) {
      orientation_change = pitch_change;
    }else {
      orientation_change = roll_change;
    }

    if (orientation_change > ANGLE_THRESHOLD) {
      // Fall has been detected
      mState = stateFallDetected;
    }else {
      // Fall not detected due to low change in orientation
      mState = stateResetTriggers;
    }

    break;
  }

  case stateFallDetected:
  {
    blinkLED();
    Serial.println("Fall Detected");
    mState = stateResetTriggers;
    break;
  }

  }

  // Increment buffer index only if we are about measure the acceleration 
  if (mState == stateMeasureAccel) {
    mBuffer_index = GetBufferPosition(mBuffer_index, 1);  
  }
   
}

/************************* Functions ***********************************************/
Imu_rotation AverageRotation(uint16_t start_index, uint8_t samples) {
  Imu_rotation average_rot = {0.0, 0.0};
  uint8_t i = 0;
  
  for (i = 0; i < samples; i++) {
    uint16_t tilt_index = GetBufferPosition(start_index, i);
    average_rot.roll += mTilt[tilt_index].roll;
    average_rot.pitch += mTilt[tilt_index].pitch;
  }

  average_rot.roll = average_rot.roll / samples;
  average_rot.pitch = average_rot.pitch / samples;
  return average_rot;
  
}


Imu_accel ReadAccel() {
  /* Get a new sensor event */
  sensors_event_t event;
  adxl.getEvent(&event);

  Imu_accel accel;
  accel.x = event.acceleration.x + OFFSET_X;
  accel.y = event.acceleration.y + OFFSET_Y;
  accel.z = event.acceleration.z + OFFSET_Z;
  return accel;
  
}


float CalcHoriz_Mag(Imu_accel* accel) {
  float horiz_mag = sqrt((accel->y * accel->y) + (accel->z * accel->z));
  return horiz_mag;
  
}


float CalcRoll(Imu_accel* accel) {
  float z_rot = atan(-1 * accel->y / sqrt(pow(accel->x, 2) + pow(accel->z, 2))) * 180 / PI;
  return z_rot;

}


float CalcPitch(Imu_accel* accel) {
  float y_rot = atan(-1 * accel->z / sqrt(pow(accel->x, 2) + pow(accel->y, 2))) * 180 / PI;
  return y_rot;

}


Imu_rotation CalcTilt(Imu_accel* accel) {
  Imu_rotation tilt;
  tilt.roll = CalcRoll(accel);
  tilt.pitch = CalcPitch(accel);
  return tilt;

}


float CalcPkPk() {
  float minVal = mVector_mag[0];
  float maxVal = mVector_mag[0];
  uint8_t i;

  // Find Min/Max in array
  for (i = 1; i < 10; i++) {

    if (mVector_mag[i] > maxVal) {
      maxVal = mVector_mag[i];
    }
    else if (mVector_mag[i] < minVal) {
      minVal = mVector_mag[i];
    }
  }
  return (maxVal - minVal);

}


float FilterLowPass(float new_input, float old_output) {
  // IIR Low pass Filter y[n] = 0.90y[n-1] + 0.10x[n]
  return (0.9 * old_output + 0.10 * new_input);

}


uint16_t GetBufferPosition(uint16_t current_pos, int8_t samples) {
  int16_t buffer_pos = current_pos + samples;
  if (buffer_pos > BUFFER_LEN - 1) {
    buffer_pos -= BUFFER_LEN;
  }

  if (buffer_pos < 0) {
    buffer_pos += BUFFER_LEN;
  }
  return (uint16_t) buffer_pos;

}


void blinkLED() {
  unsigned long current_time = millis();
  while (current_time + 2000 > millis()) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);

  }
  Serial.println("LED BLINKED");

}
