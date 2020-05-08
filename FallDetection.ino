#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

/**

   Code Written By: Tendai Makumire
   Last Updated: 08-May-2020

  Code Description:
  Code for wearable(on the waist) device designed to detect falls. Uses the ADXL345
  to detect orientation and measure the horizontal vector magnitude (HZM). The ADXl345
  is oriented so that the X axis is downwards and the Y axis points to the right. 
  Thresholds are extracted from SisFall Dataset. 

  TODO:
  - Use proper MAX values for checks
  - Automatic Calibration using Magnetometer
  - Develop interrupt based I2C driver for sensor
  - Reduce memory usage



**/

/************************* Defines *******************************************************/

#define CONSTANT_G 9.81
#define ACCEL_THRESHOLD  (1.82 * CONSTANT_G)
#define SETTLE_THRESHOLD (0.30 * CONSTANT_G)
#define ANGLE_THRESHOLD 40.0
#define BUFFER_LEN 127

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

/************************* Function Prototypes ****************************************************/

Imu_accel ReadAccel();
Imu_rotation CalcTilt(Imu_accel* accel);
Imu_rotation AverageRotation(uint16_t start_index, uint8_t samples);

float CalcRoll(Imu_accel* accel);
float CalcPitch(Imu_accel* accel);
float CalcHoriz_Mag(Imu_accel* accel);
float CalcPkPk();
float FilterLowPass(float new_input, float old_output);
uint16_t GetBufferPosition(uint16_t currentPos, int8_t samples);

/************************* Main ****************************************************/

Imu_rotation tilt[BUFFER_LEN];
float vector_mag[10] = {0.0};
uint8_t mag_index = 0;

uint16_t buffer_index = 0;
uint16_t accel_trigger = MAX_UINT16;
int16_t tilt_trigger = MAX_INT16;

float y_rotF = 0.0;
float z_rotF = 0.0;

/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified adxl = Adafruit_ADXL345_Unified(12345);

Imu_rotation current_orientation;

void setup() {

  Serial.begin(115200);

  // Disable pullups
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);

  // Enable LED
  pinMode(LED_BUILTIN, OUTPUT);

  /* Initialise the ADXL345 */
  while (!adxl.begin())
  {
    /* Flash the LED a couple of times times then try again */
    blinkLED();

  }

  adxl.setRange(ADXL345_RANGE_16_G);
  adxl.setDataRate(ADXL345_DATARATE_25_HZ);

  // Set initial values for orientation
  Imu_accel initAccel = ReadAccel();
  tilt[buffer_index] = CalcTilt(&initAccel);
  buffer_index++;

}


void loop() {
  
  Imu_accel accel = ReadAccel();
  unsigned long current_time = millis();

  // Calculate Horizontal acceleration vector magnitude
  float accel_mag = CalcHoriz_Mag(&accel);

  if (buffer_index > accel_trigger + 50) {
    vector_mag[mag_index] = accel_mag;
    mag_index++;
  }

  // Get Current Roll/Pitch angles then
  Imu_rotation filteredTilt;
  Imu_rotation currentTilt = CalcTilt(&accel);

  // Apply Low Pass IIR Filter to roll/pitch angles
  y_rotF = FilterLowPass(currentTilt.pitch, y_rotF);
  z_rotF = FilterLowPass(currentTilt.roll, z_rotF);
  filteredTilt.roll = z_rotF;
  filteredTilt.pitch = y_rotF;
  tilt[buffer_index] = filteredTilt;

  /* ------------ DEBUG Roll and Pitch  ------------*/
  Serial.print(tilt[buffer_index].pitch); Serial.print(" ");
  Serial.println(tilt[buffer_index].roll);

  // Check if acceleration above the threshold
  if (accel_mag > ACCEL_THRESHOLD) {
    mag_index = 0;
    accel_trigger = buffer_index;
    tilt_trigger = GetBufferPosition(accel_trigger, 60);

    // Get Orientation 2.4s before acceleration trigger and set as current Orientation
    // Take an average of the samples before from -2.4s to -2.0s
    uint16_t index = GetBufferPosition(accel_trigger, -60);
    current_orientation = AverageRotation(index, 10);

    /* ------------ DEBUG Roll and Pitch after High Accel  ------------*/
    Serial.print("Current Orientation is (Forward/Backward | Left/Right: ");
    Serial.print(current_orientation.pitch); Serial.print(" ");
    Serial.println(current_orientation.roll);

  }

  // Check if enough time has passed after the fall trigger
  if (buffer_index == tilt_trigger) {

    // Check if signal has settled and calculate the change in orientation
    if (CalcPkPk() < SETTLE_THRESHOLD) {
      // Check change in orientation and send message on serial monitor that a fall has occured
      Serial.println("Signal has settled, check change in orientation");

      uint16_t index = GetBufferPosition(buffer_index, -10);
      Imu_rotation new_orientation = AverageRotation(index, 10);

      /* ------------ DEBUG Roll and Pitch after High Accel  ------------*/
      Serial.print("New Orientation is (Forward/Backward | Left/Right: ");
      Serial.print(new_orientation.pitch); Serial.print(" ");
      Serial.println(new_orientation.roll);

      float orientation_change; 
      float roll_change = abs(new_orientation.roll - current_orientation.roll);
      float pitch_change = abs(new_orientation.pitch - current_orientation.pitch);
      

      if (pitch_change > roll_change) {
        orientation_change = pitch_change;
      }else {
        orientation_change = roll_change;
      }

      if (orientation_change > ANGLE_THRESHOLD) {
        // Fall Detected - Blink the LED
        blinkLED();

      }

      // Reset accel_trigger and tilt_trigger to MAX value 
      tilt_trigger = MAX_INT16;
      accel_trigger = MAX_UINT16;

    }

  }

  buffer_index = GetBufferPosition(buffer_index, 1);

  // Wait 40ms until next sample
  while(current_time + 40 < millis()) {
  }


}


/************************* Functions ***********************************************/

Imu_rotation AverageRotation(uint16_t start_index, uint8_t samples) {

  Imu_rotation average_rot = {0.0, 0.0};
  uint8_t i = 0;

  for (i = 0; i < samples; i++) {
    uint16_t tilt_index = GetBufferPosition(start_index, i);
    average_rot.roll += tilt[tilt_index].roll;
    average_rot.pitch += tilt[tilt_index].pitch;

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
  // For Prototype - Orientation --> X downwards, Y left/right, Z - towards you
  float vector_mag = sqrt((accel->y * accel->y) + (accel->z * accel->z));
  return vector_mag;
}

float CalcRoll(Imu_accel* accel) {

  // For SisFall Dataset - Orientation --> Y downwards, X left/right
  // double z_rot = atan(-1 * X_out / sqrt(pow(Y_out, 2) + pow(Z_out, 2))) * 180 / PI;

  // For Prototype - Orientation --> X downwards, Y left/right
  float z_rot = atan(-1 * accel->y / sqrt(pow(accel->x, 2) + pow(accel->z, 2))) * 180 / PI;
  return z_rot;

}


float CalcPitch(Imu_accel* accel) {

  // For SisFall Dataset Orientation - Y downwards, X left/right
  // float x_rot = atan(-1 * Z_out / sqrt(pow(X_out, 2) + pow(Y_out, 2))) * 180 / PI;

  // For Prototype Orientation - X downwards, Y left/right
  float y_rot = atan(-1 * accel->z / sqrt(pow(accel->x, 2) + pow(accel->y, 2))) * 180 / PI;
  return y_rot;

}

float CalcPkPk() {

  float minVal;
  float maxVal;
  uint8_t i = 2;

  // Initialise Min/Max
  if (vector_mag[1] > vector_mag[0]) {
    maxVal = vector_mag[1];
    minVal = vector_mag[0];
  } else {
    maxVal = vector_mag[0];
    minVal = vector_mag[1];
  }

  // Find Min/Max in array
  for (i = 2; i < 10; i++) {

    if (vector_mag[i] > maxVal) {
      maxVal = vector_mag[i];
    }
    else if (vector_mag[i] < minVal) {
      minVal = vector_mag[i];
    }

  }

  return (maxVal - minVal);
}


Imu_rotation CalcTilt(Imu_accel* accel) {

  Imu_rotation tilt;
  tilt.roll = CalcRoll(accel);
  tilt.pitch = CalcPitch(accel);
  return tilt;

}


float FilterLowPass(float new_input, float old_output) {
  // IIR Low pass Filter y[n] = 0.90y[n-1] + 0.10x[n]
  return (0.9 * old_output + 0.10 * new_input);
}


uint16_t GetBufferPosition(uint16_t currentPos, int8_t samples) {

  int16_t bufferPos = currentPos + samples;

  if (bufferPos > BUFFER_LEN - 1) {
    bufferPos -= BUFFER_LEN;
  }

  if (bufferPos < 0) {
    bufferPos += BUFFER_LEN;

  }

  return (uint16_t) bufferPos;

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
