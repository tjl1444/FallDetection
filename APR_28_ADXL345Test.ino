#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// New offsets
#define OFFSET_X 1.0702
#define OFFSET_Y 0.793
#define OFFSET_Z 0.947

unsigned long timeTrial;

/*
   Help Section:
   This sketch prints out Roll, Pitch and Acceleration Vector Magntiude on Serial plotter
   For the demonstration we want to show a fall which indicates a change in the roll or pitch angle and acceleration change
   You will need to download the Adafruit Sensor Library from: https://github.com/adafruit/Adafruit_Sensor
   You will also need to download the Adafruit Library for the ADXL345.
   You can find videos on Youtube to show you how to add libraries downloaded from the internet
   You may want to use the Low-pass filter code near the bottom for the the roll + pitch (tilt angles).
   You may want to use your own offsets for X,Y and Z (The offsets are defined at the top!)
   If your sensor is not reading 9.81 for the Z axis when lying flat then you need to change the offset!

   By Tendai Makumire
*/

/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

double roll = 0;
double pitch = 0;

void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displayDataRate(void)
{
  Serial.print  ("Data Rate:    ");

  switch (accel.getDataRate())
  {
    case ADXL345_DATARATE_3200_HZ:
      Serial.print  ("3200 ");
      break;
    case ADXL345_DATARATE_1600_HZ:
      Serial.print  ("1600 ");
      break;
    case ADXL345_DATARATE_800_HZ:
      Serial.print  ("800 ");
      break;
    case ADXL345_DATARATE_400_HZ:
      Serial.print  ("400 ");
      break;
    case ADXL345_DATARATE_200_HZ:
      Serial.print  ("200 ");
      break;
    case ADXL345_DATARATE_100_HZ:
      Serial.print  ("100 ");
      break;
    case ADXL345_DATARATE_50_HZ:
      Serial.print  ("50 ");
      break;
    case ADXL345_DATARATE_25_HZ:
      Serial.print  ("25 ");
      break;
    case ADXL345_DATARATE_12_5_HZ:
      Serial.print  ("12.5 ");
      break;
    case ADXL345_DATARATE_6_25HZ:
      Serial.print  ("6.25 ");
      break;
    case ADXL345_DATARATE_3_13_HZ:
      Serial.print  ("3.13 ");
      break;
    case ADXL345_DATARATE_1_56_HZ:
      Serial.print  ("1.56 ");
      break;
    case ADXL345_DATARATE_0_78_HZ:
      Serial.print  ("0.78 ");
      break;
    case ADXL345_DATARATE_0_39_HZ:
      Serial.print  ("0.39 ");
      break;
    case ADXL345_DATARATE_0_20_HZ:
      Serial.print  ("0.20 ");
      break;
    case ADXL345_DATARATE_0_10_HZ:
      Serial.print  ("0.10 ");
      break;
    default:
      Serial.print  ("???? ");
      break;
  }
  Serial.println(" Hz");
}

void displayRange(void)
{
  Serial.print  ("Range:         +/- ");

  switch (accel.getRange())
  {
    case ADXL345_RANGE_16_G:
      Serial.print  ("16 ");
      break;
    case ADXL345_RANGE_8_G:
      Serial.print  ("8 ");
      break;
    case ADXL345_RANGE_4_G:
      Serial.print  ("4 ");
      break;
    case ADXL345_RANGE_2_G:
      Serial.print  ("2 ");
      break;
    default:
      Serial.print  ("?? ");
      break;
  }
  Serial.println(" g");
}

void setup(void)
{
#ifndef ESP8266
  while (!Serial); // for Leonardo/Micro/Zero
#endif
  Serial.begin(115200);
  Serial.println("Accelerometer Test"); Serial.println("");

  /* Initialise the sensor */
  if (!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while (1);
  }
  
  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL345_RANGE_16_G);

  /* Set data rate appropriate for your project */
  accel.setDataRate(ADXL345_DATARATE_100_HZ);

  /* Display some basic information on this sensor */
  //displaySensorDetails();

  /* Display additional settings (outside the scope of sensor_t) */
  //displayDataRate();
  //displayRange();
  //Serial.println("");

  // Disable pullups
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);

  timeTrial = millis();
  Serial.println("Time trial started");
}

void loop(void)
{
  if (millis() >= (timeTrial + 15000)) {
    Serial.println("Walk Data collection");
    while(1);  
  }
  /* Get a new sensor event */
  sensors_event_t event;
  accel.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2) */
  //  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  //  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  //  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");

  /* Test 1 - Z Axis Acceleration */
  //Serial.println(event.acceleration.z + OFFSET_Z); Serial.print("  ");
  //Serial.println(event.acceleration.z + OFFSET_Z); Serial.print("  ");

  /* Test 2 - Signal Vector Magnitude */
  // float a = (event.acceleration.x * event.acceleration.x) + (event.acceleration.y * event.acceleration.y) + (event.acceleration.z * event.acceleration.z);
  // Serial.println(sqrt(a)); Serial.print("  ");

  /* Test 3 - Roll and Pitch Estimation and Acceleration Magnitude*/
  // Calculate Roll and Pitch (rotation around X-axis, rotation around Y-axis)
  double X_out = event.acceleration.x + OFFSET_X;
  double Y_out = event.acceleration.y + OFFSET_Y;
  double Z_out = event.acceleration.z + OFFSET_Z;
  
  roll = atan2(Y_out, sqrt(pow(X_out, 2) + pow(Z_out, 2))) * 180 / PI;
  pitch = atan2(-1 * X_out,sqrt(pow(Y_out, 2) + pow(Z_out, 2))) * 180 / PI;
  double accelerationMagnitude = sqrt((X_out * X_out) + (Y_out * Y_out) + (Z_out * Z_out));
  
  Serial.print(roll); Serial.print(" ");
  Serial.print(pitch); Serial.print(" ");
  Serial.println(accelerationMagnitude);

  //Serial.println(Z_out);
 
  // Low-pass filter for Roll and Pitch
  //  rollF = 0.94 * rollF + 0.06 * roll;
  //  pitchF = 0.94 * pitchF + 0.06 * pitch;

  delay(10);
}
