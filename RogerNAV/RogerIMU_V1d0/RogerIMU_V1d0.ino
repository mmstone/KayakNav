////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  RogerIMU V1.0  Real-time On-board GPS Enabled Rowing Inertial Measurement Unit
//
//  9 DOF Sensor (3 axis magnetometer, 3 axis accelerometer, 3 axis gyro), connected to
//    Adafruit M0 Feather Protoboard
//    Adafruit Feather Protoboard
//    Adafruit Feather Doubler
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Mahony.h>
#include <Madgwick.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
//
#define NXP_FXOS8700_FXAS21002      (2)
//
#define AHRS_VARIANT   NXP_FXOS8700_FXAS21002
//
//
// Create sensor instances.
Adafruit_FXAS21002C gyro    = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag  = Adafruit_FXOS8700(0x8700A, 0x8700B);
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
boolean trace = true;
int comCount = 0;
float heading = 0;
float roll    = 0;
float pitch   = 0;
float qw, qx, qy, qz;
float qHead   = 0;
//
//
// Mag calibration values are calculated via ahrs_calibration.
// These values must be determined for each baord/environment.
// See the image in this sketch folder for the values used
// below.
//
// Offsets applied to raw x/y/z mag values
float mag_offsets[3]            = { 18.90F, 5.48F, 110.70F };
//
// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { {  1.002,  -0.007,  -0.005 },
                                    { -0.007,   0.989,   0.007 },
                                    { -0.005,   0.007,   1.010 } };
//
float mag_field_strength        = 48.25F;
//
// Offsets applied to compensate for gyro zero-drift error for x/y/z
// Raw values converted to rad/s based on 250dps sensitiviy (1 lsb = 0.00875 rad/s)
float rawToDPS = 0.00875F;
float dpsToRad = 0.017453293F;
float gyro_zero_offsets[3]      = { 0.0F * rawToDPS * dpsToRad,
                                    0.0F * rawToDPS * dpsToRad,
                                    0.0F * rawToDPS * dpsToRad };
//
//
// Mahony is lighter weight as a filter and should be used
// on slower systems
//Mahony filter;
Madgwick filter;
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void computeEuler() {
  // Print the orientation filter output
  // Note: To avoid gimbal lock you should read quaternions not Euler
  // angles, but Euler angles are used here since they are easier to
  // understand looking at the raw values. See the ble fusion sketch for
  // and example of working with quaternion data.
  roll = filter.getRoll();
  pitch = filter.getPitch();
  heading = (359.99 - filter.getYaw());
}
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void computeQuat() {
    // Print the orientation filter output in quaternions.
    // This avoids the gimbal lock problem with Euler angles when you get
    // close to 180 degrees (causing the model to rotate or flip, etc.)
    filter.getQuaternion(&qw, &qx, &qy, &qz);
    computeHeading();
}
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void computeHeading() {
  qHead = abs(qw * 180);
  if ((qHead == 0.00) || (qHead == 180.00)) {
    return;  
    }
  if (heading > 180.00) {
    qHead = (360.00 - qHead);
    } 
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void sendHeadingRollPitch() {
  Serial1.print('H');
  Serial1.print(heading);
  Serial1.print(';');
  Serial1.print('R');
  Serial1.print(roll);
  Serial1.print(';');
  Serial1.print('P');
  Serial1.print(pitch);
  Serial1.print(';');
  Serial1.print('W');
  Serial1.print(qw);
  Serial1.print(';');
  Serial1.print('X');
  Serial1.print(qx);
  Serial1.print(';');
  Serial1.print('Y');
  Serial1.print(qy);
  Serial1.print(';');
  Serial1.print('Z');
  Serial1.print(qz);
  Serial1.print('\n');     
  Serial1.flush();
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void showHeadingRollPitch() {
  Serial.print("Heading: ");
  Serial.print(heading);
  Serial.print(" Pitch: ");
  Serial.print(pitch);
  Serial.print(" Roll: ");
  Serial.print(roll);
  Serial.print('\n'); 
  Serial.print(" Computed Heading: ");
  Serial.print(qHead);
  Serial.print(" - Quat: ");
  Serial.print(qw);
  Serial.print(" ");
  Serial.print(qx);
  Serial.print(" ");
  Serial.print(qy);
  Serial.print(" ");
  Serial.println(qz);      
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void updateIMUData() {
  sensors_event_t gyro_event;
  sensors_event_t accel_event;
  sensors_event_t mag_event;
//
  // Get new data samples
  gyro.getEvent(&gyro_event);
  accelmag.getEvent(&accel_event, &mag_event);
//
  // Apply mag offset compensation (base values in uTesla)
  float x = mag_event.magnetic.x - mag_offsets[0];
  float y = mag_event.magnetic.y - mag_offsets[1];
  float z = mag_event.magnetic.z - mag_offsets[2];
//
  // Apply mag soft iron error compensation
  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];
//
  // Apply gyro zero-rate error compensation
  float gx = gyro_event.gyro.x + gyro_zero_offsets[0];
  float gy = gyro_event.gyro.y + gyro_zero_offsets[1];
  float gz = gyro_event.gyro.z + gyro_zero_offsets[2];
//
  // The filter library expects gyro data in degrees/s, but adafruit sensor
  // uses rad/s so we need to convert them first (or adapt the filter lib
  // where they are being converted)
  gx *= 57.2958F;
  gy *= 57.2958F;
  gz *= 57.2958F;
//
  // Update the filter
  filter.update(gx, gy, gz,
                accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
                mx, my, mz);
  comCount++;
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void initIMUSensors() {
  if (trace) {
    Serial.println(F("Adafruit AHRS Fusion Example")); Serial.println("");
    }
  if(!gyro.begin()) {                                                     // Initialize the sensors.
    if (trace) {
      Serial.println("Ooops, no gyro detected ... Check your wiring!");    /* There was a problem detecting the gyro ... check your connections */
      }
    while(1);
    }
  if(!accelmag.begin(ACCEL_RANGE_4G)) {
    if (trace) {
      Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
      }
    while(1);
    }
  // Filter expects 70 samples per second
  // Based on a Bluefruit M0 Feather ... rate should be adjuted for other MCUs
  filter.begin(20);
}
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void configIOPins() {
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
}
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void configSerial() {
  trace = false;
  delay(500);
  if (trace) {
    Serial.begin(115200);
    delay(500);
    Serial.println("Starting IMU");
    }
  digitalWrite(13, LOW);
  Serial1.begin(9600);
  delay(500);
}
//
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void setup() {
  configIOPins();
  configSerial();
  initIMUSensors();
  if (trace) {
    Serial.print(" Ready! ");
    delay(500);
    }}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void loop(void) {
  updateIMUData();
  if (comCount > 90) {
    computeEuler();
    computeQuat();
    digitalWrite(13, HIGH);
    if (trace) {
      showHeadingRollPitch();
      }
    sendHeadingRollPitch();
    comCount = 0;
    }
  delay(5);
  digitalWrite(13, LOW);
}
//
//
