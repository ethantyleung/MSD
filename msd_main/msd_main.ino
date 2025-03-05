#include <math.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Ultrasonic sensor pins
const int trigPin = 9;
const int echoPin = 10;

// Rotary encoder settings
#define CLICKS_PER_REVOLUTION 700
#define PI 3.1415926
#define ARM_LENGTH 1.3 // cm

const int encoderA = 2;
const int encoderB = 3; 
volatile int encoderPos = 0;
int prevEncoderPos = 0;

// Shared time variable
unsigned long currentTime;
double armHeight = 0;
double initDist = 0;

// Initialize Accelerometer Sensor Object
Adafruit_MPU6050 mpu;
Adafruit_Sensor *accelerometer = mpu.getAccelerometerSensor();

void setup() {
  // Setup for accelerometer
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G); // set accelerometer range to +- 4G

  // Setup for ultrasonic sensor
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);  // Sets the echoPin as an Input

  // Setup for rotary encoder
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  digitalWrite(encoderA, HIGH);
  digitalWrite(encoderB, HIGH);
  attachInterrupt(digitalPinToInterrupt(encoderA), updateEncoder, RISING);

  initDist = measureDistance();
  // Serial communication
  Serial.begin(9600);
}

void loop() {
  // Update time
  currentTime = micros();

  // Measure distance using the ultrasonic sensor
  double distance = measureDistance() - initDist;

  // Calculate arm height and update position using the rotary encoder
  calculateArmHeight();

  // Get accelerometer data from the mpu6050 using getEvent call
  sensors_event_t accel_data;
  accelerometer->getEvent(&accel_data);

  // Print results in CSV format: time (ms), distance (cm), arm height, z acceleration (m/s^2)
  Serial.print(currentTime / 1e6); 
  Serial.print(" ");
  Serial.print(distance);
  Serial.print(" ");
  Serial.print(armHeight);
  Serial.print(" ");
  Serial.println(accel_data.acceleration.z);
}

double measureDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echoPin, HIGH, 30000);

  double distance = duration * 0.0344 / 2;

  // Calculating the distance (cm)
  return distance;
}

void calculateArmHeight() {
  noInterrupts();
  int currentPos = encoderPos;
  interrupts();

  if (currentPos != prevEncoderPos) {
    double angle = 2 * PI * currentPos / (double)CLICKS_PER_REVOLUTION; //in radians
    armHeight = ARM_LENGTH * sin(angle);
    prevEncoderPos = currentPos;   
  }
}

void updateEncoder() {
  if (digitalRead(encoderB) == LOW) {
    encoderPos++;
  } else {
    encoderPos--;
  }
}

//ajsdfkl