#include <RPLidar.h>
#include <MPU6050_tockn.h>
#include <AF_DCMotor.h>
#include <Servo.h>
 
RPLidar lidar;
MPU6050 mpu6050(Wire);
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);
Servo servoH;
Servo servoV;
 
// Define pin number for LIDAR motor
const int lidarMotor = 13;
 
// Define pin numbers for the sonars
const int trig1 = 12;
const int trig2 = 11;
const int trig3 = 10;
const int trig4 = 9;
const int trig5 = 8;
const int echo1 = 7;
const int echo2 = 6;
const int echo3 = 5;
const int echo4 = 4;
const int echo5 = 3;
 
// Define pin numbers for the IR Rangefinders
const int IR_in1 = 22;
const int IR_in2 = 23;
const int IR_in3 = 24;
const int IR_in4 = 25;
const int IR_out1 = 26;
const int IR_out2 = 27;
const int IR_out3 = 28;
const int IR_out4 = 29;
 
// Define pin number for the solenoid
const int solenoid = 30;
 
// Define pin numbers for the limit switches
const int limit1 = 31;
const int limit2 = 32;
 
// Basic movements
void moveForward(int t){};
void moveBackward(int t){};
void turnCW(float angle){};
void turnCCW(float angle){};
void rotateTowards(float targetDir){};
 
// Reading sensors
void readLidar(float[] &lidarDistances, float[] &lidarAngles){};
void readSonar(float[5] &sonarDistances){};
void readCompass(float &roll, float &pitch, float &yaw){};
 
// Target Finding
bool findEnemyHomeBase(float[] &lidarDistances, float[] &lidarAngles, float &targetDir){};
bool findEnemyRobot(float[5] &sonarDistances, float &targetDir){};
 
// Navigation, when there is no target found
void navigateForward(float[] &lidarDistances, float[] &lidarAngles, float[5] &sonarDistances){};
 
// Proceeding, when there is an active target found
void proceedToward(float[] &lidarDistances, float[] &lidarAngles, float[5] &sonarDistances, float &targetDir){};
 
// Attacking
void launch(){};
 
//Object Slotting
void dock(){};
void alignCanon(){};
 
// State of robot. 0-5
// Subsumption architecture
// Higher number state corresponds to higher priority
// 0: No target found. Proceeds forward while avoidaing obstacles
// 1: Enemy Robot found. Move towards the target while avoidaing obstacles
// 2: Enemy Home Base found. Move towards the target while avoidaing obstacles
// 3: Attacking enemy robot
// 4: Object slotting
// 5: Return home base
byte robotState;
 
// Global variables for the threshold values
byte maxAng;
byte maxDist;
byte minAng;
byte minDist;
 
void setup()
{  
  // Begin serial communication of LIDAR
  lidar.begin(Serial);
 
  // Set pinmode of the LIDAR, and start rotating at maximum speed
  pinMode(lidarMotor, OUTPUT);
  digitalWrite(lidarMotor, 255);
 
  // Begin serial communication of compass
  Wire.begin();
  mpu6050.begin();
  
  // Set pinmode of the compass
  pinMode(20, OUTPUT);
  pinMode(21, OUTPUT);
  
  // Set pinmode of the sonars
  pinMode(trig1, OUTPUT);
  pinMode(trig2, OUTPUT);
  pinMode(trig3, OUTPUT);
  pinMode(trig4, OUTPUT);
  pinMode(echo1, INPUT);
  pinMode(echo2, INPUT);
  pinMode(echo3, INPUT);
  pinMode(echo4, INPUT);
 
  // Set pinmode of IR Rangefinders
  pinMode(IR_in1, INPUT);
  pinMode(IR_in2, INPUT);
  pinMode(IR_in3, INPUT);
  pinMode(IR_in4, INPUT);
  pinMode(IR_out1, INPUT);
  pinMode(IR_out2, INPUT);
  pinMode(IR_out3, INPUT);
  pinMode(IR_out4, INPUT);
 
  // Set pinmode of the solenoid
  pinMode(solenoid, OUTPUT);
  digitalWrite(solenoid, LOW);
 
  // Set robot state
  robotState = 0;
}
 
void loop()
{
  float targetDir; // Stores the direction of the target
  
  float[] lidarDistances; // Distance values for a single revolution of LIDAR
  float[] lidarAngles; // Angle values for a single revolution of LIDAR
 
  float[5] sonarDistances; // Distance values from the sonars
 
  float roll, pitch, yaw; // Stores the roll, pitch, yaw angles of the robot
 
  // If return home
  if (robotState == 5)
  {
    proceedToward(homeBase, lidarDistances, sonarDistances);
  }
 
  // If object slotting
  else if (robotState == 4)
  {
    dock();
    alighCanon();
    launch();
    // Set robot state to return home base
    robotState = 5;
  }
 
  // If attacking
  else if (robotState == 3)
  {
    launch();
    // Set robot state to return home base
    robotState = 5;
  }
 
  // If enemy home base found
  else if (robotState == 2)
  {
    while (all sonarDistances > minDist)
    {
      proceedToward(targetDir, lidarDistances, sonarDistances);
    }
    // Set robot state to object slotting
    robotState = 4;
  }
 
  // If enemy robot found
  else if (robotState == 1)
  {
    // Until both the angle and the distance becomes small enough
    while (targetDir >= maxAng && sonarDistances[2] >= maxDist)
    {
      // Rotate towards the enemy robot
      rotateTowards(255, targetDir);
      // Proceed towards the enemy robot for the time that is proportional to the distance
      moveForward(sonarDistances[2]*timeFactor);
 
      // Update the direction of enemy robot
      // If enemy robot not found this time, go back to state 0
      if (!findEnemyRobot(sonarDistances, targetDir))
      {
        robotState = 0;
        break;
      }
    }
    robotState = 3;
  }
  // If no target is found
  else if (robotState == 0)
  {
    // Read sensors
    readLidar(lidarDistances, lidarAngles);
    readSonar(sonarDistances);
    readCompass(roll, pitch, yaw);
 
    // If enemy robot found
    if (findEnemyRobot(sonarDistances, targetDir)
    {
      robotState = 1;
    }
 
    // If enemy home base found
    if (findEnemyHomeBase(lidarDistances, lidarAngles, targetDir)
    {
      robotState = 2;
    }
 
    // If nothing found
    else
    {
      navigateForward(lidarDistances, lidarAngles, sonarDistances);
    }
  }
}
 
void moveForward(int t)
{
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  motor1.setSpeed(255);
  motor2.setSpeed(255);
  motor3.setSpeed(255);
  motor4.setSpeed(255);
  delay(time);
}
void moveBackward(int t)
{
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  motor1.setSpeed(255);
  motor2.setSpeed(255);
  motor3.setSpeed(255);
  motor4.setSpeed(255);
  delay(time);
}
void turnCW(float angle)
{
  float currentAngle = mpu6050.getAngleZ();
  while ( abs(mpu6050.getAngleZ() - currentAngle) >= maxAng )
  {
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);
    motor1.setSpeed(angle*speedFactor);
    motor2.setSpeed(angle*speedFactor);
    motor3.setSpeed(angle*speedFactor);
    motor4.setSpeed(angle*speedFactor);
    delay(time);
  }
}
void turnCCW(float angle)
{
  float currentAngle = mpu6050.getAngleZ();
  while ( abs(mpu6050.getAngleZ() - currentAngle) >= maxAng )
  {
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
    motor1.setSpeed(angle*speedFactor);
    motor2.setSpeed(angle*speedFactor);
    motor3.setSpeed(angle*speedFactor);
    motor4.setSpeed(angle*speedFactor);
    delay(time);
  }
}
void rotateTowards(float targetDir)
{
  if (targetDir > 0)
  {
    turnCW(targetDir);
  }
  else
  {
    turnCCW(targetDir);
  }
}
void readLidar(float[] &distances, float[] &angles)
{
  for a single revolution:
    distances[i] = lidar.getCurrentPoint().distance; //distance value in mm unit
    angle[i] = lidar.getCurrentPoint().angle; //anglue value in degree
}
void readSonar(float[5] &sonarDistances)
{
  for (int i = 0; i < 5; i++)
  {
    sonarDistances[i] = digitalRead(12 - i);
  }
}
void readCompass(float &roll, float &pitch, float &yaw)
{
  mpu6050.update();
  roll = mpu6050.getAngleX();
  pitch = mpu6050.getAngleY();
  yaw = mpu6050.getAngleZ();
}
 
// Target Finding
bool findEnemyHomeBase(float[] &lidarDistances, float[] &lidarAngles, float &targetDir)
{
  float[][] lines = lineDetect(lidarDistances, lidarAngles);
  for (angles between (-getAngleZ() - 45°) and (-getAngleZ() + 45°))
  {
    if (any 2 lines have same slope && no abrupt change in distance is observed inbetween the lines)
    {
      targetDir = (midpoint of startpoint of one line and endpoint of another line);
      return true;
    }
  }
  return false;
}
float[] houghLineTransform(float x0, float y0)
{
  float[] r;
  /* find r as a function of theta */
  return r;
}
float[][] lineDetect(float[] x, float[] y)
{
  /* use houghTransform to detect lines*/
  return lines; // lines is a 2-D array in which (slope, intercept, startpoint, endpoint) tuples are stored
}
bool findEnemyRobot(float[5] &sonarDistances, float &targetDir)
{
  for (int i = 0; i < 5; i++)
  {
    // If any of the sonar reading is below the minDist
    if (sonarDistances[i] < minDist)
    {
      // Strop the motors
      motor1.run(RELEASE);
      motor2.run(RELEASE);
      motor3.run(RELEASE);
      motor4.run(RELEASE);
 
      int sum = 0;
      // Store the sonar distance values
      float[5] previousSonarDistances = sonarDistances;
      // Wait for 3 seconds 
      delay(3000);
      // Retrieve the sonar data again
      readSonar(sonarDistances);
      // Calculate the sum of the difference
      for (int j = 0; j < 5; j++)
      {
         sum += (sonarDistances[i] - previousSonarDistances[i]);
      }
      // If the difference is significant, there must be an object that is moving around
      if (sum > thres)
      {
        robotState = 1;
        return true;
      }
      else
      {
        return false;
      }
    }
  }
}
 
// Navigation, when there is no target found
void navigateForward(float[] &lidarDistances, float[] &lidarAngles, float[5] &sonarDistances)
{
  while( min(lidarDistances) > minDist && min(sonarDistances) > minDist )
  {
    moveForward();
  }
  // If minimum distance threshold is violated by the lidar
  if (min(lidarDistances) <= minDist)
  {
    minAngle = lidarAngles[indexOf(min(lidarDistances))];
  }
  // If minimum distance threshold is violated by the sonar
  if (min(sonarDistances) <= minDist)
  {
    minAngle = indexOf(min(sonarDistances));
  }
  
  // If minimum angle occurs on the left side of the robot
  if (minAngle < 0)
  {
    // Rotate until the minimum distance is directly on the left of the robot
    // Assumption: minAngle < 90, since the minimum angle is encoutnered while traveling forward
    rotateCW(90 - minAngle);
    moveForward(100);
    // Face front and begin navigating again
    rotateTowards(0);
  }
  // If minimum angle occurs on the right side of the robot
  else if (minAngle > 0)
  {
    // Rotate until the minimum distance is directly on the right of the robot
    rotateCCW(90 - minAngle);
    moveForward(100);
    // Face front and begin navigating again
    rotateTowards(0);
  }
}
 
// Proceed Towards is exacly the same as navitage forward, but the only difference is that 
// the robot tries to rotate towards the target direction after each detour
void proceedToward(float[] &lidarDistances, float[] &lidarAngles, float[5] &sonarDistances, float &targetDir)
{
  while( min(lidarDistances) > minDist && min(sonarDistances) > minDist )
  {
    moveForward();
  }
  // If minimum distance threshold is violated by the lidar
  if (min(lidarDistances) <= minDist)
  {
    minAngle = lidarAngles[indexOf(min(lidarDistances))];
  }
  // If minimum distance threshold is violated by the sonar
  if (min(sonarDistances) <= minDist)
  {
    minAngle = indexOf(min(sonarDistances));
  }
  
  // If minimum angle occurs on the left side of the robot
  if (minAngle < 0)
  {
    // Rotate until the minimum distance is directly on the left of the robot
    // Assumption: minAngle < 90, since the minimum angle is encoutnered while traveling forward
    rotateCW(90 - minAngle);
    moveForward(100);
    // Rotate towards the target direction and begin navigating again
    rotateTowards(targetDir);
  }
  // If minimum angle occurs on the right side of the robot
  else if (minAngle > 0)
  {
    // Rotate until the minimum distance is directly on the right of the robot
    rotateCCW(90 - minAngle);
    moveForward(100);
    // Rotate towards the target direction and begin navigating again
    rotateTowards(targetDir);
  }
  // Update the direction of enemy home base
  // If enemy home base not found this time, go back to state 0
  if (!findEnemyHomeBase(targetDir))
  {
    robotState = 0;
    break;
  }
}
 
void launch()
{
  digitalWrite(solenoid, HIGH);
}
 
void dock()
{
  // Keep accelerate forward until the robot is in full contact with the enemy home base
  while( digitalRead(limit switch) == HIGH ) // HIGH means not in contact
  {
    moveForward(100); // Proceed gradually until both limit switches are pressed
  }
  // Strop the motors
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}
 
void alignCanon()
{
  initPos(); // Initializes the position of the canon to the bottom left
  while(all digitalRead(inner IR sensor)==LOW)
  {
    swipe();  // Swipe the bottom row -> elevate 30 -> swipe the middle row -> elevate 30 -> swipe the top row
              // If any IR sensor reads HIGH, break
  }
  while ( any digitalRead(inner IR sensor) == HIGH and digitalRead(corresponding outer IR sensor) == HIGH )
  {
    moveServo(inner IR sensor); // Move the servo towards the direction where both inner and outer IR sensor readings are HIGH
  }
}
