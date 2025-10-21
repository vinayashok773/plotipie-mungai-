#include <Arduino.h>

#include <Servo.h>
#include <math.h>

// =================================================================================
// -- 1. CONFIGURATION: UPDATE PARAMETERS HERE --
// =================================================================================

// -- ROBOT PHYSICAL PARAMETERS (MEASURE AND UPDATE THESE) --
const float LINK_LENGTH_1 = 10.0; // Length of the first arm link (base to elbow) in cm
const float LINK_LENGTH_2 = 10.0; // Length of the second arm link (elbow to pen) in cm

// -- SERVO PINS (UPDATE THESE) --
const int SERVO1_PIN = 8;      // Base servo (controls theta1)
const int SERVO2_PIN = 7;      // Elbow servo (controls theta2)
const int PEN_SERVO_PIN = 6;   // Pen lift servo

// -- PEN SERVO ANGLES (UPDATE THESE) --
const int PEN_UP_ANGLE = 0;   // Angle when the pen is lifted off the paper
const int PEN_DOWN_ANGLE = 90;  // Angle when the pen is touching the paper

// -- CALIBRATION (CRITICAL: UPDATE THESE FOR ACCURACY) --
const int SERVO1_ZERO_OFFSET = 90; // The servo value for theta1 = 0 degrees
const int SERVO2_ZERO_OFFSET = 0; // The servo value for theta2 = 0 degrees

// -- MOVEMENT PARAMETERS --
// This delay controls the speed of the arm. Higher value = slower and smoother movement.
const int MOVEMENT_DELAY = 55; // Delay in milliseconds between steps.

// --- PICTURE DATA ---
float array[][4] = {
       {4.814, -9.206, 4.189, -9.406},
    {4.527, -8.955, 4.289, -7.167},
    {4.939, -8.248, 4.852, -9.156},
    {5.241, -7.117, 5.416, -8.905},
    {4.214, -9.281, 4.764, -9.156},
    {9.263, -4.555, 4.914, -4.625},
    {3.635, -5.584, 3.710, -6.586},
    {10.202, -9.080, 9.738, -8.198},
    {3.585, -3.102, 4.627, -3.803},
    {3.610, -6.536, 3.585, -5.680},
    {10.431, -8.123, 10.481, -9.647},
    {9.663, -4.907, 11.816, -4.625},
    {12.091, -4.200, 12.317, -3.577},
    {2.558, -3.352, 3.410, -3.127},
    {3.385, -3.152, 3.735, -3.252},
    {4.364, -3.828, 4.264, -3.602},
    {9.588, -8.198, 9.331, -9.080},
    {11.551, -4.479, 9.538, -4.706},
    {9.688, -7.468, 9.814, -5.333},
    {5.064, -8.609, 4.964, -8.273},
    {4.086, -3.352, 2.971, -3.202},
    {5.203, -7.070, 7.433, -6.114},
    {10.202, -8.955, 10.202, -9.105},
    {4.214, -9.055, 4.514, -8.980},
    {10.406, -9.822, 9.989, -9.872},
    {7.534, -6.141, 5.241, -7.117},
    {11.501, -4.429, 12.317, -3.327},
    {3.710, -4.731, 4.389, -3.853},
    {8.404, -8.804, 8.730, -8.754},
    {2.127, -4.706, 2.971, -4.756},
    {4.877, -8.980, 5.064, -8.609},
    {4.264, -9.105, 4.161, -9.206},
    {4.514, -3.753, 4.789, -4.374},
    {4.764, -4.580, 4.464, -3.728},
    {9.663, -9.482, 10.089, -9.105},
    {1.827, -4.200, 1.932, -4.350},
    {4.577, -9.005, 4.289, -9.080},
    {4.464, -3.853, 3.835, -4.806},
    {7.934, -6.340, 8.830, -7.943},
    {8.654, -7.843, 7.909, -6.415},
    {5.391, -8.905, 5.291, -9.080},
    {10.356, -8.223, 9.713, -7.315},
    {7.483, -6.114, 7.984, -6.281},
    {10.227, -9.105, 9.763, -9.482},
    {7.909, -6.141, 7.584, -6.206},
    {2.513, -3.778, 2.483, -3.552},
    {10.039, -9.872, 9.713, -9.722},
    {1.777, -4.128, 1.907, -4.324},
    {3.560, -3.978, 3.685, -4.731},
    {5.266, -9.030, 5.303, -8.905},
    {1.777, -4.128, 2.483, -3.853},
    {9.788, -5.408, 9.563, -5.032},
    {9.814, -7.315, 10.431, -8.198},
    {8.855, -7.818, 8.880, -8.779},
    {9.613, -5.032, 9.964, -5.509},
    {2.152, -4.781, 1.752, -4.274},
    {3.024, -4.731, 3.610, -5.680},
    {8.429, -9.080, 9.076, -9.055},
    {4.214, -7.167, 3.610, -6.561},
    {3.911, -3.602, 3.560, -4.103},
    {2.558, -3.602, 2.558, -3.853},
    {3.435, -4.003, 3.936, -3.477},
    {2.951, -4.806, 2.152, -4.781},
    {12.267, -3.552, 11.551, -4.454},
    {3.685, -6.561, 4.364, -7.167},
    {5.391, -9.080, 5.266, -9.005},
    {12.242, -4.128, 11.841, -4.580},
    {3.560, -5.705, 2.951, -4.781},
    {4.086, -9.181, 4.189, -9.030},
    {3.109, -3.227, 2.583, -3.527},
    {11.741, -4.555, 12.067, -4.174},
    {9.538, -5.032, 9.688, -4.882},
    {8.254, -8.930, 8.404, -8.779},
    {4.939, -4.504, 4.764, -4.274},
    {8.429, -9.156, 8.604, -8.930},
    {14.140, -0.025, 14.090, -0.025}
};

//const int numLines = sizeof(pictureLines) / sizeof(pictureLsines[0]);


// =================================================================================
// -- 2. GLOBAL VARIABLES AND SETUP --
// =================================================================================

// Create servo objects
Servo servo1;
Servo servo2;
Servo penServo;

// Struct to hold the calculated angles from the inverse kinematics function.
struct Angles {
  float theta1;
  float theta2;
};

// Global variables to track the current state of the arm for smooth movement
// Initialized to the 'home' position angles.
float c1 = 90;
float c2 = 90;


void penUp() {
  penServo.write(PEN_UP_ANGLE);
  delay(300); // Give the servo time to move
}

void penDown() {
  penServo.write(PEN_DOWN_ANGLE);
  delay(300); // Give the servo time to move
}


//go through readme for this formulation.
Angles calculateIK(float x, float y, float l) {
  Angles result = {0.0, 0.0}; // Initialize with default failure state

  float distance = sqrt(x * x + y * y);

  if (distance > 2 * l) {
    Serial.println("Error: Target coordinates are out of reach.");
    return result;
  }
  if (distance == 0) {
    Serial.println("Error: Target cannot be the origin (0,0).");
    return result;
  }

  // --- Step 1: Calculate theta2 (This remains the same) ---
  float theta2_rad = 2 * acos(distance / (2 * l));

  // --- Step 2: Calculate theta1 (CORRECTED) ---
  // Use atan2(x, y) to measure the angle from the Y-axis, matching the original problem.
  float angle_to_target = atan2(x, y);

  float theta1_rad = angle_to_target - (theta2_rad / 2.0);

  // --- Step 3: Convert radians to degrees ---
  result.theta1 = theta1_rad * 180.0 / PI;
  result.theta2 = theta2_rad * 180.0 / PI;
  

  // Ensure theta1 is always positive
  if (result.theta1 < 0) {
    theta1_rad = angle_to_target + (theta2_rad / 2.0);
    result.theta1 = theta1_rad * 180.0 / PI;
  }
  result.theta1+=0;
  result.theta2+=7;
  return result;
}
int steps;
/*even though drawLine function looks fine( IT IS NOT USED HERE) it was generated USING gpt 
to check but i didnt bother to check why its not working, so write the linedraw with simple same idea*/
void drawLine(float x_start, float y_start, float x_end, float y_end, int steps) {
  Serial.println("Drawing line...");
  Angles final=calculateIK(x_end,y_end,10);
  penDown();
  for (int i = 0; i <= steps; i++) {
    // Calculate the intermediate point for the current step (linear interpolation)
    float t = (float)i / steps;
    float ix = x_start + t * (x_end - x_start);
    float iy = y_start + t * (y_end - y_start);

    // Calculate the required angles for this intermediate point
    Angles targetAngles = calculateIK(ix, iy, 10);

      // Command the servos to move to the new angles
    servo1.write(targetAngles.theta1);
    servo2.write(targetAngles.theta2);

      // A small delay to control the drawing speed and allow servos to move
    delay(2000);
    
  }
  Serial.println("Line drawing complete.");
  c1=final.theta1;
  c2=final.theta2;
}
//This moves the motor one i.e the one stationary 
void moveservo1(int a1){
       if(a1>c1){
        for(int i=c1;i<=a1;i++){
          servo1.write(180-i);
          delay(100);
        }
       }
       else{
        for(int i=c1;i>=a1;i--){
          servo1.write(180-i);
          delay(100);
        }
       }

       c1=a1;
 }
//this moves the motor 2 the elobow motor which is dynamic
void moveservo2(int a2){
       if(a2>c2){
        for(int i=c2;i<=a2;i++){
          servo2.write(i);
          delay(100);
        }
       }
       else{
        for(int i=c2;i>=a2;i--){
          servo2.write(i);
          delay(100);
        }
       }

       c2=a2;
 }


void linedraw(int x1,int y1,int x2,int y2,int steps){
  
  Angles initial=calculateIK(x1,y1,10);
  moveservo1(initial.theta1);
  moveservo2(initial.theta2);
  penDown();

  for(int i=0;i<steps;i++){
    float deltax=(x2-x1)/(float)steps;
    float deltay=(y2-y1)/(float)steps;

    Angles inter=calculateIK(x1+(i*(deltax)),y1+(i*deltay),10);
    moveservo1(inter.theta1);
    moveservo2(inter.theta2);

    delay(100);



  }
  penUp();


}
void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Robotic Arm Plotter Initializing...");

  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  penServo.attach(PEN_SERVO_PIN);

  Serial.println("Moving to home position.");
  penUp();

  // THE BELOW LINES ARE COMMENTED THEY WERE TO TROUBLESHOOT MOTORS AND RESET POSITION SOMETIMES I HAVE NOT REMOVED THEM HOWEVER

  // The global angle variables are already set to this home position (0, 90),
  // so this first call will confirm the physical position without jerky movement.
  //delay(1000);
 //Angles angles=calculateIK(15,0,10);
 // moveservo1(angles.theta1-10);
  //moveservo2(angles.theta2);
  //delay(2000);
  //linedraw(10,-10,5,-5,100);
  //linedraw(14,6,14,2,100);
  //linedraw(14,2,10,0.5,100);
  //linedraw(6,6,6,2,100);
 // linedraw(6,2,10,0.5,100);
  //linedraw(0,-5,,-10,150);

for(int i=0;i<120;i++){
  linedraw(array[i][0],array[i][1],array[i][2],array[i][3],70);
  delay(100);
}
 //penServo.write(90); 
 penUp();

 //servo1.write(90);
//servo2.write(10);


}

void loop() {
  // The main loop is empty.
}