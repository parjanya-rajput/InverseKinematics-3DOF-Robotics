#include <math.h>
#include <Braccio.h>
#include <Servo.h>

float L1 = 190;
float L2 = 125;
float L3 = 125; 
float theta_A = 0;
float theta_B = 0;
float theta_C = 0;

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;

float deg2rad(float deg) {
    return deg * (M_PI / 180.0);
}

float rad2deg(float rad) {
    return rad * (180.0 / M_PI);
}

void inverseKinematics(float X, float Y, float theta) {
    float la, theta1, theta2, lc, ld, theta3, x, y, l2, theta4, theta5, theta6, theta7, theta8;
    la = sqrt(pow(X, 2) + pow(Y, 2));
    theta1 = atan2(X, Y);
    theta2 = atan2(Y, X);

    lc = L1 * cos(theta);
    ld = L1 * sin(theta);

    theta3 = M_PI / 2 - theta;

    x = X - lc;
    y = Y - ld;

    l2 = sqrt(pow(x, 2) + pow(y, 2));

    theta4 = atan2(y, x);
    theta5 = atan2(x, y);

    float costheta6 = (pow(lc, 2) + pow(l2, 2) - pow(L3, 2)) / (2 * lc * l2);
    float sintheta6 = sqrt(1 - pow(costheta6, 2));

    theta6 = atan2(sintheta6, costheta6);

    float costheta7 = (pow(L3, 2) + pow(lc, 2) - pow(l2, 2)) / (2 * L3 * lc);
    float sintheta7 = sqrt(1 - pow(costheta7, 2));

    theta7 = atan2(sintheta7, costheta7);

    float costheta8 = (pow(l2, 2) + pow(L3, 2) - pow(lc, 2)) / (2 * l2 * L3);
    float sintheta8 = sqrt(1 - pow(costheta8, 2));

    theta8 = atan2(sintheta8, costheta8);

    theta_A = M_PI - theta4 + theta7;
    theta_B = theta8 - (M_PI / 2);
    theta_C = theta6 - theta5 + theta3 - M_PI / 2;

    Serial.print("Theta-1: ");
    Serial.println(rad2deg(theta_A));
    Serial.print("Theta-2: ");
    Serial.println(rad2deg(theta_B));
    Serial.print("Theta-3: ");
    Serial.println(rad2deg(theta_C));
}

void setup() {
    // Initialize Serial communication
    Serial.begin(9600);
    //Initialize Braccio Robot
    Braccio.begin();
}

void loop() {

    while (Serial.available() == 0) {
    }

    float X_input = Serial.parseInt();
    float Y_input =  Serial.parseInt();
    float theta_input = Serial.parseInt(); 
    

    theta_input = deg2rad(theta_input);

    inverseKinematics(X_input, Y_input, theta_input);

    delay(1000);

    Braccio.ServoMovement(20, 0, theta_A, theta_B, theta_C, 0,  73);

    delay(1000);
}
