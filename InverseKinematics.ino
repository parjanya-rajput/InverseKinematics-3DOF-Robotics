#include <math.h>
#include <Braccio.h>
#include <Servo.h>

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;



double l1 = 7.15; 
double l2 = 12.5; 
double l3 = 12.5;
double theta_1 = 0;
double theta_2 = 0;
double theta_3 = 0;

double deg2rad(double degrees) {
    return degrees * PI / 180.0; 
}

double rad2deg(double radians) {
    return radians * 180.0 / PI;
}

void inversekinematics(double x3, double y3, double phi) {
    double x2 = x3 - l3 * cos(phi);
    double y2 = y3 - l3 * sin(phi);

    double delta = x2 * x2 + y2 * y2;

    double costheta2 = (delta - l1 * l1 - l2 * l2) / (2 * l1 * l2);
    // Serial.println(costheta2);

    if (costheta2 * costheta2 > 1) {
        Serial.println("Cannot reach the defined point");
    } else {
        double sintheta2 = sqrt(1 - costheta2 * costheta2);

        theta_2 = atan2(sintheta2, costheta2);

        double sintheta1 = ((l1 + l2 * costheta2) * y2 - l2 * sintheta2 * x2) / delta;
        double costheta1 = ((l1 + l2 * costheta2) * x2 + l2 * sintheta2 * y2) / delta;

        theta_1 = atan2(sintheta1, costheta1);

        theta_3 = phi - theta_1 - theta_2;

        Serial.print("Theta-1: ");
        Serial.println(rad2deg(theta_1));
        Serial.print("Theta-2: ");
        Serial.println(rad2deg(theta_2));
        Serial.print("Theta-3: ");
        Serial.println(rad2deg(theta_3));
    }
}

void setup() {
    Serial.begin(9600); 

    Braccio.begin();
}

void loop() {

    double x3 = 0; 
    double y3 = 32.149; 
    double phi = 90; 
    double phirad = deg2rad(phi);
    inversekinematics(x3, y3, phirad);

    Braccio.ServoMovement(20, 0, abs(180-theta_1), abs(180-theta_2), abs(180-theta_3), 0,  73);

    delay(5000); 
}
