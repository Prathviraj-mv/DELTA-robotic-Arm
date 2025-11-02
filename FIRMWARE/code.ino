#include <Servo.h>
#include <math.h>


#define e  35.0    // end effector equilateral triangle side
#define f  80.0    // base equilateral triangle side
#define re 150.0   // lower arm length
#define rf 90.0    // upper arm length


#define SERVO_1 3
#define SERVO_2 5
#define SERVO_3 6

Servo servo1, servo2, servo3;

const float sqrt3 = sqrt(3.0);
const float pi = 3.141592653;
const float sin120 = sqrt3/2.0;
const float cos120 = -0.5;
const float tan60 = sqrt3;
const float sin30 = 0.5;
const float tan30 = 1/sqrt3;

// ------------------------------------------------------

int delta_calcAngleYZ(float x0, float y0, float z0, float &theta);
int delta_calcInverse(float x0, float y0, float z0, float &theta1, float &theta2, float &theta3);
// ------------------------------------------------------

void setup() {
  Serial.begin(9600);
  servo1.attach(SERVO_1);
  servo2.attach(SERVO_2);
  servo3.attach(SERVO_3);


  servo1.write(90);
  servo2.write(90);
  servo3.write(90);

  Serial.println("Delta Robot Ready. Input X Y Z in mm");
}

void loop() {
  if (Serial.available()) {
    float x = Serial.parseFloat();
    float y = Serial.parseFloat();
    float z = Serial.parseFloat();

    if (Serial.read() == '\n') {
      float t1, t2, t3;
      int status = delta_calcInverse(x, y, z, t1, t2, t3);
      if (status == 0) {
        servo1.write(90 - t1);
        servo2.write(90 - t2);
        servo3.write(90 - t3);

        Serial.print("Angles: ");
        Serial.print(t1); Serial.print(", ");
        Serial.print(t2); Serial.print(", ");
        Serial.println(t3);
      } else {
        Serial.println("Position out of reach!");
      }
    }
  }
}


int delta_calcAngleYZ(float x0, float y0, float z0, float &theta) {
  float y1 = -0.5 * 0.57735 * f; 
  y0 -= 0.5 * 0.57735 * e;      


  float a = (x0*x0 + y0*y0 + z0*z0 + rf*rf - re*re - y1*y1)/(2*z0);
  float b = (y1 - y0)/z0;


  float d = -(a + b*y1)*(a + b*y1) + rf*(b*b*rf + rf);
  if (d < 0) return -1;

  float yj = (y1 - a*b - sqrt(d))/(b*b + 1);
  float zj = a + b*yj;
  theta = atan(-zj/(y1 - yj)) * 180.0 / pi + ((yj > y1) ? 180.0 : 0.0);
  return 0;
}

int delta_calcInverse(float x0, float y0, float z0, float &theta1, float &theta2, float &theta3) {
  int status = delta_calcAngleYZ(x0, y0, z0, theta1);
  if (status == 0) status = delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120 - x0*sin120, z0, theta2);
  if (status == 0) status = delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120 + x0*sin120, z0, theta3);
  return status;
}
