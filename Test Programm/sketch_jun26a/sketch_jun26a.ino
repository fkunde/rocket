#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Servo.h"

MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
Servo servo1;
Servo servo2;

int val1;
int val2;
int prevVal1;
int prevVal2;

void setup()
{
Wire.begin();
Serial.begin(38400);
Serial.println("Initialize MPU");
mpu.initialize();
Serial.println(mpu.testConnection() ? "Connected" : "Connection failed");
servo1.attach(9);
servo2.attach(10);
delay(1000);
servo1.write(90);
delay(1000);
servo1.write(0);
delay(3000);
servo1.write(90);
delay(1000);
servo1.write(179);
delay(1000);
servo1.write(90);
delay(1000);
}

void loop()
{
mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
val1 = map(ay, -17000, 17000, 30, 150);
if (val1 != prevVal1)
{
servo1.write(val1);
prevVal1 = val1;
}
Serial.println(ay);
delay(100);
}
