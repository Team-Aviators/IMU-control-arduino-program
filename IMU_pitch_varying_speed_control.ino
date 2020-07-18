#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Timers
unsigned long timer = 0;
float timeStep = 0.01;
int drive1 = 8;
int speed1 = 9;
int drive2 = 2;
int speed2 = 3;
int potPin = A4;
float oValue = 0;
float value = 255;
// Pitch, Roll and Yaw values
float pitch = 0;
float ratio1 = 1;
float ratio2 = 1;
//float roll = 0;
//float yaw = 0;

void setup()
{
  Serial.begin(115200);
 
  // Initialize MPU6050
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  // mpu.setThreshold(3);
}

void loop()
{
  timer = millis();

  // Read normalized values
  Vector norm = mpu.readNormalizeGyro();

  // Calculate Pitch, Roll and Yaw
  pitch = pitch + norm.YAxis * timeStep;
  //  roll = roll + norm.XAxis * timeStep;
  //  yaw = yaw + norm.ZAxis * timeStep;

  // Output raw
  Serial.print(" Pitch = ");
  Serial.println(pitch);
  
  //  Serial.print(" Roll = ");
  //  Serial.print(roll);

  //  Serial.print(" Yaw = ");
  //  Serial.println(yaw);
  
//  oValue = analogRead(potPin);
//  value = map(oValue,0,1023,0,255);
  Serial.println(value);
  // Wait to full timeStep period
  delay(100);

  if (pitch >= -1 && pitch <= 1)
  {
    digitalWrite(drive1, HIGH);
    digitalWrite(drive2, LOW);
    analogWrite(speed1, value *0.5);
    analogWrite(speed2, value *0.5);
  }
  else if (pitch < -1)
  {
    
    ratio2 = map(pitch, -1.1, -10, 50, 100);
    ratio1 = 100 - ratio2;
    digitalWrite(drive1, HIGH);
    digitalWrite(drive2, LOW);
    analogWrite(speed1, value *(ratio1/100) );
    Serial.println(value *(ratio1/100));
    analogWrite(speed2, value *(ratio2/100) );
    Serial.println(value *(ratio2/100));
  }
  else
  {
    ratio1 = map(pitch, 1.1, 10, 50, 100);
    ratio2 = 100 - ratio1;
    digitalWrite(drive1, HIGH);
    digitalWrite(drive2, LOW);
    analogWrite(speed1, value*(ratio1/100));
    analogWrite(speed2, value*(ratio2/100));
  }
}
