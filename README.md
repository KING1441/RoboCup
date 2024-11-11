# RoboCup
Robot Code
/*----------------Code By IM_KING_2021------------------------
Sp= Setpoint: The target value or goal you want to achieve.
PV= Process Variable: The current state or measurement of the system.
Proportional gain: controls how aggressively the system reacts to the error.
[Larger Kp = faster correction but may cause oscillations.]
[Smaller Kp = slower correction but more stable, though it may take longer to reach the target.]
---------------------------------------------------------------
*/
#include <Wire.h>
#include <MPU6050.h>

int sp = 30; 
int PV = 5; 
int KP = 1; //
int Defalt_Speed = 50;
int Error = PV-sp*KP;
int R_Speed = Defalt_Speed-Error;
int L_Speed = Defalt_Speed+Error;
int test = 30;

MPU6050 mpu;

void setup() {
  // Start serial communication
  Serial.begin(115200);
  GYRO_Setup();
}

void loop(){
  //Steer();
  //test=-;
  Test_Gyro();
}

void Steer(){
  //set PV to gyro angle on port A
  if(L_Speed>24){
    Serial.println("Yes");
  }
  else{
    Serial.println("No");
  }
}

void Test_Gyro(){
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  // Read raw accelerometer and gyroscope values
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Print the raw data to the Serial Monitor
  Serial.print("Accelerometer: ");
  Serial.print("X: "); Serial.print(ax);
  Serial.print(" Y: "); Serial.print(ay);
  Serial.print(" Z: "); Serial.println(az);

  Serial.print("Gyroscope: ");
  Serial.print("X: "); Serial.print(gx);
  Serial.print(" Y: "); Serial.print(gy);
  Serial.print(" Z: "); Serial.println(gz);
  
  // Delay to slow down the output
  delay(500);
}


void GYRO_Setup(){
  // Initialize I2C communication
  Wire.begin();
  // Initialize the MPU6050 sensor
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  // Check if the sensor is connected
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
    while (1);
  }
}
