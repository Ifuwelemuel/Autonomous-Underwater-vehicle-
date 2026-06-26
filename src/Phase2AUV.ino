/*
  ROV/Sub "Dynamic Diving" Controller (Friendly Angle Version)
  
  FIXES:
  - Removed duplicate variable declarations in readSensors().
  - Cleaned up I2C reading logic.
*/

#include <Servo.h>
#include <Wire.h>
#include <MPU6050_light.h>

// ===================================================================================
//                                 USER CONFIGURATION
// ===================================================================================

// --- 1. PINS ---
const int PIN_THRUSTER      = 3;
const int PIN_SERVO_PITCH   = 9; //Horisontal servo 
const int PIN_SERVO_YAW     = 10; //vertical servo 
const int PIN_PUMP_FILL     = 5;   
const int PIN_PUMP_EMPTY    = 6;   
const int PIN_LEAK_SENSOR   = 7;   
const int PIN_DEPTH_SENSOR  = A0;

// --- 2. SERVO SETTINGS (DEGREES) ---
// 90 is usually center for most servos
const int SERVO_CENTER_DEG  = 90;  
const int SERVO_LIMIT_DEG   = 45;  // Max movement (+/- 45 degrees from center)


// --- 3. PHYSICS & TUNING ---
const float MAX_SAFE_DEPTH_M = 2.5; 
const float DEPTH_P_GAIN     = 20.0; // Angle correction per meter of error
const int   MAX_AUTO_PITCH   = 30;   // Max auto-angle (degrees)

const unsigned long PUMP_RUN_TIME_MS = 6000; 

// ===================================================================================
//                                   INTERNAL VARIABLES
// ===================================================================================

Servo servoPitch;
Servo servoYaw;
Servo servoThruster; 



struct Command {            
  float targetPitch = 0.0;      
  float targetYaw = 0;        
  int manualThrottle = 0;   
  int ballastRequest = 0;   
};
Command cmd;


MPU6050 mpu(Wire);

// 4. Global variables for IMU yaw tracking (Fixes lastMs, gyroZ_bias, yaw errors)

// Conversion factor
unsigned long lastMpuPrint = 0;





// ===================================================================================
//                                     SETUP
// ===================================================================================
void setup() {
  Serial.begin(115200);
  //Pumps 
  pinMode(PIN_PUMP_FILL, OUTPUT);
  pinMode(PIN_PUMP_EMPTY, OUTPUT);
  pinMode(PIN_LEAK_SENSOR, INPUT_PULLUP);
  
  servoPitch.attach(PIN_SERVO_PITCH);
  servoYaw.attach(PIN_SERVO_YAW);
  servoThruster.attach(PIN_THRUSTER);
  
  // Initialize to 90 degrees (Center)
  servoPitch.write(SERVO_CENTER_DEG);
  servoYaw.write(SERVO_CENTER_DEG);
  servoThruster.writeMicroseconds(1500); //tell it to stop  
  
 // mpu
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  
  // STOP! DO NOT MOVE THE SENSOR DURING THIS PART
  Serial.println(F("Calibrating... Keep sensor flat and still!"));
  delay(2000);     // let it warm up
  mpu.calcOffsets(); 
  Serial.println(F("Done!\n"));
  Serial.println(F("C<Pitch>,<Yaw>,<Throttle>,<Ballast>\n"));
}

// ===================================================================================
//                                   MAIN LOOP
// ===================================================================================
void loop() {
  unsigned long now = millis();

  readSensors();
  readSerialCommand();
  servoPitch.write(cmd.targetPitch); 
  servoYaw.write(cmd.targetYaw);
  servoThruster.writeMicroseconds(cmd.manualThrottle); 
  pumpstate(cmd.ballastRequest);
}

// ===================================================================================
//                                CONTROL FUNCTIONS
// ===================================================================================


void pumpstate(bool start){
  if (start == 1){
    startpumps();
  }
  else 
  {
    stopPumps();
  }
}

void stopPumps() {
  digitalWrite(PIN_PUMP_FILL, LOW);
  digitalWrite(PIN_PUMP_EMPTY, LOW);
}
void startpumps() {
  digitalWrite(PIN_PUMP_FILL, HIGH);
  digitalWrite(PIN_PUMP_EMPTY, LOW);
}
// ===================================================================================
//                                 HELPER FUNCTIONS
// ===================================================================================


void readSensors() {
  // --- Depth -----------------------------
  //int adc = analogRead(PIN_DEPTH_SENSOR);
 // boat.currentDepth = (adc - 50) * 0.01; 
  
  // --- IMU --------------------------------
  mpu.update();
  unsigned long now = millis();
  if (now - lastMpuPrint >= 50) { // 20Hz is better for control than 10Hz
    // Format: accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z
    // Units: m/s^2 and rad/s
    Serial.print(mpu.getAccX() * 9.806); Serial.print(",");
    Serial.print(mpu.getAccY() * 9.806); Serial.print(",");
    Serial.print(mpu.getAccZ() * 9.806); Serial.print(",");
    Serial.print(mpu.getGyroX() * DEG_TO_RAD); Serial.print(",");
    Serial.print(mpu.getGyroY() * DEG_TO_RAD); Serial.print(",");
    Serial.println(mpu.getGyroZ() * DEG_TO_RAD); 
    
    lastMpuPrint = now;
  }
}


// 1. Add this function to print the received data cleanly
void printDebugData() {
  Serial.println("--- COMMAND RECEIVED ---");
  Serial.print(" | Pitch: ");  Serial.print(cmd.targetPitch);
  Serial.print(" | Yaw: ");    Serial.print(cmd.targetYaw);
  Serial.print(" | Thruster: "); Serial.print(cmd.manualThrottle);
  Serial.print(" | Ballast: ");  Serial.println(cmd.ballastRequest);
}

void readSerialCommand() {
  if (Serial.available() <= 0) return;

  // Read one full line (example: "C 90,90,1500,1")
  String line = Serial.readStringUntil('\n');
  line.trim();  // removes \r, spaces, etc.

  if (line.length() == 0) return;

  // Must start with 'C'
  //if (line.charAt(0) != 'C') {
  //  Serial.println("Invalid header. Expected: C <Pitch>,<Yaw>,<Throttle>,<Ballast>");
   // return;
 // }

  // Remove the leading 'C'
  line.remove(0, 1);
  line.trim();  // remove the space after C

  // Find comma positions
  int c1 = line.indexOf(',');
  int c2 = line.indexOf(',', c1 + 1);
  int c3 = line.indexOf(',', c2 + 1);

  // Validate comma positions
  if (c1 < 0 || c2 < 0 || c3 < 0) {
    Serial.println("Invalid format. Use: C <Pitch>,<Yaw>,<Throttle>,<Ballast>");
    return;
  }

  // Extract values
  String pitchStr    = line.substring(0, c1);
  String yawStr      = line.substring(c1 + 1, c2);
  String throttleStr = line.substring(c2 + 1, c3);
  String ballastStr  = line.substring(c3 + 1);

  // Convert and store
  cmd.targetPitch    = pitchStr.toFloat();
  cmd.targetYaw      = yawStr.toFloat();
  cmd.manualThrottle = throttleStr.toInt();
  cmd.ballastRequest = ballastStr.toInt();

  // Optional safety limits (recommended)
  //cmd.targetPitch = constrain(cmd.targetPitch,
   //                           SERVO_CENTER_DEG - SERVO_LIMIT_DEG,
      ///                        SERVO_CENTER_DEG + SERVO_LIMIT_DEG);

  //cmd.targetYaw = constrain(cmd.targetYaw,
       //                     SERVO_CENTER_DEG - SERVO_LIMIT_DEG,
      //                      SERVO_CENTER_DEG + SERVO_LIMIT_DEG);

  cmd.manualThrottle = constrain(cmd.manualThrottle, 1100, 1900);
  cmd.ballastRequest = (cmd.ballastRequest != 0) ? 1 : 0;

  printDebugData();
}