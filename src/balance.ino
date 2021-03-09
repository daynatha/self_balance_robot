#include <Wire.h>

/* --- GLOBAL VARIABLES --- */
const float LOOP_INTERVAL_HZ    = 100.0; 
const float LOOP_INTERVAL_MILLI = 1000.0 / LOOP_INTERVAL_HZ;
const int MPU_ADDR   = 0x68;
const int ACCEL_REGS = 0x3B;
const int GYRO_REGS  = 0x43;
const float pi       = 3.14159265;

const float ACCEL_ROLL_OFFSET  = 4.84;
const float ACCEL_PITCH_OFFSET = 0.08;

short gyro_x, gyro_y, gyro_z;
short acc_x, acc_y, acc_z;
short temp;
int gyro_x_offs, gyro_y_offs, gyro_z_offs;

float pitch_out, roll_out;
float accel_roll, accel_pitch;

float roll_angle, pitch_angle, yaw_angle;
double start_of_loop;

/* --- FUNCTIONS --- */
void calculate_accel_angle_offsets();
void configure_MPU();
void calibrate_MPU(const int readings);
void calculate_angles(bool configuration);
float square(float s) {return s*s;}
void print_pitch_roll();
void print_raw_MPU_data();
float get_pitch() {return pitch_out;}

/*
const int ENABLE        = 4;
const int DIR           = 5;
const int STEP          = 6;
const int STEPS_PER_REV = 200;
*/

unsigned int print_inc = 0;


/* ----------------------------------- START SETUP ----------------------------------- */
void setup() {
  Wire.begin();
  Serial.begin(9600);
  
  configure_MPU();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADDR, 1);
  int setting = Wire.read();
  Serial.print("Gyro config register contents: ");
  Serial.println(setting);

//  calculate_accel_angle_offsets();
  
  delay(500);                                          // delay 2 seconds for MPU to initialize
  
  calibrate_MPU(1000);
  /*
  pinMode(STEP,OUTPUT);
  pinMode(DIR,OUTPUT);
  pinMode(ENABLE,OUTPUT);
  digitalWrite(ENABLE,LOW);
 */
}
/* ----------------------------------- END SETUP ------------------------------------ */

/* ----------------------------------- START LOOP ----------------------------------- */
void loop() {
  start_of_loop = micros();
  
  calculate_angles(false);
  
  if (print_inc++ % 5 == 0) { print_pitch_roll(); }
//  print_raw_MPU_data();
  
  while (micros() - start_of_loop < LOOP_INTERVAL_MILLI * 1000.0) { }                        // wait for next loop (200Hz)
  
}
/* ----------------------------------- END LOOP ------------------------------------ */


void calculate_accel_angle_offsets() {
  Serial.print("Place robot on completely level surface. Calibration will start in 3 seconds");
  delay(500);
  Serial.print('.');
  delay(1000);
  Serial.print('.');
  delay(1000);
  Serial.println('.');
  delay(1000);
  Serial.println("Calculating pitch/roll from accelerometer data, do not move robot");
  
  int total_accel;
  accel_roll  = 0;
  accel_pitch = 0;
  for (int i = 0; i < 800; ++i) {
    gather_MPU_data();
    total_accel = sqrt((acc_x*acc_x) + (acc_y*acc_y) + (acc_z*acc_z));
    accel_roll  += -1 * asin(float(acc_x)/total_accel) * (180.0 / pi);
    accel_pitch += asin(float(acc_y)/total_accel) * (180.0 / pi);
    delay(5);
  }
  accel_roll  /= 800;
  accel_pitch /= 800;
  Serial.print("ACCEL_ROLL_OFFSET: ");
  Serial.print(accel_roll);
  Serial.print("\tACCEL_PITCH_OFFSET: ");
  Serial.println(accel_pitch);
  Serial.println("Done");
  while (true) { delay(10); }                     // loop indefinetly 
}

void configure_MPU() {
  Wire.beginTransmission(MPU_ADDR);           
  Wire.write(0x6B);                       
  Wire.write(0x00);                       
  Wire.endTransmission(); 
  // setup the gyro 
  Wire.beginTransmission(MPU_ADDR);                                        
  Wire.write(0x1B);                                                   
  Wire.write(0b00001000);                                            // FS_SEL=1 (set bits 3,4 in gyro_config register to 0b01)      
  Wire.endTransmission();                 
  // setup the accel
  Wire.beginTransmission(MPU_ADDR);          
  Wire.write(0x1C);                                                 
  Wire.write(0b00010000);                                            // AFS_SEL=1 (set bits 3,4 in gyro_config register to 0b10)            
  Wire.endTransmission();                                               
}

void calibrate_MPU(const int readings) {
  /* This function records the MPU6050 gyroscope x,y,z values and gets 
   * their average over 3000 recordings.  Requires that the device be 
   * stationary during execution of the function
   */
   //TODO: add functionality that runs motors while calibrating for errant noise 

  if (readings <= 0) return;
  
  gyro_x_offs = gyro_y_offs = gyro_z_offs = 0;
  roll_angle = pitch_angle = yaw_angle    = 0;
  
  int dot_inc = readings / 21;
  
  Serial.print("Calibrating MPU6050...\n");
  
  for (int i = 0; i < readings; i++) {
    if (i % dot_inc == 0) { Serial.print("."); }
    gather_MPU_data();
    gyro_x_offs = gyro_x_offs + gyro_x;
    gyro_y_offs = gyro_y_offs + gyro_y;
    gyro_z_offs = gyro_z_offs + gyro_z;
    delay(3);
  }
  
  gyro_x_offs /= readings;
  gyro_y_offs /= readings;
  gyro_z_offs /= readings;

  Serial.print("\nx avg: ");
  Serial.print(gyro_x_offs);
  Serial.print("\ty avg: ");
  Serial.print(gyro_y_offs);
  Serial.print("\tz avg: ");
  Serial.println(gyro_z_offs);
  
  calculate_angles(true);
}

void gather_MPU_data() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADDR, 14);

  while (Wire.available() < 14) { }

  acc_x  = Wire.read() << 8 | Wire.read();                                  //Add the low and high byte to the acc_x variable
  acc_y  = Wire.read() << 8 | Wire.read();                                  //Add the low and high byte to the acc_y variable
  acc_z  = Wire.read() << 8 | Wire.read();                                  //Add the low and high byte to the acc_z variable
  temp   = Wire.read() << 8 | Wire.read();                            //Add the low and high byte to the temperature variable
  gyro_x = Wire.read() << 8 | Wire.read();                                 //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read() << 8 | Wire.read();                                 //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read() << 8 | Wire.read(); 
}

void calculate_angles(bool calibration) {
  /* Calculates the current angle of the device. 
   * calibration - true this function is called from calibrate_MPU(),
   *               otherwise this should be false
   */
  float drift_comp  = 0.0008;                                       // change this value to play around with drift settings 
  float filter      = 0.1;                                         // use a complementary filter
  
//  gather_gyro_data();
  gather_MPU_data();

  gyro_x -= gyro_x_offs;
  gyro_y -= gyro_y_offs;
  gyro_z -= gyro_z_offs;
  
  roll_angle   += ((float)gyro_y / 65.5) / LOOP_INTERVAL_HZ;                            // 200Hz and raw gyro output is 65.5deg/s
  pitch_angle  += ((float)gyro_x / 65.5) / LOOP_INTERVAL_HZ;
  yaw_angle    += ((float)gyro_z / 65.5) / LOOP_INTERVAL_HZ;

//  gather_accel_data();

  float total_accel = sqrt((acc_x*acc_x) + (acc_y*acc_y) + (acc_z*acc_z));
  accel_roll  = -1 * asin(float(acc_x)/total_accel) * (180.0 / pi) - ACCEL_ROLL_OFFSET;
  accel_pitch = asin(float(acc_y)/total_accel) * (180.0 / pi) - ACCEL_PITCH_OFFSET;

  float Gs = total_accel / 4096.0;                                                // convert total acceleration to terms of Gs

//  float deg_C = float(temp) / 340.0 + 36.53;                                                      // convert raw temperature data to C
//  Serial.println(deg_C);

//  Serial.println(Gs);
  
  if (Gs < 1.04 && Gs > 0.97) {
    drift_comp = 0.01;
  }
  else {
    drift_comp = 0.0008;
  }
  
  /*
  Serial.print("Roll (acc): ");
  Serial.print(accel_roll);
  Serial.print("\tPitch (acc): ");
  Serial.println(accel_pitch);
  */
  
  if (calibration) {
    roll_out  = accel_roll;
    pitch_out = accel_pitch;
//    roll_angle  = accel_roll;
//    pitch_angle = accel_pitch;
  }
  else {
    roll_angle  = roll_angle * (1 - drift_comp) + accel_roll * drift_comp;
    pitch_angle = pitch_angle * (1 - drift_comp) + accel_pitch * drift_comp;
  }
  // complementary filter
  roll_out  = roll_out * 0.9 + roll_angle * 0.1;
  pitch_out = pitch_out * 0.9 + pitch_angle * 0.1;
//roll_out = roll_angle;
//pitch_out = pitch_angle;
}

void print_pitch_roll() {
  Serial.print("Roll: ");
  Serial.print(roll_out);
  Serial.print("\tPitch: ");
  Serial.print(pitch_out);
  Serial.print("\tRoll (acc): ");
  Serial.print(accel_roll);
  Serial.print("\tPitch (acc): ");
  Serial.println(accel_pitch);
}

void print_raw_MPU_data() {
  Serial.print("gyro_x: ");
  Serial.print(gyro_x);
  Serial.print("\tgyro_y: ");
  Serial.print(gyro_y);
  Serial.print("\tgyro_z: ");
  Serial.print(gyro_z);
  Serial.print("\tacc_x: ");
  Serial.print(acc_x);
  Serial.print("\tacc_y: ");
  Serial.print(acc_y);
  Serial.print("\tacc_z: ");
  Serial.println(acc_z);
}

  /*
  digitalWrite(DIR,HIGH);
  for (int i=0; i < STEPS_PER_REV*3; ++i) {
    digitalWrite(STEP,HIGH);
    delayMicroseconds(500);
    digitalWrite(STEP,LOW);
    delayMicroseconds(500);
  }

  delay(500);
  
  digitalWrite(DIR,LOW);
  for (int i=0; i < STEPS_PER_REV; ++i) {
    digitalWrite(STEP,HIGH);
    delayMicroseconds(2000);
    digitalWrite(STEP,LOW);
    delayMicroseconds(2000);
  }
  */
