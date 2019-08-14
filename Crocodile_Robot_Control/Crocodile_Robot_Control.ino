// Aaron Barlev 2019

#include <MPU9250.h>
#include <Servo.h>

Servo servos[8];

// Angle that the leg will move forward or backward at the hip (1/2 of the total sweep angle)
int left_sweep = 8; // ADJUST
int right_sweep = 20; // ADJUST
// Determines how much a given path error affects leg sweep
double K = 0.4; // ADJUST

int lift_angle = 60; // Angle that the leg will move up at the knee
int sweep_max = 20; // Maximum angle that either leg will swep when adjusted for path deviation
int sweep_min = 8; // Minimum angle that either leg will swep when adjusted for path deviation

// Robot starts with FL/BR back and FR/BL forward, all 4 feet on the ground
// For hip, CCW is larger angle
// For knee, from side, CCW is larger angle
int zero[] = {80, 95, 17, 90, 32, 100, 80, 88};

int left_sweep_init = left_sweep;
int right_sweep_init = right_sweep;
double heading = 0;
double heading_init = 0;
double heading_filtered = 0;
double heading_degrees = 0;
double ym = 0;
double xm = 0;
double error = 0;

int lift_delay = 150;
int sweep_delay = 150;

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);
int status;

void write_angles(double front_left_lift, double front_left_sweep, double back_right_lift, double back_right_sweep, 
      double front_right_lift, double front_right_sweep, double back_left_lift, double back_left_sweep) {
  servos[0].write(front_left_lift);
  servos[1].write(front_left_sweep);
  servos[2].write(back_right_lift);
  servos[3].write(back_right_sweep);
  servos[4].write(front_right_lift);
  servos[5].write(front_right_sweep);
  servos[6].write(back_left_lift);
  servos[7].write(back_left_sweep);
}

void update_heading() {
  // get X magnetic field reading, get Y magnetic field reading
  // Set heading = atan2(Y, X)
  // Set heading_avg = heading_avg*0.85 + heading*0.15
  
  IMU.readSensor();
  ym = IMU.getMagY_uT();
  xm = IMU.getMagX_uT();
  heading = atan2(ym, xm);
  double declination = 0.174533; 
  heading += declination;
  
  // Correcting when signs are reveresed
  if(heading <0) heading += 2*PI;
  // Correcting due to the addition of the declination angle
  if(heading > 2*PI)heading -= 2*PI;
  heading_degrees = heading * 180/PI; // The heading in Degrees unit
  // Smoothing the output angle / Low pass filter 
  heading_filtered = heading_filtered*0.65 + heading_degrees*0.35;
}

void setup() {
  Serial.begin(9600);
  servos[0].attach(10);
  servos[1].attach(3);
  servos[2].attach(4);
  servos[3].attach(5);
  servos[4].attach(6);
  servos[5].attach(7);
  servos[6].attach(8);
  servos[7].attach(9);
  write_angles(zero[0]-lift_angle, zero[1], zero[2]+lift_angle, zero[3], zero[4]+lift_angle, zero[5], zero[6]-lift_angle, zero[7]);

// start communication with IMU 
  status = IMU.begin();
  while (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    delay(2000);
  }

  // Record compass data for 5 seconds then set the initial heading, heading_init
  unsigned long t = millis();
  while (millis() - t < 5000) {
    update_heading();
    write_angles(zero[0]-lift_angle, zero[1], zero[2]+lift_angle, zero[3], zero[4]+lift_angle, zero[5], zero[6]-lift_angle, zero[7]);
    delay(10);
  }
  heading_init = heading_filtered;
  
}

void loop() {

  // Zero
  //write_angles(zero[0], zero[1]);
  //delay(lift_delay);

  // Raise FLBR, drop FRBL
  // write_angle(FL lift, FL sweep, BR lift, BR sweep, FR lift, FR sweep, BL lift, BL sweep)
  write_angles(zero[0]-lift_angle, zero[1]-left_sweep, zero[2]+lift_angle, zero[3]+right_sweep, 
        zero[4], zero[5]-right_sweep, zero[6], zero[7]+left_sweep);
  delay(lift_delay);

  // Sweep forward FRBL, sweep back FLBR
  write_angles(zero[0]-lift_angle, zero[1]+left_sweep, zero[2]+lift_angle, zero[3]-right_sweep, 
        zero[4], zero[5]+right_sweep, zero[6], zero[7]-left_sweep);
  delay(sweep_delay);
  
  // Raise FRBL, drop FLBR
  write_angles(zero[0], zero[1]+left_sweep, zero[2], zero[3]-right_sweep, 
        zero[4]+lift_angle, zero[5]+right_sweep, zero[6]-lift_angle, zero[7]-left_sweep);
  delay(lift_delay);

  // Sweep forward FLBR, sweep back FRBL
  write_angles(zero[0], zero[1]-left_sweep, zero[2], zero[3]+right_sweep, 
        zero[4]+lift_angle, zero[5]-right_sweep, zero[6]-lift_angle, zero[7]+left_sweep);
  delay(sweep_delay);

//  Serial.println("Looped");

  update_heading();

  // Set error = heading_avg - heading_init
  // Set left_sweep = left_sweep_init - K*error (within a max/min)
  // Set right_sweep = right_sweep_init - K*error (within a max/min)
  error = heading_filtered - heading_init;
  left_sweep = left_sweep_init - K*error;
  right_sweep = right_sweep_init + K*error; 
  
  // Adjust within a max/min
  if (left_sweep > sweep_max) { left_sweep = sweep_max; }
  if (right_sweep > sweep_max) { right_sweep = sweep_max; }
  if (left_sweep < sweep_min) { left_sweep = sweep_min; }
  if (right_sweep < sweep_min) { right_sweep = sweep_min; }
  delay(10);

  Serial.print("Error: ");
  Serial.println(error);
  Serial.print("Left sweep: ");
  Serial.println(left_sweep);
  Serial.print("Right sweep: ");
  Serial.println(right_sweep);
  
}
