#include <AccelStepper.h>
#include <EEPROM.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>

#define EEPROM_SIZE 36

#define PULSE_PIN 5
#define DIRECTION_PIN 6
#define ENABLE_PIN 7
#define LIMIT_SWITCH_1 2
 

#define ACCELERATION 200
#define MAX_SPEED 3000
#define MAX_POSITION 16000
#define MIN_POSITION 0

#define SERIAL_WRITE_INTERVAL 101 // ms
#define EEPROM_WRITE_INTERVAL 100 // ms

#define CONTROL_MODE_ADDR 8
#define POSITION_ADDR 16

ros::NodeHandle nh;
sensor_msgs::JointState joint_state_message;
ros::Publisher joint_state_publisher("/joint_state", &joint_state_message);

AccelStepper stepper(AccelStepper::DRIVER, PULSE_PIN, DIRECTION_PIN); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

/*
TODO:
Two Modes: Position Control, Velocity Control

  Velocity Control:
    Take speed from ROS (Serial) unit: steps/sec.
    Move at speed set.

  Position Control:
    Take position from  ROS (serial) unit: step.
    Move to position using a set max speed and acceleration.

Keep sending speed and position to ROS (serial).

If position changed store the new position in EEPROM (read position at start from EEPROM). !!! initialize EEPROM with position 0 at start.

Logic to Switch between Postion and Velocity Control (store it in EEPROM)


Serial Inputs:

mode 0  // position
mode 1  // velocity

21134 // position/velocity


0 to 17000

0 to 16384 (0 to 2^14)

100 = 6mm
1000 = 24mm

lead/ pitch = 4mm
total length of ball screw = 500
steps per rev = 200
1 rev = 4mm
200 steps = 4mm
50 steps = 1mm

y = 50   * x - 200  (mm to steps)
y = 0.02 * x + 4    (steps to mm)

*/

enum class ControlMode{POSITION, VELOCITY};

ControlMode control_mode;
long current_position = 0;
long set_position = 0;
float set_velocity = 0;
long prev_position = 0;
long serial_timer = 0;
long eeprom_writer_timer = 0;
float pos[1], vel[1];



void modeCallback(const std_msgs::Bool& msg) {
  if(msg.data) {
    control_mode = ControlMode::POSITION;
    set_position = current_position;
    stepper.moveTo(set_position);
  } else {
    control_mode = ControlMode::VELOCITY;
    set_velocity = 0;
    stepper.setSpeed(set_velocity);
  }
  EEPROM.put(CONTROL_MODE_ADDR, control_mode);
//  EEPROM.commit();
}

void velCallback(const std_msgs::Float32& msg) {
  if(control_mode == ControlMode::VELOCITY) {
    set_velocity = msg.data;
    stepper.setSpeed(set_velocity);
  }
}

void posCallback(const std_msgs::Float32& msg) {
  if(control_mode == ControlMode::POSITION) {
    set_position = msg.data;
    set_position = set_position > MAX_POSITION ? MAX_POSITION : (set_position < MIN_POSITION ? MIN_POSITION : set_position);
    stepper.moveTo(set_position);
  }
}

ros::Subscriber<std_msgs::Bool> mode_sub("slider/mode", &modeCallback );
ros::Subscriber<std_msgs::Float32> vel_sub("/slider/joint_velocity", &velCallback );
ros::Subscriber<std_msgs::Float32> pos_sub("/slider/joint_position", &posCallback );


volatile bool limit_switch_flag = false;
//bool limit_switch_flag = true;

void limitSwitchInterrupt() {
  if(current_position != 0) {
    limit_switch_flag = true;
  }
}

void setup()
{
//  EEPROM.begin(EEPROM_SIZE);
  // Initialize IOs
  pinMode(PULSE_PIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(LIMIT_SWITCH_1, INPUT);

  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_1), limitSwitchInterrupt, FALLING); 

  // Initialize EEPROM
//  control_mode = ControlMode::POSITION;
//  EEPROM.put(CONTROL_MODE_ADDR, ControlMode::POSITION);
//  current_position = 1000;
//  delay(100);
//  EEPROM.put(POSITION_ADDR, current_position);
//  delay(100);
//  Serial.begin(57600);
//  while(1) {
//    Serial.print(sizeof(control_mode));
//    Serial.print(" ");
//    Serial.println(sizeof(current_position));
//  }
  EEPROM.get(CONTROL_MODE_ADDR, control_mode);
  EEPROM.get(POSITION_ADDR, current_position);
  set_position = current_position;

  // Initialize Serial
//  Serial.begin(9600);  
  
  // Initialize AccelStepper Library
  stepper.setEnablePin(ENABLE_PIN);
  stepper.setMinPulseWidth(5);
  stepper.setPinsInverted(false, true, true);
  
  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setSpeed(0);
  stepper.setAcceleration(ACCELERATION);
  stepper.setCurrentPosition(current_position);

  joint_state_message.position_length = 1;
  joint_state_message.velocity_length = 1;
  pos[0] = current_position;
  vel[0] = stepper.speed();
  joint_state_message.position = pos;
  joint_state_message.velocity = vel;

  nh.initNode();
  nh.advertise(joint_state_publisher);
  nh.subscribe(mode_sub);
  
  nh.subscribe(vel_sub);
  nh.subscribe(pos_sub);

  serial_timer = millis();	
  eeprom_writer_timer = millis();
}

bool reset_flag = false;

void loop()
{  
  current_position = stepper.currentPosition();
  if(limit_switch_flag) {
    current_position = 0;
    stepper.setCurrentPosition(current_position);
    limit_switch_flag = false;
    nh.loginfo("Interrupt!");
  }

  if(control_mode == ControlMode::POSITION) {
    stepper.run();
  } else {
    if(((current_position >= MAX_POSITION) && set_velocity > 0) || ((current_position <= MIN_POSITION) && set_velocity < 0)) {
      set_velocity = 0;
      stepper.setSpeed(set_velocity);
    }
    stepper.runSpeed();
  }

  if(millis() - serial_timer > SERIAL_WRITE_INTERVAL) {
    pos[0] = current_position;
    vel[0] = stepper.speed();
    joint_state_message.position = pos; 
    joint_state_message.velocity = vel;
    joint_state_publisher.publish(&joint_state_message);
    serial_timer = millis();
  }

  nh.spinOnce();
//  nh.loginfo("test1");
  if(millis() - eeprom_writer_timer > EEPROM_WRITE_INTERVAL) {
    if(current_position != prev_position) {
      EEPROM.put(POSITION_ADDR, current_position);
//      EEPROM.commit();
//      nh.loginfo("test");
    }
    eeprom_writer_timer = millis();
    prev_position = current_position;
  }
  
}
