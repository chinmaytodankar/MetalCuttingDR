
//################# Include Required Libraries #################

#include <AccelStepper.h>
#include <EEPROM.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>

// #############################################################

//################# Declare Constant Variables #################

// Set the pins of stepper motor
const byte PULSE_PIN = 5;
const byte DIRECTION_PIN = 6;
const byte ENABLE_PIN = 7;

// Set the pin of limit switch
const byte LIMIT_SWITCH = 2;

// Set the pins of motor driver for linear actuator
const byte MOTOR_PIN_1 = 11;
const byte MOTOR_PIN_2 = 12;

// Set the constraints of stepper motor
const byte ACCELERATION = 200;
const int MAX_SPEED = 3000;
const int MAX_POSITION = 16000;
const int MIN_POSITION = 0;

// Set interval durations
const byte PUBLISH_INTERVAL = 101;
const byte EEPROM_WRITE_INTERVAL = 100;

// Set Eeprom addresses
const byte CONTROL_MODE_ADDR = 4;
const byte POSITION_ADDR = 6;

// #############################################################

//################## Declare Global Variables ##################

ros::NodeHandle nh;
sensor_msgs::JointState joint_msg;

// ROS Publishers
ros::Publisher joint_publisher("/slider/joint_state", &joint_msg);

AccelStepper stepper(AccelStepper::DRIVER, PULSE_PIN, DIRECTION_PIN);

enum class ControlMode{POSITION, VELOCITY};

ControlMode control_mode;
int current_position = 0;
int set_position = 0;
float set_velocity = 0;
int prev_position = 0;
long publish_timer = 0;
long eeprom_write_timer = 0;
float pos[1], vel[1];
bool lever_state = false, prev_lever_state = false;
volatile bool limit_switch_flag = false;

// #############################################################

// ####################### ROS Subscribers #######################

void modeCallback(const std_msgs::Bool& msg) 
{
    if(msg.data) 
    {
        control_mode = ControlMode::POSITION;
        set_position = current_position;
        stepper.moveTo(set_position);
    } 
    else 
    {
        control_mode = ControlMode::VELOCITY;
        set_velocity = 0;
        stepper.setSpeed(set_velocity);
    }
    EEPROM.put(CONTROL_MODE_ADDR, control_mode);
}

ros::Subscriber<std_msgs::Bool> mode_sub("/slider/mode", &modeCallback);

void posCallback(const std_msgs::Float32& msg) 
{
    if(control_mode == ControlMode::POSITION) 
    {
        set_position = msg.data;
        set_position = set_position > MAX_POSITION ? MAX_POSITION : (set_position < MIN_POSITION ? MIN_POSITION : set_position);
        stepper.moveTo(set_position);
    }
}
ros::Subscriber<std_msgs::Float32> pos_sub("/slider/joint_position", &posCallback);

void velCallback(const std_msgs::Float32& msg) 
{
    if(control_mode == ControlMode::VELOCITY) 
    {
        set_velocity = msg.data;
        stepper.setSpeed(set_velocity);
    }
}
ros::Subscriber<std_msgs::Float32> vel_sub("/slider/joint_velocity", &velCallback);

void leverStateCallback(const std_msgs::Bool& msg)
{
    lever_state = msg.data;
}
ros::Subscriber<std_msgs::Bool> lever_sub("/slider/lever_state", &leverStateCallback);

// #############################################################

// #################### Lever Functions #####################

void change_lever_state()
{
    digitalWrite(MOTOR_PIN_1, lever_state);
    digitalWrite(MOTOR_PIN_2, !lever_state);
}

// #############################################################

// ####################### Initialize EEPROM #######################
void initEEPROM() 
{
    control_mode = ControlMode::POSITION;
    EEPROM.put(CONTROL_MODE_ADDR, ControlMode::POSITION);
    current_position = MAX_POSITION;
    EEPROM.put(POSITION_ADDR, current_position);
}
// #################################################################

// #################### Interrupt Routine ######################

void limitSwitchInterrupt() {
    if(current_position != 0) 
    {
        limit_switch_flag = true;
    }
}

// #############################################################

// ####################### Arduino Setup #######################

void setup() 
{

    // Initialize pin modes
    pinMode(PULSE_PIN, OUTPUT);
    pinMode(DIRECTION_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(LIMIT_SWITCH, INPUT);
    pinMode(MOTOR_PIN_1, OUTPUT);
    pinMode(MOTOR_PIN_2, OUTPUT);
    digitalWrite(MOTOR_PIN_1, LOW);
    digitalWrite(MOTOR_PIN_2, LOW);
    attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH), limitSwitchInterrupt, FALLING);    // Attach Interrupt to the limit switch pin 

    // Initialize EEPROM
     initEEPROM();

    // Get Values stored in EEPROM
    EEPROM.get(CONTROL_MODE_ADDR, control_mode);
    EEPROM.get(POSITION_ADDR, current_position);
    set_position = current_position;

    // Initialize stepper motor values
    stepper.setEnablePin(ENABLE_PIN);
    stepper.setMinPulseWidth(5);
    stepper.setPinsInverted(false, true, true);
    stepper.setMaxSpeed(MAX_SPEED);
    stepper.setSpeed(0);
    stepper.setAcceleration(ACCELERATION);
    stepper.setCurrentPosition(current_position);

    // Initialize ROS
    nh.initNode();
    nh.advertise(joint_publisher);
    nh.subscribe(mode_sub);
    nh.subscribe(pos_sub);
    nh.subscribe(vel_sub);
    nh.subscribe(lever_sub);

    // Initialize Joint Message
    joint_msg.position_length = 1;
    joint_msg.velocity_length = 1;
    pos[0] = current_position;
    vel[0] = stepper.speed();
    joint_msg.position = pos;
    joint_msg.velocity = vel;

    // Initialize Timers
    publish_timer = millis();
    eeprom_write_timer = millis();
}

// #############################################################

// ####################### Arduino Loop ########################

void loop() 
{
    current_position = stepper.currentPosition();                           // Get stepper motors current position.

    // actuate the lever if the state has been changed.
    if(lever_state != prev_lever_state) {
        change_lever_state();
    }
    prev_lever_state = lever_state;

    // If the limit switch was triggered, reset the position of the stepper motor.
    if(limit_switch_flag) 
    {
        current_position = 0;
        stepper.setCurrentPosition(current_position);
        limit_switch_flag = false;
        nh.loginfo("Interrupt!");
    }
    
    // Check Controller Mode.
    if(control_mode == ControlMode::POSITION)
    {
        stepper.run();                                                      // Increment a step if position not reached in position control mode.
    }
    else                                                                    // Else it is velocity controller.
    {
        if(((current_position >= MAX_POSITION) && set_velocity > 0) ||      // If position goes out of constraints, stop the motor.
            ((current_position <= MIN_POSITION) && set_velocity < 0))
        {
            set_velocity = 0;
            stepper.setSpeed(set_velocity);
        }
        stepper.runSpeed();                                                 // Run stepper at set speed.
    }

    // Publish Message if the set interval has passed
    if(millis() - publish_timer >= PUBLISH_INTERVAL)
    {
        pos[0] = current_position;
        vel[0] = stepper.speed();
        joint_msg.position = pos;
        joint_msg.velocity = vel;
        joint_publisher.publish(&joint_msg);
        publish_timer = millis();
    }

    // Write values to eeprom if the set interval has passed
    if(millis() - eeprom_write_timer >= EEPROM_WRITE_INTERVAL)
    {
        if(current_position != prev_position)                       // Write only if the position has been changed
        {
            EEPROM.put(POSITION_ADDR, current_position);
        }
        eeprom_write_timer = millis();
        prev_position = current_position;
    }

    nh.spinOnce();
}

// #############################################################
