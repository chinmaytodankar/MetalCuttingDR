#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, Float32, Bool
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import JointState
import numpy as np
import sys
from enum import Enum, auto

# Enumerator to track phases/stages of cutting
class Phase(Enum):
    INIT = auto()
    TORCH_ON = auto()
    PREHEATING = auto()
    CUTTING = auto()
    FINISHED = auto()
    OTHER = auto()

class Controller:
    def __init__(self):

        # Creating all the publishers
        self.slider_mode_publisher = rospy.Publisher("/slider/mode", Bool, queue_size=1)
        self.slider_position_publisher = rospy.Publisher("/slider/joint_position", Float32, queue_size=1)
        self.slider_lever_state_publisher = rospy.Publisher("/slider/lever_state", Bool, queue_size=1)
        
        # Creating all the subscribers
        self.heat_pool_strength_subscriber = rospy.Subscriber("/image_processing/ellipse/strength", Float32, self.heatPoolStrengthCallback, queue_size=1)
        self.heat_pool_eccentricity_subscriber = rospy.Subscriber("/image_processing/ellipse/eccentricity", Float32, self.heatPoolEccentricityCallback, queue_size=1)
        self.calibration_status_subscriber = rospy.Subscriber("/image_processing/torch_calibration/status", Bool, self.calibrationStatusCallback, queue_size=1)
        self.calibration_strength_subscriber = rospy.Subscriber("/image_processing/torch_calibration/strength", Float32, self.calibrationStrengthCallback, queue_size=1)
        self.poses_subscriber = rospy.Subscriber("/control/inputs", PoseArray, self.posesCallback, queue_size=1)
        self.joint_state_subscriber = rospy.Subscriber("/slider/joint_state", JointState, self.jointStateCallback, queue_size=1)
        
        # Heat pool variables
        self.heat_pool_strength = -1
        self.heat_pool_strength_buffer = np.zeros(5)        # buffer to average out values
        self.heat_pool_strength_index = 0
        self.heat_pool_eccentricity = -1
        self.calibration_strength = -1
        
        # timer to call control algorithm at interval specified in duration in seconds
        rospy.Timer(rospy.Duration(0.1), self.controlLoop)
        
        # Initialize the phase variables
        self.current_phase = Phase.INIT
        self.previous_phase = Phase.OTHER
        
        # flags used
        self.first_run = True
        self.position_published = False

        # steps array received as input
        self.steps_array = []

        # variables to control cutting
        self.time_delay = 0
        self.kp = 1
        self.start_time = 0

        rospy.loginfo("Controller Started")


    def controlLoop(self, event):

        # If the phase is initial move to position 0 and wait for starting the torch and calibrating.
        if(self.current_phase is Phase.INIT):
            if(self.first_run):
                rospy.loginfo("Moving to position 0")
                self.first_run = False
                self.slider_mode_publisher.publish(True)
                self.slider_position_publisher.publish(0.0)
            if(self.previous_phase != self.current_phase):
                rospy.loginfo("INITIAL PHASE: Start the torch after the slider moves to 0 position and then calibrate the image processing node to start")
            self.previous_phase = self.current_phase
            return
        
        # phase changes to TORCH ON once the calibration status turns true, i.e., calibration is completed. 
        # after that we move the torch to the first pose in the list of poses received and change to preheating phase 
        # once reached there
        if(self.current_phase is Phase.TORCH_ON):
            if(self.previous_phase != self.current_phase):
                rospy.loginfo("Calibration complete, moving to initial position for preheating")
            self.previous_phase = self.current_phase
            
            if(len(self.steps_array) == 0):
                rospy.logwarn("Send path poses to continue")
                return
            if not self.position_published:
                self.published_position = self.steps_array.pop(0)
                self.slider_position_publisher.publish(self.published_position)
                self.position_published = True

            
            if self.jointState.position[0] == self.published_position:
                self.current_phase = Phase.PREHEATING

            return

        # In preheating phase we wait at the position until the heatpool strength crosses a threshold, 
        # after that we press the bypass lever and start the cutting phase. 
        if(self.current_phase is Phase.PREHEATING):
            if(self.previous_phase != self.current_phase):
                rospy.loginfo("Preheating the metal...")
            self.previous_phase = self.current_phase
            
            if self.heat_pool_strength > 1600:
                rospy.loginfo("Preheating Complete")
                self.current_phase = Phase.CUTTING
                self.position_published = False
                self.start_time = rospy.Time.now()
            return

        # Cutting Phase
        if(self.current_phase is Phase.CUTTING):
            if(self.previous_phase != self.current_phase):
                rospy.loginfo("Cutting Phase")
            self.previous_phase = self.current_phase

            # if there are no more steps left, the cutting is finished
            if len(self.steps_array) == 0:
                rospy.loginfo("Finished Cutting")
                self.current_phase = Phase.FINISHED
                return

            # bypass lever is pressed in the cutting phase.
            self.slider_lever_state_publisher.publish(True)

            # if we are not moving between two poses
            if not self.position_published:
                
                # ***not tested control logic using proportional controller***
                # higher above 0 the error that means the heat pool is smaller than reference, 
                # that means we need to slow down hence greater the time delay between moving from one pose to other
                # conversely, lower the error below 0, heat pool is bigger than ideal case, hence we can speed up a bit
                # thus lower the time delay (time delay cannot be negative, thus clipping it to 0, 
                # probably should clip it above some const so that there is atleast some delay between two poses, could be determined by testing)
                
                # get the error of the heatpool with some reference
                self.error = 3000 - self.heat_pool_strength

                # multiply the error with some gain and add it in the time delay
                self.time_delay += self.error * self.kp

                # clip the time delay above 0
                if self.time_delay < 0:
                    self.time_delay = 0

                
                rospy.loginfo(self.time_delay)
                
                # if the time difference is greater than the time delay since we started cutting at current position, we move to next pose.
                if (rospy.Time.now() - self.start_time > rospy.Duration(self.time_delay/1000)):
                    self.published_position = self.steps_array.pop(0)
                    self.slider_position_publisher.publish(self.published_position)
                    self.position_published = True
                    rospy.loginfo("Moving to position {}".format(self.published_position))
                return

            # if we are moving between two poses we wait till the torch reaches next pose.
            if self.position_published:
                if self.jointState.position[0] == self.published_position:
                    self.position_published = False
                    self.start_time = rospy.Time.now()
                    rospy.loginfo("Moved to position {}".format(self.published_position))

            return
            

        # if we are finished cutting, we can release the bypass lever
        if(self.current_phase is Phase.FINISHED):
            if(self.previous_phase != self.current_phase):
                rospy.loginfo("Finished Phase.")
            self.previous_phase = self.current_phase
            self.slider_lever_state_publisher.publish(False)
            return


        # other phase in case of some error maybe? just added in case required in future.
        if(self.current_phase is Phase.OTHER):
            rospy.logerr("Error: Unknown Phase Encountered")
            self.previous_phase = self.current_phase
            return

    # monitors whether the calibration is completed, if yes change from init phase to torch on phase
    def calibrationStatusCallback(self, msg):
        if(msg.data and self.current_phase is Phase.INIT):
            self.current_phase = Phase.TORCH_ON

    # callback to get the poses, i.e., the steps that we have to go cut at in case of 1d slider
    def posesCallback(self, msg):
        self.steps_array = []
        for pose in msg.poses:
            self.steps_array.append(self.sliderPositionToStepsConversion(pose))
        rospy.loginfo("Poses Received")
        print(self.steps_array)

    # converts the received poses from mm to stepper steps
    def sliderPositionToStepsConversion(self, pose):
        position = pose.position.y
        return int(50 * position)

    # callback to get the heatpool strength, averages last 5 values received
    def heatPoolStrengthCallback(self, msg):
        self.heat_pool_strength_buffer[self.heat_pool_strength_index] = msg.data
        self.heat_pool_strength_index = (self.heat_pool_strength_index + 1) % 5
        self.heat_pool_strength = np.mean(self.heat_pool_strength_buffer)

    # callback to get the eccentricity, not used in control algorithm right now
    def heatPoolEccentricityCallback(self, msg):
        self.heat_pool_eccentricity = msg.data

    # joint state callback to monitor current location of the torch
    def jointStateCallback(self, msg):
        self.jointState = msg

    # callback to get the strength of heatpool when we were calibrating the torch position.
    # not used, could be useful to get the ratio of heatpool strength while cutting the metal 
    # wrt to heatpool strength of idle torch to use in the control algoritm maybe? 
    def calibrationStrengthCallback(self, msg):
        self.calibration_strength = msg.data

def main(args):

    rospy.init_node("slider_controller", anonymous=True)
    rospy.loginfo("Starting Node")
    controller = Controller()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")

if __name__ == '__main__':
    main(sys.argv)