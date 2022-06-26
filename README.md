# MetalCuttingDR

## Recorder:

Run the following command to launch the recorder node

```
roslaunch realsense_recorder recorder.launch
```

### Recorder Topics
`/realsense_record`: Start/Stop the recording from the realsense camera. (Message type std_msgs/Bool)


## Slider:

Upload the DR_Slider.ino code to arduino (if it is not already uploaded).

Initialize the EEPROM values by uncommenting the `initEEPROM()` line in the arduino setup (only if uploading on a new arduino).

Open 2 terminals and run the following commands to use the slider:

### First Terminal:

```
roscore
```

### Second Terminal:

```
rosrun rosserial_python serial_node.py port_number
```

port_number will be something like `/dev/ttyACM0`

### Slider Topics:

`/slider/joint_velocity`: Control velocity of the slider. (Message type std_msgs/Float32)

`/slider/joint_position`: Control position of the slider. (Message type std_msgs/Float32)

`/slider/mode`: Switch between position control and velocity control. (Message type std_msgs/Bool)

`/slider/lever_state`: Actuate the oxygen bypass lever. (Message type std_msgs/Bool)

`/slider/joint_state`: Monitor the slider position and velocity. (Message type sensor_msgs/JointState)