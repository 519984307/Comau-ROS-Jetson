# comau_gripper_handler 

# Overview of the comau ros gripper handler

## Prelogue

This package currently gives the user the ability to command the desired state of an I/O pin using a ROS service.

# How to control a I/O pins using this package 

**ATTENTION!!!**\
Always check the robot surroundings. Make sure that no one is near the robot.

# Robot side

Clone the comau_pdl in the PC that has access to robot's controller FTP.

```bash
git clone https://github.com/LMS-Robotics-VR/comau_pdl.git
```

1. Load `pdl_tcp_functions`. (NO HOLD PDL program with utility functions for the TCP/IP communication)
2. Load `gripper_handler`. (NO HOLD PDL program that executes the I/O commands)

**IMPORTANT NOTE**\
The I/O pins must be set up on the controller side. Undefined I/O pins commands will trigger an error.

The pin(s) that are actuated by this package need to be declared on the `receive_gripper_message` routine of the `gripper_handler.pdl`.

*e.g.*
```bash
  ------------------------------------------------------------------
  --- Routine: receive_gripper_message
  --- Brief: Receiving the gripper command and send it for execution 
  ------------------------------------------------------------------
ROUTINE receive_gripper_message : BOOLEAN
VAR
  t_i : INTEGER
  t_msg : GRIPPER_MESSAGE_TYPE
BEGIN
  READ vi_server_netlun (t_msg.id::4)             -- Read the gripper message unique identifier
  READ vi_server_netlun (t_msg.gripper_command::4) -- Read the open close flag
  vi_gripper_command := t_msg.gripper_command
  gripper_state := vi_gripper_command
  IF gripper_state = TRUE THEN
    $DOUT[19] = ON
  ENDIF
  IF si_verbose = 1 THEN 
    WRITE vs_screen_lun (cs_main_log, "Received gripper message : ", gripper_state, NL)
  ENDIF
  RETURN(TRUE)
END receive_gripper_message
```

# PC side

## Configuration

At **comau_gripper_handler/config/gripper_handler.yaml** we can find the configuration parameters for the network setup. Please configure the robot's IP there.


## Starting Sequence

1. Ensure that you have connection with the robot or RoboShop.
2. Once the ROS workspace is built (see instructions in main [README](../README.md)), you are good to go ahead starting the handler with the following commands in different terminals.

```bash
# Launch gripper handler service server
roslaunch comau_gripper_handler comau_gripper_handler.launch 

# Service call
rosservice call /gripper_command "gripper_command: false" #Or true
```