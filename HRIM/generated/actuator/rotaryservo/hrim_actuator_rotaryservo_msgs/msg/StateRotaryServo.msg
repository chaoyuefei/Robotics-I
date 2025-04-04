# motor condition and the reason in case of error

std_msgs/Header header # motor state is at this time

float64 goal # commanded position

float64 position # current position encoder

float64 error # difference between current and goal positions

float64 velocity # current velocity

float64 effort # current effort of the actuator

float64 load # load imposed on the motor

bool moving # whether the motor is currently in motion

uint8 FAULT_NONE=0
uint8 FAULT_CURRENT=1
uint8 FAULT_TEMPERATURE=2
uint8 FAULT_BUS_VOLTAGE=3
uint8 FAULT_BRIDGE_DRIVER=4
uint8 fault # fault cause

uint8 CONTROL_TYPE_NONE=0
uint8 CONTROL_TYPE_POSITION=1
uint8 CONTROL_TYPE_EFFORT=2
uint8 CONTROL_TYPE_VELOCITY=3
uint8 CONTROL_TYPE_POSITION_VELOCITY=4
uint8 CONTROL_TYPE_POSITION_EFFORT=5
uint8 CONTROL_TYPE_VELOCITY_EFFORT=6
uint8 CONTROL_TYPE_POSITION_VELOCITY_EFFORT=7
uint8 control_type # rotary servomotor control type

