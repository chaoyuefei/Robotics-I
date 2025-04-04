# control the position, velocity or/and effort

std_msgs/Header header

uint8 CONTROL_TYPE_NONE=0
uint8 CONTROL_TYPE_POSITION=1
uint8 CONTROL_TYPE_EFFORT=2
uint8 CONTROL_TYPE_VELOCITY=3
uint8 CONTROL_TYPE_POSITION_VELOCITY=4
uint8 CONTROL_TYPE_POSITION_EFFORT=5
uint8 CONTROL_TYPE_VELOCITY_EFFORT=6
uint8 CONTROL_TYPE_POSITION_VELOCITY_EFFORT=7
uint8 control_type # rotary servomotor control type

float64 position # goal position

float32 velocity # movement velocity

float32 acceleration # movement acceleration

float32 effort # movement torque

