#!/usr/bin/python3

import rclpy
from rclpy.qos import qos_profile_sensor_data
from hrim_actuator_rotaryservo_msgs.msg import GoalRotaryServo
from time import sleep

# -------- #

rclpy.init(args=None)

# Create Node with name "mara_minimal_publisher"
node = rclpy.create_node("mara_minimal_publisher")


# Create a publisher on topic "/hrim_actuation_servomotor_000000000001/goal_axis1"
pub = node.create_publisher(
    GoalRotaryServo, '/hrim_actuator_rotaryservo_000000000001/goal_axis1', qos_profile=qos_profile_sensor_data)
pub2 = node.create_publisher(
    GoalRotaryServo, '/hrim_actuator_rotaryservo_000000000002/goal_axis1', qos_profile=qos_profile_sensor_data)
pub3 = node.create_publisher(
    GoalRotaryServo, '/hrim_actuator_rotaryservo_000000000001/goal_axis2', qos_profile=qos_profile_sensor_data)
pub4 = node.create_publisher(
    GoalRotaryServo, '/hrim_actuator_rotaryservo_000000000003/goal_axis1', qos_profile=qos_profile_sensor_data)

# Create message with the same type as the topic, GoalRotaryServo
msg = GoalRotaryServo()
msg2 = GoalRotaryServo()
msg3 = GoalRotaryServo()
msg4 = GoalRotaryServo()


# Desired position in degrees
position_deg = 25.
position_deg2 = -70.
position_deg3 = 0.
position_deg4 = -70

delta = 0*10
while rclpy.ok():
    print("Next Pos =", position_deg)
    # Fill message content
    msg.position = position_deg * 3.1416/180  # Position to rads
    msg.velocity = 0.4  # Velocity in rads/s
    msg.control_type = 4  # Position and velocity control

    msg2.position = position_deg2 * 3.1416/180  # Position to rads
    msg2.velocity = 0.4  # Velocity in rads/s
    msg2.control_type = 4  # Position and velocity control

    msg3.position = position_deg3 * 3.1416/180  # Position to rads
    msg3.velocity = 0.4  # Velocity in rads/s
    msg3.control_type = 4  # Position and velocity control

    msg4.position = position_deg4 * 3.1416/180  # Position to rads
    msg4.velocity = 0.4  # Velocity in rads/s
    msg4.control_type = 4  # Position and velocity control

    # Publish message!
    pub.publish(msg)
    pub2.publish(msg2)
    pub3.publish(msg3)
    pub4.publish(msg4)

    # Spin not really needed here since we don't have callbacks
    # rclpy.spin_once(node)

    # Sleep 1 second per loop
    sleep(1.)

    position_deg += delta
    if position_deg > 90.0:
        delta = -10
    elif position_deg < -90.0:
        delta = 10
    else:
        pass


node.destroy_node()
rclpy.shutdown()





