gazebo_client = node.create_client(SetModelState, '/gazebo/set_model_state')
while not gazebo_client.wait_for_service(timeout_sec=1.0):
    node.get_logger().info('service not available, waiting again...')
resp = gazebo_client.call_async(req)
rclpy.spin_until_future_complete(node, resp)

state_msg = ModelState()

state_msg = ModelState()
state_msg.model_name = 'cube1'

state_msg.pose.position.x = 0
state_msg.pose.position.y = -0.5
state_msg.pose.position.z = 1
state_msg.pose.orientation.w = 1
state_msg.pose.orientation.x = 0
state_msg.pose.orientation.y = 0
state_msg.pose.orientation.z = 0


class MinimalClient(Node):

    def __init__(self):
        super().__init__('mara_minimal_client')

        # Create a client for service "/hrim_actuation_gripper_000000000004/goal"
        self.client = self.create_client(
            ControlFinger, "/hrim_actuator_gripper_000000000004/fingercontrol")

        # Wait for service to be avaiable before calling it
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        # Create request with the same type as the service, ControlFinger
        self.req = ControlFinger.Request()

    def send_request(self):
        #self.req.goal_angularposition = 0.86
        #self.req.goal_angularposition = 0.37
        #self.req.goal_linearposition = 0
        self.req.goal_effort = .01
        #self.req.goal_velocity = 0.
        self.future = self.client.call_async(self.req)
