# Importing necessary libraries from ROS2 Python client library
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray  # Import the String message type from standard ROS2 message library
import numpy as np

# Defining the SimpleSubscriber class which inherits from Node
class ForwardKinematics(Node):

    def __init__(self):
        super().__init__('forward_kinematics')  # Initialize the node with the name 'simple_subscriber'
        # Create a subscription object that listens to messages of type String
        # on the topic 'topic'. The 'listener_callback' function is called
        # when a new message is received. '10' is the queue size.
        self.subscription = self.create_subscription(Float32MultiArray,'joint_state',self.listener_callback,10)
        self.subscription  # Dummy expression to avoid unused variable warning
        self.publisher_ = self.create_publisher(Float32MultiArray, 'joint_pos_rel', 10)
        self.l1 = 0.1
        self.l2 = 0.1
        self.l3 = 0.1
        self.target = np.array([0,0])

                 # 初始化关节电流
        self.friction_coefficient = 1.06  # 摩擦力系数
        self.friction_offset = 10.3  # 摩擦力偏移

        # added by Jiaheng
        self.Kpvd1 = 2.0
        self.Kdvd1 = 0.6
        self.Kivd1 = 0.6
        self.Kpvd2 = 2.0
        self.Kdvd2 = 0.6
        self.Kivd2 = 0.6    
        self.Kpvd3 = 2.0
        self.Kdvd3 = 0.6
        self.Kivd3 = 0.6

        self.Pre_vel_err1 = 0 # 储存前项
        self.Int_vel_err1 = 0 # 储存积分
        self.Pre_vel_err2 = 0 # 储存前项
        self.Int_vel_err2 = 0 # 储存积分
        self.Pre_vel_err3 = 0 # 储存前项
        self.Int_vel_err3 = 0 # 储存积分        
        self.abs_int_limit = 10 # 储存积分上界
        self.k_cur_torque = 0.1 # T = K*I
        self.abs_u_limit = 50

    def listener_callback(self, msg):
        # # Callback function that is invoked when a new message is received
        # self.get_logger().info('I heard: "%s"' % msg.data)  # Log the received message
        self.publisher_callback(msg.data)

    def publisher_callback(self, data):
        angles = np.radians(data[:3])
        vel1 = data[3]
        vel2 = data[4]
        vel3 = data[5]
        pos = Float32MultiArray()  # Creating a float position object
        fk_pos = self.forward_kinematic_pos(angles)
        J = self.jacobian(angles)
        J_inverse = np.linalg.pinv(J)

        if self.target.all() == 0:
            self.target = fk_pos

        d_pos = self.target - fk_pos
        d_q = np.dot(J_inverse,d_pos)
        d_q = np.rad2deg(d_q)

        pos.data = [d_q[0],d_q[1],d_q[2]]
        self.publisher_.publish(pos)  # Publishing the message

        # Logging the published message to the console
        self.get_logger().info('Publishing: "%s"' % d_q)

    def jacobian(self,thetas):
        J = [[-self.l1 * np.sin(thetas[0]) - self.l2 * np.sin(thetas[0]+thetas[1]) - self.l3 * np.sin(thetas[0]+thetas[1]+thetas[2]),
              -self.l2 * np.sin(thetas[0]+thetas[1]) - self.l3 * np.sin(thetas[0]+thetas[1]+thetas[2]),
              -self.l3 * np.sin(thetas[0]+thetas[1]+thetas[2])],
             [self.l1 * np.cos(thetas[0]) + self.l2 * np.cos(thetas[0]+thetas[1]) + self.l3 * np.cos(thetas[0]+thetas[1]+thetas[2]),
              self.l2 * np.cos(thetas[0]+thetas[1]) + self.l3 * np.cos(thetas[0]+thetas[1]+thetas[2]),
              self.l3 * np.cos(thetas[0]+thetas[1]+thetas[2])]]
        return np.array(J)

    def forward_kinematic_pos(self,thetas):
        x = (self.l1 * np.cos(thetas[0]) + self.l2 * np.cos(thetas[0]+thetas[1]) + self.l3 * np.cos(np.sum(thetas)))
        y = (self.l1 * np.sin(thetas[0]) + self.l2 * np.sin(thetas[0]+thetas[1]) + self.l3 * np.sin(np.sum(thetas)))
        return np.array([x, y])

# The main function which serves as the entry point for the program
def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS2 Python client library
    forward_kinematics = ForwardKinematics()  # Create an instance of the SimpleSubscriber

    try:
        rclpy.spin(forward_kinematics)  # Keep the node alive and listening for messages
    except KeyboardInterrupt:  # Allow the program to exit on a keyboard interrupt (Ctrl+C)
        pass

    forward_kinematics.destroy_node()  # Properly destroy the node
    rclpy.shutdown()  # Shutdown the ROS2 Python client library

# This condition checks if the script is executed directly (not imported)
if __name__ == '__main__':
    main()  # Execute the main function
