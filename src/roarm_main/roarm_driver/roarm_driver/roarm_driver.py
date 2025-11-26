import rclpy
from rclpy.node import Node
from serial import SerialException
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32
import os
from roarm_sdk.roarm import roarm

class RoarmDriver(Node):
    def __init__(self):
        super().__init__('roarm_driver')
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 115200)
        serial_port_name = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.roarm_type = os.environ['ROARM_MODEL']
        self.roarm = roarm(roarm_type=self.roarm_type, port=serial_port_name, baudrate=baud_rate)
        self.last_roarm_sent_data = None

        self.joint_states_sub = self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)    
        self.led_ctrl_sub = self.create_subscription(Float32, 'led_ctrl', self.led_ctrl_callback, 10)
        
    def handle_m2_joint_states(self,name,position):
        base = position[name.index('base_link_to_link1')]
        shoulder = position[name.index('link1_to_link2')]
        elbow =  position[name.index('link2_to_link3')]
        hand =  position[name.index('link3_to_gripper_link')]
                
        radians = [base, shoulder, elbow, hand]  
        return radians
    
    def handle_m3_joint_states(self,name,position):
        base = position[name.index('base_link_to_link1')]
        shoulder = position[name.index('link1_to_link2')]
        elbow =  position[name.index('link2_to_link3')]
        wrist =  position[name.index('link3_to_link4')]
        roll = position[name.index('link4_to_link5')] 
        hand =  position[name.index('link5_to_gripper_link')]
                
        radians = [base, shoulder, elbow, wrist, roll, hand]  
        return radians
        
    def joint_states_callback(self, msg):

        header = {
            'stamp': {
                'sec': msg.header.stamp.sec,
                'nanosec': msg.header.stamp.nanosec,
            },
            'frame_id': msg.header.frame_id,
        }
        
        name = msg.name
        position = msg.position
        velocity = msg.velocity
        effort = msg.effort

        switch_dict = {
            "roarm_m2": self.handle_m2_joint_states,
            "roarm_m3": self.handle_m3_joint_states,
        }
        radians = switch_dict[self.roarm_type](name,position)   

        if radians == self.last_roarm_sent_data:
            return
        self.last_roarm_sent_data = radians       

        try:
            self.roarm.joints_radian_ctrl(radians=radians,speed=1000,acc=50)
            self.roarm.feedback_get()
        except SerialException as e:
            self.get_logger().error(f"{e}")
            
    def led_ctrl_callback(self, msg):
        led = msg.data
        self.roarm.led_ctrl(led=led)
        
def main(args=None):
    rclpy.init(args=args)
    roarm_driver = RoarmDriver()
    rclpy.spin(roarm_driver)
    roarm_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

