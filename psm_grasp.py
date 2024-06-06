import rospy
import math
from geometry_msgs.msg import PoseStamped 
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
import numpy as np

class PSM:
    def __init__(self):
        # Initialize the node
        rospy.init_node("Grasp", anonymous=True)
        
        # Define the publisher
        self.servoCp_pose_pub = rospy.Publisher("/PSM1/servo_cp", TransformStamped, queue_size=10)
        self.servoJp_pose_pub = rospy.Publisher("/PSM1/jaw/servo_jp", JointState, queue_size=10)

        # Define the subscriber for CP
        self.cp_pose_sub = rospy.Subscriber("/PSM1/measured_cp", PoseStamped, self.cb_pose)
        
        # Define the subscriber for the gripper 
        self.js_pose_sub = rospy.Subscriber("/PSM1/jaw/measured_js", JointState, self.cb_jaw)
        
        self.measured_cp = PoseStamped()
        self.measured_jaw = JointState()
        rospy.sleep(1)  # Give time for publishers/subscribers to initialize
        
    def cb_pose(self, msg):
        self.measured_cp = msg
        rospy.loginfo("Received measured_cp: %s", msg)
        
    def cb_jaw(self, msg):
        self.measured_jaw = msg
        rospy.loginfo("Received measured_jaw: %s", msg)

    def move_tcp_tp(self, target, v, dt):
        pos_current_np = np.array([self.measured_cp.pose.position.x,
                                   self.measured_cp.pose.position.y,
                                   self.measured_cp.pose.position.z])
        
        pos_target_np = np.array(target)
        d = np.linalg.norm(pos_current_np - pos_target_np)
        T = d / v
        N = int(math.floor(T / dt))

        X = np.linspace(pos_current_np[0], pos_target_np[0], N)
        Y = np.linspace(pos_current_np[1], pos_target_np[1], N)
        Z = np.linspace(pos_current_np[2], pos_target_np[2], N)
        
        rate = rospy.Rate(1.0 / dt)

        for i in range(N):
            if rospy.is_shutdown():
                break 
            p = TransformStamped()
            p.header.stamp = rospy.Time.now()
            p.header.frame_id = "world"
            p.transform.translation.x = X[i]
            p.transform.translation.y = Y[i]
            p.transform.translation.z = Z[i]
            
            rospy.loginfo("Publishing pose: %s", p)
            self.servoCp_pose_pub.publish(p)
            rate.sleep()


    def move_jaw_to(self, target, omega, dt):

        d = abs (target - self.measured_jaw.position[0])
        T = d / omega
        N = int(math.floor(T / dt))

        X = np.linspace( self.measured_jaw.position[0], target[0], N)

        
        rate = rospy.Rate(1.0 / dt)

        for i in range(N):
            if rospy.is_shutdown():
                break 
            
            j = JointState()
            j.position[0] = X[i]

            self.servoJp_pose_pub.publish(j)
            rate.sleep()

if __name__ == "__main__":
    psm = PSM()
    rospy.sleep(1)
#    psm.move_tcp_tp([0.0, 0.05, -0.12], 0.01, 1)
    psm.move_jaw_to(0.0,0.1,0.01)
