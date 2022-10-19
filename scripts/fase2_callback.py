import rospy
from geometry_msgs.msg import PoseStamped

class fase2_altura():
    def __init__(self):
        ########## Subscribers ##################
        self.local_atual = rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped, self.local_callback)
        rospy.loginfo("Trying to get altitude message")
        rospy.wait_for_message('/mavros/vision_pose/pose', PoseStamped)
        rospy.loginfo("Services are up")
        rospy.spin()
    
    def local_callback(self, data):
        print("Altitude em relação ao tubo: " str(data.pose.position.z) + "m")

if __name__ == "__main__":
    fase2_altura()