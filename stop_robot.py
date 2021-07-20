import rospy
import time
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

state_msg = ModelState()
state_msg.model_name = 'robot'
state_msg.pose.position.x = -0.2
state_msg.pose.position.y = 0.15
state_msg.pose.position.z = 0.1
state_msg.pose.orientation.x = 0
state_msg.pose.orientation.y = 0
state_msg.pose.orientation.z = 0
state_msg.pose.orientation.w = 0

rospy.wait_for_service('/gazebo/set_model_state')

try:
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    resp = set_state(state_msg)
except e:
    print ("Service call failed:" , e)


vel_msg = Twist()

vel_msg.linear.x = 0
vel_msg.linear.y = 0
vel_msg.linear.z = 0
vel_msg.angular.x = 0
vel_msg.angular.y = 0
vel_msg.angular.z = 0

rospy.init_node('vector_controller', anonymous=True)
velocity_publisher = rospy.Publisher('/vector/cmd_vel', Twist, queue_size=10)
for i in range(10):
    velocity_publisher.publish(vel_msg)
    time.sleep(0.01)
