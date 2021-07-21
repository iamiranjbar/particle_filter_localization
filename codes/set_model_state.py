import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

state_msg = ModelState()
state_msg.model_name = 'robot'
state_msg.pose.position.x = -0.2
state_msg.pose.position.y = 0.15
# state_msg.pose.position.x = 0.2
# state_msg.pose.position.y = 0.7
state_msg.pose.position.z = 0
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
