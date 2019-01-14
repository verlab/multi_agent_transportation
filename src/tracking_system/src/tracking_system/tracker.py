
import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates

from system_manager.srv import *

class SimulationTracker(object):

  def __init__(self, model_name, output_topic):
    self.model_name = model_name
    self.output_topic = output_topic

    self.publisher = None
    self.current_pose = None

    self.scale = rospy.get_param("/world_scale", 1)

  def model_state_handler(self, data):
    if self.model_name in data.name:
      idx = data.name.index( self.model_name )

      self.current_pose = data.pose[idx]

      self.publisher.publish( self.current_pose )

  def handler_service_current_pose_scalled(self, data):
    res = CurrentPoseScalledResponse()

    pose = self.current_pose.position

    res.x = round( pose.x / self.scale )
    res.y = round( pose.y / self.scale )

    return res

  def run(self):
    # Create the publisher to output the pose
    self.publisher = rospy.Publisher( self.output_topic, Pose, queue_size = 1 )

    # Subscribe to gazebo output model states
    rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_state_handler)

    rospy.Service( "current_pose_scalled", CurrentPoseScalled, self.handler_service_current_pose_scalled )
