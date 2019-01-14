import rospy
from geometry_msgs.msg import Pose
from ar_track_alvar_msgs.msg import AlvarMarkers

class RealTracker(object):
  """
  RealTracker
  """

  def __init__(self, marker_id = -1, output_topic = None, marker_id_list = None, callback = None):
    self.marker_id = marker_id
    self.marker_id_list = marker_id_list if marker_id_list is not None else []
    self.output_topic = output_topic
    self.callback = callback

    self.publisher = None

  def marker_callback(self, marker_info):

    valid_list = []

    for marker in marker_info.markers:
      if (marker.id == self.marker_id) or (marker.id in self.marker_id_list):

        if self.publisher:
          self.publisher.publish( marker.pose.pose )

        valid_list.append( marker )

    if self.callback:
      self.callback( valid_list )

  def run(self):
    if self.output_topic:
      self.publisher = rospy.Publisher( self.output_topic, Pose, queue_size = 10 )

    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.marker_callback)
