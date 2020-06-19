#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Path
import copy

from apriltag_msgs.msg import ApriltagPoseStamped
from std_srvs.srv import Trigger, TriggerResponse
from mav_manager.srv import Vec4, Vec4Request

def add_two_ints_server():

    rospy.spin()

class TagSearch():
  def __init__(self):
    rospy.init_node('tag_search')
    self.trigger_search = False
    self.tag_sub = rospy.Subscriber("hires/apriltag_pose_estimator/apriltag_poses", ApriltagPoseStamped, self.tag_cb)
    self.srv = rospy.Service('tag_search', Trigger, self.tag_trigger)
    tag_id = rospy.get_param('~tag_id', 0)
    self.req_tag_id = tag_id
    rospy.loginfo("Requested tag id %d", tag_id)

  def tag_trigger(self, req):
    try:
      set_vel_client = rospy.ServiceProxy('mav_services/setDesVelInWorldFrame', Vec4)
      vec4_goal = Vec4Request()
      vec4_goal.goal = [0, 0, 0, 0.4]
      resp1 = set_vel_client(vec4_goal)

      rospy.loginfo('Searching tag by yawing')
      self.trigger_search = True

      resp = TriggerResponse()
      resp.success = True
      return resp
    except rospy.ServiceException, e:
      rospy.logerr("Service call failed: %s", e)
      resp = TriggerResponse()
      resp.success = False
      return resp

  def tag_cb(self, data):

    if not self.trigger_search:
      return

    for tag in data.apriltags:
      if self.req_tag_id == tag.id:
        rospy.loginfo('Found tag')
        self.trigger_search = False

        try:
          set_vel_client = rospy.ServiceProxy('mav_services/setDesVelInWorldFrame', Vec4)
          vec4_goal = Vec4Request()
          vec4_goal.goal = [0, 0, 0, 0.0]
          resp1 = set_vel_client(vec4_goal)
          rospy.loginfo('Setting zero velocity')
        except rospy.ServiceException, e:
          rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
  TagSearch()
  rospy.spin()