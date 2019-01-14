#!/usr/bin/python

import rospy
import roslaunch
import rospkg

def create_rags(**kargs):
  args = []
  for key in kargs:
    args.append( "%s:=%s" % ( key, kargs[key] ) )

  return args

def launch(package, launch_name, args = []):
  ros_master_id = rospy.get_param("/run_id")

  pkg_path = rospkg.RosPack().get_path( package )
  launch_path = "%s/launch/%s" % ( pkg_path, launch_name)

  loader = roslaunch.XmlLoader()
  config = roslaunch.ROSLaunchConfig()

  loader.load( launch_path, config, argv=args )

  runner = roslaunch.ROSLaunchRunner( ros_master_id, config )
  runner.launch()
