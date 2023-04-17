#!/usr/bin/env python

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')

    # Create the broadcaster
    broadcaster = tf.TransformBroadcaster()

    # Broadcast the transforms
    while not rospy.is_shutdown():
        # world to map transform
        #broadcaster.sendTransform((0, 0, 0), (0, 0, 0, 1),
        #                          rospy.Time.now(), "map", "world")

        # map to base_link transform
        broadcaster.sendTransform((0, 0, 0), (0, 0, 0, 1),
                                  rospy.Time.now(), "map", "base_link")

        # odom to map transform
        broadcaster.sendTransform((0, 0, 0), (0, 0, 0, 1),
                                  rospy.Time.now(), "odom", "map")

        rospy.sleep(0.01)