#!/usr/bin/env python
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    broadcaster = tf.TransformBroadcaster()

    rate = rospy.Rate(10) # 10 Hz

    while not rospy.is_shutdown():
        # Get the current time
        now = rospy.Time.now()

        # Define the transform from base_link to map
        broadcaster.sendTransform(
            (0, 0, 0), # Translation
            (0, 0, 0, 1), # Rotation
            now,
            "base_link",
            "map"
        )

        rate.sleep()