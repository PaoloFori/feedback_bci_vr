#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def main():
    rospy.init_node('unity_scene_manager')

    target_scene = rospy.get_param('~paradigm', 'Menu')

    pub = rospy.Publisher('/vr/load_scene', String, queue_size=1, latch=True)
    
    rospy.loginfo(f"[SCENE MANAGER] Initial paradigm: {target_scene}")
    
    rospy.sleep(0.5)
    pub.publish(target_scene)
    
    rospy.spin()

if __name__ == '__main__':
    main()