#!usr/bin/env python3

import rospy
from gazebo_msgs.srv import GetLinkState, GetModelState
from gazebo_msgs.msg import LinkState
from geometry_msgs.msg import Pose, TwistStamped
from std_msgs.msg import Bool

class getLinkState():
    def __init__(self, link_name, ref_frame):
        self.link_name = link_name
        self.ref_frame = ref_frame
        self.link_state = LinkState()
        
    def get_link_state(self):
        try: 
            get_link = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
            self.link_state = get_link(self.link_name, self.ref_frame).link_state
            return self.link_state.pose.position.x, self.link_state.pose.position.y, self.link_state.pose.position.z, \
                    self.link_state.twist.angular.x, self.link_state.twist.angular.y, self.link_state.twist.angular.z
        
        except rospy.ServiceException as e:
            rospy.loginfo("Get Link State service call failed:  {0}".format(e))
            pass
        
if __name__ == '__main__':
    try:
        rospy.init_node('get_link_state_node')
        body_coordinate_pub = rospy.Publisher('/body_pose', Pose, queue_size=10)
        body_velocity_pub = rospy.Publisher('/body_scalar', TwistStamped, queue_size=10)
        body_state_pub = rospy.Publisher('/state', Bool, queue_size=10)
        link_pose = Pose()
        link_omega_vel = TwistStamped()
        link_state = Bool()
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            getLink = getLinkState("base_link", "world")
            x_pos, y_pos, z_pos, w_x, w_y, w_z = getLink.get_link_state()
            
            # Publishing /body_pose topic
            link_pose.position.x = x_pos
            link_pose.position.y = y_pos
            link_pose.position.z = z_pos
            body_coordinate_pub.publish(link_pose)
            
            # Publishing /body_scalar topic
            link_omega_vel.twist.angular.x = w_x
            link_omega_vel.twist.angular.y = w_y
            link_omega_vel.twist.angular.z = w_z
            body_velocity_pub.publish(link_omega_vel)
            
            # Publishing /state topic
            if 0.0 < z_pos < 0.06:
                link_state.data = False
            elif 0.06 < z_pos < 0.15: 
                link_state.data = True
            else: 
                link_state.data = False
                
            body_state_pub.publish(link_state)
            
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass