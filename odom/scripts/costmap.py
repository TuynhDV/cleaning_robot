#!/usr/bin/env python

import rospy
from dynamic_reconfigure.srv import Reconfigure, ReconfigureRequest
from dynamic_reconfigure.msg import Config, BoolParameter
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
import time

class CostmapClearer:
    def __init__(self):
        rospy.init_node('costmap_clearer_node', anonymous=True)
        
        # Subscribe to the move_base_simple/goal topic
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
        rospy.loginfo("CostmapClearer node initialized.")

    def toggle_obstacle_layer(self, enable):
        """Bật hoặc tắt obstacle layer."""
        rospy.loginfo("Setting obstacle layer to %s", "ENABLED" if enable else "DISABLED")
        service_name = '/move_base/global_costmap/obstacle_layer/set_parameters'
        
        try:
            # Đợi service sẵn sàng
            rospy.wait_for_service(service_name, timeout=5)
            
            # Khởi tạo service proxy
            dynamic_reconfig_costmap = rospy.ServiceProxy(service_name, Reconfigure)

            # Tạo yêu cầu dynamic reconfigure
            reconfigure_req = ReconfigureRequest()
            bool_param = BoolParameter()
            bool_param.name = "enabled"
            bool_param.value = enable

            conf = Config()
            conf.bools.append(bool_param)
            reconfigure_req.config = conf

            # Gọi dịch vụ để cập nhật trạng thái của obstacle layer
            dynamic_reconfig_costmap(reconfigure_req)
            rospy.loginfo("Obstacle Layer successfully %s", "enabled" if enable else "disabled")
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call %s: %s" % (service_name, e))
        except rospy.ROSException as e:
            rospy.logerr("Service %s not available: %s" % (service_name, e))

    def goal_callback(self, msg):
        rospy.loginfo("Received new goal. Clearing obstacle layers...")

        # Tắt obstacle layer để làm sạch costmap
        self.toggle_obstacle_layer(False)

        # Đợi một chút để costmap có thể được làm sạch
        rospy.sleep(1)

        # Bật lại obstacle layer sau khi đã làm sạch
        self.toggle_obstacle_layer(True)

if __name__ == '__main__':
    try:
        costmap_clearer = CostmapClearer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
