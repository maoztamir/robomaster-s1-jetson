"""Accept only fresh, healthy RTAB odometry pairs; set provisional noise floors."""
import math
import rclpy
import message_filters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from rtabmap_msgs.msg import OdomInfo


class Gates(Node):
    def __init__(self):
        super().__init__('fusion_input_gates')
        self.inputs=[]
        for kind in ('lidar','visual'):
            pub=self.create_publisher(Odometry, '/fusion/'+kind+'/accepted',10)
            odom=message_filters.Subscriber(self,Odometry,'/fusion/'+kind+'/odom',qos_profile=qos_profile_sensor_data)
            info=message_filters.Subscriber(self,OdomInfo,'/fusion/'+kind+'/odom_info',qos_profile=qos_profile_sensor_data)
            sync=message_filters.TimeSynchronizer([odom,info],10)
            sync.registerCallback(lambda o,i,k=kind,p=pub:self.receive(k,p,o,i))
            self.inputs.append((odom,info,sync))

    def receive(self,kind,pub,odom,info):
        age=(self.get_clock().now().nanoseconds-(odom.header.stamp.sec*10**9+odom.header.stamp.nanosec))/1e9
        quality=info.inliers>=20 if kind=='visual' else info.icp_inliers_ratio>=.3
        q=odom.pose.pose.orientation
        pose=odom.pose.pose.position
        cov=list(odom.pose.covariance)
        values=[pose.x,pose.y,pose.z,q.x,q.y,q.z,q.w]+cov
        if (info.lost or not quality or not -.1<=age<=1.5 or
            not all(math.isfinite(v) for v in values) or
            not .9 <= q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w <=1.1 or
            any(cov[i]<0 for i in (0,7,35))):
            return
        # Floors are conservative initial settings, not measured uncertainty.
        for index in (0,7,35):cov[index]=max(cov[index],.0025 if kind=='lidar' else .01)
        odom.pose.covariance=cov
        pub.publish(odom)


def main():
    rclpy.init();node=Gates()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
