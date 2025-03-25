import rospy
import math
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from lidar_setup import setup_lidar, get_lidar_data

def publish_lidar():
    rospy.init_node("ydlidar_publisher", anonymous=True)
    pub = rospy.Publisher("/lidar_points", PointCloud2, queue_size=10)

    laser = setup_lidar()
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        points = get_lidar_data(laser)
        
        if points:
            header = rospy.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "laser_frame"

            # Cria mensagem PointCloud2
            cloud_msg = pc2.create_cloud_xyz32(header, points)
            pub.publish(cloud_msg)

        rate.sleep()

if __name__ == "__main__":
    try:
        publish_lidar()
    except rospy.ROSInterruptException:
        pass
