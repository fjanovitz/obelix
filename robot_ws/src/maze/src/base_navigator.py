import rospy
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import Twist
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Float64

class ReactiveNavigator:
    def __init__(self):
        rospy.init_node('reactive_navigator')

        # Parâmetros
        self.wall_follow_distance = 0.3  # metros
        self.forward_speed = 0.1         # m/s
        self.turn_speed = 0.5            # rad/s
        self.goal_color_lower = np.array([20, 100, 100])  # Ex: amarelo HSV
        self.goal_color_upper = np.array([30, 255, 255])

        # Controle de servo
        self.servo_pub = rospy.Publisher('/camera_controller.py', Float64, queue_size=10)
        self.servo_angles = [-60, -30, 0, 30, 60]  # Ângulos para escanear

        # Publicadores e subscritores
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/lidar_points', PointCloud2, self.lidar_callback)
        rospy.Subscriber('/camera/image_raw', Image, self.image_callback)

        # CV Bridge
        self.bridge = CvBridge()
        self.latest_image = None

        self.twist = Twist()
        self.state = 'WALL_FOLLOW'

    def lidar_callback(self, msg):
        points = list(pc2.read_points(msg, field_names=("x", "y"), skip_nans=True))
        distances = self.organize_lidar(points)

        if self.detect_bifurcation(distances):
            self.state = 'SCAN'
        else:
            self.wall_follow_strategy(distances)
            self.cmd_pub.publish(self.twist)

    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def organize_lidar(self, points):
        left, front, right = [], [], []
        for x, y in points:
            angle = np.degrees(np.arctan2(y, x))
            dist = np.hypot(x, y)
            if -30 <= angle <= 30:
                front.append(dist)
            elif 30 < angle <= 90:
                left.append(dist)
            elif -90 <= angle < -30:
                right.append(dist)
        return {
            'front': np.min(front) if front else float('inf'),
            'left': np.min(left) if left else float('inf'),
            'right': np.min(right) if right else float('inf')
        }

    def detect_bifurcation(self, distances):
        free_paths = sum([dist > 0.6 for dist in distances.values()])
        return free_paths >= 2

    def wall_follow_strategy(self, distances):
        right = distances['right']
        left = distances['left']

        if right < self.wall_follow_distance:
            self.twist.linear.x = 0.0
            self.twist.angular.z = -self.turn_speed
        elif left < self.wall_follow_distance:
            self.twist.linear.x = 0.0
            self.twist.angular.z = self.turn_speed
        else:
            self.twist.linear.x = self.forward_speed
            self.twist.angular.z = 0.0

    def scan_for_goal(self):
        if not self.latest_image:
            return None

        max_area = 0
        best_angle = None

        for angle in self.servo_angles:
            self.servo_pub.publish(Float64(np.radians(angle)))
            rospy.sleep(0.5)

            frame = self.latest_image
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.goal_color_lower, self.goal_color_upper)
            area = cv2.countNonZero(mask)

            if area > max_area:
                max_area = area
                best_angle = angle

        return best_angle

    def decide_direction(self, angle):
        if angle is None:
            self.twist.linear.x = self.forward_speed
            self.twist.angular.z = 0.0
        elif angle < -20:
            self.twist.linear.x = 0.0
            self.twist.angular.z = -self.turn_speed
        elif angle > 20:
            self.twist.linear.x = 0.0
            self.twist.angular.z = self.turn_speed
        else:
            self.twist.linear.x = self.forward_speed
            self.twist.angular.z = 0.0

        self.cmd_pub.publish(self.twist)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.state == 'SCAN':
                angle = self.scan_for_goal()
                self.decide_direction(angle)
                self.state = 'WALL_FOLLOW'
            rate.sleep()

if __name__ == '__main__':
    nav = ReactiveNavigator()
    nav.run()
