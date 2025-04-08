#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import math
from maze.srv import Finder

class ReactiveNavigator:
    def __init__(self):
        rospy.init_node('reactive_navigator')

        # --- Parameters ---
        # Navigation
        self.forward_speed = 10.0          # Max forward speed (m/s)
        self.turn_speed = 10.0              # Max turning speed (rad/s)
        self.goal_direction = 0.0           # Target direction in degrees (0 = straight ahead) Initially straight.
        self.obstacle_threshold = 0.35      # Consider sectors below this distance blocked (m) - Bifurcation/Scan trigger
        self.wall_follow_distance = 0.4     # Target distance for wall following (m)

        # VFH Parameters
        self.vfh_num_sectors = 72           # 360 / 5 degrees per sector
        self.vfh_valley_threshold = 5       # Min number of consecutive free sectors to form a valley
        self.vfh_robot_width_sectors = 3    # How many sectors robot width blocks (+/-)
        self.vfh_target_weight = 5.0        # How strongly to prefer valleys near the target
        self.vfh_width_weight = 1.0         # How strongly to prefer wider valleys

        # Controller Gains
        self.steer_gain = 1.0               # Proportional gain for VFH steering
        self.wall_follow_kp = 1.5           # Proportional gain for wall following fallback

        # State Machine
        self.state = 'NAVIGATING' # Initial state: NAVIGATING, SCANNING, WALL_FOLLOWING

        # Shared data
        self.latest_polar_histogram = np.full(self.vfh_num_sectors, float('inf'))
        self.lidar_ready = False
        self.current_twist = Twist() # Store the current command

        # --- ROS Setup ---
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/lidar_points', PointCloud2, self.lidar_callback, queue_size=1)

        # Service Client for Target Finder
        rospy.loginfo("Waiting for target finder service...")
        service_name = '/camera_controller/find_target'
        try:
            rospy.wait_for_service(service_name, timeout=15.0)
            self.find_target_service = rospy.ServiceProxy(service_name, Finder)
            rospy.loginfo("Target finder service connected.")
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(f"Failed to connect to target finder service: {e}")
            rospy.signal_shutdown("Target finder service not available")
            return # Exit if service not found

        rospy.loginfo("Navigator initialized. Starting in NAVIGATING state.")

    # --- Lidar and VFH Processing ---

    def lidar_callback(self, msg):
        """Processes lidar data, updates VFH histogram."""
        try:
            points_generator = pc2.read_points(msg, field_names=("x", "y"), skip_nans=True)
            self.update_vfh_histogram(points_generator)
            self.lidar_ready = True
        except Exception as e:
            rospy.logerr(f"Error in lidar_callback: {e}")
            self.lidar_ready = False # Mark as not ready if error

    def update_vfh_histogram(self, points):
        """Calculates the VFH polar histogram from lidar points."""
        histogram = np.full(self.vfh_num_sectors, float('inf'))
        sector_angle = 360.0 / self.vfh_num_sectors

        for x, y in points:
            dist = math.hypot(x, y)
            # Ignore points too close or too far for VFH calculation
            if dist < 0.05 or dist > 6.0:
                continue

            angle_rad = math.atan2(y, x)
            angle_deg = math.degrees(angle_rad)

            # Normalize angle to 0-360
            angle_deg_normalized = (angle_deg + 360.0) % 360.0

            sector_index = int(angle_deg_normalized / sector_angle)

            # Update histogram with minimum distance in sector
            histogram[sector_index] = min(histogram[sector_index], dist)

        self.latest_polar_histogram = histogram
        # Debug: Log histogram values occasionally
        # if rospy.Time.now().to_sec() % 5 < 0.1: # Log every 5 seconds
        #      rospy.loginfo(f"Hist Min: {np.min(self.latest_polar_histogram[np.isfinite(self.latest_polar_histogram)]):.2f}")

    def find_best_valley(self):
        """Analyzes histogram to find the best obstacle-free direction (valley)."""
        if not self.lidar_ready or not np.any(np.isfinite(self.latest_polar_histogram)):
            rospy.logwarn_throttle(5, "VFH: Histogram not ready or contains no valid data.")
            return None # Cannot find valley without data

        # 1. Create Binary Histogram (Blocked=1, Free=0)
        binary_hist = (self.latest_polar_histogram < self.obstacle_threshold).astype(int)

        # 2. Apply Masking for Robot Width (Widen obstacles)
        masked_hist = np.copy(binary_hist)
        blocked_indices = np.where(binary_hist == 1)[0]
        for idx in blocked_indices:
            for offset in range(-self.vfh_robot_width_sectors, self.vfh_robot_width_sectors + 1):
                masked_hist[(idx + offset) % self.vfh_num_sectors] = 1

        # 3. Find Valleys (contiguous sequences of 0s)
        valleys = []
        in_valley = False
        start_index = -1
        # Iterate through histogram twice to handle wrap-around
        hist_extended = np.concatenate((masked_hist, masked_hist))
        for i, sector_state in enumerate(hist_extended):
            current_index = i % self.vfh_num_sectors
            if sector_state == 0 and not in_valley: # Start of a new valley
                in_valley = True
                start_index = current_index
            elif (sector_state == 1 or i == len(hist_extended) - 1) and in_valley: # End of a valley
                in_valley = False
                end_index = (i - 1) % self.vfh_num_sectors
                # Calculate valley width considering wrap-around
                if end_index < start_index: # Wrapped around
                    width = (self.vfh_num_sectors - start_index) + end_index + 1
                else:
                    width = end_index - start_index + 1

                if width >= self.vfh_valley_threshold:
                     # Calculate center angle (handle wrap-around carefully)
                    if end_index < start_index:
                        center_offset = ((self.vfh_num_sectors - start_index) + end_index) / 2.0
                        center_index = (start_index + center_offset) % self.vfh_num_sectors
                    else:
                        center_index = (start_index + end_index) / 2.0

                    sector_angle_deg = 360.0 / self.vfh_num_sectors
                    center_angle = center_index * sector_angle_deg
                    # Convert center angle to -180 to 180 range for easier comparison
                    center_angle = (center_angle + 180) % 360 - 180
                    valleys.append({'center_angle': center_angle, 'width': width, 'start': start_index, 'end': end_index})

                # Break early if we have processed the first full circle and found the start again
                if i >= self.vfh_num_sectors and start_index != -1 and current_index == start_index:
                    break


        if not valleys:
            rospy.logwarn("VFH: No suitable valleys found.")
            return None

        # 4. Select Best Valley based on Cost
        best_valley = None
        min_cost = float('inf')

        for valley in valleys:
            # Cost = Weighted difference from target + Weighted inverse width
            angle_diff = abs(valley['center_angle'] - self.goal_direction)
            # Handle angle wrap around difference (e.g., diff between -170 and 170 is 20, not 340)
            angle_diff = min(angle_diff, 360.0 - angle_diff)

            cost = (self.vfh_target_weight * angle_diff +
                    self.vfh_width_weight * (self.vfh_num_sectors / valley['width'])) # Penalize narrow valleys more

            if cost < min_cost:
                min_cost = cost
                best_valley = valley

        # rospy.loginfo(f"VFH Best Valley: Angle={best_valley['center_angle']:.1f}, Width={best_valley['width']}") # Debug
        return best_valley['center_angle']

    # --- State Actions ---

    def execute_navigation(self):
        """Uses VFH to navigate towards goal_direction, avoiding obstacles."""
        best_angle_deg = self.find_best_valley()

        if best_angle_deg is None:
            # Trigger for potential bifurcation/dead end scan
            rospy.logwarn("NAVIGATING: No clear path found via VFH. Switching to SCANNING.")
            self.state = 'SCANNING'
            # Stop the robot immediately before scanning
            self.current_twist = Twist()
            self.cmd_pub.publish(self.current_twist)
            return # Exit this cycle, scanning starts next cycle

        # Calculate steering command
        angle_error_rad = math.radians(best_angle_deg) # VFH angle is relative to robot's front
        turn_command = self.steer_gain * angle_error_rad
        turn_command = np.clip(turn_command, -self.turn_speed, self.turn_speed)

        # Adjust forward speed based on how much we need to turn
        # More turn = less forward speed
        speed_scale = max(0.0, 1.0 - abs(turn_command) / self.turn_speed)
        forward_command = self.forward_speed * speed_scale

        # Further reduce speed based on clearance in the chosen direction (simplistic check)
        target_sector = int(((best_angle_deg + 360) % 360) / (360.0 / self.vfh_num_sectors))
        clearance = self.latest_polar_histogram[target_sector]
        if clearance < self.obstacle_threshold * 1.5: # If getting close
             forward_command *= (clearance / (self.obstacle_threshold * 1.5)) # Scale down speed
             forward_command = max(0.05, forward_command) # Ensure minimum motion if clear

        self.current_twist.linear.x = forward_command
        self.current_twist.angular.z = turn_command

    def execute_scan_sequence(self):
        """Stops robot and sequentially scans angles for the target."""
        rospy.loginfo("SCANNING: Initiating target scan sequence.")
        # Ensure robot is stopped
        self.current_twist = Twist()
        self.cmd_pub.publish(self.current_twist)
        rospy.sleep(0.2) # Short pause to ensure stop command is processed

        target_found = False
        found_angle = 0.0

        try:
            # Call the target finding service (this blocks until service returns)
            response = self.find_target_service() # Request scan
            target_found = response.found
            if target_found:
                found_angle = response.angle_degrees
                rospy.loginfo(f"SCANNING: Target found by service at {found_angle:.1f} degrees!")
            else:
                rospy.loginfo("SCANNING: Target not found by service during scan.")

        except rospy.ServiceException as e:
            rospy.logerr(f"SCANNING: Service call failed: {e}. Assuming target not found.")
            target_found = False
        except Exception as e:
             rospy.logerr(f"SCANNING: Unexpected error during service call: {e}")
             target_found = False


        # --- Decide Next State ---
        if target_found:
            rospy.loginfo("SCANNING: Target acquired. Setting goal direction and returning to NAVIGATING.")
            self.goal_direction = found_angle # Update the VFH target direction
            self.state = 'NAVIGATING'
             # Let VFH handle the turn and movement in the next NAVIGATING cycle
        else:
            rospy.loginfo("SCANNING: Target not found. Switching to WALL_FOLLOWING fallback.")
            self.state = 'WALL_FOLLOWING'
            # Reset goal direction to straight ahead for wall following / VFH if it recovers
            self.goal_direction = 0.0

        # The actual movement/turning will happen in the next cycle based on the new state/goal_direction

    def execute_wall_following(self):
        """Fallback strategy: Simple right-wall following."""
        if not self.lidar_ready:
            rospy.logwarn_throttle(5, "WALL_FOLLOWING: Lidar not ready, stopping.")
            self.current_twist = Twist()
            return

        # Check if a clear path forward opens up via VFH
        vfh_best_angle = self.find_best_valley()
        if vfh_best_angle is not None and abs(vfh_best_angle) < 45: # If a reasonably forward path exists
            rospy.loginfo("WALL_FOLLOWING: Clear path ahead detected by VFH. Switching back to NAVIGATING.")
            self.state = 'NAVIGATING'
            self.goal_direction = 0.0 # Reset goal to straight
            return # Let NAVIGATING take over next cycle

        # --- Simple Wall Following Logic ---
        # Get min distance in the right sector(s)
        right_start_sector = int(((-90 + 360) % 360) / (360.0 / self.vfh_num_sectors))
        right_end_sector = int(((-30 + 360) % 360) / (360.0 / self.vfh_num_sectors))
        if right_start_sector > right_end_sector: # Handle wrap around if sector definitions change
             right_indices = list(range(right_start_sector, self.vfh_num_sectors)) + list(range(0, right_end_sector + 1))
        else:
             right_indices = list(range(right_start_sector, right_end_sector + 1))

        valid_right_dists = self.latest_polar_histogram[right_indices]
        valid_right_dists = valid_right_dists[np.isfinite(valid_right_dists)]
        right_dist = np.min(valid_right_dists) if len(valid_right_dists) > 0 else float('inf')

        # Get min distance in the front sector(s)
        front_start_sector = int(((-45 + 360) % 360) / (360.0 / self.vfh_num_sectors))
        front_end_sector = int(((45 + 360) % 360) / (360.0 / self.vfh_num_sectors))
        if front_start_sector > front_end_sector:
            front_indices = list(range(front_start_sector, self.vfh_num_sectors)) + list(range(0, front_end_sector+1))
        else:
            front_indices = list(range(front_start_sector, front_end_sector+1))

        valid_front_dists = self.latest_polar_histogram[front_indices]
        valid_front_dists = valid_front_dists[np.isfinite(valid_front_dists)]
        front_dist = np.min(valid_front_dists) if len(valid_front_dists) > 0 else float('inf')


        # Priority: Avoid front collision
        if front_dist < self.obstacle_threshold * 1.2: # More sensitive threshold here
            rospy.loginfo("WALL_FOLLOWING: Obstacle Detected - Turning Left")
            self.current_twist.linear.x = 0.0
            self.current_twist.angular.z = self.turn_speed # Sharp left turn
        # Proportional control for right wall distance
        elif right_dist == float('inf') or right_dist > self.wall_follow_distance * 2.0:
             rospy.loginfo("WALL_FOLLOWING: Right wall lost, turning right gently.")
             self.current_twist.linear.x = self.forward_speed * 0.3 # Slow down
             self.current_twist.angular.z = -self.turn_speed * 0.4 # Gentle right turn
        else:
            error = self.wall_follow_distance - right_dist
            turn_cmd = self.wall_follow_kp * error
            turn_cmd = np.clip(turn_cmd, -self.turn_speed, self.turn_speed)

            fwd_speed_scale = max(0.1, 1.0 - abs(turn_cmd) / self.turn_speed)
            self.current_twist.linear.x = self.forward_speed * fwd_speed_scale * 0.7 # Generally slower in wall follow
            self.current_twist.angular.z = turn_cmd
            # rospy.loginfo(f"WallFollow: R_Dist={right_dist:.2f}, Err={error:.2f}, Turn={turn_cmd:.2f}, Fwd={self.current_twist.linear.x:.2f}") # Debug

    # --- Main Loop ---

    def run(self):
        """Main control loop managing states."""
        rate = rospy.Rate(10) # 10 Hz control loop

        while not rospy.is_shutdown():
            start_time = rospy.Time.now()

            if not self.lidar_ready:
                rospy.logwarn_throttle(2, "Waiting for Lidar data...")
                # Keep publishing zero twist if lidar isn't ready
                self.cmd_pub.publish(Twist())
                rate.sleep()
                continue

            # --- State Machine Logic ---
            if self.state == 'NAVIGATING':
                self.execute_navigation()
                # State transition to SCANNING happens inside execute_navigation
            elif self.state == 'SCANNING':
                # This state's action is blocking and handles its own transitions
                self.execute_scan_sequence()
                 # Ensure twist is potentially updated based on scan result for next cycle
                continue # Skip publishing twist this cycle, scan handled it or will next cycle
            elif self.state == 'WALL_FOLLOWING':
                self.execute_wall_following()
                # State transition back to NAVIGATING happens inside execute_wall_following
            else:
                rospy.logerr(f"Unknown state: {self.state}. Stopping.")
                self.current_twist = Twist()
                self.state = 'NAVIGATING' # Attempt recovery to default state

            # Publish the calculated Twist command
            self.cmd_pub.publish(self.current_twist)

            # Optional: Log loop duration
            # loop_duration = (rospy.Time.now() - start_time).to_sec()
            # rospy.loginfo(f"Loop time: {loop_duration:.4f}s, State: {self.state}")

            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                rospy.loginfo("ROSInterruptException received, shutting down.")
                break
            except Exception as e:
                rospy.logerr(f"Error in control loop sleep: {e}")


if __name__ == '__main__':
    try:
        nav = ReactiveNavigator()
        if nav.find_target_service: # Only run if service connection succeeded
            nav.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unhandled exception in ReactiveNavigator main: {e}")
    finally:
        # Ensure robot stops on exit
        rospy.loginfo("Navigator shutting down. Sending stop command.")
        # Create a publisher specifically for shutdown
        shutdown_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        stop_twist = Twist()
        # Publish multiple times to increase chance of receipt
        for _ in range(3):
            shutdown_pub.publish(stop_twist)
            rospy.sleep(0.05)