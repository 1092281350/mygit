import rosbag
import matplotlib.pyplot as plt
import numpy as np
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PointStamped
from scipy.interpolate import interp1d
from trajectory_msgs.msg import MultiDOFJointTrajectory

def quaternion_normalize(q):
    n=np.linalg.norm(q)
    return q/ n if n >0 else q
def calculate_euclidean_error(bag_file, gps_topic, mpc_topic):
    # Initialize lists to hold GPS and MPC data and their timestamps
    gps_x, gps_y, gps_timestamps = [], [], []
    mpc_x, mpc_y, mpc_timestamps = [], [], []

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[gps_topic, mpc_topic]):
            timestamp = t.to_sec()
            if topic == gps_topic:
                gps_x.append(msg.point.x)
                gps_y.append(msg.point.y)
                gps_timestamps.append(timestamp)
            elif topic == mpc_topic:
                for point in msg.points:
                    if point.transforms:
                        mpc_x.append(point.transforms[0].translation.x)
                        mpc_y.append(point.transforms[0].translation.y)
                        mpc_timestamps.append(timestamp)

    # Convert lists to numpy arrays for interpolation
    gps_x, gps_y, gps_timestamps = np.array(gps_x), np.array(gps_y), np.array(gps_timestamps)
    mpc_x, mpc_y, mpc_timestamps = np.array(mpc_x), np.array(mpc_y), np.array(mpc_timestamps)

    # Interpolate GPS and MPC data to a common set of timestamps for comparison
    common_timestamps = np.linspace(max(min(gps_timestamps), min(mpc_timestamps)), min(max(gps_timestamps), max(mpc_timestamps)), num=max(len(gps_timestamps), len(mpc_timestamps)))
    gps_x_interp = interp1d(gps_timestamps, gps_x, kind='linear')(common_timestamps)
    gps_y_interp = interp1d(gps_timestamps, gps_y, kind='linear')(common_timestamps)
    mpc_x_interp = interp1d(mpc_timestamps, mpc_x, kind='linear')(common_timestamps)
    mpc_y_interp = interp1d(mpc_timestamps, mpc_y, kind='linear')(common_timestamps)

    # Calculate Euclidean distance for each point
    distances = np.sqrt((gps_x_interp - mpc_x_interp)**2 + (gps_y_interp - mpc_y_interp)**2)

    # Calculate total and average error
    total_error = np.sum(distances)
    average_error = np.mean(distances)

    print("Total Euclidean Error: {}".format(total_error))
    print("Average Euclidean Error: {}".format(average_error))
    return total_error, average_error


def plot_attitude_comparison2(bag_file, imu_topic, mpc_topic):
    # Initialize lists to hold IMU and MPC orientation data and their timestamps
    imu_roll, imu_pitch, imu_yaw, imu_timestamps = [], [], [], []
    mpc_roll, mpc_pitch, mpc_yaw, mpc_timestamps = [], [], [], []
    
    first_timestamp = None  # To store the first timestamp across all topics

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[imu_topic, mpc_topic]):
            # Convert ROS Time to seconds
            timestamp = t.to_sec()
            if first_timestamp is None:
                first_timestamp = timestamp  # Set the first timestamp
            
            # Calculate relative timestamp
            rel_timestamp = timestamp - first_timestamp
            
            if topic == imu_topic:
                # Convert quaternion to Euler angles
                orientation_q = msg.orientation
                euler = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
                imu_roll.append(euler[0])
                imu_pitch.append(euler[1])
                imu_yaw.append(euler[2])
                imu_timestamps.append(rel_timestamp)
            elif topic == mpc_topic:
                # Assuming each point only has one transform and using the first one for simplicity
                transform = msg.points[0].transforms[0]
                euler = euler_from_quaternion(quaternion_normalize([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]))
                mpc_roll.append(euler[0])
                mpc_pitch.append(euler[1])
                mpc_yaw.append(euler[2])
                mpc_timestamps.append(rel_timestamp)

    # Convert lists to numpy arrays
    imu_roll, imu_pitch, imu_yaw, imu_timestamps = map(np.array, (imu_roll, imu_pitch, imu_yaw, imu_timestamps))
    mpc_roll, mpc_pitch, mpc_yaw, mpc_timestamps = map(np.array, (mpc_roll, mpc_pitch, mpc_yaw, mpc_timestamps))

    # Plotting
    plt.figure(figsize=(15, 10))

    # Roll
    plt.subplot(3, 1, 1)
    plt.plot(imu_timestamps,np.degrees(imu_roll), label='IMU Roll')
    plt.plot(mpc_timestamps, mpc_roll, label='MPC Roll')
    plt.ylabel('Roll (degrees)')
    plt.legend()

    # Pitch
    plt.subplot(3, 1, 2)
    plt.plot(imu_timestamps, np.degrees(imu_pitch), label='IMU Pitch')
    plt.plot(mpc_timestamps,mpc_pitch, label='MPC Pitch')
    plt.ylabel('Pitch (degrees)')
    plt.legend()

    # Yaw
    # plt.subplot(3, 1, 3)
    # plt.plot(imu_timestamps, np.degrees(imu_yaw), label='IMU Yaw')
    # plt.plot(mpc_timestamps, np.degrees(mpc_yaw), label='MPC Yaw')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Yaw (degrees)')
    # plt.legend()

    plt.tight_layout()

    plt.savefig('attitude_comparison.jpg', format='jpg', dpi=300)

    plt.show()

def plot_gps_odometry_comparison1(bag_file, gps_topic, odom_topic, mpc_topic):
    # Initialize lists to hold GPS, Odometry, and MPC data and their timestamps
    gps_x, gps_y, gps_timestamps = [], [], []
    odom_x, odom_y, odom_timestamps = [], [], []
    mpc_x, mpc_y, mpc_timestamps = [], [], []

    first_timestamp = None  # To store the first timestamp across all topics

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[gps_topic, odom_topic, mpc_topic]):
            # Convert ROS Time to seconds
            timestamp = t.to_sec()
            if first_timestamp is None:
                first_timestamp = timestamp  # Set the first timestamp
            
            # Calculate relative timestamp
            rel_timestamp = timestamp - first_timestamp

            if topic == gps_topic:
                gps_x.append(msg.point.x)
                gps_y.append(msg.point.y)
                gps_timestamps.append(rel_timestamp)
            elif topic == odom_topic:
                odom_x.append(msg.pose.pose.position.x)
                odom_y.append(msg.pose.pose.position.y)
                odom_timestamps.append(rel_timestamp)
            elif topic == mpc_topic:
                # Assuming each point only has one transform
                for point in msg.points:
                    if point.transforms:  # Check if transforms are available
                        mpc_x.append(point.transforms[0].translation.x)
                        mpc_y.append(point.transforms[0].translation.y)
                        mpc_timestamps.append(rel_timestamp)  # Use the same timestamp for all points in a message

    # Convert lists to numpy arrays
    gps_x, gps_y, gps_timestamps = map(np.array, (gps_x, gps_y, gps_timestamps))
    odom_x, odom_y, odom_timestamps = map(np.array, (odom_x, odom_y, odom_timestamps))
    mpc_x, mpc_y, mpc_timestamps = map(np.array, (mpc_x, mpc_y, mpc_timestamps))

    # Plotting
    plt.figure(figsize=(15, 10))

    plt.subplot(2, 1, 1)
    plt.plot(gps_timestamps, gps_x, label='Truth X')
    # plt.plot(odom_timestamps, odom_x, label='Odom X')
    plt.plot(mpc_timestamps, mpc_x, label='Desired X')
    plt.xlabel('Time (s)')
    plt.ylabel('X Coordinate')
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(gps_timestamps, gps_y, label='Truth Y')
    # plt.plot(odom_timestamps, odom_y, label='Odom Y')
    plt.plot(mpc_timestamps, mpc_y, label='Desired Y')
    plt.xlabel('Time (s)')
    plt.ylabel('Y Coordinate')
    plt.legend()

    plt.tight_layout()
    plt.savefig("mpc.jpg",format="jpg",dpi=300)
    plt.show()

def plot_gps_odometry_comparison(bag_file, gps_topic, odom_topic):
    # Initialize lists to hold GPS and Odometry data
    gps_points = []
    odom_positions = []

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[gps_topic, odom_topic]):
            if topic == gps_topic:
                point = msg.point
                gps_points.append((point.x, point.y, point.z))
            elif topic == odom_topic:
                position = msg.pose.pose.position
                odom_positions.append((position.x, position.y, position.z))

    # Convert lists to numpy arrays for easier manipulation
    gps_points = np.array(gps_points)
    odom_positions = np.array(odom_positions)

    # Create time arrays
    gps_time = np.arange(len(gps_points))
    odom_time = np.arange(len(odom_positions))

    # Plotting
    plt.figure(figsize=(15, 10))

    # GPS Points
    plt.subplot(2, 1, 1)
    plt.plot(gps_time, gps_points[:, 0], label='GPS X (Longitude)')
    plt.plot(gps_time, gps_points[:, 1], label='GPS Y (Latitude)')
    plt.plot(gps_time, gps_points[:, 2], label='GPS Z (Altitude)')
    plt.ylabel('GPS Coordinates')
    plt.legend()

    # Odometry Position
    if len(odom_positions) > 0:
        plt.subplot(2, 1, 2)
        plt.plot(odom_time, odom_positions[:, 0], label='Odom X')
        plt.plot(odom_time, odom_positions[:, 1], label='Odom Y')
        plt.plot(odom_time, odom_positions[:, 2], label='Odom Z')
        plt.ylabel('Odometry Position (meters)')
        plt.legend()

    plt.tight_layout()
    plt.show()


def plot_imu_odometry_comparison(bag_file, imu_topic, odom_topic,vel_topic):
    # Initialize lists to hold IMU and Odometry data
    imu_times = []
    imu_orientations = []
    imu_angular_velocities = []
    vel_times =[]
    vel_linear_velocities = []
    odom_times = []
    odom_orientations = []
    odom_linear_velocities = []
    odom_angular_velocities = []

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[imu_topic, odom_topic,vel_topic]):
            time = t.to_sec()
            if topic == imu_topic:
                # Process IMU data
                imu_times.append(time)
                orientation = msg.orientation
                euler = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
                imu_orientations.append(euler)
                # vel_linear_velocities.append((msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z))
                imu_angular_velocities.append((msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z))
            elif topic == odom_topic:
                # Process Odometry data
                odom_times.append(time)
                orientation = msg.pose.pose.orientation
                euler = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
                odom_orientations.append(euler)
                odom_linear_velocities.append((msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z))
                odom_angular_velocities.append((msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z))
            elif topic == vel_topic:
                vel_times.append(time)
                vel_linear_velocities.append((msg.vector.x, msg.vector.y, msg.vector.z))

    # Convert lists to numpy arrays for easier manipulation
    imu_times = np.array(imu_times)
    imu_orientations = np.array(imu_orientations)
    vel_linear_velocities = np.array(vel_linear_velocities)
    imu_angular_velocities = np.array(imu_angular_velocities)
    vel_times=np.array(vel_times)
    odom_times = np.array(odom_times)
    odom_orientations = np.array(odom_orientations)
    odom_linear_velocities = np.array(odom_linear_velocities)
    odom_angular_velocities = np.array(odom_angular_velocities)


    common_times = np.linspace(max(imu_times[0], odom_times[0],vel_times[0]), min(imu_times[-1], odom_times[-1],vel_times[-1]), num=max(len(imu_times), len(odom_times),len(vel_times)))

    # Interpolation functions
    interpolate_imu_orientations = interp1d(imu_times, imu_orientations, axis=0, kind='linear', fill_value="extrapolate")
    interpolate_odom_orientations = interp1d(odom_times, odom_orientations, axis=0, kind='linear', fill_value="extrapolate")
    interpolate_vel_linear_velocities = interp1d(vel_times, vel_linear_velocities, axis=0, kind='linear', fill_value="extrapolate")
    interpolate_odom_linear_velocities = interp1d(odom_times, odom_linear_velocities, axis=0, kind='linear', fill_value="extrapolate")

    # Interpolated data
    imu_orientations_interp = interpolate_imu_orientations(common_times)
    odom_orientations_interp = interpolate_odom_orientations(common_times)
    vel_linear_velocities_interp = interpolate_vel_linear_velocities(common_times)
    odom_linear_velocities_interp = interpolate_odom_linear_velocities(common_times)

    # Plotting
    plt.figure(figsize=(15, 10))

    # IMU and Odometry Orientations
    plt.subplot(3, 2, 1)
    plt.plot(common_times, imu_orientations_interp[:, 0], label='IMU Roll')
    plt.plot(common_times, odom_orientations_interp[:, 0], label='Odom Roll')
    plt.ylabel('Roll (radians)')
    plt.legend()

    plt.subplot(3, 2, 3)
    plt.plot(common_times, imu_orientations_interp[:, 1], label='IMU Pitch')
    plt.plot(common_times, odom_orientations_interp[:, 1], label='Odom Pitch')
    plt.ylabel('Pitch (radians)')
    plt.legend()

    plt.subplot(3, 2, 5)
    plt.plot(common_times, imu_orientations_interp[:, 2], label='IMU Yaw')
    plt.plot(common_times, odom_orientations_interp[:, 2], label='Odom Yaw')
    plt.ylabel('Yaw (radians)')
    plt.legend()

    # IMU and Odometry Linear Velocities
    plt.subplot(3, 2, 2)
    plt.plot(common_times, vel_linear_velocities_interp[:, 0], label='IMU Linear Velocity X')
    plt.plot(common_times, odom_linear_velocities_interp[:, 0], label='Odom Linear Velocity X')
    plt.ylabel('Linear Velocity X (m/s)')
    plt.legend()

    plt.subplot(3, 2, 4)
    plt.plot(common_times, vel_linear_velocities_interp[:, 1], label='IMU Linear Velocity Y')
    plt.plot(common_times, odom_linear_velocities_interp[:, 1], label='Odom Linear Velocity Y')
    plt.ylabel('Linear Velocity Y (m/s)')
    plt.legend()

    plt.subplot(3, 2, 6)
    plt.plot(common_times, vel_linear_velocities_interp[:, 2], label='IMU Linear Velocity Z')
    plt.plot(common_times, odom_linear_velocities_interp[:, 2], label='Odom Linear Velocity Z')
    plt.ylabel('Linear Velocity Z (m/s)')
    plt.legend()

    plt.tight_layout()
    plt.show()
# Example usage
# plot_imu_odometry_comparison('/home/qzh/catkin_ws/mp.bag', '/dji_sdk/imu', '/firefly/robot_fuse/odom','/dji_sdk/velocity')
final_bag='/home/qzh/catkin_ws/ha_mpc.bag'
plot_gps_odometry_comparison1(final_bag, '/dji_sdk/local_position', '/firefly/robot_fuse/odom','/firefly/command/trajectory')
plot_attitude_comparison2(final_bag, '/dji_sdk/imu','/firefly/command/trajectory')
calculate_euclidean_error(final_bag, '/dji_sdk/local_position', '/firefly/command/trajectory')