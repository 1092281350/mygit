import rosbag
import matplotlib.pyplot as plt
import numpy as np
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PointStamped
from scipy.interpolate import interp1d

def plot_gps_odometry_comparison1(bag_file, gps_topic, odom_topic,mpc_topic):
    # Initialize lists to hold GPS and Odometry data
    gps_x = []
    gps_y = []
    gps_z = []
    odom_x = []
    odom_y = []
    odom_z = []
    mpc_x = []
    mpc_y = []

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[gps_topic, odom_topic,mpc_topic]):
            if topic == gps_topic:
                gps_x.append(msg.point.x)
                gps_y.append(msg.point.y)
                gps_z.append(msg.point.z)
            elif topic == odom_topic:
                odom_x.append(msg.pose.pose.position.x)
                odom_y.append(msg.pose.pose.position.y)
                odom_z.append(msg.pose.pose.position.z)
            # elif topic == mpc_topic:
                # mpc_x.append(msg.points.transforms.translation.x)
                # mpc_y.append(msg.points.transforms.translation.y)

    # Convert lists to numpy arrays for easier manipulation
    gps_x = np.array(gps_x)
    gps_y = np.array(gps_y)
    gps_z = np.array(gps_z)
    odom_x = np.array(odom_x)
    odom_y = np.array(odom_y)
    odom_z = np.array(odom_z)
    mpc_x = np.array(mpc_x)
    mpc_y = np.array(mpc_y)

    # Create time arrays
    gps_time = np.arange(len(gps_x))
    odom_time = np.arange(len(odom_x))
    mpc_time= np.arange(len(mpc_x))

    # Plotting
    plt.figure(figsize=(15, 10))

    # GPS X vs. Odom X
    plt.subplot(3, 1, 1)
    plt.plot(gps_time, gps_x, label='GPS X')
    plt.plot(odom_time, odom_x, label='Odom X')
    # plt.plot(mpc_time, mpc_x, label='MPC X')
    plt.ylabel('X Coordinate')
    plt.legend()

    # GPS Y vs. Odom Y
    plt.subplot(3, 1, 2)
    plt.plot(gps_time, gps_y, label='GPS Y')
    plt.plot(odom_time, odom_y, label='Odom Y')
    # plt.plot(mpc_time, mpc_y, label='MPC Y')
    plt.ylabel('Y Coordinate')
    plt.legend()

# GPS Y vs. Odom Y
    plt.subplot(3, 1, 3)
    plt.plot(gps_time, gps_z, label='GPS Z')
    plt.plot(odom_time, odom_z, label='Odom Z')
    plt.ylabel('Z Coordinate')
    plt.legend()

    plt.tight_layout()
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
plot_imu_odometry_comparison('/home/qzh/catkin_ws/mp.bag', '/dji_sdk/imu', '/firefly/robot_fuse/odom','/dji_sdk/velocity')
plot_gps_odometry_comparison1('/home/qzh/catkin_ws/mp.bag', '/dji_sdk/local_position', '/firefly/robot_fuse/odom','/firefly/command/trajectory')