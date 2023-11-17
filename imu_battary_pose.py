import rosbag
import datetime
bag_file = 'test2.bag'  # Replace with the path to your ROS bag file

# Define the topics and output file names
topics = ['/Quad13/Dynamics/IMU', '/mavros/battery', '/mavros/vision_pose/pose']
output_files = [f'{bag_file}_imu_data.txt', f'{bag_file}battery_data.txt', f'{bag_file}vision_pose.txt']

for topic, output_file in zip(topics, output_files):
    # Define the column headers for each topic
    if topic == '/Quad13/Dynamics/IMU':
        headers = ['Time', 'Angular Velocity X', 'Angular Velocity Y', 'Angular Velocity Z', 'Linear Acceleration X', 'Linear Acceleration Y', 'Linear Acceleration Z']

    elif topic == '/mavros/vision_pose/pose':
        headers = ['Time', 'Position X', 'Position Y', 'Position Z', 'Quaternion X', 'Quaternion Y', 'Quaternion Z', 'Quaternion W', 'Roll', 'Pitch', 'Yaw']

    with open(output_file, 'w') as file:
        bag = rosbag.Bag(bag_file)
        
        # Write the column headers to the file
        file.write('\t'.join(headers) + '\n')
        
        for _, msg, t in bag.read_messages(topics=[topic]):
            # Convert the time stamp to its original format
            time = t.to_nsec()
            timestamp_fraction = time / 10**4

            # Convert timestamp to datetime object
            dt = datetime.datetime.fromtimestamp(timestamp_fraction / 100000.0)

            # Convert datetime object to a formatted string with 1/10000th of a second precision
            formatted_time = dt.strftime('%Y-%m-%d %H:%M:%S.%f')
            # Extract the desired data from the message based on the topic
            if topic == '/Quad13/Dynamics/IMU':
                angular_velocity_x = msg.gyro_x
                angular_velocity_y = msg.gyro_y
                angular_velocity_z = msg.gyro_z
                linear_acceleration_x = msg.acc_x
                linear_acceleration_y = msg.acc_y
                linear_acceleration_z = msg.acc_z
                data = [time, angular_velocity_x, angular_velocity_y,angular_velocity_z, linear_acceleration_x, linear_acceleration_y, linear_acceleration_z]
            elif topic == '/mavros/vision_pose/pose':
                position_x = msg.pose.position.x
                position_y = msg.pose.position.y
                position_z = msg.pose.position.z
                quaternion_x = msg.pose.orientation.x
                quaternion_y = msg.pose.orientation.y
                quaternion_z = msg.pose.orientation.z
                quaternion_w = msg.pose.orientation.w
                data = [formatted_time, position_x, position_y, position_z,quaternion_x,quaternion_y,quaternion_z,quaternion_w]

            # Write the data to the file with appropriate spacing
            file.write('\t'.join(str(value) for value in data) + '\n')

        bag.close()
