import rosbag

# Specify the path to your rosbag file
bag_file = "test3.bag"

# Specify the topic you want to export
topic = "/Quad13/Dynamics/RotorSpeed"

# Specify the output text file
output_file = f"{bag_file}_RotorSpeed.txt"

# Open the rosbag file
bag = rosbag.Bag(bag_file)

# Create a dictionary to store the rotor speeds
rotor_speeds = {
    'time': [],
    'rotor1_speed': [],
    'rotor2_speed': [],
    'rotor3_speed': [],
    'rotor4_speed': []
}

# Iterate over the messages in the specified topic
for _, msg, t in bag.read_messages(topics=[topic]):
    # Append the rotor speeds to the dictionary4
    # time = t.to_nsec();

    # rotor_speeds['time'].append(time)
    rotor_speeds['rotor1_speed'].append(msg.rotor1_speed)
    rotor_speeds['rotor2_speed'].append(msg.rotor2_speed)
    rotor_speeds['rotor3_speed'].append(msg.rotor3_speed)
    rotor_speeds['rotor4_speed'].append(msg.rotor4_speed)

# Close the rosbag file
bag.close()

# Export the data to a text file
with open(output_file, 'w') as file:
    # Write the column titles
    # file.write("rotor1_speed, rotor2_speed, rotor3_speed, rotor4_speed\n")
    
    # Write the data rows
    for i in range(len(rotor_speeds['rotor1_speed'])):
        file.write(f"{rotor_speeds['time']},{rotor_speeds['rotor1_speed'][i]}, {rotor_speeds['rotor2_speed'][i]}, {rotor_speeds['rotor3_speed'][i]}, {rotor_speeds['rotor4_speed'][i]}\n")

print("Data exported successfully.")


