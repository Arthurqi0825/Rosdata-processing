import rosbag

# Specify the path to your rosbag file
bag_file = "test2.bag"

# Specify the topic you want to inspect
topic = "/Quad13/Dynamics/RotorSpeed"

# Open the rosbag file
bag = rosbag.Bag(bag_file)

# Iterate over the messages in the specified topic
for _, msg, _ in bag.read_messages(topics=[topic]):
    # Print the raw message data
    print("Raw Message Data:")
    print(msg)
    print("")

# Close the rosbag file
bag.close()