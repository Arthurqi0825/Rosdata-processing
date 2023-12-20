import rosbag

# Specify the path to your rosbag file
bag_file = 'test2.bag'
# Open the rosbag file
bag = rosbag.Bag(bag_file)

# Get the list of topics in the rosbag
topics = bag.get_type_and_topic_info()[1].keys()

# Print the topics
for topic in topics:
    print(topic)

# Close the rosbag file
bag.close()