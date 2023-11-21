import matplotlib.pyplot as plt

# Specify the path to your exported data text file
data_file = '/home/zhangziqi/Desktop/Rosdata/Rosdata-processing/test2.bag_RotorSpeed.txt'


# Read the data from the text file
data = []

# Parameters
div = 100*60/16384 #unit change of rotor speed
kv = 1.473e-8 # thrust perameters

# Open file and convert into float
with open(data_file, 'r') as file:
    for line in file:
        values = line.strip().split(', ')
        data.append([(float(value)*div) for value in values])


# Extract the rotor speeds and calculate Ri^2 for each row
rotor1_speeds = [row[0] for row in data]
rotor2_speeds = [row[1] for row in data]
rotor3_speeds = [row[2] for row in data]
rotor4_speeds = [row[3] for row in data]


#  get the thrust data by Kv*ri^2
ri_squared = [r1**2 + r2**2 + r3**2 + r4**2 for r1, r2, r3, r4 in data]
r1_thrust = [kv* i**2 for i in rotor1_speeds]
r2_thrust = [kv* i**2 for i in rotor2_speeds]
r3_thrust = [kv* i**2 for i in rotor3_speeds]
r4_thrust = [kv* i**2 for i in rotor4_speeds]

total_thrust = [kv* i for i in ri_squared]

# Create separate subplots for each rotor
fig, axs = plt.subplots(5, 1, figsize=(8, 12))

# Plot rotor 1
axs[0].plot(r1_thrust)
axs[0].set_ylabel('Rotor 1 Speed')

# Plot rotor 2
axs[1].plot(r2_thrust)
axs[1].set_ylabel('Rotor 2 Speed')

# Plot rotor 3
axs[2].plot(r3_thrust)
axs[2].set_ylabel('Rotor 3 Speed')

# Plot rotor 4
axs[3].plot(r4_thrust)
axs[3].set_ylabel('Rotor 4 Speed')

# Plot the sum of squares
axs[4].plot(total_thrust, color='red')
axs[4].set_ylabel('Thrust')

# Adjust the layout
plt.tight_layout()

# Show the plot
plt.show()