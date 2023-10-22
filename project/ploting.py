import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Read the CSV file
df = pd.read_csv('odometry.csv')

# Convert the pandas series to numpy arrays before plotting
x = np.array(df['x'])
y = np.array(df['y'])
time = np.array(df['time'])

# Create the plot
plt.figure(figsize=(10,5))
plt.scatter(x, y, c=time, cmap='viridis')

# Set the range of x and y axes
plt.xlim(-10, 15)
plt.ylim(-25, 13)

# Add a colorbar
cbar = plt.colorbar()
cbar.set_label('Time')

# Add labels and title
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Graph of y vs x with Time as color')

# Show the plot
plt.savefig('data_image.png')
plt.show()

# ----------------------------
# Calculate total time elapsed
total_time_elapsed = df['time'].iloc[-1] - df['time'].iloc[0]

# Calculate total distance traveled
df['distance_traveled'] = np.sqrt(df['x'].diff()**2 + df['y'].diff()**2)
total_distance_traveled = df['distance_traveled'].sum()

# Calculate mean velocity for the entire journey
mean_velocity = total_distance_traveled / total_time_elapsed

# Create a new DataFrame to store these values
data = pd.DataFrame({
    'total_time_elapsed': [total_time_elapsed],
    'total_distance_traveled': [total_distance_traveled],
    'mean_velocity': [mean_velocity]
})


# Write the new data to a CSV file
data.to_csv('data_info.csv', index=False)
