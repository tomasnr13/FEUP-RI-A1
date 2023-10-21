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
plt.show()
