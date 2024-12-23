import matplotlib.pyplot as plt

# File path
file_path = 'trajectory_data_processed.txt'

# Lists to hold x and y coordinates
x_coords = []
y_coords = []

# Read and parse the file
with open(file_path, 'r') as file:
    lines = file.readlines()
    for i, line in enumerate(lines):
        if line.strip().startswith('x:'):
            x = float(line.split(':')[1].strip())
            y = float(lines[i + 1].split(':')[1].strip())
            x_coords.append(x)
            y_coords.append(y)

# Plot the trajectory as dots
plt.figure(figsize=(8, 6))
plt.scatter(x_coords, y_coords, c='blue', s=10)  # Just dots
plt.title('2D Trajectory (Positions)')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.grid(True)
plt.show()

