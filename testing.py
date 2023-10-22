import subprocess
import random
import yaml
import math

def random_point_in_circle(center, radius):
    # Generate random angle and radius within the circle
    angle = random.uniform(0, 2 * math.pi)
    r = math.sqrt(random.uniform(0, 1)) * radius

    # Convert to Cartesian coordinates
    x = center[0] + r * math.cos(angle)
    y = center[1] + r * math.sin(angle)

    return [x, y]

def modify_yaml_and_execute():
    with open('./src//world/turtle.model.yaml', 'r') as file:
        yaml_data = yaml.safe_load(file)

    circle = random_point_in_circle([3,0], 3)

    yaml_data['bodies'][0]['pose'][0] = circle[0] #center 3, min -1 max 7
    yaml_data['bodies'][0]['pose'][1] = circle[1] #center 0 min -4 max 4

    with open('./src//world/turtle.model.yaml', 'w') as file:
        yaml.dump(yaml_data, file)

    subprocess.run(['colcon', 'build'])
    subprocess.run(['source', 'install/setup.bash'], shell=True)
    subprocess.run(['ros2', 'launch', 'turtle', 'turtle.launch.py'])

if __name__ == "__main__":
    modify_yaml_and_execute()
