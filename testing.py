import subprocess

constants_code_template = """
# Constants
MAX_LIN_VEL = {}
MAX_ANG_VEL = {}
LIN_ACC = {}
LIN_DEC = {}
ANG_ACC = {}
ANG_DEC = {}
MIN_DIST_FROM_WALL = {}
LASER_RANGE = {}
LASER_FREQ = {}
STOP_WALL_LEN = {}
STOP_TOLERANCE = {}
FILENAME = {}
"""

def write_constants_and_execute():

    counter = 0

    MAX_LIN_VEL = 2.0
    MAX_ANG_VEL = 3.0
    LIN_ACC = 1.5
    LIN_DEC = 3.0
    ANG_ACC = 6.0
    ANG_DEC = 6.0
    MIN_DIST_FROM_WALL = 1.0
    LASER_RANGE = 3
    LASER_FREQ = 10
    STOP_WALL_LEN = 4.18
    STOP_TOLERANCE = 0.1

    for i in range(5):
        counter += 1
        FILENAME = f"\'odometry_{MAX_LIN_VEL}_{MAX_ANG_VEL}_{LIN_ACC}_{LIN_DEC}_{ANG_ACC}_{ANG_DEC}_{MIN_DIST_FROM_WALL}_{LASER_RANGE}_{LASER_FREQ}_{STOP_WALL_LEN}_{STOP_TOLERANCE}.csv\'"
        constants_code = constants_code_template.format(MAX_LIN_VEL, MAX_ANG_VEL, LIN_ACC, LIN_DEC, ANG_ACC, ANG_DEC, MIN_DIST_FROM_WALL, LASER_RANGE, LASER_FREQ, STOP_WALL_LEN, STOP_TOLERANCE, FILENAME)

        with open('./src/c_turtle/params.py', 'w') as f:
            f.write(constants_code)

        subprocess.run(['colcon', 'build'])
        subprocess.run(['source', 'install/setup.bash'], shell=True)
        subprocess.run(['ros2', 'launch', 'c_turtle', 'c_turtle.launch.py'])

write_constants_and_execute()