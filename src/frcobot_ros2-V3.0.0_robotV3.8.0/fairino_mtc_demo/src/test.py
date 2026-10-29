import Robot

# Establish a connection with the robot controller and return a robot object if the connection is successful
ROBOT_IP_ADDRESS = '192.168.58.2'
robot = Robot.RPC(ROBOT_IP_ADDRESS)
status = 1
smooth = 0
block = 0

robot.SetDO(0,0,0,0)
robot.SetDO(1,1,0,0)