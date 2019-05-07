from MapLib_python3 import Map2D
from RobotDrawer_python3 import plot_log_with_map_path

map_path = "./maps/mapa3.txt"
myMap = Map2D(map_path)

goal_x = 5
goal_y = 3

myMap.fillCostMatrix([goal_x, goal_y])
route = myMap.planPath([0, 0], [goal_x, goal_y])

plot_log_with_map_path(route, map_path)
