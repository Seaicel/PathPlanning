from numpy.core.numeric import full
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from path.msg import OccupancyGrid
from path.msg import pointArray
import octmap
from surface import Surface
import numpy as np
import DstarLite as ds
import LocalTarget
import open3d as o3d

pub = None

def callback(data):
    global pub
    width = data.info.width
    height = data.info.height
    map_grid = np.ndarray(shape=(height, width), buffer = np.array(data.data, dtype=float))
    cur_pos = [data.info.origin.position.x, data.info.origin.position.y]

    map_grid[(0 <= map_grid) & (map_grid < 50)] = 0
    map_grid[map_grid >= 50] = 1
    map_grid[map_grid == -1] = 2
    print(map_grid)
    target = LocalTarget.cal_local_target(map_grid, int(cur_pos[0]), int(cur_pos[1]), None)

    print(cur_pos, "---->", target)

    path_grid = map_grid.copy()
    path_grid[(path_grid == 1) | (path_grid == 2)] = np.inf

    path, sensed_map, updated_points = ds.Main(path_grid, int(target[0]), int(target[1]), int(cur_pos[0]), int(cur_pos[1]))

    print("move to ", cur_pos, path)
    cur_pos = target
    point_msg = pointArray()
    point_msg.cur_pos = Point(cur_pos[0], cur_pos[1], 0)
    point_msg.points = [Point(p[0], p[1], 0) for p in path]
    pub.publish(point_msg)


def planning():
    global pub
    rospy.init_node('plan', anonymous=True)

    rospy.Subscriber('points', OccupancyGrid, callback)
    pub = rospy.Publisher('cur_pos', pointArray, queue_size=2)

    rospy.spin()



if __name__ == '__main__':
    planning()
