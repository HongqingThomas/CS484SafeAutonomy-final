# import carla
# import time
# import numpy as np
# import math
import matplotlib.pyplot as plt
import numpy as np
import math
import random

from shapely.geometry import Polygon, Point



class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None


class RrtStar:

    # def __init__(self, x_start, x_goal, step_len,
    #              goal_sample_rate, search_radius, iter_max, delta, boundary, obstacle):
    #TODO：为了debug修改了boundary
    def __init__(self, x_start, x_goal, step_len,
                 goal_sample_rate, search_radius, iter_max, delta, boundary, obstacle):

        # comment: x_start = input waypoint[0]
        # comment: x_goal = input waypoint[-1]
        self.s_start = Node(x_start)
        self.s_goal = Node(x_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
        self.iter_max = iter_max
        self.vertex = [self.s_start]
        self.path = []
        self.delta = delta

        self.boundary = boundary

        # TODO:为了debug修改了
        self.poly = Polygon([[self.boundary[0][0][0], self.boundary[0][0][1]],
                             [self.boundary[0][10][0], self.boundary[0][10][1]],
                             [self.boundary[0][-1][0], self.boundary[0][-1][1]],
                             [self.boundary[1][-1][0], self.boundary[1][-1][1]],
                             [self.boundary[1][10][0], self.boundary[1][10][1]],
                             [self.boundary[1][0][0], self.boundary[1][0][1]],
                             [self.boundary[0][0][0], self.boundary[0][0][1]]])
        # # TODO:修改了
        # self.poly = Polygon([[self.boundary[0][0].transform.location.x + self.delta*(self.boundary[1][0].transform.location.x-self.boundary[0][0].transform.location.x), self.boundary[0][0].transform.location.y + self.delta*(self.boundary[1][0].transform.location.y-self.boundary[0][0].transform.location.y)],
        #                      [self.boundary[0][10].transform.location.x + self.delta*(self.boundary[1][10].transform.location.x-self.boundary[0][10].transform.location.x), self.boundary[0][10].transform.location.y + self.delta*(self.boundary[1][10].transform.location.y-self.boundary[0][10].transform.location.y)],
        #                      [self.boundary[0][-1].transform.location.x + self.delta*(self.boundary[1][-1].transform.location.x-self.boundary[0][-1].transform.location.x), self.boundary[0][-1].transform.location.y + self.delta*(self.boundary[1][-1].transform.location.y-self.boundary[0][-1].transform.location.y)],
        #                      [self.boundary[1][-1].transform.location.x - self.delta*(self.boundary[1][-1].transform.location.x-self.boundary[0][-1].transform.location.x), self.boundary[1][-1].transform.location.y - self.delta*(self.boundary[1][-1].transform.location.y-self.boundary[0][-1].transform.location.y)],
        #                      [self.boundary[1][10].transform.location.x - self.delta*(self.boundary[1][10].transform.location.x-self.boundary[0][10].transform.location.x), self.boundary[1][10].transform.location.y - self.delta*(self.boundary[1][10].transform.location.y-self.boundary[0][10].transform.location.y)],
        #                      [self.boundary[1][0].transform.location.x - self.delta*(self.boundary[1][0].transform.location.x-self.boundary[0][0].transform.location.x), self.boundary[1][0].transform.location.y - self.delta*(self.boundary[1][0].transform.location.y-self.boundary[0][0].transform.location.y)],
        #                      [self.boundary[0][0].transform.location.x + self.delta*(self.boundary[1][0].transform.location.x-self.boundary[0][0].transform.location.x), self.boundary[0][0].transform.location.y + self.delta*(self.boundary[1][0].transform.location.y-self.boundary[0][0].transform.location.y)]])

        self.obstacle = obstacle

        self.all_obstacle = []

    def planning(self):

        self.all_obstacle = self.obstacle_parameter(self.obstacle)

        for k in range(self.iter_max):

            # TODO: needs to change the function. done.
            node_rand = self.generate_random_node(self.goal_sample_rate)

            # TODO: done.(vertex are node list)
            node_near = self.nearest_neighbor(self.vertex, node_rand)

            # TODO: done. using new_state() and get_distance_and_angle()
            node_new = self.new_state(node_near, node_rand)

            # TODO: is_collision() needs to write how to check collsion. using is_collision(),
            # is_inside_obs(), is_intersect_rec(), get_obs_vertex()
            if node_new and not self.is_collision(node_near, node_new):

                # TODO:done. using is_collision()[can be modify before] and find_near_neighbor()
                neighbor_index = self.find_near_neighbor(node_new)

                # TODO:done
                self.vertex.append(node_new)
                # print(self.vertex)

                if neighbor_index:
                    # TODO:done. using choose_parent(), get_new_cost() and cost().
                    self.choose_parent(node_new, neighbor_index)

                    # TODO:done. using rewire(), get_new_cost() and cost().
                    self.rewire(node_new, neighbor_index)

        # TODO: almost done. using search_goal_parent(), is_collision()[can be modify before].
        index = self.search_goal_parent()

        # TODO: done. using extract_path() maybe add if statement here to let the car stop when there is no optimal path.
        self.path = self.extract_path(self.vertex[index])
        return self.path


    def generate_random_node(self, goal_sample_rate):
        if np.random.random() > goal_sample_rate:
            min_x, min_y, max_x, max_y = self.poly.bounds
            within = False
            while (within == False):
                random_point = Point(random.uniform(min_x, max_x), random.uniform(min_y, max_y))
                if (random_point.within(self.poly)):
                    within = True

            return random_point
        else:
            return self.s_goal

    def nearest_neighbor(self, node_list, n):
        # b = []
        # for nd in node_list:
        #     n_x = n.x * np.ones(len(nd.x))
        #     n_y = n.y * np.ones(len(nd.y))
        #     print('n_x', n_x)
        #     # print('node', node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y) ]))])
        #     # return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y) ]))]#for nd in node_list]))]
        #     # print('nd',nd)
        #     print('nd.x', nd.x)
        #
        #     # print('nd.xtype', type(nd.x))
        #     print('n.x', n.x)
        #
        #     a = []
        #     for i in range(len(nd.x)):
        #         a.append(math.hypot(nd.x[i] - n.x, nd.y[i] - n.y))
        #     # a = math.hypot(nd.x - n_x, nd.y - n_y)
        #     b.append(min(a))
        #
        # index = np.argmin(b)
        # print('b', b)
        # print('index', index)
        # # print(index.type)
        # return node_list[int(index)]
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]

    def new_state(self, node_start, node_goal):
        dist, theta = self.get_distance_and_angle(node_start, node_goal)

        dist = min(self.step_len, dist)
        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))

        node_new.parent = node_start

        return node_new

    def get_distance_and_angle(self, node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)

    def judge(self, M, N, P):
        return ((P.y - M.y) * (N.x - M.x) > (N.y - M.y) * (P.x - M.x))

    def obstacle_parameter(self, obstacle):
        all_obstacle = []
        for i in range(len(obstacle)):
            a = 5.0
            b = 3.0
            #TODO:因为debug被迫修改
            obstacle_position = [obstacle[i][0], obstacle[i][1]]
            obstacle_angle = np.arctan2(obstacle[i][3], obstacle[i][2])
            # obstacle_position = [obstacle[i].get_location().x, obstacle[i].get_location().y]
            # obstacle_angle = np.arctan2(obstacle[i].get_velocity().x, obstacle[i].get_velocity().y)
            #TODO:正式代码需要修改的地方
            obstacle_leftup = Node([
                (-a / 2) * np.cos(obstacle_angle) - (b / 2) * np.sin(obstacle_angle) + obstacle_position[0],
                (-a / 2) * np.sin(obstacle_angle) + (b / 2) * np.cos(obstacle_angle) + obstacle_position[1]])
            obstacle_leftdown = Node([
                (-a / 2) * np.cos(obstacle_angle) - (-b / 2) * np.sin(obstacle_angle) + obstacle_position[0],
                (-a / 2) * np.sin(obstacle_angle) + (-b / 2) * np.cos(obstacle_angle) + obstacle_position[1]])
            obstacle_rightdown = Node([
                (a / 2) * np.cos(obstacle_angle) - (-b / 2) * np.sin(obstacle_angle) + obstacle_position[0],
                (a / 2) * np.sin(obstacle_angle) + (-b / 2) * np.cos(obstacle_angle) + obstacle_position[1]])
            obstacle_rightup = Node([
                (a / 2) * np.cos(obstacle_angle) - (b / 2) * np.sin(obstacle_angle) + obstacle_position[0],
                (a / 2) * np.sin(obstacle_angle) + (b / 2) * np.cos(obstacle_angle) + obstacle_position[1]])
            new_obstacle = [obstacle_leftup, obstacle_leftdown, obstacle_rightdown, obstacle_rightup]
            all_obstacle.append(new_obstacle)
        return all_obstacle

    def is_collision(self, start, end):
        # A,B,C,D as vertex of the rectangle
        # TODO: obstacle
        for i in range(len(self.all_obstacle)):
            bool_1 = (self.judge(start, self.all_obstacle[i][0], self.all_obstacle[i][3]) != self.judge(end,self.all_obstacle[i][0],self.all_obstacle[i][3])) and (self.judge(start, end, self.all_obstacle[i][0]) != self.judge(start, end,self.all_obstacle[i][3]))
            bool_2 = (self.judge(start, self.all_obstacle[i][0], self.all_obstacle[i][1]) != self.judge(end,self.all_obstacle[i][0],self.all_obstacle[i][1])) and (self.judge(start, end, self.all_obstacle[i][0]) != self.judge(start, end,self.all_obstacle[i][1]))
            bool_3 = (self.judge(start, self.all_obstacle[i][2], self.all_obstacle[i][3]) != self.judge(end,self.all_obstacle[i][2],self.all_obstacle[i][3])) and (self.judge(start, end, self.all_obstacle[i][2]) != self.judge(start, end,self.all_obstacle[i][3]))
            bool_4 = (self.judge(start, self.all_obstacle[i][1], self.all_obstacle[i][2]) != self.judge(end,self.all_obstacle[i][1],self.all_obstacle[i][2])) and ( self.judge(start, end, self.all_obstacle[i][1]) != self.judge(start, end,self.all_obstacle[i][2]))
        return (bool_1 or bool_2 or bool_3 or bool_4)

    def find_near_neighbor(self, node_new):
        n = len(self.vertex) + 1
        r = min(self.search_radius * math.sqrt((math.log(n) / n)), self.step_len)

        dist_table = [math.hypot(nd.x - node_new.x, nd.y - node_new.y) for nd in self.vertex]
        dist_table_index = [ind for ind in range(len(dist_table)) if dist_table[ind] <= r and
                            not self.is_collision(node_new, self.vertex[ind])]

        return dist_table_index

    def choose_parent(self, node_new, neighbor_index):
        cost = [self.get_new_cost(self.vertex[i], node_new) for i in neighbor_index]

        cost_min_index = neighbor_index[int(np.argmin(cost))]
        node_new.parent = self.vertex[cost_min_index]

    def get_new_cost(self, node_start, node_end):
        dist, _ = self.get_distance_and_angle(node_start, node_end)

        return self.cost(node_start) + dist

    def cost(self, node_p):
        node = node_p
        cost = 0.0

        while node.parent:
            # The Math.hypot() function returns the square root of the sum of squares of its arguments.
            cost += math.hypot(node.x - node.parent.x, node.y - node.parent.y)
            node = node.parent

        return cost

    def rewire(self, node_new, neighbor_index):
        for i in neighbor_index:
            node_neighbor = self.vertex[i]

            if self.cost(node_neighbor) > self.get_new_cost(node_new, node_neighbor):
                node_neighbor.parent = node_new

    def extract_path(self, node_end):
        path = [[self.s_goal.x, self.s_goal.y]]
        node = node_end

        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path

    def search_goal_parent(self):
        dist_list = [math.hypot(n.x - self.s_goal.x, n.y - self.s_goal.y) for n in self.vertex]
        # using step_len as threshold.
        node_index = [i for i in range(len(dist_list)) if dist_list[i] <= self.step_len]

        if len(node_index) > 0:
            cost_list = [dist_list[i] + self.cost(self.vertex[i]) for i in node_index
                         if not self.is_collision(self.vertex[i], self.s_goal)]
            return node_index[int(np.argmin(cost_list))]
        # maybe have better way to figure it out, like return a large number or return none to let the car stop when there is no optimal path.
        return len(self.vertex) - 1

    def extract_path(self, node_end):
        path = [[self.s_goal.x, self.s_goal.y]]
        node = node_end

        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path

if __name__ == '__main__':
    obstacles_location = [177.33601379, 96.1129837]
    obstacles_velocity = [6.65619373, -1.21003282]
    filtered_obstacles = np.array([[177.33601379, 96.1129837,6.65619373, -1.21003282]])
    boundary = np.array([[[158.31506348, 85.67432404],
                          [159.31506348, 85.67114258],
                          [160.31504822, 85.66796875],
                          [161.31503296, 85.66478729],
                          [162.31503296, 85.66161346],
                          [163.31503296, 85.65843964],
                          [164.31503296, 85.65525818],
                          [165.31503296, 85.65208435],
                          [166.3150177, 85.64890289],
                          [167.31500244, 85.64572906],
                          [168.31500244, 85.64255524],
                          [169.31500244, 85.63937378],
                          [170.31500244, 85.63619995],
                          [171.31498718, 85.63301849],
                          [172.31498718, 85.62984467],
                          [173.26547241, 85.62653351],
                          [173.94592285, 85.6088028],
                          [174.62535095, 85.567276],
                          [175.30291748, 85.50205231],
                          [175.9777832, 85.41318512]],
                         [[158.36198425, 100.44425201],
                          [159.36198425, 100.44107056],
                          [160.36196899, 100.43789673],
                          [161.36195374, 100.43471527],
                          [162.36195374, 100.43154144],
                          [163.36195374, 100.42836761],
                          [164.36195374, 100.42518616],
                          [165.36195374, 100.42201233],
                          [166.36193848, 100.41883087],
                          [167.36192322, 100.41565704],
                          [168.36192322, 100.41248322],
                          [169.36192322, 100.40930176],
                          [170.36192322, 100.40612793],
                          [171.36190796, 100.40294647],
                          [172.36190796, 100.39977264],
                          [173.39245605, 100.39598846],
                          [174.58892822, 100.3647995],
                          [175.78358459, 100.29179382],
                          [176.97497559, 100.17710876],
                          [178.16162109, 100.02085114]]])

    waypoints = [[172.0, 93.0, 0.0], [181.0, 91.0, 0.0], [190.0, 86.0, 0.0], [197.0, 78.0, 0.0], [200.0, 69.0, 0.0],
     [200.0, 57.0, 0.0], [194.0, 45.0, 0.0], [184.0, 39.0, 0.0], [173.0, 35.0, 0.0], [166.0, 33.0, 0.0],
     [159.0, 31.0, 0.0], [153.0, 31.0, 0.0], [150.0, 36.0, 0.0], [151.0, 41.0, 0.0], [153.0, 47.0, 0.0],
     [156.0, 52.0, 0.0], [158.0, 60.0, 0.0], [159.0, 66.0, 0.0], [155.0, 69.0, 0.0], [150.0, 70.0, 0.0],
     [144.0, 69.0, 0.0], [136.0, 68.0, 0.0], [122.0, 67.0, 0.0], [109.0, 65.0, 0.0], [97.0, 64.0, 0.0],
     [82.0, 62.0, 0.0], [70.0, 58.0, 0.0], [59.0, 51.0, 0.0], [51.0, 41.0, 0.0], [44.0, 30.0, 0.0]]

    rrt_stars = RrtStar(x_start=np.array([162,93]), x_goal=np.array(waypoints)[1,:2], step_len=4, goal_sample_rate=0.60,
                        search_radius=3, iter_max=30, delta=1, boundary=boundary,
                        obstacle=filtered_obstacles)

    path = rrt_stars.planning()
    print(path[-1][0])

    all_obstacle = rrt_stars.obstacle_parameter(filtered_obstacles)
    _obstacle_coord = []
    for i in range(len(all_obstacle)):
        for j in range(len(all_obstacle[0])):
            _obstacle = [all_obstacle[i][j].x, all_obstacle[i][j].y]
            _obstacle_coord.append(_obstacle)
    print('all_obstacle:',_obstacle_coord)

    plt.scatter(np.array(waypoints)[:2,0], np.array(waypoints)[:2,1])
    plt.scatter(boundary[:,:,0], boundary[:,:,1], s=10)
    plt.scatter(np.array(path)[:,0], np.array(path)[:,1], s=10)
    plt.scatter(np.array(_obstacle_coord)[:, 0], np.array(_obstacle_coord)[:, 1],color='blue', s=10)
    plt.scatter(162, 93, color='yellow', label='start point')
    # plt.
    plt.legend()
    plt.show()
