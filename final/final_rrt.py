import carla
import time
import numpy as np
import math

import numpy as np
import math
import random

from shapely.geometry import Polygon, Point


# def circleRadius(b, c, d):
#     temp = c[0]2 + c[1]2
#     bc = (b[0]2 + b[1]2 - temp) / 2
#     cd = (temp - d[0]2 - d[1]2) / 2
#     det = (b[0] - c[0]) * (c[1] - d[1]) - (c[0] - d[0]) * (b[1] - c[1])

#     if abs(det) < 1.0e-10:
#     return 0


class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None


class RrtStar:

    def __init__(self, x_start, x_goal, step_len,
                 goal_sample_rate, search_radius, iter_max, delta, boundary, obstacle):

        # comment: x_start = input waypoint[0]
        # comment: x_goal = input waypoint[-1]
        # print('x_start',x_start)
        # print('x_start shape ',x_start[0].shape())
        # x_start = [[162. 93.]]
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

        self.poly = Polygon([[self.boundary[0][0].transform.location.x + self.delta * (
                    self.boundary[1][0].transform.location.x - self.boundary[0][0].transform.location.x),
                              self.boundary[0][0].transform.location.y + self.delta * (
                                          self.boundary[1][0].transform.location.y - self.boundary[0][
                                      0].transform.location.y)],
                             [self.boundary[0][10].transform.location.x + self.delta * (
                                         self.boundary[1][10].transform.location.x - self.boundary[0][
                                     10].transform.location.x),
                              self.boundary[0][10].transform.location.y + self.delta * (
                                          self.boundary[1][10].transform.location.y - self.boundary[0][
                                      10].transform.location.y)],
                             [self.boundary[0][-1].transform.location.x + self.delta * (
                                         self.boundary[1][-1].transform.location.x - self.boundary[0][
                                     -1].transform.location.x),
                              self.boundary[0][-1].transform.location.y + self.delta * (
                                          self.boundary[1][-1].transform.location.y - self.boundary[0][
                                      -1].transform.location.y)],
                             [self.boundary[1][-1].transform.location.x - self.delta * (
                                         self.boundary[1][-1].transform.location.x - self.boundary[0][
                                     -1].transform.location.x),
                              self.boundary[1][-1].transform.location.y - self.delta * (
                                          self.boundary[1][-1].transform.location.y - self.boundary[0][
                                      -1].transform.location.y)],
                             [self.boundary[1][10].transform.location.x - self.delta * (
                                         self.boundary[1][10].transform.location.x - self.boundary[0][
                                     10].transform.location.x),
                              self.boundary[1][10].transform.location.y - self.delta * (
                                          self.boundary[1][10].transform.location.y - self.boundary[0][
                                      10].transform.location.y)],
                             [self.boundary[1][0].transform.location.x - self.delta * (
                                         self.boundary[1][0].transform.location.x - self.boundary[0][
                                     0].transform.location.x), self.boundary[1][0].transform.location.y - self.delta * (
                                          self.boundary[1][0].transform.location.y - self.boundary[0][
                                      0].transform.location.y)],
                             [self.boundary[0][0].transform.location.x + self.delta * (
                                         self.boundary[1][0].transform.location.x - self.boundary[0][
                                     0].transform.location.x), self.boundary[0][0].transform.location.y + self.delta * (
                                          self.boundary[1][0].transform.location.y - self.boundary[0][
                                      0].transform.location.y)]])

        self.obstacle = obstacle

        self.all_obstacle = []

    def planning(self):

        self.all_obstacle = self.obstacle_parameter(self.obstacle)

        for k in range(self.iter_max):

            # TODO: needs to change the function. done.
            node_rand = self.generate_random_node(self.goal_sample_rate)

            # print(self.vertex[0].x, self.vertex[0].y, 'self.vertex')

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

    # def generate_random_node(self, goal_sample_rate):
    #     # TODO: 1. set the environment instead of using x_range and y_range;
    #     # 2. generate random nodes in different windows(needs to add parameter for different window)
    #     # in that case, goal point is not self.s_goal but next waypoint
    #     delta = self.delta
    #     if np.random.random() > goal_sample_rate:
    #
    #         return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
    #                      np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))
    #
    #     return self.s_goal

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
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y) for nd in node_list]))]

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
            a = 3.0 + self.delta
            b = 1.0 + self.delta
            obstacle_position = [obstacle[i].get_location().x, obstacle[i].get_location().y]
            obstacle_angle = np.arctan2(obstacle[i].get_velocity().y, obstacle[i].get_velocity().x)
            # TODO: add velocity parameter
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
            bool_1 = (self.judge(start, self.all_obstacle[i][0], self.all_obstacle[i][3]) != self.judge(end,
                                                                                                        self.all_obstacle[
                                                                                                            i][0],
                                                                                                        self.all_obstacle[
                                                                                                            i][
                                                                                                            3])) and (
                                 self.judge(start, end, self.all_obstacle[i][0]) != self.judge(start, end,
                                                                                               self.all_obstacle[i][3]))
            bool_2 = (self.judge(start, self.all_obstacle[i][0], self.all_obstacle[i][1]) != self.judge(end,
                                                                                                        self.all_obstacle[
                                                                                                            i][0],
                                                                                                        self.all_obstacle[
                                                                                                            i][
                                                                                                            1])) and (
                                 self.judge(start, end, self.all_obstacle[i][0]) != self.judge(start, end,
                                                                                               self.all_obstacle[i][1]))
            bool_3 = (self.judge(start, self.all_obstacle[i][2], self.all_obstacle[i][3]) != self.judge(end,
                                                                                                        self.all_obstacle[
                                                                                                            i][2],
                                                                                                        self.all_obstacle[
                                                                                                            i][
                                                                                                            3])) and (
                                 self.judge(start, end, self.all_obstacle[i][2]) != self.judge(start, end,
                                                                                               self.all_obstacle[i][3]))
            bool_4 = (self.judge(start, self.all_obstacle[i][1], self.all_obstacle[i][2]) != self.judge(end,
                                                                                                        self.all_obstacle[
                                                                                                            i][1],
                                                                                                        self.all_obstacle[
                                                                                                            i][
                                                                                                            2])) and (
                                 self.judge(start, end, self.all_obstacle[i][1]) != self.judge(start, end,
                                                                                               self.all_obstacle[i][2]))
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


class Agent():
    def __init__(self, vehicle=None):
        self.vehicle = vehicle
        self.theta_ref = 0
        self.accum_theta_ref = 0
        self.pre_waypoints = np.zeros(shape=(30, 3))
        self.pre_waypoints[0][0] = 101
        self.pre_waypoints[0][1] = 93

        # initialization for predict
        self.theta_ref_predict = []
        self.accum_theta_ref_predict = []

        # self.path_plan = 1
        #
        # # # rrt variables
        self.idx = 0
        # # self.non_obstacle = 0
        # self.is_rrt = 0

        #
        self.path = []
        self.mod_choose = 1
        # self.last_waypoint = []
        # self.kept_waypoints = []
        # self.pre_keptwaypoints = []
        # self.after_rrt = 0

    def run_step(self, filtered_obstacles, waypoints, vel, transform, boundary):

#目前的思路：返回waypoints和pre_waypoints,如果进入rrt就全部跑完然后最后一个点是最后一个prewaypoints，否则就用

        # TODO: change waypoints
        if self.mod_choose == 1:
            if filtered_obstacles != []:
                obstacle_position = np.array(
                    [filtered_obstacles[0].get_location().x, filtered_obstacles[0].get_location().y])
                obstacle_velocity = np.array(
                    [filtered_obstacles[0].get_velocity().x, filtered_obstacles[0].get_velocity().y])
                distance_matrix = np.sum(abs(np.array(waypoints)[:, :2] - obstacle_position), axis=1)
                index = np.where(distance_matrix == np.min(distance_matrix))
                # time to do the rrt* plan
                if index[0][0] == 0:
                    x_start = np.array([transform.location.x, transform.location.y])
                    step_len = 2
                    rrt_stars = RrtStar(x_start=x_start, x_goal=np.array(waypoints)[1, :2], step_len=step_len,
                                        goal_sample_rate=0.60, search_radius=3, iter_max=50, delta=0.2, boundary=boundary,
                                        obstacle=filtered_obstacles)
                    self.path = rrt_stars.planning()
                    self.mod_choose = 0
                else:
                    waypoints = waypoints
            else:
                waypoints = waypoints
        if self.mod_choose == 0: #生成新的path
            # self.path.reverse()
            path_len = len(self.path)
            waypoints[0][0] = self.path[path_len - 1 - self.idx][0]
            waypoints[0][1] = self.path[path_len - 1 - self.idx][1]

            cur_x, cur_y = transform.location.x, transform.location.y
            target_x, target_y = self.path[path_len - 1 - self.idx]
            distance = math.sqrt((cur_x - target_x) ** 2 + (cur_y - target_y) ** 2)
            # TODO: modify one
            distance = min((cur_x - target_x),(cur_y - target_y))
            # TODO: modify two
            # stoelength = function(two waypoints)
            # TODO:

            if distance < 0.8:
                self.idx += 1

            if self.idx >= (path_len-1):
                self.mod_choose = 1
                self.idx = 0


        x_ref = waypoints[0][0]
        y_ref = waypoints[0][1]
        x_b = transform.location.x
        y_b = transform.location.y
        pre_theta_ref = self.accum_theta_ref

        if (self.pre_waypoints[0][0] != waypoints[0][0] or self.pre_waypoints[0][1] != waypoints[0][1]):
            self.theta_ref = np.arctan2(-self.pre_waypoints[0][1] + waypoints[0][1],
                                        -self.pre_waypoints[0][0] + waypoints[0][0])  # *180/math.pi
            self.accum_theta_ref = np.rad2deg(
                np.arctan2(waypoints[0][1] - self.pre_waypoints[0][1], waypoints[0][0] - self.pre_waypoints[0][0]))

        v_ref = 10  # 20
        k_x = 0.2  # 2
        k_y = 1.5  # 1.5
        k_v = 1  # 1
        k_theta = 0.135  # 0.135

        self.pre_waypoints = waypoints

        theta_b = transform.rotation.yaw

        v_b = np.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

        delta_x = math.cos(self.theta_ref) * (x_ref - x_b) + math.sin(self.theta_ref) * (y_ref - y_b)
        delta_y = -math.sin(self.theta_ref) * (x_ref - x_b) + math.cos(self.theta_ref) * (y_ref - y_b)

        delta_theta = self.accum_theta_ref - theta_b
        if abs(delta_theta) > 180:
            if theta_b < 0:
                delta_theta = self.accum_theta_ref - 360 - theta_b
            else:
                delta_theta = self.accum_theta_ref + 360 - theta_b

        delta_v = v_ref - v_b

        delta = np.array([delta_x, delta_y, delta_theta, delta_v])

        K = np.array([[k_x, 0, 0, k_v], [0, k_y, k_theta, 0]])
        u = np.dot(K, delta)

        # control the vehicle speed
        control = carla.VehicleControl()
        if u[0] >= 0:
            control.throttle = 0.8
        elif u[0] < 0:
            control.brake = 1

        # if curving is decteted, hit the brake
        radius = abs(pre_theta_ref - self.accum_theta_ref)
        threshold = 1
        if (radius > threshold):
            control.brake = 1
            # v_ref = 10

        steer_threshold = np.pi  # 1.221

        if u[1] > steer_threshold:  # 1.39 [-1,1] [-3.14, 3.14 RADS]
            control.steer = 1
        elif u[1] < -steer_threshold:
            control.steer = -1
        else:
            control.steer = u[1] / steer_threshold
        # print('throttle:', control.throttle)
        # print('steer:', control.steer)
        return control