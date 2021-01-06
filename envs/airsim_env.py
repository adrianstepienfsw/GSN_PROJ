from envs.base_env import BaseEnv
import gym
import numpy as np
import airsim
import numpy as np
import math
import time
import matplotlib.pyplot as plt
import matplotlib.colors as pcol

import gym
from gym import spaces

class AirSimEnv(BaseEnv):
    def __init__(self, env_name, id, seed):
        super().__init__(env_name, id)

        self.debug = False

        if self.debug:
            plt.ion()

        self.make()

        self.distance = 0
        self.distances = []
        self.timestamp = self.car_state.timestamp
        self.action_space = spaces.Discrete(6)
        self.observation_space = spaces.Discrete(6)
        self.lidar_range = 15
        self.action = None
        self.state = {
            "pose": None,
            "collision": None,
            "prev_pose": None,
        }

    def make(self):
        self.car = airsim.CarClient()
        self.car_controls = airsim.CarControls()
        self.car_state = self.car.getCarState()

        return self.car

    def step(self, action):
        self.do_action(action)
        time.sleep(0.4)
        obs = self.get_observation_space()
        reward, done = self.compute_reward()
        info = {'reward': reward}

        self.distance += self.calculate_distance()

        return obs, reward, done, info

    def reset(self):
        self.setup_car()
        self.do_action(1)
        if self.distance != 0:
            self.distances.append(self.distance)
        if len(self.distances)>=20:
            print("Mean distance form 20 rounds", np.array(self.distances).sum()/20)
            self.distances = []
        self.distance = 0
        return self.get_observation_space()

    def get_action_space(self):
        return self.action_space

    def get_observation_space(self):
        width = 20
        lidar_matrix = np.array([])
        while (lidar_matrix.size < 1):
            lidar = self.car.getLidarData()
            if (len(lidar.point_cloud) / 3) > 500:
                lidar_points = self.process_lidar_data(lidar)
                lidar_matrix = self.change_lidar_points_to_matrix(lidar_points, width)
        self.car_state = self.car.getCarState()
        collision = self.car.simGetCollisionInfo().has_collided

        self.state["prev_pose"] = self.state["pose"]
        self.state["pose"] = self.car_state.kinematics_estimated
        self.state["collision"] = collision

        return lidar_matrix.reshape([width, width, 1])

    def render(self):
        pass

    def compute_reward(self):
        beta = 2
        MAX_SPEED = 10
        MIN_SPEED = 0

        z = self.state["pose"].position.z_val
        pts = [[
                np.array([23, -83, z]),
                np.array([23, -413, z]),
                np.array([68.5, -414, z])
            ],[
                np.array([23, -100, z]),
                np.array([38, -100.5, z]),
                np.array([48, -100, z]),
                np.array([57, -100.7, z]),
                np.array([74, -102, z]),
                np.array([100, -102, z]),
                np.array([111, -100, z]),
                np.array([120, -99, z])
        ]]

        pd = self.state["pose"].position
        car_pt = np.array([pd.x_val, pd.y_val, pd.z_val])

        dist = 10000000
        for ii in range(len(pts)):
            for i in range(0, len(pts[ii]) - 1):
                dist = min(
                    dist,
                    np.linalg.norm(np.cross((car_pt - pts[ii][i]), (car_pt - pts[ii][i + 1])))
                    / np.linalg.norm(pts[ii][i] - pts[ii][i + 1]),
                )

        #print(dist)
        reward_dist = 0
        reward_speed = 0
        if self.state["collision"]:
            reward = -3
        else:
            reward_dist = math.exp(-beta * dist)
            reward_speed = (
                                   (self.car_state.speed - MIN_SPEED) / (MAX_SPEED - MIN_SPEED)
                           )
            reward = reward_dist + reward_speed - 0.5

        done = 0
        if reward < -10:
            done = 1
        if self.car_controls.brake == 0:
            if self.car_state.speed == 0:
                done = 1
        if self.state["collision"]:
            done = 1
        #print(self.distance)
        #print("Reward, dist, done: ",reward ,dist ,done)
        #print("Reward, reward_dist, reward_speed", reward, reward_dist, reward_speed)
        return reward, done

    def process_lidar_data(self, data):
        points = np.array(data.point_cloud, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 3), 3))
        out = {}
        out["x"] = -points[:, 0]
        out["y"] = points[:, 1]
        out["z"] = (-points[:, 2]) + 2.5

        return out

    def change_lidar_points_to_matrix(self, _points, width):
        matrix = np.zeros(width * width).reshape(width, width)
        points = _points.copy()
        points_buckets = []

        for i in range(width):
            points_buckets.append([])
            for ii in range(width):
                points_buckets[i].append([])

        for i_points in range(len(points["x"])):
            x = int((points["x"][i_points] + self.lidar_range) / (self.lidar_range*2/width))
            y = int((points["y"][i_points] + self.lidar_range) / (self.lidar_range*2/width))
            points_buckets[x][y].append(points["z"][i_points])

        for i in range(len(matrix)):
            for ii in range(len(matrix)):
                if len(points_buckets[i][ii]) > 0:
                    matrix[i][ii] = np.mean(points_buckets[i][ii])
                else:
                    matrix[i][ii] = -200
        matrix = self.fill_lidar_gaps(matrix, 10)

        if self.debug == True:
            plt.imshow(matrix, norm=pcol.Normalize(vmin=0, vmax=1, clip=False))
            plt.show(block=False)
            plt.pause(.001)

        return matrix

    def fill_lidar_gaps(self, lidar, radius):
        output = np.zeros_like(lidar)
        for x in range(lidar.shape[0]):
            for y in range(lidar.shape[1]):
                if lidar[x][y] == -200:
                    for i_radius in range(1, radius+1):
                        gap = self.fill_gap(lidar, x, y, i_radius)
                        if gap != -200:
                            output[x][y] = gap
                            break
                    if output[x][y] == -200:
                        output[x][y] = 0
                else:
                    output[x][y] = lidar[x][y]
        return output


    def fill_gap(self, matrix, x, y, radius):
        sum = 0
        count = 0
        for x_neighbour in range(max(0, x - radius), min(matrix.shape[0], x + radius + 1)):
            for y_neighbour in range(max(0, y - radius), min(matrix.shape[1], y + radius + 1)):
                if matrix[x_neighbour][y_neighbour] != -200:
                    sum += matrix[x_neighbour][y_neighbour]
                    count += 1
        if count == 0:
            return -200
        else:
            return sum / max(1, count)

    def setup_car(self):
        self.car.reset()
        pose = airsim.Pose(airsim.Vector3r(23, -83, -0.2), airsim.to_quaternion(0, 0, -1.56))  # PRY in radians
        self.car.simSetVehiclePose(pose, ignore_collison=True)
        self.car.enableApiControl(True)
        self.car.armDisarm(True)
        time.sleep(0.5)

    def do_action(self, action):
        #print("Action: ",action)
        self.car_controls.brake = 0
        self.car_controls.throttle = 0.5
        if action == 0:
            self.car_controls.steering = 0
        elif action == 1:
            self.car_controls.steering = 0.5
        elif action == 2:
            self.car_controls.steering = -0.5
        elif action == 3:
            self.car_controls.steering = 0.25
        elif action == 4:
            self.car_controls.steering = -0.25
        else:
            self.car_controls.brake = 1
            self.car_controls.throttle = 0
        self.car.setCarControls(self.car_controls)

    def calculate_distance(self):
        prev_timestamp = self.timestamp
        self.timestamp = self.car_state.timestamp
        time = (self.timestamp - prev_timestamp) / 1000000000
        distance = (self.car_state.speed) * time / 2
        return distance
