import numpy as np
import airsim
import time
import numpy as np

class RandomAgent:
    def step(self, car_controls, car_state, client):
        action = np.random.randint(0, 6)
        print("Action: ", action)
        car_controls = self.interpret_action(car_controls, action)
        return car_controls

    def interpret_action(self, car_controls, action):
        car_controls.brake = 0
        car_controls.throttle = 1
        if action == 0:
            car_controls.throttle = 0
            car_controls.brake = 1
        elif action == 1:
            car_controls.steering = 0
        elif action == 2:
            car_controls.steering = 0.5
        elif action == 3:
            car_controls.steering = -0.5
        elif action == 4:
            car_controls.steering = 0.25
        else:
            car_controls.steering = -0.25
        return car_controls

def calculate_distance(prev_speed, speed, prev_timestamp, timestamp):
    time = (timestamp - prev_timestamp) / 1000000000
    distance = (prev_speed+speed) * time / 2
    return distance


# connect to the AirSim simulator
client = airsim.CarClient()
client.confirmConnection()
client.enableApiControl(False)
car_controls = airsim.CarControls()

agent = RandomAgent()

car_state = client.getCarState()

prev_timestamp = car_state.timestamp
prev_speed = car_state.speed
distance = 0

while True:
    # get state of the car
    car_state = client.getCarState()
    print(client.simGetVehiclePose().position)

    # upgrade traveled distance
    speed = car_state.speed
    timestamp = car_state.timestamp
    distance += calculate_distance(prev_speed, speed, prev_timestamp, timestamp)
    # print("Traveled distance: ", distance)

    # set the controls for car
    car_controls = agent.step(car_controls, car_state, client)
    client.setCarControls(car_controls)

    # let car drive a bit
    time.sleep(0.1)

    collision_info = client.simGetCollisionInfo()
    if collision_info.has_collided:
        print("Collision. Reset Environment.")
        distance = 0
        #client.reset()

    prev_timestamp = timestamp
    prev_speed = speed
