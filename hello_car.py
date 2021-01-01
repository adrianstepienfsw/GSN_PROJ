# ready to run example: PythonClient/car/hello_car.py
import airsim
import time
import numpy as np
import matplotlib.pyplot as plt


# connect to the AirSim simulator
client = airsim.CarClient()
client.confirmConnection()
client.enableApiControl(True)
car_controls = airsim.CarControls()

while True:
    # get state of the car
    car_state = client.getCarState()
    print("Speed %d, Gear %d" % (car_state.speed, car_state.gear))

    # set the controls for car
    car_controls.throttle = 0.2
    car_controls.steering = 0
    client.setCarControls(car_controls)

    # let car drive a bit
    time.sleep(1)

    lidar = client.getLidarData()
    points = np.array(lidar.point_cloud, dtype=np.dtype('f4'))
    points = np.reshape(points, (int(points.shape[0] / 3), 3))
    x = points[:,0]
    y = points[:,1]
    z = -points[:,2]
    print(points.shape)
    ax = plt.axes(projection='3d')
    ax.scatter(x, y, z, c=z, cmap='viridis', linewidth=0.5);

    plt.show()

    collision_info = client.simGetCollisionInfo()
    if collision_info.has_collided:
        print("collision")
        #break




    # get camera images from the car
    #responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
    #print('Retrieved images: %d', len(responses))

    # do something with images
    #for response in responses:
    #    if response.pixels_as_float:
    #        print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
    #        airsim.write_pfm('py1.pfm', airsim.get_pfm_array(response))
    #    else:
    #        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
    #        airsim.write_file('py1.png', response.image_data_uint8)
    #    img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)
    #    plt.imshow(img1d)