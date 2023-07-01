#! /usr/bin/env python
# Import ROS.
import rospy
import random
import threading 
from iq_gnc.py_gnc_functions import *
from iq_gnc.PrintColours import *
from iq_gnc.ManageDrones import manageDrones, LANDING, LANDED, TAKING_OFF, TAKEOFF, PRE_LANDING
from std_msgs.msg import String
from tf2_ros import Buffer
from geometry_msgs.msg import PoseStamped, Vector3, TransformStamped, Point

drone = gnc_api()
manageDrone = manageDrones(1)
canReceive = True
currentX = 0
currentY = -6

def set_destination(drone, coordinates_arr):
    if(float(coordinates_arr[0]) == 0 and float(coordinates_arr[1]) == 0):
        drone.set_destination(x=float(coordinates_arr[0]), y=float(coordinates_arr[1]), z=1, psi=float(coordinates_arr[3]))
    else:
        drone.set_destination(x=float(coordinates_arr[0]) - currentX, y=float(coordinates_arr[1]) - currentY, z=float(coordinates_arr[2]), psi=float(coordinates_arr[3]))
    drone.needCheckReach = True

def callback(data):
    global canReceive
    if canReceive == True or data.data.find('drone3: ') != -1 or data.data.find('drone3true') != -1:
        rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)
        if data.data == 'out' or data.data.find('drone3out') != -1:
            drone.set_destination(x=0, y=0, z=1, psi=0)
        elif data.data == 'land':
            random_number = random.randrange(0, 11, 1) / 2.0
            rospy.loginfo(rospy.get_caller_id() + " wait %s to land", str(random_number))
            destination_arr = [0, 0, 1, 0]
            timer = threading.Timer(random_number, set_destination, [drone, destination_arr]) 
            timer.start()
            manageDrone.updateState('drone3 ', PRE_LANDING)
        else:
            analizeData = data.data.split(': ')
            if analizeData[0] == TAKEOFF and manageDrone.state == LANDED:
                drone.takeoff(1)
                manageDrone.updateState('drone3 ', TAKING_OFF)
            elif manageDrone.state == TAKEOFF and analizeData[0] == 'drone3':
                random_number = random.randrange(0, 11, 1) / 2.0
                rospy.loginfo(rospy.get_caller_id() + " wait %s", str(random_number))
                coordinates_arr = analizeData[1].split()
                timer = threading.Timer(random_number, set_destination, [drone, coordinates_arr]) 
                timer.start()
                # drone.set_destination_sub_drones(x=float(coordinates_arr[0]), y=float("0"), z=float(coordinates_arr[2]), psi=0, topicChild='drone2_map', topicParent='master_map2')
                # if(float(coordinates_arr[0]) == 0 and float(coordinates_arr[1]) == 0):
                #     drone.set_destination(x=float(coordinates_arr[0]), y=float(coordinates_arr[1]), z=1, psi=float(coordinates_arr[3]))
                # else:
                #     drone.set_destination(x=float(coordinates_arr[0]) - currentX, y=float(coordinates_arr[1]) - currentY, z=float(coordinates_arr[2]), psi=float(coordinates_arr[3]))
                # drone.needCheckReach = True
            elif analizeData[0] == 'master':
                frames = analizeData[1].split()
                drone.masterFrame = [float(frames[0]), float(frames[1]), float(frames[2])]
        canReceive = False

def callback2(data):
    print(data)

def main():
    # Initializing ROS node.
    rospy.init_node("drone3", anonymous=True)

    chatter_pub3 = rospy.Publisher(name='drone3_listen', data_class=String, queue_size=100)
    rospy.Subscriber(name="/drone1/master", data_class=String, queue_size=10, callback=callback)
    # rospy.Subscriber(name="/drone3/mavros/setpoint_position/local", data_class=PoseStamped, queue_size=10, callback=callback2)

    drone.wait4connect()
    drone.set_mode("GUIDED")
    drone.initialize_local_frame()
    # Specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish.
    rate = rospy.Rate(2)

    counter = 0

    while not rospy.is_shutdown():
        rate.sleep()
        if manageDrone.state == LANDED:
            # print('---------------        Land Succesfully        ---------------')
            a = 0
        if manageDrone.state == LANDING:
            print('---------------             Landing            ---------------')
            print('---------------  You can\'t public your image  ---------------')
            if drone.check_waypoint_reached():
                manageDrone.updateState('drone3 ', LANDED)
        elif manageDrone.state == PRE_LANDING:
            if drone.check_waypoint_reached():
                manageDrone.updateState('drone3 ', LANDING)
                drone.land()
        if counter > 8 and drone.check_waypoint_reached():
            if drone.needCheckReach == True:
                data = 'reached:3'
                rospy.loginfo(data)
                chatter_pub3.publish(data)
                drone.needCheckReach = False
            global canReceive
            canReceive = True
        elif counter == 8:
            manageDrone.updateState('drone3 ', TAKEOFF)
            data = 'takeoff:3'
            rospy.loginfo(data)
            chatter_pub3.publish(data)
            counter += 1
        elif manageDrone.state == TAKING_OFF:
            counter += 1
        else:
            print('waiting for another signal')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
