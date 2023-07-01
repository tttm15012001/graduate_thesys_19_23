#! /usr/bin/env python
# Import ROS.
import rospy
import math
import itertools
import random
import numpy as np
from scipy.optimize import linear_sum_assignment
# Import the API.
from iq_gnc.py_gnc_functions import *
# To print colours (optional).
from iq_gnc.PrintColours import *
from std_msgs.msg import String
from iq_gnc.ManageDrones import manageDrones, coordinates, LANDING, LANDED, TAKING_OFF, TAKEOFF, PRE_LANDING, MIN_DISTANCE, MAX_DISTANCE, HEIGHT, TIME_TO_LAND, WAIT_TIME
import tf2_ros

# Create an object for the API.
drone = gnc_api()
manageDrone = manageDrones(9)
base = [[0, 0], [-200, 0], [-400, 0], [0, 200], [-200, 200], [-400, 200], [0, 400], [-200, 400], [-400, 400], [0, 600], [-200, 600], [-400, 600]]
# base2 = [[321, 119], [1321, 1119], [821, 1119], [821, 619], [-200, 200], [-400, 200], [321, 619], [321, 1119], [-400, 400]]
# current_coordinate_all_drones = [[1, 2], [3, 4], [5, 6], [7, 8], [9, 10]]
# current_coordinate_all_drones = [[0, 0], [-200, 0], [-400, 0], [837, 457], [-200, 200], [-400, 200], [189, 553], [201, 1053], [-400, 400]]
current_coordinate_all_drones = [[0, 0], [-300, 0], [-600, 0], [0, 300], [-300, 300], [-600, 300], [0, 600], [-300, 600], [-600, 600]]
# just for debugging
# current_coordinate_all_drones = [[321, 119], [-200, 0], [-400, 0], [821, 619], [-200, 200], [-400, 200], [321, 619], [-200, 400], [-400, 400]]
# current_coordinate_all_drones = [[321, 119], [1321, 1119], [821, 1119], [821, 619], [-200, 200], [-400, 200], [321, 619], [321, 1119], [-400, 400]]
# current_coordinate_all_drones = [[321, 119], [321, 619], [821, 619], [0, 200], [-200, 200], [-400, 200], [0, 400], [-200, 400], [-400, 400]]
# current_coordinate_all_drones = [[321, 119], [321, 619], [321, 1119], [821, 1119], [1321, 1119], [821, 619], [0, 400], [-200, 400], [-400, 400]]
X = 0
Y = 1

# fill [0, 0] to drones not chosen
def fill_coors(new_coordinates, drones_select):
    coords_return = [[0, 0] for i in range(len(current_coordinate_all_drones))]
    for index, value in enumerate(drones_select):
        coords_return[current_coordinate_all_drones.index(value)] = new_coordinates[index]
    return coords_return

#--------------  Hungrian ALgorithm --------------#

def calculate_distance(drone_coords, destinations):
    distances = []
    for drone in drone_coords:
        distances.append([np.linalg.norm(np.array(drone) - np.array(dest)) for dest in destinations])
    return distances

def minimize_path_cost(drone_coords, destinations):
    distances = calculate_distance(drone_coords, destinations)
    cost_matrix = np.array(distances)
    row_ind, col_ind = linear_sum_assignment(cost_matrix)
    assignments = [(drone_coords[row], destinations[col]) for row, col in zip(row_ind, col_ind)]
    total_cost = cost_matrix[row_ind, col_ind].sum()
    return assignments, total_cost

def fill(assignments):
    coords = [[0, 0] for i in range(len(current_coordinate_all_drones))]
    for index, assignment in enumerate(assignments):
        coords[current_coordinate_all_drones.index(assignment[0])] = assignment[1]
    return coords

def update_position(current_coordinate_all_drones, coords):
    for index, coord in enumerate(coords):
        if(coord[X] != 0 or coord[Y] != 0):
            current_coordinate_all_drones[index] = coord

def updateFrame(postXYCoors, min):
    for i in range(len(postXYCoors)):
        postXYCoors[i][X] -= min
        postXYCoors[i][X] -= min
    return postXYCoors

#--------------  End Hungrian ALgorithm --------------#


#--------------          Scale          --------------#

def find_closest_coords(coords):
    min_distance = 1000000
    for index in range(len(coords) - 1):
        distance = euclidean_distance(coords[index + 1], coords[index])
        if distance < min_distance:
            min_distance = distance
    return min_distance

def scale_coords(coords, scale):
    coords_return = []
    for index in range(len(coords)):
        temp = [coords[index][X] * scale, coords[index][Y] * scale]
        coords_return.insert(len(coords_return), temp)
    return coords_return

#--------------        End Scale        --------------#

#---------    update coordinate of drones    ---------#

def update_coordinates_chosen_drone(current_coordinates_all_drones, chosen_drones, new_coords):
    # Create a dictionary mapping the chosen drone coordinates to their index in the current coordinates list
    drone_indices = {tuple(drone): i for i, drone in enumerate(current_coordinates_all_drones)}

    # Replace the chosen drones with their new coordinates in the current coordinates list
    for i, new_coord in enumerate(new_coords):
        current_coordinates_all_drones[drone_indices[tuple(chosen_drones[i])]] = new_coord

    # Return the updated current coordinates list
    return current_coordinates_all_drones

#---------  End update coordinate of drones  ---------#

def update_coordinates_for_all_drones(current_coordinates_all_drones, new_coords):
    for i in range(len(current_coordinate_all_drones)):
        if new_coords[i][X] != 0 and new_coords[i][Y] != 0:
            current_coordinate_all_drones[i] = new_coords[i]

    # Return the updated current coordinates list
    return current_coordinates_all_drones

# calculate total cost
def total_cost(drones, destination):
    total = 0
    for i in range(len(destination)):
        total += euclidean_distance(drones[i], destination[i])
    return total

class Particle:
    def __init__(self):
        self.position = []  # Current position (drone assignments)
        self.best_position = []  # Personal best position
        self.best_fitness = float('inf')  # Personal best fitness

def iterator(num_drones, dest_coords, num_loop = 600000):
    best_cost = float('inf')
    best_drones = []
    for _ in range(num_loop):
        particle = Particle()
        particle.position = random.sample(range(len(current_coordinate_all_drones)), num_drones)
        drones_coords = []
        for index in particle.position:
            drones_coords.insert(len(drones_coords), current_coordinate_all_drones[index])
        cost = total_cost(drones_coords, dest_coords)
        if(cost < best_cost):
            best_cost = cost
            best_drones = drones_coords
    return [best_drones, best_cost]

# choose which drone to shape
def choose_drones(drones, coords):
    drone_combinations = list(itertools.combinations(drones, int(len(coords))))
    
    # Find the minimum path cost among all subsets
    min_cost = float('inf')
    min_subset = None
    for subset in drone_combinations:
        subset_cost = total_cost(subset, coords)
        if subset_cost < min_cost:
            min_cost = subset_cost
            min_subset = subset

    return min_subset

# Define the Euclidean distance between two points
def euclidean_distance(point1, point2):
    return math.sqrt((point1[X] - point2[X]) ** 2 + (point1[Y] - point2[Y]) ** 2)

# Define the cost function for a path
def path_cost(path):
    cost = 0
    for i in range(len(path) - 1):
        cost += euclidean_distance(path[i], path[i+1])
    return cost

def updateLTZero(postXYCoors):
    minX = 0
    minY = 0
    for idx in range(len(postXYCoors)):
        if postXYCoors[idx][X] < minX:
            minX = postXYCoors[idx][X]
        if postXYCoors[idx][Y] < minY:
            minY = postXYCoors[idx][Y]
    if minX < 0:
        for idx in range(len(postXYCoors)):
            postXYCoors[idx][X] -= minX
    if minY < 0:
        for idx in range(len(postXYCoors)):
            postXYCoors[idx][Y] -= minY
    return postXYCoors

def find_nearest_drones(current_coordinates, destinations):
    drone_count = len(current_coordinates)
    destination_count = len(destinations)
    if destination_count > drone_count:
        raise ValueError("There are more destinations than drones.")

    drones = []
    total = 0
    
    for i in range(destination_count):
        destination = destinations[i]
        nearest_drone = None
        nearest_distance = float('inf')

        for j in range(drone_count):
            drone = current_coordinates[j]
            distance = math.sqrt((destination[0] - drone[0])**2 + (destination[1] - drone[1])**2)

            if distance < nearest_distance:
                nearest_distance = distance
                nearest_drone = j

        drones.insert(len(drones), current_coordinates[nearest_drone])
        total += nearest_distance
        current_coordinates.pop(nearest_drone)
        drone_count -= 1

    return [drones, total]

def super_abb(drones, new_coords):
    n = len(new_coords)
    min_cost = float('inf')
    min_path = None

    # Generate all possible permutations of new coordinates
    coord_permutations = list(itertools.permutations(new_coords))

    # Iterate over each permutation
    for coord_permutation in coord_permutations:
        current_cost = 0
        path = []

        # Calculate the total cost for the current permutation
        for i in range(n):
            current_cost += euclidean_distance(drones[i], coord_permutation[i])
            path.append((i, coord_permutation[i]))

        # Update the minimum cost and path if the current cost is lower
        if current_cost < min_cost:
            min_cost = current_cost
            min_path = path

    # Extract the new coordinates for each drone from the minimum path
    new_coords = [None] * n
    for i, coord in min_path:
        new_coords[i] = coord

    return new_coords

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    splitData = data.data.split(':')
    if splitData[0] == 'takeoff':
        manageDrone.updateReady(int(splitData[1]) - 2)
    elif splitData[0] == 'reached':
        manageDrone.updateSignal(int(splitData[1]) - 2)

def checkDistance(arr, XSubAt, YSubAt):
    newArr = []
    preXChange = False
    preYChange = False
    for i in range(len(arr)):
        if(i != 0):
            pre = arr[i - 1]
            cur = arr[i]
            item = []
            item.insert(X, newArr[i - 1][X])
            item.insert(Y, newArr[i - 1][Y])
            if i in XSubAt:
                if(abs(pre[X] - cur[X]) > 100 and abs(pre[X] - cur[X]) < MIN_DISTANCE):
                    item[X] -= MIN_DISTANCE
                    preXChange = True
                else:
                    if(preXChange):
                        item[X] += cur[X] - pre[X]
                    else:
                        item[X] = pre[X] + cur[X] - pre[X]
                        preXChange = False
            else:
                if(abs(pre[X] - cur[X]) > 100 and abs(pre[X] - cur[X]) < MIN_DISTANCE):
                    item[X] += MIN_DISTANCE
                    preXChange = True
                else:
                    if(preXChange):
                        item[X] += cur[X] - pre[X]
                    else:
                        item[X] = pre[X] + cur[X] - pre[X]
                        preXChange = False
            if i in YSubAt:
                if(abs(pre[Y] - cur[Y]) > 100 and abs(pre[Y] - cur[Y]) < MIN_DISTANCE):
                    item[Y] -= MIN_DISTANCE
                    preYChange = True
                else:
                    if(preYChange):
                        item[Y] += cur[Y] - pre[Y]
                    else:
                        item[Y] = pre[Y] + cur[Y] - pre[Y]
                        preYChange = False
            else:
                if(abs(pre[Y] - cur[Y]) > 100 and abs(pre[Y] - cur[Y]) < MIN_DISTANCE):
                    item[Y] += MIN_DISTANCE
                    preYChange = True
                else:
                    if(preYChange):
                        item[Y] += cur[Y] - pre[Y]
                    else:
                        item[Y] = pre[Y] + cur[Y] - pre[Y]
                        preYChange = False
            newArr.insert(i, item)
        else:
            newArr.insert(i, arr[i])
    return newArr

def mirror(dataCoordinate):
    splitCoors = dataCoordinate.split()
    coords = []
    for item in enumerate(splitCoors):
        coord = item[1]
        coordY = coord.split(',')[Y]
        newY = 2000 - int(coordY)
        coord = coord.replace(str(coordY), str(newY))
        coords.insert(len(coords), coord)
    return coords

def handleForMoreDrones(numDrones = 0, dataCoordinate = []):
    # splitCoors = dataCoordinate.split()
    splitCoors = dataCoordinate
    numTops = len(splitCoors)
    preXYCoors = []
    postXYCoors = []

    # Handle num drones use
    for idx in range(numTops):
        subCoors = []
        subCoors.insert(len(subCoors), int(splitCoors[idx].split(',')[X]))
        subCoors.insert(len(subCoors), int(splitCoors[idx].split(',')[Y]))
        preXYCoors.insert(len(preXYCoors), subCoors) 
    multiple = int((numDrones / numTops))
    realNumDrones = multiple * numTops
    if realNumDrones - 1 != manageDrone.numBefore:
        print(str(realNumDrones) + ' cc ' + str(manageDrone.numBefore))
        manageDrone.needPublish = True
    manageDrone.requestXDroneTakeOff(realNumDrones - 1)
    XsubAt = []
    YsubAt = []

    # Handle add coordinates inside each edge when num drones use greater than num vertexs
    for i in range(len(preXYCoors)):
        cur = preXYCoors[i]
        next = preXYCoors[(i + 1) % numTops]
        postXYCoors.insert(len(postXYCoors), preXYCoors[i])
        if(next[X] < cur[X]):
            for i2 in range(multiple):
                XsubAt.insert(len(XsubAt), i * multiple + i2 + 1)
        if(next[Y] < cur[Y]):
            for i2 in range(multiple):
                YsubAt.insert(len(YsubAt), i * multiple + i2 + 1)
        for j in range(multiple - 1):
            newSubCoors = []
            smallerX = cur[X] if (cur[X] < next[X]) else next[X]
            smallerY = cur[Y] if (cur[Y] < next[Y]) else next[Y]
            XCoor = round(smallerX + abs(cur[X] - next[X]) / multiple * (j + 1), 2)
            YCoor = round(smallerY + abs(cur[Y] - next[Y]) / multiple * (j + 1), 2)
            newSubCoors.insert(len(newSubCoors), XCoor)
            newSubCoors.insert(len(newSubCoors), YCoor)
            postXYCoors.insert(i * multiple + j + 1, newSubCoors)
    min_distance_between_two_coords = find_closest_coords(postXYCoors)
    scale = MIN_DISTANCE / min_distance_between_two_coords

    # Scale to prevent from collision
    postXYCoors = scale_coords(postXYCoors, scale)

    minX = float('inf')
    minY = float('inf')
    for index in range(len(postXYCoors)):
        if(postXYCoors[index][X] < minX):
            minX = postXYCoors[index][X]
        if(postXYCoors[index][Y] < minY):
            minY = postXYCoors[index][Y]
    min = minX if(minX < minY) else minY
    min = (min - 200) if(min > 200) else 0

    postXYCoors = updateFrame(postXYCoors, min)

    # Hungarian algorithm

    assignments, total_cost_var = minimize_path_cost(current_coordinate_all_drones, postXYCoors)

    total_cost_without_algorithms = total_cost(current_coordinate_all_drones, postXYCoors)

    coords = fill(assignments)
    update_position(current_coordinate_all_drones, coords)
    rospy.loginfo("Total cose without using min path: %s", str(total_cost_without_algorithms))
    rospy.loginfo("Total cose using min path: %s", str(total_cost_var))
    return coords

def pushCoorToQueue(XYCoors):
    if(len(manageDrone.coordinatesAll) < 20):
        queue = []
        for idx in range(len(XYCoors)):
            data = XYCoors[idx]
            newCoordinates = coordinates(int(data[0]) / 100, int(data[1]) / 100, HEIGHT)
            queue.insert(len(queue), newCoordinates)
        manageDrone.coordinatesAll.insert(0, queue)

def handleDataFromServer(data):
    rospy.loginfo(rospy.get_caller_id() + " I heard %s from server", data.data)
    if manageDrone.state != LANDING:
        numDrones = int(data.data.split(' - ')[0])
        if(numDrones <= 12):
            dataCoordinate = data.data.split(' - ')[1]
            dataArr = mirror(dataCoordinate)
            coordinates = handleForMoreDrones(numDrones, dataArr)
            if manageDrone.state == LANDED:
                drone.takeoff(1)
                manageDrone.updateState('drone1 ', TAKING_OFF)
                manageDrone.needPublish = True
            pushCoorToQueue(coordinates)
        else:
            rospy.loginfo('---------------              Over Load             ---------------')

def switch(topic, condition, countDownSleep, stay):
    if condition == LANDED:
        # print('---------------        Land Succesfully        ---------------')
        a = 0
    if manageDrone.state == LANDING:
        print('---------------             Landing            ---------------')
        print('---------------  You can\'t public your image  ---------------')
        if drone.check_waypoint_reached():
            manageDrone.updateState('drone1 ', LANDED)
    elif condition == PRE_LANDING:
        if drone.check_waypoint_reached():
            manageDrone.updateState('drone1 ', LANDING)
            drone.land()
    elif condition == 'takingoff':
        if manageDrone.needPublish:
            dataSend = 'takeoff: '
            for idx in range(len(manageDrone.requestTakeOff)):
                # dataSend += 'drone' + str(idx + 2) + str(manageDrone.requestTakeOff[idx]).lower() + ' '
                dataSend += 'drone' + str(idx + 2) + 'true '
            manageDrone.createData(dataSend)
            rospy.loginfo(manageDrone.dataSend)
            topic.publish(manageDrone.dataSend)
            manageDrone.needPublish = False
    elif condition == 'takeoff':
        if drone.check_waypoint_reached() and manageDrone.isReachAll():
            if countDownSleep > 0 and stay <= 0:
                if len(manageDrone.coordinatesAll) > 0:
                    manageDrone.unReachAll()
                    countDownSleep = TIME_TO_LAND
                    stay = WAIT_TIME
                    indexDestination = len(manageDrone.coordinatesAll) - 1
                    coor = manageDrone.coordinatesAll[indexDestination]
                    print('length: ' + str(indexDestination))
                    if(coor[0].coor_x == 0 and coor[0].coor_y == 0):
                        drone.set_destination(x=coor[0].coor_x, y=coor[0].coor_y, z=1, psi=coor[0].coor_psi)
                    else:
                        drone.set_destination(x=coor[0].coor_x, y=coor[0].coor_y, z=coor[0].coor_z, psi=coor[0].coor_psi)
                    data = 'master: ' + str(drone.frameData.transform.translation.x) + ' 0 0'
                    rospy.loginfo(data)
                    topic.publish(data)
                    for idx in range(1, len(coor)):
                        data = 'drone' + str(idx + 1) + ': ' + str(coor[idx].coor_x) + ' ' + str(coor[idx].coor_y) + ' ' + str(coor[idx].coor_z) + ' ' + str(coor[idx].coor_psi)
                        manageDrone.createData(data)
                        rospy.loginfo(manageDrone.dataSend)
                        topic.publish(manageDrone.dataSend)
                    manageDrone.coordinatesAll.pop(indexDestination)
                else:
                    stay -= 1
                    countDownSleep -= 1
                    print('Time remaining before sleep: ' + str(countDownSleep))
            elif countDownSleep > 0 and stay > 0:
                stay -= 1
                countDownSleep -= 1
                print('Time remaining before nexr shape: ' + str(stay))
                print('Time remaining before sleep: ' + str(countDownSleep))
            else:
                data = "land"
                rospy.loginfo(data)
                topic.publish(data)
                drone.set_destination(x=0, y=0, z=1, psi=0)
                manageDrone.updateState('drone1 ', PRE_LANDING)
                # rospy.loginfo(CGREEN2 + "All waypoints reached landing now." + CEND)
        # elif stay % 4 == 0:
            # rospy.loginfo(manageDrone.dataSend)
            # topic.publish(manageDrone.dataSend)
            # a = 1
        # stay = (stay + 1) % 16
    elif condition == 'landed':
        print('---Landed---')
    return (countDownSleep, stay)

def main():
    # Initializing ROS node.
    rospy.init_node("drone1", anonymous=True)
    chatter_pub = rospy.Publisher(name='master', data_class=String, queue_size=100)

    # Subscribe for all sub drones
    rospy.Subscriber(name="/drone2/drone2_listen", data_class=String, queue_size=10, callback=callback)
    rospy.Subscriber(name="/drone3/drone3_listen", data_class=String, queue_size=10, callback=callback)
    rospy.Subscriber(name="/drone4/drone4_listen", data_class=String, queue_size=10, callback=callback)
    rospy.Subscriber(name="/drone5/drone5_listen", data_class=String, queue_size=10, callback=callback)
    rospy.Subscriber(name="/drone6/drone6_listen", data_class=String, queue_size=10, callback=callback)
    rospy.Subscriber(name="/drone7/drone7_listen", data_class=String, queue_size=10, callback=callback)
    rospy.Subscriber(name="/drone8/drone8_listen", data_class=String, queue_size=10, callback=callback)
    rospy.Subscriber(name="/drone9/drone9_listen", data_class=String, queue_size=10, callback=callback)
    rospy.Subscriber(name="/drone10/drone10_listen", data_class=String, queue_size=10, callback=callback)
    rospy.Subscriber(name="/server", data_class=String, queue_size=10, callback=handleDataFromServer)

    # Wait for FCU connection.
    drone.wait4connect()
    # Wait for the mode to be switched.
    # drone.wait4start()
    drone.set_mode("GUIDED")

    # Create local reference frame.
    drone.initialize_local_frame()
    # Request takeoff with an altitude of 3m.

    # Specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish.
    rate = rospy.Rate(2)

    counter = 0
    stay = 0
    countDownSleep = TIME_TO_LAND

    ## Create a frame name 'master_map'
    buffer = tf2_ros.Buffer()

    drone.frameData.header.stamp = rospy.Time.now()
    drone.frameData.header.frame_id = 'map'

    # Set the child frame ID
    drone.frameData.child_frame_id = 'master_map'

    # Set the transform from the parent frame to the child frame
    drone.frameData.transform.rotation.x = 0
    drone.frameData.transform.rotation.y = 0
    drone.frameData.transform.rotation.z = 0
    drone.frameData.transform.rotation.w = 1
    while not rospy.is_shutdown():
        rate.sleep()
        # buffer.set_transform(drone.frameData, "default_authority")
        # drone.frameData = buffer.lookup_transform_core('map', 'master_map', rospy.Time(0))
        print(manageDrone.state)
        if manageDrone.state == LANDED:
            counter = 0
        elif counter > 8:
            (countDownSleep, stay) = switch(chatter_pub, manageDrone.state, countDownSleep, stay)
            print(str(countDownSleep) + ' --- ' + str(stay))
        elif counter == 8:
            manageDrone.updateState('drone1 ', TAKEOFF)
            counter += 1
        elif manageDrone.state == TAKING_OFF:
            switch(chatter_pub, manageDrone.state, countDownSleep, stay)
            counter += 1
        else:
            print('waiting for signal')
    # XYCoors = handleForMoreDrones(10, "111,452 318,603 199,833 433,703 838,958 650,575 855,474 613,463 534,138 397,453")
    # XYCoors = handleForMoreDrones(10, "231,112 234,438 971,436 970,111")
    # XYCoors = mirror("679,311 128,1035 809,1381")
    # handleForMoreDrones(9, XYCoors)
    # XYCoors = handleForMoreDrones(9, "679,311 128,1035 809,1381")
    # XYCoors = handleForMoreDrones(6, "679,311 128,1035 809,1381")
    # XYCoors = handleForMoreDrones(4, "415,356 153,709 783,1116 1001,790")
    # XYCoors = handleForMoreDrones(8, "415,356 153,709 783,1116 1001,790")
    # pushCoorToQueue(XYCoors[0])

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
