#!/usr/bin/env python

import rospy
import rospkg
import os
import sys
import yaml
import random
import time
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

yamlpath = 'pu2_data.yaml'

if __name__ == '__main__':

    # Initialize rospack
    rospack = rospkg.RosPack()

    # Get path to yaml
    pu2_path = rospack.get_path('lab2pkg_py')
    yamlpath = os.path.join(pu2_path, 'scripts', 'pu2_data.yaml')

    with open(yamlpath, 'r') as f:
        try:
	    print("trying to open yaml... ")
            # Load the data as a dict
            data = yaml.load(f)
            # Load block position
            block_xy_pos = data['block_xy_pos']
	    tray_pos = data['tray_pos']
	    bowl_pos = data['bowl_pos']
            
        except:
	    print("couldn't open... ")
            sys.exit()

    # Initialize ROS node
    rospy.init_node('ur3_gazebo_spawner', anonymous=True)

    # Initialize ROS pack
    rospack = rospkg.RosPack()

    ## Get file path for stuff we need
    ur_path = rospack.get_path('ur_description')

    #rice_path = os.path.join(ur_path, 'urdf', 'rice.urdf')
    chicken_path = os.path.join(ur_path, 'urdf', 'chicken.urdf')
    cheese_path = os.path.join(ur_path, 'urdf', 'cheese.urdf')
    lettuce_path = os.path.join(ur_path, 'urdf', 'lettuce.urdf')
    steak_path = os.path.join(ur_path, 'urdf', 'steak.urdf')
    blob1_path = os.path.join(ur_path, 'urdf', 'white_blob.urdf')
    blob2_path = os.path.join(ur_path, 'urdf', 'black_blob.urdf')
    #blob3_path = os.path.join(ur_path, 'urdf', 'green_blob.urdf')
    blob4_path = os.path.join(ur_path, 'urdf', 'yellow_blob.urdf')
    tray_path = os.path.join(ur_path, 'urdf', 'tray.urdf')
    bowl_path = os.path.join(ur_path, 'urdf', 'bowl.urdf')

    print("Got paths. ")

    # sys.exit()

    # Wait for service to start
    rospy.wait_for_service('gazebo/spawn_urdf_model')
    spawn = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)
    delete = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    '''
    # Spawn trays
    trays = ['tray1','tray2','tray3','tray4','tray5','tray6']

    pose1 = Pose(Point(tray_pos[0][0][0],tray_pos[0][0][1],0), Quaternion(0,0,0,0))
    pose2 = Pose(Point(tray_pos[0][1][0],tray_pos[0][1][1],0), Quaternion(0,0,0,0))
    pose3 = Pose(Point(tray_pos[1][0][0],tray_pos[1][0][1],0), Quaternion(0,0,0,0))
    pose4 = Pose(Point(tray_pos[1][1][0],tray_pos[1][1][1],0), Quaternion(0,0,0,0))
    pose5 = Pose(Point(tray_pos[2][0][0],tray_pos[2][0][1],0), Quaternion(0,0,0,0))
    pose6 = Pose(Point(tray_pos[2][1][0],tray_pos[2][1][1],0), Quaternion(0,0,0,0))

    spawn(trays[0], open(tray_path, 'r').read(), 'block', pose1, 'world')
    print("spawned block 1")
    time.sleep(0.5);
    spawn(trays[1], open(tray_path, 'r').read(), 'block', pose2, 'world')
    print("spawned block 2")
    time.sleep(0.5);
    spawn(trays[2], open(tray_path, 'r').read(), 'block', pose3, 'world')
    print("spawned block 3")
    time.sleep(0.5);
    spawn(trays[3], open(tray_path, 'r').read(), 'block', pose4, 'world')
    print("spawned block 4")
    time.sleep(0.5);
    spawn(trays[4], open(tray_path, 'r').read(), 'block', pose5, 'world')
    print("spawned block 5")
    time.sleep(0.5);
    spawn(trays[5], open(tray_path, 'r').read(), 'block', pose6, 'world')
    print("Trays done")  
    '''

    # Delete and spawn food
    food = ['White_Rice','Brown_Rice','Chicken','Steak','Cheese','Lettuce']

    pose7 = Pose(Point(tray_pos[0][0][0],tray_pos[0][0][1],0.05), Quaternion(0,0,0,0))
    pose8 = Pose(Point(tray_pos[0][1][0],tray_pos[0][1][1],0.05), Quaternion(0,0,0,0))
    pose9 = Pose(Point(tray_pos[1][0][0],tray_pos[1][0][1],0.02), Quaternion(0,0,0,0))
    pose10 = Pose(Point(tray_pos[1][1][0],tray_pos[1][1][1],0.02), Quaternion(0,0,0,0))
    pose11 = Pose(Point(tray_pos[2][0][0],tray_pos[2][0][1],0.05), Quaternion(0,0,0,0))
    pose12 = Pose(Point(tray_pos[2][1][0],tray_pos[2][1][1],0.02), Quaternion(0,0,0,0))

    spawn(food[0], open(blob1_path, 'r').read(), 'block', pose7, 'world')
    print("spawned block 1")
    time.sleep(0.5);
    spawn(food[1], open(blob2_path, 'r').read(), 'block', pose8, 'world')
    print("spawned block 2")
    time.sleep(0.5);
    spawn(food[2], open(chicken_path, 'r').read(), 'block', pose9, 'world')
    print("spawned block 3")
    time.sleep(0.5);
    spawn(food[3], open(steak_path, 'r').read(), 'block', pose10, 'world')
    print("spawned block 4")
    time.sleep(0.5);
    spawn(food[4], open(blob4_path, 'r').read(), 'block', pose11, 'world')
    print("spawned block 5")
    time.sleep(0.5);
    spawn(food[5], open(lettuce_path, 'r').read(), 'block', pose12, 'world')
    print("Food done") 

    # Spawn bowls

    for x in range(1):
	bowl = 'bowl' + str(x+1)
	delete(bowl)
	pose = Pose(Point(bowl_pos[0][0][0],bowl_pos[0][0][1],0),Quaternion(0,0,0,0))
	spawn(bowl, open(bowl_path, 'r').read(), 'block', pose, 'world')
	print(bowl) 
    
    '''
    # Delete and Spawn test object
    bowl = 'bowl'
    pose = Pose(Point(tobj_pos[0][0][0],tobj_pos[0][0][1],0), Quaternion(0,0,0,0))
    spawn(bowl, open(bowl_path, 'r').read(), 'block', pose, 'world')
    print("test object done")
    '''
   
    '''
    # Starting location ?
    starting_location = None
    while not starting_location:
        starting_location = raw_input("Enter starting location number <Either 1 2 or 3>: ")
        starting_location = int(starting_location)
        if (starting_location != 1) and (starting_location != 2) and (starting_location != 3):
            starting_location = None
            print("Wrong input \n\n")

    # 0-indexed
    starting_location -= 1

    # Missing block stuff if you want it
    # Missing block ?
    missing_block = None
    while missing_block is None:
        missing_block = raw_input("Missing Block?(y/n): ")
        missing_block = str(missing_block)
        if (missing_block != 'y') and (missing_block != 'n'):
            missing_block = None
            print("Wrong input \n\n")
        
    missing_block = (missing_block == 'y')
    
    # Delete previous blocks
    for height in range(3):
        block_name = 'block' + str(height + 1)
        delete(block_name)

    #Missing block stuff if you want it
    if not missing_block:
        # Spawn three blocks
        for height in range(3):
            block_name = 'block' + str(height + 1)
            pose = Pose(Point(block_xy_pos[starting_location][height][0], 
                            block_xy_pos[starting_location][height][1], 0), Quaternion(0, 0, 0, 0))
            spawn(block_name, open(block_paths[2-height], 'r').read(), 'block', pose, 'world')
    
    else:
        missing_block_height = random.randint(0, 2)
        # Spawn two blocks
        for height in range(3):
            if height == missing_block_height:
                continue
            block_name = 'block' + str(height + 1)
            pose = Pose(Point(block_xy_pos[starting_location][height][0], 
                            block_xy_pos[starting_location][height][1], 0), Quaternion(0, 0, 0, 0))
            spawn(block_name, open(block_paths[2-height], 'r').read(), 'block', pose, 'world')
   
    for height in range(3):
            block_name = 'block' + str(height + 1)
            pose = Pose(Point(block_xy_pos[starting_location][height][0], 
                            block_xy_pos[starting_location][height][1], 0), Quaternion(0, 0, 0, 0))
            spawn(block_name, open(block_paths[2-height], 'r').read(), 'block', pose, 'world')
    '''
    

