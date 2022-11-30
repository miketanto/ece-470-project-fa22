#!/usr/bin/env python

import rospy
import rospkg
import os
import sys
import yaml
import random
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
    print("PU2 PATH")
    print(pu2_path)
    print("END PU2 PATH")
    yamlpath = os.path.join(pu2_path, 'scripts', 'pu2_data.yaml')

    with open(yamlpath, 'r') as f:
        try:
            # Load the data as a dict
            data = yaml.load(f)
            # Load block position
            block_xy_pos = data['block_xy_pos']
            blob_pos = data['blob_pos']
	    tobj_pos = data['tobj_pos']
            
        except:
            sys.exit()

    # Initialize ROS node
    rospy.init_node('ur3_gazebo_spawner', anonymous=True)
    # Initialize ROS pack
    rospack = rospkg.RosPack()
    # Get path to block
    ur_path = rospack.get_path('ur_description')
    print("UR PATH")
    print(ur_path)
    print("END UR PATH")
    block_path = os.path.join(ur_path, 'urdf', 'block.urdf')
    block1_path = os.path.join(ur_path, 'urdf', 'block_red.urdf')
    block2_path = os.path.join(ur_path, 'urdf', 'block_yellow.urdf')
    block3_path = os.path.join(ur_path, 'urdf', 'block_green.urdf')
    block_paths = [block1_path, block2_path, block3_path]
    blob_path = os.path.join(ur_path, 'urdf', 'blob.urdf')
    tobj_path = os.path.join(ur_path, 'urdf', 'bowl.urdf')
    
    print("Got paths")
    # Wait for service to start
    rospy.wait_for_service('gazebo/spawn_urdf_model')
    spawn = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)
    delete = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    
    # Delete and Spawn test blob 
    blob = 'blob'
    delete(blob)
    pose = Pose(Point(blob_pos[0][0][0],blob_pos[0][0][1],0), Quaternion(0,0,0,0))
    spawn(blob, open(blob_path, 'r').read(), 'block', pose, 'world')
    print("Blobs done")    
    
    # Delete and Spawn test object
    tobj = 'bowl'
    pose = Pose(Point(tobj_pos[0][0][0],tobj_pos[0][0][1],0), Quaternion(0,0,0,0))
    spawn(tobj, open(tobj_path, 'r').read(), 'block', pose, 'world')
    print("test object done")
   
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

    """ # Missing block stuff if you want it
    # Missing block ?
    missing_block = None
    while missing_block is None:
        missing_block = raw_input("Missing Block?(y/n): ")
        missing_block = str(missing_block)
        if (missing_block != 'y') and (missing_block != 'n'):
            missing_block = None
            print("Wrong input \n\n")
        
    missing_block = (missing_block == 'y')
    """
    # Delete previous blocks
    for height in range(3):
        block_name = 'block' + str(height + 1)
        delete(block_name)

    """ #Missing block stuff if you want it
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
    """
    for height in range(3):
            block_name = 'block' + str(height + 1)
            pose = Pose(Point(block_xy_pos[starting_location][height][0], 
                            block_xy_pos[starting_location][height][1], 0), Quaternion(0, 0, 0, 0))
            spawn(block_name, open(block_paths[2-height], 'r').read(), 'block', pose, 'world')

