## Lab 6
## Contributors: Bojun Yang and Zihuan Wu
import cozmo
from cozmo.util import degrees, distance_mm, speed_mmps
import math
import sys
import time
import random

from cmap import *
from gui_rrt import *
from utils import *
from go_to_goal_cozmo import *

MAX_NODES = 20000

last_pose = cozmo.util.Pose(0,0,0,angle_z=cozmo.util.Angle(degrees=0))
flag_odom_init = False

Map_filename = "map_arena.json"
grid = CozGrid(Map_filename)
gui = GUIWindow(grid, show_camera=True)
pf = ParticleFilter(grid)
pickup = (5, 8, 270);
pickup_x, pickup_y, pickup_h = pickup
storage = (22, 13, 270)
storage_x, storage_y, storage_h = storage

def step_from_to(node0, node1, limit=75):
    ########################################################################
    # TODO: please enter your code below.
    # 1. If distance between two nodes is less than limit, return node1
    # 2. Otherwise, return a node in the direction from node0 to node1 whose
    #    distance to node0 is limit. Recall that each iteration we can move
    #    limit units at most
    # 3. Hint: please consider using np.arctan2 function to get vector angle
    # 4. Note: remember always return a Node object
    
    
    #temporary cod below to be replaced
    dist = get_dist(node0, node1)
    if dist < limit:
        return node1
    return Node(
        (
            node0.x + (node1.x - node0.x) * limit / dist,
            node0.y + (node1.y - node0.y) * limit / dist
        )
    )
    ############################################################################

    
    
    


def node_generator(cmap):
    rand_node = None
    ############################################################################
    # TODO: please enter your code below.
    # 1. Use CozMap width and height to get a uniformly distributed random node
    # 2. Use CozMap.is_inbound and CozMap.is_inside_obstacles to determine the
    #    legitimacy of the random node.
    # 3. Note: remember always return a Node object
    
    
    #temporary cod below to be replaced
    if random.random() < 0.05:
        goal = cmap.get_goals()[0]
        return Node((goal.x, goal.y))
    w, h = cmap.get_size()
    while rand_node == None:
        rand_node = Node((random.random() * w, random.random() * h))
        if not cmap.is_inbound(rand_node) or cmap.is_inside_obstacles(rand_node):
            rand_node = None
        else:
            return rand_node
    ############################################################################
    


def RRT(cmap, start):
    cmap.add_node(start)
    map_width, map_height = cmap.get_size()
    while (cmap.get_num_nodes() < MAX_NODES):
        ########################################################################
        # TODO: please enter your code below.
        # 1. Use CozMap.get_random_valid_node() to get a random node. This
        #    function will internally call the node_generator above
        # 2. Get the nearest node to the random node from RRT
        # 3. Limit the distance RRT can move
        # 4. Add one path from nearest node to random node
        #
        rand_node = cmap.get_random_valid_node()
        nearest_node = None
        shortest = float('inf')

        for node in cmap.get_nodes():
            if get_dist(node, rand_node) < shortest:
                shortest = get_dist(node, rand_node)
                nearest_node = node
        rand_node = step_from_to(nearest_node, rand_node)
        ########################################################################
        
        
        time.sleep(0.01)
        cmap.add_path(nearest_node, rand_node)
        if cmap.is_solved():
            break

    path = cmap.get_path()
    smoothed_path = cmap.get_smooth_path()

    if cmap.is_solution_valid():
        print("A valid solution has been found :-) ")
        print("Nodes created: ", cmap.get_num_nodes())
        print("Path length: ", len(path))
        print("Smoothed path length: ", len(smoothed_path))
    else:
        print("Please try again :-(")



async def CozmoPlanning(robot: cozmo.robot.Robot):
    # Allows access to map and stopevent, which can be used to see if the GUI
    # has been closed by checking stopevent.is_set()
    global cmap, stopevent
    global flag_odom_init, last_pose
    global grid, gui, pf

    # particle filter stuff
    # start streaming

    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = False
    robot.camera.enable_auto_exposure()
    await robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()

    fx, fy = robot.camera.config.focal_length.x_y
    cx, cy = robot.camera.config.center.x_y
    camera_settings = np.array([
        [fx,  0, cx],
        [ 0, fy, cy],
        [ 0,  0,  1]
    ], dtype=np.float)

    execute = True
    convergence = False
    looking = False
    odometry_info = compute_odometry(robot.pose)
    last_pose = robot.pose
    markers, camera_image = await marker_processing(robot, camera_settings, show_diagnostic_image=False)
    original_pose = robot.pose
    

    while execute:
        if robot.is_picked_up:
            print("robot is picked up")
            looking = False
            time.sleep(0.5)
            last_pose = cozmo.util.Pose(0,0,0,angle_z=cozmo.util.Angle(degrees=0))
            convergence = False
            pf = ParticleFilter(grid)
            await robot.drive_straight(cozmo.util.distance_mm(0.1), cozmo.util.speed_mmps(10.0)).wait_for_completed()
            continue

        odometry_info = compute_odometry(robot.pose)
        last_pose = robot.pose
        markers, camera_image = await marker_processing(robot, camera_settings, show_diagnostic_image=False)

        if not convergence:
            # do converging stuff
            m_x, m_y, m_h, converged = pf.update(odometry_info, markers)

            if converged:
                robot.stop_all_motors()
                x,y,h = m_x, m_y, m_h
                looking = False
                goal_x, goal_y, goal_h = goal
                dx, dy = goal_x - m_x, goal_y - m_y
                convergence = True
                execute = False
                # turn to 0 degrees
                print('turning to 0 degrees')
                await robot.go_to_pose(original_pose).wait_for_completed()

                await robot.turn_in_place(cozmo.util.degrees(diff_heading_deg(180, 0)), num_retries=2).wait_for_completed()
                await robot.drive_straight(cozmo.util.distance_inches(8), cozmo.util.speed_mmps(60)).wait_for_completed()
                await robot.turn_in_place(cozmo.util.degrees(diff_heading_deg(180, 270)), num_retries=2).wait_for_completed()
                print('hi')
            else:
                # actively look around
                if not looking:
                    robot.drive_wheel_motors(-30, 30)
                    looking = True
                # await robot.turn_in_place(cozmo.util.degrees(20)).wait_for_completed()
        else:
            if robot.is_picked_up:
                robot.stop_all_motors()
                looking = False
                await robot.drive_wheels(0.0, 0,0)
                convergence = False
                pf = ParticleFilter(grid)
                await robot.drive_straight(cozmo.util.distance_mm(0.1), cozmo.util.speed_mmps(10.0)).wait_for_completed()

    ########################################################################
    # TODO: please enter your code below.
    # Description of function provided in instructions. Potential pseudcode is below

    #assume start position is in cmap and was loaded from emptygrid.json as [50, 35] already
    #assume start angle is 0
    # start_node = Node((50, 35))
    await robot.say_text("I am ready to begin delivery!").wait_for_completed()
    start_node = Node((100, 200)) # 4, 8
    delivery_node = Node((550, 200)) # 22, 8
    #Add final position as goal point to cmap, with final position being defined as a point that is at the center of the arena 
    #you can get map width and map weight from cmap.get_size()
    map_w, map_h = cmap.get_size()
    cmap.set_start(start_node)
    cmap.add_goal(delivery_node)
    
    #reset the current stored paths in cmap
    cmap.reset_paths()
    #call the RRT function using your cmap as input, and RRT will update cmap with a new path to the target from the start position
    RRT(cmap, cmap.get_start())
    #get path from the cmap
    path = cmap.get_smooth_path()
    path = cmap.get_path()
    
    go = True
    picking_up = True
    delivering = False
    cubes = []

    while go:
        if picking_up:
            saved_pose = robot.pose
            # await robot.go_to_pose(saved_pose).wait_for_completed()
            await robot.say_text('give').wait_for_completed()
            time.sleep(0.5)
            lookaround = robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
            cubes = await robot.world.wait_until_observe_num_objects(num=1, object_type=cozmo.objects.LightCube, timeout=60)
            lookaround.stop()
            if len(cubes) < 1:
                print("Error: need 1 Cubes but only found", len(cubes), "Cube(s)")
            else:
                await robot.pickup_object(cubes[0], num_retries=5).wait_for_completed()
                await robot.go_to_pose(saved_pose).wait_for_completed()
                # go to delivery
                await follow_path(robot, path, delivery_node)
                picking_up, delivering = delivering, picking_up
        if delivering:
            saved_pose = robot.pose
            await robot.drive_straight(cozmo.util.distance_inches(4), cozmo.util.speed_mmps(90)).wait_for_completed()
            await robot.place_object_on_ground_here(cubes[0]).wait_for_completed()
            # await robot.drive_straight(cozmo.util.distance_inches(-4), cozmo.util.speed_mmps(90)).wait_for_completed()
            # robot.drive_straight(cozmo.util.distance_inches(2), cozmo.util.speed_mmps(60)).wait_for_completed()
            await robot.go_to_pose(saved_pose).wait_for_completed()
            # go back to pick up area
            await follow_path(robot, path[::-1], start_node)
            picking_up, delivering = delivering, picking_up
    
    # marked = {}
    # update_cmap = False

    # i = 0
    # curr_node = path[i]
    
    # up_angle = robot.pose.rotation.angle_z.degrees
    # await robot.turn_in_place(cozmo.util.degrees(-90), num_retries=2).wait_for_completed()
    # curr_robot_angle = 0
    
    # #while the current cosmo position is not at the goal:
    # while curr_node.x != delivery_node.x or curr_node.y != delivery_node.y:
    #     #break if path is none or empty, indicating no path was found
    #     if path == None or len(path) == 0:
    #         break
        
    #     # Get the next node from the path
    #     i+=1
    #     next_node = path[i]
    #     #drive the robot to next node in path. #First turn to the appropriate angle, and then move to it
    #     #you can calculate the angle to turn through a trigonometric function
    #     # TODO: the angle calculatin is wrong. redo it
    #     dx, dy = next_node.x - curr_node.x, next_node.y - curr_node.y
    #     angle_wrt_global = math.atan2(dy, dx)
    #     angle_wrt_robot = angle_wrt_global - curr_robot_angle
    #     await robot.turn_in_place(cozmo.util.radians(angle_wrt_robot), is_absolute=False).wait_for_completed()
    #     await robot.drive_straight(cozmo.util.distance_mm(get_dist(next_node, curr_node)), cozmo.util.speed_mmps(50)).wait_for_completed()
    #     # Update the current Cozmo position (cozmo_pos and cozmo_angle) to be new node position and angle 
    #     curr_node = next_node
    #     curr_robot_angle = angle_wrt_global

    # # turn towards delivery
    # robot.turn_in_place(cozmo.util.degrees(diff_heading_deg(up_angle, robot.pose.rotation.angle_z.degrees)), num_retries=2).wait_for_completed()
    ########################################################################
    
async def follow_path(robot, path, goal_node):
    i = 0
    curr_node = path[i]
    
    up_angle = robot.pose.rotation.angle_z.degrees
    await robot.turn_in_place(cozmo.util.degrees(-90), num_retries=2).wait_for_completed()
    curr_robot_angle = 0
    
    #while the current cosmo position is not at the goal:
    while curr_node.x != goal_node.x or curr_node.y != goal_node.y:
        #break if path is none or empty, indicating no path was found
        if path == None or len(path) == 0:
            break
        
        # Get the next node from the path
        i+=1
        next_node = path[i]
        #drive the robot to next node in path. #First turn to the appropriate angle, and then move to it
        #you can calculate the angle to turn through a trigonometric function
        # TODO: the angle calculatin is wrong. redo it
        dx, dy = next_node.x - curr_node.x, next_node.y - curr_node.y
        angle_wrt_global = math.atan2(dy, dx)
        angle_wrt_robot = angle_wrt_global - curr_robot_angle
        await robot.turn_in_place(cozmo.util.radians(angle_wrt_robot), is_absolute=False).wait_for_completed()
        await robot.drive_straight(cozmo.util.distance_mm(get_dist(next_node, curr_node)), cozmo.util.speed_mmps(90)).wait_for_completed()
        # Update the current Cozmo position (cozmo_pos and cozmo_angle) to be new node position and angle 
        curr_node = next_node
        curr_robot_angle = angle_wrt_global

    # turn towards up
    await robot.turn_in_place(cozmo.util.degrees(diff_heading_deg(up_angle, robot.pose.rotation.angle_z.degrees)), num_retries=2).wait_for_completed()
    
def get_global_node(local_angle, local_origin, node):
    """Helper function: Transform the node's position (x,y) from local coordinate frame specified by local_origin and local_angle to global coordinate frame.
                        This function is used in detect_cube_and_update_cmap()
        Arguments:
        local_angle, local_origin -- specify local coordinate frame's origin in global coordinate frame
        local_angle -- a single angle value
        local_origin -- a Node object
        Outputs:
        new_node -- a Node object that decribes the node's position in global coordinate frame
    """
    ########################################################################
    # TODO: please enter your code below.
    
    cos, sin = math.cos(local_angle), math.sin(local_angle)
    return Node(
        (
            node.x * cos - node.y * sin + local_origin.x,
            node.y * sin + node.y * cos + local_origin.y
        )
    )
    ########################################################################


async def detect_cube_and_update_cmap(robot, marked, cozmo_pos):
    """Helper function used to detect obstacle cubes and the goal cube.
       1. When a valid goal cube is detected, old goals in cmap will be cleared and a new goal corresponding to the approach position of the cube will be added.
       2. Approach position is used because we don't want the robot to drive to the center position of the goal cube.
       3. The center position of the goal cube will be returned as goal_center.

        Arguments:
        robot -- provides the robot's pose in G_Robot
                 robot.pose is the robot's pose in the global coordinate frame that the robot initialized (G_Robot)
                 also provides light cubes
        cozmo_pose -- provides the robot's pose in G_Arena
                 cozmo_pose is the robot's pose in the global coordinate we created (G_Arena)
        marked -- a dictionary of detected and tracked cubes (goal cube not valid will not be added to this list)

        Outputs:
        update_cmap -- when a new obstacle or a new valid goal is detected, update_cmap will set to True
        goal_center -- when a new valid goal is added, the center of the goal cube will be returned
    """
    global cmap

    # Padding of objects and the robot for C-Space
    cube_padding = 40.
    cozmo_padding = 100.

    # Flags
    update_cmap = False
    goal_center = None

    # Time for the robot to detect visible cubes
    time.sleep(1)

    for obj in robot.world.visible_objects:

        if obj.object_id in marked:
            continue

        # Calculate the object pose in G_Arena
        # obj.pose is the object's pose in G_Robot
        # We need the object's pose in G_Arena (object_pos, object_angle)
        dx = obj.pose.position.x - robot.pose.position.x
        dy = obj.pose.position.y - robot.pose.position.y

        object_pos = Node((cozmo_pos.x+dx, cozmo_pos.y+dy))
        object_angle = obj.pose.rotation.angle_z.radians

        # Define an obstacle by its four corners in clockwise order
        obstacle_nodes = []
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((cube_padding, cube_padding))))
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((cube_padding, -cube_padding))))
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((-cube_padding, -cube_padding))))
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((-cube_padding, cube_padding))))
        cmap.add_obstacle(obstacle_nodes)
        marked[obj.object_id] = obj
        update_cmap = True

    return update_cmap, goal_center, marked


class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        # Please refrain from enabling use_viewer since it uses tk, which must be in main thread
        cozmo.run_program(CozmoPlanning,use_3d_viewer=False, use_viewer=False)
        stopevent.set()


class RRTThread(threading.Thread):
    """Thread to run RRT separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        while not stopevent.is_set():
            RRT(cmap, cmap.get_start())
            time.sleep(100)
            cmap.reset_paths()
        stopevent.set()


if __name__ == '__main__':
    global cmap, stopevent
    stopevent = threading.Event()
    robotFlag = False
    for i in range(0,len(sys.argv)): #reads input whether we are running the robot version or not
        if (sys.argv[i] == "-robot"):
            robotFlag = True
    if (robotFlag):
        #creates cmap based on empty grid json
        #"start": [50, 35],
        #"goals": [] This is empty
        cmap = CozMap("map.json", node_generator) 
        robot_thread = RobotThread()
        robot_thread.start()
    else:
        cmap = CozMap("map.json", node_generator)
        sim = RRTThread()
        sim.start()
    visualizer = Visualizer(cmap)
    visualizer.start()
    stopevent.set()
