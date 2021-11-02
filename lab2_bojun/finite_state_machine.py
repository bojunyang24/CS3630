import re
import sys
import numpy as np
import cozmo
import time

from imgclassification.py import ImageClassifier
from sklearn import svm, metrics
from skimage import io, feature, filters, exposure, color
from cozmo.util import degrees, distance_mm, speed_mmps

def idle(robot: cozmo.robot.Robot):



def order(robot: cozmo.robot.Robot):
	robot.say_text("order").wait_for_completed()
    time.sleep(0.5)

    lookaround = robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
    cubes = robot.world.wait_until_observe_num_objects(num=1, object_type=cozmo.objects.LightCube, timeout=60)
    lookaround.stop()

    if len(cubes) < 1:
        print("Error: need 1 Cubes but only found", len(cubes), "Cube(s)")
    else:
        action = robot.pickup_object(cubes[0], num_retries=5)
    action.wait_for_completed()
    if action.has_failed:
        code, reason = action.failure_reason
        result = action.result
        print("Pickup Cube failed: code=%s reason=‘%s’ result=%s" % (code, reason, result))
    robot.drive_straight(distance_mm(330), speed_mmps(60)).wait_for_completed()
    robot.place_object_on_ground_here(cubes[0]).wait_for_completed()
    robot.drive_straight(distance_mm(-490), speed_mmps(50)).wait_for_completed()

	idle(robot)

def drone(robot: cozmo.robot.Robot):
	robot.say_text("drone").wait_for_completed()
    time.sleep(0.5)
	turn_left = 90 
 
    # turn robot 90 degrees to the left.
    robot.turn_in_place(degrees(turn_left)).wait_for_completed()
    robot.drive_wheels(80, 40, duration=8)
    robot.drive_wheels(40, 80, duration=8)

    robot.play_anim_trigger(cozmo.anim.Triggers.CodeLabWin).wait_for_completed()
	idle(robot)


def inspection(robot: cozmo.robot.Robot):
	robot.say_text('inspection').wait_for_completed()
    time.sleep(0.5)
    for t in range(8)
        if t % 2 == 0:
            robot.move_lift(1)
        else:
            robot.move_lift(-1)
        robot.drive_straight(distance_mm(200), speed_mmps(50), in_parallel = True)
        robot.say_text('I am not a spy', in_parallel=True).wait_for_completed()
        robot.turn_in_place(degrees(90), in_parallel=True).wait_for_completed()

	idle(robot)
