## Lab 2
## Contributors: Bojun Yang and Zihuan Wu

import joblib
import cozmo, time, datetime
from cozmo.util import degrees, distance_mm, speed_mmps
from skimage import io, feature, filters, exposure, color, measure
import numpy as np

class CozmoSpy:
  
  def __init__(self):
    self.run = True
    # 1 - idle; 2 - order; 3 - drone; 4 - inspection
    self.state = 1
    self.states = {'none': 1, 'order': 2, 'drone': 3, 'inspection': 4}
    self.classifier = joblib.load('classifier.pkl')

  def activate(self, robot: cozmo.robot.Robot):
    robot.say_text('activated').wait_for_completed()

    while self.run:
      next_state = 1

      if self.state == 2:
        next_state = self.order(robot)
      elif self.state == 3:
        next_state = self.drone(robot)
      elif self.state == 4:
        next_state = self.inspection(robot)
      else:
        # self.state == 1
        next_state = self.idle(robot)

      if next_state not in self.states:
        self.state = 1
      else:
        self.state = self.states[next_state]
      print("Proceeding to '{}' state".format(next_state))

  def idle(self, robot: cozmo.robot.Robot):

    robot.say_text("Idle").wait_for_completed()

    time.sleep(3)

    latest_image = robot.world.latest_image
    new_image = latest_image.raw_image

    timestamp = datetime.datetime.now().strftime("%dT%H%M%S%f")

    new_image.save("./captured_images/" + "_" + timestamp + ".bmp")
    np_image = np.array(new_image)
    img_features = extract_image_features(np_image)
    symbol = self.classifier.predict(img_features)[0]
    robot.say_text("I see a {} symbol".format(symbol)).wait_for_completed()
    print("Saw '{}' symbol".format(symbol))
    return symbol

  def order(self, robot: cozmo.robot.Robot):
    robot.say_text('Order mission').wait_for_completed()
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
    robot.drive_straight(distance_mm(300), speed_mmps(60)).wait_for_completed()
    robot.place_object_on_ground_here(cubes[0]).wait_for_completed()
    robot.drive_straight(distance_mm(-350), speed_mmps(50)).wait_for_completed()
    return 'none'

  def drone(self, robot: cozmo.robot.Robot):
    robot.say_text('Drone mission').wait_for_completed()
    time.sleep(0.5)
    turn_left = 90

    # turn robot 90 degrees to the left.
    robot.turn_in_place(degrees(turn_left)).wait_for_completed()
    robot.drive_wheels(160, 60, duration=3)
    robot.drive_wheels(60, 200, duration=3)

    robot.play_anim_trigger(cozmo.anim.Triggers.CodeLabWin).wait_for_completed()
    return 'none'

  def inspection(self, robot: cozmo.robot.Robot):
    robot.say_text('Inspection mission').wait_for_completed()
    time.sleep(0.5)
    robot.set_lift_height(0.0).wait_for_completed()
    i = 0
    self.should_move = True
    self.should_turn = False
    self.should_lift = True
    while i < 4:
      if robot.lift_position.ratio <= 0.1 and self.should_lift:
        self.should_lift = False
        robot.set_lift_height(1.0, 2.0, 5.0, 2.5, True).on_completed(self.done_lifting)
      if robot.lift_position.ratio >= 0.9 and self.should_lift:
        self.should_lift = False
        robot.set_lift_height(0.0, 2.0, 5.0, 2.5, True).on_completed(self.done_lifting)
      if self.should_move:
        robot.say_text('I am not a spy', in_parallel=True)
        self.should_move = False
        robot.drive_straight(distance_mm(200), speed_mmps(50), in_parallel = True).on_completed(self.done_moving)
      if self.should_turn:
        self.should_turn = False
        if i == 3:
          robot.turn_in_place(degrees(90), in_parallel=True).wait_for_completed()
        else:
          robot.turn_in_place(degrees(90), in_parallel=True).on_completed(self.done_turning)
        i+=1
    robot.wait_for_all_actions_completed()
    robot.set_lift_height(0.0).wait_for_completed()
    return 'none'
  
  def done_lifting(self, evt, *, failure_reason, state, action, **kwargs):
    self.should_lift = True

  def done_moving(self, evt, *, failure_reason, state, action, **kwargs):
    self.should_move = False
    self.should_turn = True

  def done_turning(self, evt, *, failure_reason, state, action, **kwargs):
    self.should_move = True
    self.should_turn = False

def extract_image_features(img):
  # extract feature vector from image data
  feature_data = []
  y_max, x_max, _ = img.shape
  greyscaled = filters.gaussian(img[:,:,0], sigma=1)
  hog_descriptor = feature.hog(greyscaled, orientations=10, pixels_per_cell=(64,64), cells_per_block=(2,1), block_norm='L2-Hys', visualize=False)
  feature_data.append(hog_descriptor)

  feature_data = np.vstack(feature_data)
  return(feature_data)

agent = CozmoSpy()
cozmo.run_program(agent.activate, use_viewer=True, force_viewer_on_top=False)