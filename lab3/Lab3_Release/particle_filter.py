# Contributors
# Bojun Yang
# Wu Zihuan

from grid import CozGrid
from particle import Particle
from utils import grid_distance, rotate_point, diff_heading_deg, add_odometry_noise
import setting
import math
import numpy as np
from queue import PriorityQueue


def motion_update(particles, odom, grid):
    """ Particle filter motion update

        Arguments:
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*
        grid -- grid world map, which contains the marker information,
                see grid.py and CozGrid for definition
                Can be used for boundary checking
        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """
    
    motion_particles = []
    dx, dy, dh = odom

    if dx == 0 and dy == 0 and dh == 0:
        return particles
    
    for p in particles:
        x, y, h = p.xyh
        dx_r, dy_r = rotate_point(dx, dy, h)
        noisy_dx, noisy_dy, noisy_dh = add_odometry_noise(
            (x + dx_r, y + dy_r, h + dh),
            setting.ODOM_HEAD_SIGMA,
            setting.ODOM_TRANS_SIGMA)
        # p.x, p.y, p.h = x + noisy_dx, y + noisy_dy, h + noisy_dh
        # x_f, y_f, h_f = x + noisy_dx, y + noisy_dy, (h + noisy_dh) % 360
        motion_particles.append(Particle(noisy_dx, noisy_dy, noisy_dh))

    return motion_particles

# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments:
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before measurement update (but after motion update)

        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree

                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
				* Note that the robot can see mutliple markers at once, and may not see any one

        grid -- grid world map, which contains the marker information,
                see grid.py and CozGrid for definition
                Can be used to evaluate particles

        Returns: the list of particles represents belief p(x_{t} | u_{t})
                after measurement update
    """
    measured_particles, weights = [], []

    for p in particles:
        if not grid.is_in(p.x, p.y) or not grid.is_free(p.x, p.y):
            # if out of bounds or on occupied space
            weights.append(0.0)
        else:
            prob = get_probability(p, measured_marker_list, grid)
            weights.append(prob)
    
    if np.sum(weights) == 0.0:
        # reset weights if all weights are 0
        weights = np.ones(len(particles))
    # normalize
    weights = np.divide(weights, np.sum(weights))

    #resampling
    # random_number = np.random.randint(low = 0.05 * setting.PARTICLE_COUNT, high = .1 * setting.PARTICLE_COUNT)
    random_number = int(.1 * setting.PARTICLE_COUNT)
    measured_particles = np.random.choice(particles, size = setting.PARTICLE_COUNT - random_number, replace=True, p=weights)
    measured_particles = np.ndarray.tolist(measured_particles) + Particle.create_random(random_number, grid)

    return measured_particles

# probability that particle sees what the robot sees
def get_probability(p, robot_marker_list, grid):
    particle_marker_list = p.read_markers(grid)

    marker_pairs = [] # (robot_marker, particle_marker)

    # for each robot marker, match it with the closest particle marker
    for robot_marker in robot_marker_list:
        if len(particle_marker_list) > 0:
            # find a match between robot marker and particle marker. use closest distance to match
            particle_match = particle_marker_list[0]
            particle_match_diff = float('inf')
            for particle_marker in particle_marker_list:
                dist_diff = grid_distance(robot_marker[0], robot_marker[1], particle_marker[0], particle_marker[1])
                if dist_diff < particle_match_diff:
                    particle_match_diff = dist_diff
                    particle_match = particle_marker
            marker_pairs.append((robot_marker, particle_match))
            particle_marker_list.remove(particle_match)
    
    prob = 1.0
    # for each match of markers, use the equation to calculate a score for how different they are
    for robot_marker, particle_marker in marker_pairs:
        dist_diff = grid_distance(robot_marker[0], robot_marker[1], particle_marker[0], particle_marker[1])
        angle_diff = diff_heading_deg(robot_marker[2], particle_marker[2])
        prob *= probability_helper(dist_diff, angle_diff)
    
    # possible robot detecting non-existent markers
    prob *= setting.SPURIOUS_DETECTION_RATE ** (len(robot_marker_list) - len(marker_pairs))
    # possible that robot did not detect the markers the particle sees
    prob *= setting.DETECTION_FAILURE_RATE ** len(particle_marker_list)
    return prob
                

def probability_helper(dist_diff, angle_diff):
    failure_rate = setting.SPURIOUS_DETECTION_RATE * setting.DETECTION_FAILURE_RATE
    dist_exp = (dist_diff ** 2) / (2 * (setting.MARKER_TRANS_SIGMA ** 2))
    angle_exp = (angle_diff ** 2) / (2 * (setting.MARKER_ROT_SIGMA ** 2))
    return max(
        failure_rate, 
        np.exp(-1 * (dist_exp + angle_exp))
    )
