from math import atan2, pi, sqrt, hypot
# In the docstring, "Pose" refers to a tuple (x,y,yaw) in (meters, meters, degrees)
# In world coordinates
def compute_control(cur_pose, prev_pose):
    """Given the current and previous odometry poses, this function extracts
    the control information based on the odometry motion model.
    Args:
        cur_pose  ([Pose]): Current Pose
        prev_pose ([Pose]): Previous Pose 
    Returns:
        [delta_rot_1]: Rotation 1  (degrees)
        [delta_trans]: Translation (meters)
        [delta_rot_2]: Rotation 2  (degrees)
    """
    x_dist = cur_pose[0]-prev_pose[0] # distance traveled in x direction [m]
    y_dist = cur_pose[1]-prev_pose[1] # distance traveled in y direction [m]
    trans_angle = atan2(y_dist, x_dist)*180/pi # angle at which translation happened [deg]
    delta_rot_1 = mapper.normalize_angle(trans_angle-prev_pose[2]) # rotaion from start pose to translation angle
    delta_trans = hypot(x_dist, y_dist) # euclidean distance
    delta_rot_2 = mapper.normalize_angle(cur_pose[2]-trans_angle) # rotation from translation angle to final pose

    return delta_rot_1, delta_trans, delta_rot_2

# In world coordinates
def odom_motion_model(cur_pose, prev_pose, u):
    """ Odometry Motion Model
    Args:
        cur_pose  ([Pose]): Current Pose
        prev_pose ([Pose]): Previous Pose
        u (rot1, trans, rot2) (float, float, float): A tuple with control data in the format format (rot1, trans, rot2) with units (degrees, meters, degrees)                                               
    Returns:
        prob [float]: Probability p(x'|x, u)
    """
    # called Transition Probability / Action Model in lecture notes
    
    # find the movement between previous and current timestep
    movement = compute_control(cur_pose, prev_pose)
    
    # easier to understand implimentation:
    #P_rot_1 = loc.gaussian(movement[0], u[0], loc.odom_rot_sigma) # probability of rotation 1 given control
    #P_trans = loc.gaussian(movement[1], u[1], loc.odom_trans_sigma) # probability of translation given control
    #P_rot_2 = loc.gaussian(movement[2], u[2], loc.odom_rot_sigma) # probability of rotation 2 given control
    #prob = P_rot_1*P_trans*P_rot_2 # assume independent probabilities
    
    # faster implimentaion:
    prob = loc.gaussian(movement[0], u[0], loc.odom_rot_sigma) * loc.gaussian(movement[1], u[1], loc.odom_trans_sigma) * loc.gaussian(movement[2], u[2], loc.odom_rot_sigma)
    return prob

def prediction_step(cur_odom, prev_odom):
    """ Prediction step of the Bayes Filter.
    Update the probabilities in loc.bel_bar based on loc.bel from the previous time step and the odometry motion model.

    Args:
        cur_odom  ([Pose]): Current Pose
        prev_odom ([Pose]): Previous Pose
    """
    u = compute_control(cur_odom, prev_odom)
    # for all previous state indices
    for xp in range(0, mapper.MAX_CELLS_X):
        # print(xp) # line to wath progress of update step
        for yp in range(0, mapper.MAX_CELLS_Y):
            for ap in range(0, mapper.MAX_CELLS_A):
                if loc.bel[xp, yp, ap]>0.0001:   #  only run through current states if previous beliefs are not negligable
                    # for all current state indices
                    for xc in range(0, mapper.MAX_CELLS_X): 
                        for yc in range(0, mapper.MAX_CELLS_Y):
                            for ac in range(0, mapper.MAX_CELLS_A):
                                loc.bel_bar[xc, yc, ac] = loc.bel_bar[xc, yc, ac] + odom_motion_model(mapper.from_map(xc, yc, ac), mapper.from_map(xp, yp, ap), u) * loc.bel[xp, yp, ap]

def update_step():
    """ Update step of the Bayes Filter.
    Update the probabilities in loc.bel based on loc.bel_bar and the sensor model.
    """
    # for all states
    for x in range(0, mapper.MAX_CELLS_X):
        for y in range(0, mapper.MAX_CELLS_Y):
            for a in range(0, mapper.MAX_CELLS_A):
                if loc.bel_bar[x, y, a]>0.0001:
                    loc.bel[x, y, a] = np.prod(loc.gaussian(loc.obs_range_data, mapper.obs_views[x, y, a, :], loc.sensor_sigma)) * loc.bel_bar[x, y, a]
                    loc.bel = loc.bel / np.sum(loc.bel) # normalize belief grid
    
    
