# In world coordinates
def compute_control(cur_pose, prev_pose):
    """ Given the current and previous odometry poses, this function extracts
    the control information based on the odometry motion model.

    Args:
        cur_pose  ([Pose]): Current Pose
        prev_pose ([Pose]): Previous Pose 

    Returns:
        [delta_rot_1]: Rotation 1  (degrees)
        [delta_trans]: Translation (meters)
        [delta_rot_2]: Rotation 2  (degrees)
    """
    x_dist = cur_pose[0]-prev_pose[0] # distance traveled in x direction
    y_dist = cur_pose[1]-prev_pose[1] # distance traveled in y direction
    trans_angle = atan2(y_dist, x_dist) # angle at which translation happened
    delta_rot_1 = trans_angle-prev_pose[2] # rotaion from start pose to translation angle
    if abs(delta_rot_1)>180: # correct to turn the shortest angle
        delta_rot_1 = sign(delta_rot_1)*(abs(delta_rot_1)-360)
    delta_trans = sqrt(x_dist**2+y_dist**2) # euclidean distance
    delta_rot_2 = cur_pose-trans_angle # rotation from translation angle to final pose
    if abs(delta_rot_2)>180: # correct to turn the shortest angle
        delta_rot_1 = sign(delta_rot_2)*(abs(delta_rot_2)-360)
    return delta_rot_1, delta_trans, delta_rot_2

# In world coordinates
def odom_motion_model(cur_pose, prev_pose, u):
    """ Odometry Motion Model

    Args:
        cur_pose  ([Pose]): Current Pose
        prev_pose ([Pose]): Previous Pose
        (rot1, trans, rot2) (float, float, float): A tuple with control data in the format 
                                                   format (rot1, trans, rot2) with units (degrees, meters, degrees)

    Returns:
        prob [float]: Probability p(x'|x, u)
    """
        
    return prob

def prediction_step(cur_odom, prev_odom, xt):
    """ Prediction step of the Bayes Filter.
    Update the probabilities in loc.bel_bar based on loc.bel from the previous time step and the odometry motion model.

    Args:
        cur_odom  ([Pose]): Current Pose
        prev_odom ([Pose]): Previous Pose
    """
    # added xt as an input so that when looping through all states the prediction can be done for state, xt
    u = compute_control(cur_odom, prev_odom)
    sum = 0 #initialize sum
    # for all possible previous states
    for all x_positons in grid:   
        for all y_posiions in grid:
            for all yaw_angles in grid:
                x_prev = [x_posiotn, y_position, yaw_angle] # previous pose
                # sum product of transition probability from each previous pose and belief at that previous pose
                sum = sum + odom_motion_model(xt, [x_prev], u)*loc.bel[x_pev]
    loc.bel_bar = sum # prediction
        
def sensor_model(obs, xt):
    """ This is the equivalent of p(z|x).

    Args:
        obs ([ndarray]): A 1D array consisting of the measurements made in rotation loop

    Returns:
        [ndarray]: Returns a 1D array of size 18 (=loc.OBS_PER_CELL) with the likelihood of each individual measurements
    """
    prob_array = array of 18 0s # initialize probability array to be filled in
    views = get_views(xt[0], xt[1], xt[2])  # expected views at particular state xt
    for index in [0:17]:
        # pull probabilities from a normal distribution centered at the expected sensor reading
        prob_array[index] = loc.gaussian(obs[index], views[index], loc.sensor_sigma)
    return prob_array

def update_step(xt, z):
    """ Update step of the Bayes Filter.
    Update the probabilities in loc.bel based on loc.bel_bar and the sensor model.
    """
    # update for state xt with measurement z
    sens = sensor_model(z, xt) # P(z|xt)                    
    loc.bel = sensor_model*loc.bel_bar
    # assume that eta is calculated later