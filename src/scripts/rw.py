import math
def reward_function(params):

    # Import package (needed for heading)
    first_racingpoint_index = 1
    verbose = False


    ################## HELPER FUNCTIONS ###################

    def dist_2_points(x1, x2, y1, y2):
        return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

    def closest_2_racing_points_index(racing_coords, car_coords):

        # Calculate all distances to racing points
        distances = []
        for i in range(len(racing_coords)):
            distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                     y1=racing_coords[i][1], y2=car_coords[1])
            distances.append(distance)

        # Get index of the closest racing point
        closest_index = distances.index(min(distances))

        # Get index of the second closest racing point
        distances_no_closest = distances.copy()
        distances_no_closest[closest_index] = 999
        second_closest_index = distances_no_closest.index(
            min(distances_no_closest))

        return [closest_index, second_closest_index]

    def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):

        # Calculate the distances between 2 closest racing points
        a = abs(dist_2_points(x1=closest_coords[0],
                              x2=second_closest_coords[0],
                              y1=closest_coords[1],
                              y2=second_closest_coords[1]))

        # Distances between car and closest and second closest racing point
        b = abs(dist_2_points(x1=car_coords[0],
                              x2=closest_coords[0],
                              y1=car_coords[1],
                              y2=closest_coords[1]))
        c = abs(dist_2_points(x1=car_coords[0],
                              x2=second_closest_coords[0],
                              y1=car_coords[1],
                              y2=second_closest_coords[1]))

        # Calculate distance between car and racing line (goes through 2 closest racing points)
        # try-except in case a=0 (rare bug in DeepRacer)
        try:
            distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                           (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
        except:
            distance = b

        return distance

    # Calculate which one of the closest racing points is the next one and which one the previous one
    def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

        # Virtually set the car more into the heading direction
        heading_vector = [math.cos(math.radians(
            heading)), math.sin(math.radians(heading))]
        new_car_coords = [car_coords[0]+heading_vector[0],
                          car_coords[1]+heading_vector[1]]

        # Calculate distance from new car coords to 2 closest racing points
        distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                    x2=closest_coords[0],
                                                    y1=new_car_coords[1],
                                                    y2=closest_coords[1])
        distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                           x2=second_closest_coords[0],
                                                           y1=new_car_coords[1],
                                                           y2=second_closest_coords[1])

        if distance_closest_coords_new <= distance_second_closest_coords_new:
            next_point_coords = closest_coords
            prev_point_coords = second_closest_coords
        else:
            next_point_coords = second_closest_coords
            prev_point_coords = closest_coords

        return [next_point_coords, prev_point_coords]

    def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):

        # Calculate the direction of the center line based on the closest waypoints
        next_point, prev_point = next_prev_racing_point(closest_coords,
                                                        second_closest_coords,
                                                        car_coords,
                                                        heading)

        # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
        track_direction = math.atan2(
            next_point[1] - prev_point[1], next_point[0] - prev_point[0])

        # Convert to degree
        track_direction = math.degrees(track_direction)

        # Calculate the difference between the track direction and the heading direction of the car
        direction_diff = abs(track_direction - heading)
        if direction_diff > 180:
            direction_diff = 360 - direction_diff

        return direction_diff

    # Gives back indexes that lie between start and end index of a cyclical list
    # (start index is included, end index is not)
    def indexes_cyclical(start, end, array_len):

        if end < start:
            end += array_len

        return [index % array_len for index in range(start, end)]

    # Calculate how long car would take for entire lap, if it continued like it did until now
    def projected_time(first_index, closest_index, step_count, times_list):

        # Calculate how much time has passed since start
        current_actual_time = (step_count-1) / 15

        # Calculate which indexes were already passed
        indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

        # Calculate how much time should have passed if car would have followed optimals
        current_expected_time = sum([times_list[i] for i in indexes_traveled])

        # Calculate how long one entire lap takes if car follows optimals
        total_expected_time = sum(times_list)

        # Calculate how long car would take for entire lap, if it continued like it did until now
        try:
            projected_time = (current_actual_time/current_expected_time) * total_expected_time
        except:
            projected_time = 9999

        return projected_time

    #################### RACING LINE ######################s
    # Optimal racing line for the Spain track
    # Each row: [x,y,speed,timeFromPreviousPoint]
    racing_track =   [[-3.60352, -0.02478, 1.23043, 0.14426],
                        [-3.63021, -0.19976, 1.30254, 0.13589],
                        [-3.63398, -0.37819, 1.35997, 0.13123],
                        [-3.61621, -0.55876, 1.25889, 0.14413],
                        [-3.57612, -0.74035, 1.16457, 0.15968],
                        [-3.51247, -0.92156, 1.16457, 0.16493],
                        [-3.42259, -1.10038, 1.16457, 0.17185],
                        [-3.30309, -1.27367, 1.16457, 0.18076],
                        [-3.14573, -1.43447, 1.16457, 0.19319],
                        [-2.94306, -1.56774, 1.85375, 0.13085],
                        [-2.72294, -1.6859, 2.20249, 0.11343],
                        [-2.48963, -1.79229, 2.71844, 0.09433],
                        [-2.24547, -1.89099, 3.54671, 0.07425],
                        [-1.99099, -1.98644, 3.24648, 0.08372],
                        [-1.72926, -2.08033, 2.85845, 0.09728],
                        [-1.46725, -2.16901, 2.52667, 0.10947],
                        [-1.20498, -2.25125, 2.24411, 0.12248],
                        [-0.94248, -2.32582, 1.98381, 0.13756],
                        [-0.67983, -2.39142, 1.75578, 0.15419],
                        [-0.41713, -2.44571, 1.55119, 0.17293],
                        [-0.15464, -2.48588, 1.55119, 0.17119],
                        [0.10723, -2.50863, 1.55119, 0.16945],
                        [0.36773, -2.50969, 1.55119, 0.16794],
                        [0.62558, -2.48364, 1.55119, 0.16707],
                        [0.87821, -2.42323, 1.6982, 0.15296],
                        [1.12468, -2.33405, 1.82615, 0.14353],
                        [1.3643, -2.21965, 1.97943, 0.13414],
                        [1.59671, -2.08338, 2.07936, 0.12957],
                        [1.82124, -1.92703, 2.18219, 0.12538],
                        [2.03718, -1.75232, 2.29079, 0.12125],
                        [2.24389, -1.56112, 2.39576, 0.11753],
                        [2.44072, -1.3553, 2.48877, 0.11443],
                        [2.62702, -1.13667, 2.56432, 0.11201],
                        [2.80212, -0.90695, 2.48166, 0.11639],
                        [2.9653, -0.66773, 2.31675, 0.12499],
                        [3.11608, -0.42065, 2.15388, 0.13439],
                        [3.25373, -0.16709, 1.93133, 0.14938],
                        [3.37683, 0.09185, 1.74027, 0.16475],
                        [3.48382, 0.35494, 1.55136, 0.18307],
                        [3.57222, 0.62087, 1.34988, 0.2076],
                        [3.63942, 0.88793, 1.19979, 0.22953],
                        [3.6812, 1.15385, 1.05755, 0.25453],
                        [3.69347, 1.4154, 1.05755, 0.2476],
                        [3.67161, 1.6682, 1.05755, 0.23993],
                        [3.60935, 1.9055, 1.05755, 0.23198],
                        [3.5023, 2.11853, 1.05755, 0.22544],
                        [3.34616, 2.29336, 1.0905, 0.21496],
                        [3.15251, 2.42486, 1.22404, 0.19123],
                        [2.93582, 2.51689, 1.33586, 0.17624],
                        [2.70476, 2.57264, 1.45934, 0.16288],
                        [2.4654, 2.59598, 1.58902, 0.15135],
                        [2.22182, 2.59064, 1.71605, 0.14198],
                        [1.97673, 2.55972, 1.86274, 0.13262],
                        [1.73197, 2.50639, 2.01692, 0.1242],
                        [1.48876, 2.43339, 2.1723, 0.11689],
                        [1.24792, 2.34305, 2.37012, 0.10853],
                        [1.00987, 2.23786, 2.60532, 0.09989],
                        [0.77478, 2.12022, 2.86121, 0.09188],
                        [0.54263, 1.99219, 2.24299, 0.1182],
                        [0.31341, 1.85523, 1.80852, 0.14764],
                        [0.08749, 1.71039, 1.80852, 0.14839],
                        [-0.12845, 1.56278, 1.80852, 0.14464],
                        [-0.34746, 1.42246, 1.80852, 0.14382],
                        [-0.57334, 1.29736, 1.80852, 0.14277],
                        [-0.80923, 1.19541, 2.05058, 0.12532],
                        [-1.05255, 1.11153, 2.1091, 0.12203],
                        [-1.30255, 1.04452, 1.9551, 0.13238],
                        [-1.55862, 0.99357, 1.82263, 0.14325],
                        [-1.82049, 0.95867, 1.67244, 0.15796],
                        [-2.08807, 0.9411, 1.52003, 0.17641],
                        [-2.34302, 0.90544, 1.36741, 0.18826],
                        [-2.58338, 0.85148, 1.0, 0.24634],
                        [-2.80739, 0.77949, 1.0, 0.23529],
                        [-3.01277, 0.6895, 1.0, 0.22423],
                        [-3.19674, 0.58154, 1.0, 0.2133],
                        [-3.35579, 0.45579, 1.0, 0.20276],
                        [-3.47105, 0.30677, 1.08129, 0.17423],
                        [-3.5516, 0.14495, 1.16195, 0.15556],
                        [-3.60352, -0.02478, 1.23043, 0.14426]]
################# INPUT PARAMETERS ###################

    # Read all input parameters
    all_wheels_on_track = params['all_wheels_on_track']
    x = params['x']
    y = params['y']
    distance_from_center = params['distance_from_center']
    is_left_of_center = params['is_left_of_center']
    heading = params['heading']
    progress = params['progress']
    steps = params['steps']
    speed = params['speed']
    steering_angle = params['steering_angle']
    track_width = params['track_width']
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    is_offtrack = params['is_offtrack']

    ############### OPTIMAL X,Y,SPEED,TIME ################

    # Get closest indexes for racing line (and distances to all points on racing line)
    closest_index, second_closest_index = closest_2_racing_points_index(
        racing_track, [x, y])

    # Get optimal [x, y, speed, time] for closest and second closest index
    optimals = racing_track[closest_index]
    optimals_second = racing_track[second_closest_index]

    # Save first racingpoint of episode for later
    # if verbose == True:
    #     first_racingpoint_index = 0 # this is just for testing purposes
    if steps == 1:
        first_racingpoint_index = closest_index

    ################ REWARD AND PUNISHMENT ################

    ## Define the default reward ##
    reward = 1

    ## Reward if car goes close to optimal racing line ##
    DISTANCE_MULTIPLE = 1
    dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
    distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
    reward += distance_reward * DISTANCE_MULTIPLE

    ## Reward if speed is close to optimal speed ##
    SPEED_DIFF_NO_REWARD = 1
    SPEED_MULTIPLE = 0
    speed_diff = abs(optimals[2]-speed)
    if speed_diff <= SPEED_DIFF_NO_REWARD:
        # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
        # so, we do not punish small deviations from optimal speed
        speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
    else:
        speed_reward = 0
    reward += speed_reward * SPEED_MULTIPLE

    # Reward if less steps
    REWARD_PER_STEP_FOR_FASTEST_TIME = 1
    STANDARD_TIME = 11
    FASTEST_TIME = 9
    times_list = [row[3] for row in racing_track]
    projected_time = projected_time(first_racingpoint_index, closest_index, steps, times_list)
    try:
        steps_prediction = projected_time * 15 + 1
        reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                       (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
        steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
    except:
        steps_reward = 0
    reward += steps_reward

    # Zero reward if obviously wrong direction (e.g. spin)
    direction_diff = racing_direction_diff(
        optimals[0:2], optimals_second[0:2], [x, y], heading)
    
    if direction_diff > 30:
        reward = 1e-3
    elif direction_diff > 15:
        reward *= 0.5
    # Zero reward of obviously too slow 속도 엄격함 완화
    speed_diff_zero = optimals[2]-speed
    if speed_diff_zero > 3:
        reward = 1e-3
    elif speed_diff_zero > 1:
        reward *= 0.2
    elif speed_diff_zero > 0.5:
        reward *= 0.5



    # 트랙이탈시 큰 보상 감수
    if is_offtrack : 
        reward = 1e-3
    else : 
        pass

    ## Incentive for finishing the lap in less steps ##
    REWARD_FOR_FASTEST_TIME = 150 # should be adapted to track length and other rewards
    STANDARD_TIME = 13 # seconds (time that is easily done by model)
    FASTEST_TIME = 11  # seconds (best time of 1st place on the track)
    if progress == 100:
        finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                  (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
    else:
        finish_reward = 0
    reward += finish_reward

    ## Zero reward if off track ## 제거
    # if all_wheels_on_track == False:
    #     reward = 1e-3

    ####################### VERBOSE #######################

    # if self.verbose == True:
    #     print("Closest index: %i" % closest_index)
    #     print("Distance to racing line: %f" % dist)
    #     print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
    #     print("Optimal speed: %f" % optimals[2])
    #     print("Speed difference: %f" % speed_diff)
    #     print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
    #     print("Direction difference: %f" % direction_diff)
    #     print("Predicted time: %f" % projected_time)
    #     print("=== Steps reward: %f ===" % steps_reward)
    #     print("=== Finish reward: %f ===" % finish_reward)

    #################### RETURN REWARD ####################

    # Always return a float value
    return float(reward)