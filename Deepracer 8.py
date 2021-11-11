import math
def reward_function(params):

    # Read input parameters 
    all_wheels_on_track = params['all_wheels_on_track']
    distance_from_center = params['distance_from_center']
    track_width = params['track_width']
    steering_angle = params['steering_angle']
    abs_steering = abs(params['steering_angle']) # Only need the absolute steering angle
    speed= params['speed']
    objects_distance = params['objects_distance']
    _, next_object_index = params['closest_objects']
    previous_object_index, _ = params['closest_objects']
    objects_left_of_center = params['objects_left_of_center']
    is_left_of_center = params['is_left_of_center']
    objects_location = params['objects_location']
    heading = params['heading']
    x = params['x']
    y = params['y']
    is_crashed = params['is_crashed']

    # Initialize reward with a small number but not zero
    # because zero means off-track or crashed
    reward = 1e-3

    # track width marks
    marker_0 = 0
    marker_1 = 0.1 * track_width
    marker_2 = 0.2 * track_width
    marker_2_5 = 0.25 * track_width
    marker_3 = 0.3 * track_width
    marker_4 = 0.4 * track_width
    marker_5 = 0.5 * track_width
    marker_10 = track_width
    marker_20 = 2 * track_width
    marker_30 = 3 * track_width
    marker_40 = 4* track_width

    reward_1 = 1
    reward_2 = 1
    reward_3 = 1
    reward_4 = 1
    reward_5 = 0
    reward_6 = 0
    reward_7 = 0
    

    reward_lane = 0
    reward_avoid = 0
    
    angle = 0
    
    # Decide if the agent and the next object is on the same lane
    is_same_lane = objects_left_of_center[next_object_index] == is_left_of_center
    
    # Steering penalty threshold, change the number based on your action space setting
    ABS_STEERING_THRESHOLD = 15

    if abs_steering > ABS_STEERING_THRESHOLD:
    	reward_1 = reward_1 - 0.6 * speed
    else:
        reward_1 = 1


    # Distance to the next object
    distance_next_object = objects_distance[next_object_index]
    # Distance to the previous object
    distance_previous_object = objects_distance[previous_object_index]

    if distance_next_object < marker_40 or distance_previous_object < marker_10:
        if is_same_lane:
            reward_2 = 0.1 * reward_2-1
        else:
            if marker_2 <= distance_from_center <= marker_3:
                reward_2 = 1.0 * reward_2 + 0.3 * speed
            elif marker_1 <= distance_from_center <= marker_4:
                reward_2 = 0.5 * reward_2 + 0.2 * speed
            elif marker_0 <= distance_from_center <= marker_5:
                reward_2 = 0.1 * reward_2 + 0.1 * speed
            else:
                reward_2 = 0.1 * reward_2-1 # likely crashed/ close to off track
      
        if distance_from_center >= marker_3:
            reward_2 = reward_2 - 0.8
    else:
        if distance_from_center <= marker_1:
       	    reward_2 = 1.0 * reward_2 + 0.5 * speed
        elif distance_from_center <= marker_2_5:
            reward_2 = 0.5 * reward_2 + 0.3 * speed
       	elif distance_from_center <= marker_3:
            reward_2 = 0.1 * reward_2 + 0.1 * speed
       	else:
            reward_2 = 0.1 * reward_2-1 # likely crashed/ close to off track	
    
    if all_wheels_on_track and (0.5*track_width - distance_from_center) >= 0.005:
    	reward_3 = reward_3
    else: 
     	reward_3 = -1

    reward_lane = reward_1 + reward_2 + reward_3
    

    if is_same_lane:
        if marker_20 <= distance_next_object <= marker_40:
            reward_4 = 0.8 - 0.1 * speed
        elif marker_10 <= distance_next_object < marker_20:
            reward_4 = 0.6 - 0.2 * speed
        elif distance_next_object < marker_10:
            reward_4 = 0.4 - 0.3 * speed


    #Calculate the angle between the direction of the car and the obstacle
    next_object_loc = objects_location[next_object_index]
    angle = math.atan2(next_object_loc[1]-y,next_object_loc[0]-x) * 180/math.pi - heading
   
    if angle < -90:
        angle = angle + 180
    elif angle > 90:
        angle = angle - 180

    abs_angle = abs(angle)

    if is_same_lane:
        if is_left_of_center:
            if steering_angle >= 0:
                reward_5 = reward_5 - 1
            else:
                reward_5 = reward_5
        else:
            if steering_angle <= 0:
                reward_5 = reward_5 - 1
            else:
                reward_5 = reward_5
    
    
    if is_same_lane and (distance_next_object<marker_20):
        if 0 <= abs_angle < 6:
            reward_6 = reward_6 - 1
        elif 6 <= abs_angle < 12:
            reward_6 = reward_6 - 0.5
        elif 12 <= abs_angle < 16:
            reward_6 = reward_6 - 0.2
    
    if is_crashed:
        reward_7 = reward_7- 5
    
    reward_avoid = reward_4 + reward_5 + reward_6 + reward_7

    reward = reward_lane + 2 * reward_avoid

    return float(reward)