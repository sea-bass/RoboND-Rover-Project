import numpy as np

# Set throttle value to throttle setting, 
# scaled proportionally based on number of free pixels less than a max distance away
def proportional_throttle(Rover):
    max_dist = 100;
    num_free_pixels = len(Rover.nav_angles[Rover.nav_dists < max_dist])
    return  Rover.throttle_set * \
            (np.clip(num_free_pixels,Rover.stop_forward,Rover.go_forward)-Rover.stop_forward) \
            /(Rover.go_forward-Rover.stop_forward)

# Set the steering angle and modify throttle if needed
# If the throttle is on but the max angle is greater than 15 degrees, turn it off
def steer_rover(Rover):
    mean_angle = np.mean(Rover.nav_angles * 180/np.pi)
    if abs(mean_angle) < 15:
        steer = mean_angle
        throttle = Rover.throttle
    else:
        steer = np.clip(mean_angle,-15,15)
        throttle = 0;
    return throttle, steer
	
# Drive to a rock at low speed, trying to brake in front of it
def follow_rock(Rover):
    # Calculate the min distance and mean angle
    mean_angle = np.mean(Rover.rock_angles * 180/np.pi)
    mean_dist = np.mean(Rover.rock_dists)
    # Set steering and speed to track the rock
    min_vel = 0.05
    max_vel = 0.4
    steer = np.clip(mean_angle,-15,15)
    speed_set = np.clip(mean_dist*0.025,min_vel,max_vel);
	# Brake if going too fast to track rock
    if (Rover.vel>(speed_set+0.05)):
        throttle = 0
        brake = Rover.brake_set
    # Throttle to match a speed proportional to distance to the rock
    elif (Rover.vel < (speed_set-0.05)):
        throttle = Rover.throttle_set;
        brake = 0
	# Else, coast
    else:
        throttle = 0
        brake = 0
    return throttle, steer

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Check if we have vision data to make decisions with
	# Prioritize rocks vs. navigable terrain
    if len(Rover.rock_dists) > 0:
        Rover.throttle, Rover.steer = follow_rock(Rover)
        Rover.mode = 'stop';
    elif Rover.nav_angles is not None:
        num_free_pixels = len(Rover.nav_angles)
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if num_free_pixels >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting, 
                    # scaled proportionally based on number of free pixels
                    Rover.throttle = proportional_throttle(Rover)
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.throttle, Rover.steer = steer_rover(Rover)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = 15
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = proportional_throttle(Rover)
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.throttle, Rover.steer = steer_rover(Rover)
                    Rover.mode = 'forward'
					
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True

    # If in a state where pitch and roll are not normal, back up
    pitch_thresh = 1
    roll_thresh = 2
    if ( (Rover.pitch<180) & (Rover.pitch>pitch_thresh) ) | \
       ( (Rover.pitch>180) & ((360-Rover.pitch)>pitch_thresh) ) | \
	   ( (Rover.roll<180) & (Rover.roll>roll_thresh) ) | \
       ( (Rover.roll>180) & ((360-Rover.roll)>roll_thresh) ):
        if Rover.vel > -Rover.max_vel:
            Rover.throttle = -0.2
        else:
            Rover.throttle = 0
        Rover.steer = 0
        Rover.brake = 0    

    return Rover

