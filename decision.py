import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):
 
   # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    #######################################################################################################################################################
    #                                                           Edited by: Engy Mohamed                                                                   #
    #######################################################################################################################################################
    
    
    # Only apply left wall hugging when out of the starting point (after 10s)
    # to avoid getting stuck in a circle
    if Rover.total_time > 10:
        

    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status. I made Rover.mode a stack
        if Rover.mode[-1] == 'forward':

            if Rover.home is None:
                # note down home
                Rover.home = Rover.pos

            if Rover.samples_collected >= 3 and Rover.mapped > 50:
                Rover.mode.append('home')
                        

            # if sample rock on sight (in the left side only) and relatively close
            if Rover.samples_angles is not None and np.mean(Rover.samples_angles) > -0.4 and np.min(Rover.samples_dists) < 40:
                # Rover.steer = np.clip(np.mean(Rover.samples_angles * 180 / np.pi), -15, 15)
                Rover.rock_time = Rover.total_time
                Rover.mode.append('rock')

            # Check the extent of navigable terrain
            elif len(Rover.nav_angles) >= Rover.stop_forward:
                # If mode is forward, navigable terrain looks good
                # Except for start, if stopped means stuck.
                # Alternates between stuck and forward modes
                if Rover.vel <= 0.1 and Rover.total_time - Rover.stuck_time > 4:
                    # Set mode to "stuck" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode.append('stuck')
                    Rover.stuck_time = Rover.total_time
                # if velocity is below max, then throttle
                elif Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                # Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                # Hug left wall by setting the steer angle slightly to the left
                Rover.steer = np.clip(np.mean((Rover.nav_angles) * 180 / np.pi), -15, 15)

            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward or Rover.vel <= 0:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode.append('stop')
                    
    #####################################################################################################################################################
    #                                                                                                                                                   #
    #####################################################################################################################################################
    
    
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
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
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
    
    return Rover



#########################################################
#           Coded by: Habiba ahmed                       #
#########################################################
# Check if the rover is making circle
def check_circle(Rover):
    if Rover.steer == 15 and Rover.vel > 0.5:
        if Rover.circle_time > 5:
            Rover.circle_time = 0
            Rover.is_stuck = True
            Rover.stuck_time = Rover.total_time
            Rover.last_mode = Rover.mode  
            Rover.mode = 'unstuck'
        elif Rover.steer != 15:
            Rover.circle_time = 0
            Rover.is_stuck = False
            Rover.stuck_time = 0
        else:
            Rover.circle_time += 1
    elif Rover.steer == -15 and Rover.vel > 0.5:
        if Rover.circle_time > 5:
            Rover.circle_time = 0
            Rover.is_stuck = True
            Rover.stuck_time = Rover.total_time
            Rover.last_mode = Rover.mode  
            Rover.mode = 'unstuck'
        elif Rover.steer != -15:
            Rover.circle_time = 0
            Rover.is_stuck = False
            Rover.stuck_time = 0
        else:
            Rover.circle_time += 1