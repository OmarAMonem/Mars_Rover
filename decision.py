import numpy as np
from perception import to_polar_coords

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):
 
   # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    #######################################################################################################################################################
    #                                                           Edited by: Engy Mohamed                                                                   #
    #######################################################################################################################################################
    
    
   
        

    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status. I made Rover.mode a stack
        if Rover.mode[-1] == 'forward':

            if Rover.home is None:
                # note down home
                Rover.home = Rover.pos

            if Rover.samples_collected >= 5 and Rover.mapped > 90:
                dist, angles = to_polar_coords(Rover.home[0] - Rover.pos[0],
                                                Rover.home[1] - Rover.pos[1])
                if dist < 10:
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

    #########################################################
    #           Coded by: Shiry Ezzat                       #
    #########################################################
                # If we're already in "stuck". Stay here for 1 sec
        elif Rover.mode[-1] == 'stuck':
            # if 1 sec passed go back to previous mode
            if Rover.total_time - Rover.stuck_time > 1:
                # Set throttle back to stored value
                Rover.throttle = Rover.throttle_set
                # Release the brake
                Rover.brake = 0
                # Set steer to mean angle
                # Hug left wall by setting the steer angle slightly to the left
                Rover.steer = np.clip(np.mean((Rover.nav_angles) * 180 / np.pi), -15, 15)
                Rover.mode.pop() # returns to previous mode
            # Now we're stopped and we have vision data to see if there's a path forward
            else:
                Rover.throttle = 0
                # Release the brake to allow turning
                Rover.brake = 0
                # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                # Since hugging left wall steering should be to the right:
                Rover.steer = -15

    #########################################################
    #           Coded by: Omar Osama                        #
    #########################################################
        elif Rover.mode[-1] == 'home':
            if dist > 2:
                Rover.steer = np.clip(np.mean(angles * 180 / np.pi), -15, 15)
            else:
                Rover.is_done = True
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
    #########################################################
    #           Coded by: Maram Ahmed                       #
    #########################################################
        elif Rover.mode[-1] == 'rock':
            # Steer torwards the sample
            mean = np.mean(Rover.samples_angles * 180 / np.pi)
            if not np.isnan(mean):
                Rover.steer = np.clip(mean, -15, 15)
            else:
                Rover.mode.pop() # rock is not in sight anymore. Go to previous state

            # if 20 sec passed and it's still looking for the rock, give up and go back to previous mode
            if Rover.total_time - Rover.rock_time > 20:
                Rover.mode.pop()  # returns to previous state

            # stop when close enough to pick up the sample
            if Rover.near_sample:
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set

            # if got stuck go to stuck mode
            elif Rover.vel <= 0 and Rover.total_time - Rover.stuck_time > 10:
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode.append('stuck')
                Rover.stuck_time = Rover.total_time
            else:
                # Approach slowly
                slow_speed = Rover.max_vel / 2
                if Rover.vel < slow_speed:
                    Rover.throttle = 0.1
                    Rover.brake = 0
                else:  # Else brake
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set/2
                    
    #########################################################
    #           Coded by: Habiba ahmed                       #
    #########################################################
        elif Rover.mode[-1] == 'stop':
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
                    # Since hugging left wall steering should be to the right:
                    Rover.steer = -15
                # If we're stopped but see sufficient navigable terrain in front then go
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    # Hug left wall by setting the steer angle slightly to the left
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -15, 15)
                    Rover.mode.pop()  # returns to previous mode                
    #####################################################################################################################################################
    #                                                                                                                                                   #
    #####################################################################################################################################################

    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover




