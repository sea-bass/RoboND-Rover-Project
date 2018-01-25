import numpy as np
import cv2

# Identify pixels above and below a threshold
# If you specify no upper threshold, it will assume none (255,255,255)
def color_thresh(img, lower_thresh=(160,160,160), upper_thresh=(255,255,255)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] >= lower_thresh[0]) & (img[:,:,0] <= upper_thresh[0]) \
                 & (img[:,:,1] >= lower_thresh[1]) & (img[:,:,1] <= upper_thresh[1]) \
                 & (img[:,:,2] >= lower_thresh[2]) & (img[:,:,2] <= upper_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    sourcePts = np.float32([[13, 139],
                     [302, 139],
                     [200, 94],
                     [120, 94]])
    pixel_size = 10;                     
    base_x = 0.5*Rover.img.shape[1];
    base_y = Rover.img.shape[0];
    destPts = np.float32([[base_x-pixel_size/2, base_y], 
                 [base_x+pixel_size/2, base_y], 
                 [base_x+pixel_size/2, base_y-pixel_size], 
                 [base_x-pixel_size/2, base_y-pixel_size]])
				 
    # 2) Apply perspective transform
    warped = perspect_transform(Rover.img,sourcePts,destPts)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    # Find navigable terrain
    nav_img = color_thresh(warped,lower_thresh=(160, 160, 160))
	# Find rocks
    rock_img = color_thresh(warped,lower_thresh=(120,100,0), upper_thresh=(200,200,50))
	# Find obstacle terrain, which is the complement of the navigable terrain 
	# minus the parts of the warped image that weren't in the rover's field of sight
    obs_img = 1-nav_img
    obs_img[warped[:,:,0]==0] = 0

	# 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,2] = nav_img*255
    Rover.vision_image[:,:,1] = rock_img*255
    Rover.vision_image[:,:,0] = obs_img*255;
		
    # 5) Convert map image pixel values to rover-centric coords
    x_pix_nav,y_pix_nav = rover_coords(nav_img)
    x_pix_obs,y_pix_obs = rover_coords(obs_img)
    x_pix_rock,y_pix_rock = rover_coords(rock_img)
	
    # 6) Convert rover-centric pixel values to world coordinates
    x_world,y_world = pix_to_world(x_pix_nav, y_pix_nav, \
					Rover.pos[0], Rover.pos[1], Rover.yaw, \
                    Rover.worldmap.shape[0], pixel_size)
    x_rock,y_rock = pix_to_world(x_pix_rock, y_pix_rock, \
					Rover.pos[0], Rover.pos[1], Rover.yaw, \
                    Rover.worldmap.shape[0], pixel_size)
    x_obs,y_obs = pix_to_world(x_pix_obs, y_pix_obs, \
					Rover.pos[0], Rover.pos[1], Rover.yaw, \
                    Rover.worldmap.shape[0], pixel_size)

    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    # Do this only for acceptable pitch and roll values
    pitch_thresh = 0.25
    roll_thresh = 2
    if ( (abs(Rover.pitch)<pitch_thresh) | (abs(Rover.pitch-360)<pitch_thresh) ) \
       & ( (abs(Rover.roll)<roll_thresh) | (abs(Rover.roll-360)<roll_thresh) ):
        # Loop through all the navigable pixels found
        for i in range(y_world.shape[0]):
            # Add to the navigable space
            if Rover.worldmap[y_world[i],x_world[i],2] < 255:
                Rover.worldmap[y_world[i],x_world[i],2] += 1;
			
        for i in range(y_rock.shape[0]):			
		    # Add to the rock space
            if Rover.worldmap[y_rock[i],x_rock[i],1] < 255:
                Rover.worldmap[y_rock[i],x_rock[i],1] += 1;
		  
		# Loop through all the obstacle pixels found
        for i in range(y_obs.shape[0]):
			# Add to the obstacle space
            if (Rover.worldmap[y_obs[i],x_obs[i],0] < 255):
                Rover.worldmap[y_obs[i],x_obs[i],0] += 1;
				
		# Remove obstacle detections from navigable space located
        freeIndices = np.where((Rover.worldmap[:,:,2]>0) | (Rover.worldmap[:,:,1]>0));
        Rover.worldmap[freeIndices[0],freeIndices[1],0] = 0;		
			
	# 8) Convert rover-centric pixel positions to polar coordinates
	# Update Rover pixel distances and angles
		# Rover.nav_dists = rover_centric_pixel_distances
		# Rover.nav_angles = rover_centric_angles
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(x_pix_nav,y_pix_nav)
    Rover.rock_dists, Rover.rock_angles = to_polar_coords(x_pix_rock,y_pix_rock)
	
    return Rover