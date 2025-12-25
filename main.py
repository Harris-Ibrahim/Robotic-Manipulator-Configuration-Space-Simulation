import pygame
import numpy as np
import math
from shapely.geometry import Polygon

# Initialize the pygame 
pygame.init()

# Setup a clock
clock = pygame.time.Clock()
fps = 60

# Set the dimensions in pixels of the pygame screen
SCREEN_WIDTH, SCREEN_HEIGHT = 1400, 700 
# Set the origin ,in pixels, for the c-space plot
PLOT_X0, PLOT_Y0 = 800, 90
# Set the step size to map the c-space grid to the c-space plot on pygame screen
PIXELS_PER_GRIDPOINT = 8

# Set the resolution / Step size of the configuration space
cs_resolution = 0.1

# provide the colour channels of various objects in the simulation
COLOUR_LINK1 = (21, 94, 149)
COLOUR_LINK2 = (106, 128, 185)
COLOUR_LINK1_COLLISION = (210, 0, 0)
COLOUR_LINK2_COLLISION= (240, 0, 0)
COLOUR_JOINT = (246, 199, 148)
COLOUR_SCREEN = (251, 251, 251)
COLOUR_OBSTACLE =  (249, 110, 42)
COLOUR_TEXT =  (40, 40, 40)
COLOUR_GOAL = (0, 128, 0)

# provide the lengths and the width of the rectangular links in pixels
link_length_1, link_length_2 = 200, 100
link_width = 20
half_width = link_width//2

# Set the lower and upp joint constraints of both joints
# in radians, in range 0 to 2Pi for lower limit, and in range 0 to -2Pi for upper limit
joint_1_lower_lim = 0  
joint_1_upper_lim = +2*math.pi 
joint_2_lower_lim =  -2.96706 + +2*math.pi
joint_2_upper_lim = +2.96706

# Define the location of the connection of the manipulator to the ground in pixels
# This position is also the reference point for positioning of obstacles and robot links
ground_x, ground_y = 350, 350

# provide the centre coordinates, lengths and the width of the rectangular/ square obstacles in pixels
obs_width, obs_height= 120, 80

# Set the distance between the robot's joint 1 and the centre of the obstacle 1 and 2 in pixels, just multiply real world offset with the scale and do not change the signs
obs_centre_offset_x1 = 0
obs_centre_offset_y1 = -300 
obs_centre_offset_x2 = 0
obs_centre_offset_y2 = 200

# Setup a pygame screen at the given screen sizes
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

# A function to convert any value of theta to its positive equivalent in 0-2pi range. For example -pi get converted to pi, 4pi to 2pi
def convert_to_two_pi_range (theta):
    # If the thetas exceed 0 - 2*pi range, set them back to an equivalent 0 - 2*pi values, 
    # otherwise the contraints cannot be checked due to periodic natures of angles
    if theta < 0 :
        corrected_theta = theta + 2*math.pi
    elif theta > 2*math.pi :
        corrected_theta = theta - 2*math.pi
    else :
        corrected_theta = theta 
    return corrected_theta

# Get the x,y index corresponding to the given theta values, and ranges of theta and grid indices
def theta_to_index_value (theta_1, theta_2, max_theta, size_of_grid) :
    theta_xi = int((theta_1/(max_theta))*size_of_grid)
    theta_yi = int((theta_2/(max_theta))*size_of_grid)
    return theta_xi, theta_yi

# Get the theta_1, theta_2 corresponding to the given index value, and ranges of theta and grid indices
def index_to_theta_value (theta_xi, theta_yi, max_theta, size_of_grid):
    theta_1 = (theta_xi/(size_of_grid))*max_theta
    theta_2 = (theta_yi/(size_of_grid))*max_theta
    return theta_1, theta_2

# A function to draw a 2link RR manipulators, 2 links as rectangular polygons, 2 joints as circles
def draw_manipulator(ground_x, ground_y, link_length_1, link_length_2, half_width, theta_1, theta_2, COLOUR_L1, COLOUR_L2 ):

    # For link 1: Calculate the start coordinates and the difference to the end coordinates of the centreline
    start_x, start_y = ground_x, ground_y
    delta_x, delta_y = link_length_1*math.cos(theta_1), -link_length_1*math.sin(theta_1)  # negative sign for flipped y axis in pygame

    # Rectangle's four corners given the start point and the difference to the end point of the centreline of link 1
    corner_tuples_1 = [
        (start_x - half_width * math.sin(theta_1), start_y - half_width * math.cos(theta_1)),
        (start_x + half_width * math.sin(theta_1), start_y + half_width * math.cos(theta_1)),
        (start_x + delta_x + half_width * math.sin(theta_1), start_y + delta_y + half_width * math.cos(theta_1)),
        (start_x + delta_x - half_width * math.sin(theta_1), start_y + delta_y - half_width * math.cos(theta_1))
    ]
    # Draw a polygon given the 4 corners to represent the rectangular link 1
    pygame.draw.polygon(surface=screen, color=COLOUR_L1, points=corner_tuples_1)

    # Draw a circle where link 1 connects with the ground, to represent joint 1
    pygame.draw.circle(surface=screen, color=COLOUR_JOINT, center=(start_x,start_y), radius= half_width)

    # For link 1: Calculate the start coordinates and the difference to the end coordinates of the centreline
    start_x, start_y = ground_x+delta_x, ground_y+delta_y # The start point of link 2 is the end point of the link 1
    delta_x, delta_y = link_length_2*math.cos(theta_1+theta_2), -link_length_2*math.sin(theta_1+theta_2)  # negative sign for flipped y axis in pygame

    # Rectangle's four corners given the start point and the difference to the end point of the centreline of link 1
    corner_tuples_2 = [
        (start_x - half_width * math.sin(theta_1+theta_2), start_y - half_width * math.cos(theta_1+theta_2)),
        (start_x + half_width * math.sin(theta_1+theta_2), start_y + half_width * math.cos(theta_1+theta_2)),
        (start_x + delta_x + half_width * math.sin(theta_1+theta_2), start_y + delta_y + half_width * math.cos(theta_1+theta_2)),
        (start_x + delta_x - half_width * math.sin(theta_1+theta_2), start_y + delta_y - half_width * math.cos(theta_1+theta_2))
    ]
    # Draw a polygon given the 4 corners to represent the rectangular link 2
    pygame.draw.polygon(surface=screen, color=COLOUR_L2, points=corner_tuples_2)

    # Draw a circle where link 2 connects with the link 1, to represent joint 2
    pygame.draw.circle(surface=screen, color= COLOUR_JOINT, center=(start_x,start_y), radius= half_width)

    # Create and return a shapely polygon given the corner points, useful for checking collisions 
    return Polygon(corner_tuples_1), Polygon (corner_tuples_2)

# A function to draw a 2link RR manipulators, 2 links as rectangular polygons, 2 joints as circles
# Identical to the other function, except this one only calculates a polygon and doesnot attempt to draw on pygame
def create_manipulator_polygons(ground_x, ground_y, link_length_1, link_length_2, half_width, theta_1, theta_2):

    # For link 1: Calculate the start coordinates and the difference to the end coordinates of the centreline
    start_x, start_y = ground_x, ground_y
    delta_x, delta_y = link_length_1*math.cos(theta_1), -link_length_1*math.sin(theta_1)  # negative sign for flipped y axis in pygame

    # Rectangle's four corners given the start point and the difference to the end point of the centreline of link 1
    corner_tuples_1 = [
        (start_x - half_width * math.sin(theta_1), start_y - half_width * math.cos(theta_1)),
        (start_x + half_width * math.sin(theta_1), start_y + half_width * math.cos(theta_1)),
        (start_x + delta_x + half_width * math.sin(theta_1), start_y + delta_y + half_width * math.cos(theta_1)),
        (start_x + delta_x - half_width * math.sin(theta_1), start_y + delta_y - half_width * math.cos(theta_1))
    ]
    # For link 1: Calculate the start coordinates and the difference to the end coordinates of the centreline
    start_x, start_y = ground_x+delta_x, ground_y+delta_y # The start point of link 2 is the end point of the link 1
    delta_x, delta_y = link_length_2*math.cos(theta_1+theta_2), -link_length_2*math.sin(theta_1+theta_2)  # negative sign for flipped y axis in pygame

    # Rectangle's four corners given the start point and the difference to the end point of the centreline of link 1
    corner_tuples_2 = [
        (start_x - half_width * math.sin(theta_1+theta_2), start_y - half_width * math.cos(theta_1+theta_2)),
        (start_x + half_width * math.sin(theta_1+theta_2), start_y + half_width * math.cos(theta_1+theta_2)),
        (start_x + delta_x + half_width * math.sin(theta_1+theta_2), start_y + delta_y + half_width * math.cos(theta_1+theta_2)),
        (start_x + delta_x - half_width * math.sin(theta_1+theta_2), start_y + delta_y - half_width * math.cos(theta_1+theta_2))
    ]
    # Create and return a shapely polygon given the corner points, useful for checking collisions 
    return Polygon(corner_tuples_1), Polygon (corner_tuples_2)

# A function to draw an obstacle as a polygon given its centre coordinates, length and width
def draw_obstacle (obs_centre_x, obs_centre_y, obs_width, obs_height, OBSTACLE_COLOUR):
    # Half length and width
    obs_halfwidth, obs_halfheight = obs_width//2, obs_height//2

    # Rectangle's four corners given the start point and the difference to the end point of the centreline of link 1
    corner_tuples = [
    (obs_centre_x - obs_halfwidth, obs_centre_y - obs_halfheight),
    (obs_centre_x - obs_halfwidth, obs_centre_y + obs_halfheight),
    (obs_centre_x + obs_halfwidth, obs_centre_y + obs_halfheight),
    (obs_centre_x + obs_halfwidth, obs_centre_y - obs_halfheight)
    ]
    # Draw a polygon given the 4 corners to represent the rectangular obstacle 
    pygame.draw.polygon(surface=screen, color=OBSTACLE_COLOUR, points=corner_tuples)

    # Create a shapely polygon given the corner points, useful for checking collisions 
    return Polygon (corner_tuples)

# A function to create an obstacle as a polygon given its centre coordinates, length and width
# Identical to the other function, except this one only calculates a polygon and doesnot attempt to draw on pygame
def create_obstacle_polygon (obs_centre_x, obs_centre_y, obs_width, obs_height):
    # Half length and width
    obs_halfwidth, obs_halfheight = obs_width//2, obs_height//2

    # Rectangle's four corners given the start point and the difference to the end point of the centreline of link 1
    corner_tuples = [
    (obs_centre_x - obs_halfwidth, obs_centre_y - obs_halfheight),
    (obs_centre_x - obs_halfwidth, obs_centre_y + obs_halfheight),
    (obs_centre_x + obs_halfwidth, obs_centre_y + obs_halfheight),
    (obs_centre_x + obs_halfwidth, obs_centre_y - obs_halfheight)
    ]
    # Create a shapely polygon given the corner points, useful for checking collisions 
    return Polygon (corner_tuples)

# Draw a configuration space on pygame using the provided c-space grid map, which should be a square grid with 1s in indexes corresponding to obstacles 0 otherwise
def draw_configuration_space (cspace_grid):
    cs_space_size = len(cspace_grid)
    for i in range (cs_space_size):
        for j in range (cs_space_size):
            if cspace_grid[i,j] == 1: 
                COLOUR = COLOUR_OBSTACLE
            else:
                COLOUR = COLOUR_SCREEN
            pygame.draw.rect(surface=screen, color=COLOUR, rect= (PLOT_X0+(j*(PIXELS_PER_GRIDPOINT)), PLOT_Y0+((cs_space_size-i)*PIXELS_PER_GRIDPOINT), PIXELS_PER_GRIDPOINT, PIXELS_PER_GRIDPOINT))
            # pygame.draw.rect(surface=screen, color=COLOUR, rect= (PLOT_X0+j*(PLOT_WIDTH//PLOT_RESOLUTION), PLOT_Y0+(len(cspace_grid)-i)*(PLOT_HEIGHT//PLOT_RESOLUTION), (PLOT_WIDTH//PLOT_RESOLUTION), (PLOT_HEIGHT//PLOT_RESOLUTION)))

    # Compute the corners of the c-space plot
    top_left = (PLOT_X0, PLOT_Y0)
    top_right = (PLOT_X0+PIXELS_PER_GRIDPOINT*cs_space_size, PLOT_Y0)
    bottom_left = (PLOT_X0, PLOT_Y0+PIXELS_PER_GRIDPOINT*cs_space_size+PIXELS_PER_GRIDPOINT)
    bottom_right = (PLOT_X0+PIXELS_PER_GRIDPOINT*cs_space_size, PLOT_Y0+PIXELS_PER_GRIDPOINT*cs_space_size+PIXELS_PER_GRIDPOINT)

    # Create a border around the plot given the corners
    pygame.draw.line(surface=screen, color= COLOUR_TEXT, start_pos=top_left, end_pos= top_right, width=PIXELS_PER_GRIDPOINT)
    pygame.draw.line(surface=screen, color= COLOUR_TEXT, start_pos=bottom_left, end_pos= bottom_right, width=PIXELS_PER_GRIDPOINT)
    pygame.draw.line(surface=screen, color= COLOUR_TEXT, start_pos=top_left, end_pos= bottom_left, width=PIXELS_PER_GRIDPOINT)
    pygame.draw.line(surface=screen, color= COLOUR_TEXT, start_pos=top_right, end_pos= bottom_right, width=PIXELS_PER_GRIDPOINT)

# Create Configuration Space grid map 
def create_configuration_space (cs_resolution):

    # number of steps from 0 - 2pi should so that step size is 0.001 rad. 2pi / step size  = number of steps 
    theta_space = np.arange (0, 2*math.pi, cs_resolution)

    # create a configuration space grid with a resolution of 0.001 rad, axis ranges equal to the entire uncontrained space of thetas, 
    # All values set to 1 for obstacles. All grid points are obstacles unless found to be free
    cspace_grid = np.full ((len(theta_space), len(theta_space)), 1)

    # Draw an obstacle as a polygon, given its centre coordinates, width and height 
    obs_centre_x, obs_centre_y = ground_x + obs_centre_offset_x1, ground_y + obs_centre_offset_y1 
    obs_1_polygon = create_obstacle_polygon (obs_centre_x, obs_centre_y, obs_width, obs_height)

    # Draw an obstacle as a polygon, given its centre coordinates, width and height 
    obs_centre_x, obs_centre_y = ground_x + obs_centre_offset_x2, ground_y + obs_centre_offset_y2
    obs_2_polygon = create_obstacle_polygon (obs_centre_x, obs_centre_y, obs_width, obs_height)

    # iterate through the whole grid and search for free spaces or obstacles and update grid accordingly
    # bypass theta values that are out of joint constraints
    for i, theta_1 in enumerate (theta_space):
        if (((theta_1 < joint_1_upper_lim) and (theta_1 >= 0)) or ((theta_1 > joint_1_lower_lim) and (theta_1 <= 2*math.pi))):

            for j, theta_2 in enumerate (theta_space):
                # bypass theta values that are out of joint constraints
                if (((theta_2 < joint_2_upper_lim) and (theta_2 >= 0)) or ((theta_2 > joint_2_lower_lim) and (theta_2 <= 2*math.pi))):
                    
                    # Draw a 2link RR manipulator, 2 links as rectangular polygons, 2 joints as circles, Given the current thetas
                    link1_polygon, link2_polygon = create_manipulator_polygons(ground_x, ground_y, link_length_1, link_length_2, half_width, theta_1, theta_2)

                    # Check for collisions between Link polygons and the obstacles, if it doesnot  occur then mark grid point as 0 for free
                    if not (link1_polygon.intersects(obs_1_polygon) or link1_polygon.intersects(obs_2_polygon) or link2_polygon.intersects(obs_1_polygon) or link2_polygon.intersects(obs_2_polygon)):
                        cspace_grid [j, i] = 0 # each column is for a unique theta1, rows along each column for a all the theta2 values
                else:
                    continue
        else:
            continue
        
    return cspace_grid

# A function to draw a rectangle in the cs plot to denote the goal position
def draw_goal_point_in_cs (goal_theta_1, goal_theta_2, COLOUR, cspace_grid ):
    cs_space_size = len (cspace_grid)
    goal_theta_i = int((goal_theta_1/(2*math.pi))*cs_space_size)
    goal_theta_j = int((goal_theta_2/(2*math.pi))*cs_space_size)
    pygame.draw.rect(surface=screen, color=COLOUR, rect= (PLOT_X0+(goal_theta_i*(PIXELS_PER_GRIDPOINT)), PLOT_Y0+((cs_space_size-goal_theta_j)*PIXELS_PER_GRIDPOINT), 3*PIXELS_PER_GRIDPOINT, 3*PIXELS_PER_GRIDPOINT))

# A function to draw a rectangle in the cs plot to denote the current position
def draw_current_point_in_cs (theta_1, theta_2, COLOUR, cspace_grid ):
    cs_space_size = len (cspace_grid)
    theta_i = int((theta_1/(2*math.pi))*cs_space_size)
    theta_j = int((theta_2/(2*math.pi))*cs_space_size)
    pygame.draw.rect(surface=screen, color=COLOUR, rect= (PLOT_X0+(theta_i*(PIXELS_PER_GRIDPOINT)), PLOT_Y0+((cs_space_size-theta_j)*PIXELS_PER_GRIDPOINT), 3*PIXELS_PER_GRIDPOINT, 3*PIXELS_PER_GRIDPOINT))

# A function to just simulate the manipulator following the generated theta trajectories
def draw_motion_plan (cspace_grid, theta_trajectory):

    pygame.time.delay(500)

    last_theta_x , last_theta_y = 0, 0 # used later for maintaining configuration
    for theta_list in theta_trajectory:
        # Set the colour of the screen to all white
        # clear the screen of any past output
        screen.fill(color=COLOUR_SCREEN)

        # Draw a line to separate ws and cs in the pygame window
        pygame.draw.line(surface=screen, color= COLOUR_TEXT, start_pos=(700,0), end_pos= (700,700), width=5)

        # Draw text to: label Workspace and Cspace in the game window
        font = pygame.font.SysFont(name= None, size= 30 )
        title1_text = font.render ("Workspace", True, COLOUR_TEXT)
        title2_text = font.render ("C-Space",  True, COLOUR_TEXT)
        screen.blit(source= title1_text, dest= (300, 670))
        screen.blit(source= title2_text, dest= (1000, 670))

        # simulate motion
        # Theta coordinates along the trajectory
        theta_x, theta_y = theta_list[0], theta_list[1]
        theta_x, theta_y = convert_to_two_pi_range(theta_x), convert_to_two_pi_range(theta_y)

        # Draw an obstacle as a polygon, given its centre coordinates, width and height 
        obs_centre_x, obs_centre_y = ground_x + obs_centre_offset_x1, ground_y + obs_centre_offset_y1
        obs_1_polygon = draw_obstacle (obs_centre_x, obs_centre_y, obs_width, obs_height, COLOUR_OBSTACLE)

        # Draw an obstacle as a polygon, given its centre coordinates, width and height 
        obs_centre_x, obs_centre_y = ground_x + obs_centre_offset_x2, ground_y + obs_centre_offset_y2
        obs_2_polygon = draw_obstacle (obs_centre_x, obs_centre_y, obs_width, obs_height, COLOUR_OBSTACLE)

        # Draw a manipulator again but after checking for collision with obstacles and changing the colours to simulate collision
        # Draw a 2link RR manipulator, 2 links as rectangular polygons, 2 joints as circles, Given the current thetas
        link1_polygon, link2_polygon = draw_manipulator(ground_x, ground_y, link_length_1, link_length_2, half_width, theta_x, theta_y, COLOUR_LINK1, COLOUR_LINK2)
    
        # Draw a configuration space on pygame using the provided c-space grid map, which should be a square grid with 1s in indexes corresponding to obstacles 0 otherwise
        draw_configuration_space (cspace_grid)

        # Draw a rectangle in the cs plot to show the goal config
        draw_goal_point_in_cs (goal_theta_1, goal_theta_2, COLOUR_GOAL, cspace_grid )

        # Draw a rectangle in the cs plot to show the Current config
        draw_current_point_in_cs (theta_x, theta_y, COLOUR_LINK1, cspace_grid )

        pygame.time.delay(40)

        # Update the display with the new changes, update clock
        pygame.display.update()
        clock.tick(fps)
        last_theta_x, last_theta_y = theta_x, theta_y
    return last_theta_x, last_theta_y

# A flag to run the while loop for pygame until the window is closed
run = True 

# Give the Initial joint angles/ thetas of the robot in radians
theta_1, theta_2 =  convert_to_two_pi_range (math.pi/6), convert_to_two_pi_range (math.pi/4)

# Give the Target joint angles/ thetas of the robot in radians
goal_theta_1, goal_theta_2 =  convert_to_two_pi_range (5*math.pi/6), convert_to_two_pi_range (-math.pi/4)

# obtain a configuration space for the starting configuration
cspace_grid = create_configuration_space (cs_resolution)

# Load the already calculate trajectory
theta_trajectory = np.load ("demo_trajectory.npy")

while run:

    # initialize the temporary theta variables to track current changes
    temp_theta_1, temp_theta_2 = theta_1, theta_2

    # initialize the temporary obstacles centre relative position to track current changes
    obs_centre_dis_x1, obs_centre_dis_y1, obs_centre_dis_x2, obs_centre_dis_y2 = obs_centre_offset_x1, obs_centre_offset_y1, obs_centre_offset_x2, obs_centre_offset_y2 

    # Set the colour of the screen to all white
    # clear the screen of any past output
    screen.fill(color=COLOUR_SCREEN)

    # Draw a line to separate ws and cs in the pygame window
    pygame.draw.line(surface=screen, color= COLOUR_TEXT, start_pos=(700,0), end_pos= (700,700), width=5)

    # Draw text to: label Workspace and Cspace in the game window
    font = pygame.font.SysFont(name= None, size= 30 )
    title1_text = font.render ("Workspace", True, COLOUR_TEXT)
    title2_text = font.render ("C-Space",  True, COLOUR_TEXT)
    screen.blit(source= title1_text, dest= (300, 670))
    screen.blit(source= title2_text, dest= (1000, 670))
    

    # Check for Key Presses, perform an action according to the pressed key
    key = pygame.key.get_pressed()
    # pressing the following keys increases/ decreases "temporary" values of thetas
    if key[pygame.K_a]:
        temp_theta_1 -= 0.03
    if key[pygame.K_d]:
        temp_theta_1 += 0.03
    if key[pygame.K_w]:
        temp_theta_2 += 0.03
    if key[pygame.K_s]:
        temp_theta_2 -= 0.03

    # Simulate the motion trajectory
    if key[pygame.K_g]:
       temp_theta_1, temp_theta_2= draw_motion_plan (cspace_grid, theta_trajectory)

    # If the thetas exceed 0 - 2*pi range, set them back to an equivalent 0 - 2*pi values, 
    # otherwise the contraints cannot be checked due to periodic natures of angles
    temp_theta_1 = convert_to_two_pi_range (temp_theta_1)
    temp_theta_2 = convert_to_two_pi_range (temp_theta_2)
    
    #  If the changed value of theta is within limits, set it as the actual theta, otherwise don't allow the change
    if (((temp_theta_1 < joint_1_upper_lim) and (temp_theta_1 > 0)) or ((temp_theta_1 > joint_1_lower_lim) and (temp_theta_1 < 2*math.pi))):
        theta_1 = temp_theta_1
    if (((temp_theta_2 < joint_2_upper_lim) and (temp_theta_2 > 0)) or ((temp_theta_2 > joint_2_lower_lim) and (temp_theta_2 < 2*math.pi))):
        theta_2 = temp_theta_2

    # pressing the following keys increases/ decreases "temporary" values of obstacle coordinates
    if key[pygame.K_4]:
        obs_centre_dis_x1 -= 10
    if key[pygame.K_6]:
        obs_centre_dis_x1 += 10
    if key[pygame.K_8]:
        obs_centre_dis_y1 -= 10
    if key[pygame.K_5]:
        obs_centre_dis_y1 += 10

    # Create an obstacle as a polygon, given its centre coordinates, width and height 
    obs_centre_x, obs_centre_y = ground_x + obs_centre_dis_x1, ground_y + obs_centre_dis_y1
    obs_1_polygon = create_obstacle_polygon (obs_centre_x, obs_centre_y, obs_width, obs_height)

    # Create an obstacle as a polygon, given its centre coordinates, width and height 
    obs_centre_x, obs_centre_y = ground_x + obs_centre_dis_x2, ground_y + obs_centre_dis_y2
    obs_2_polygon = create_obstacle_polygon (obs_centre_x, obs_centre_y, obs_width, obs_height)

    # Create a manipulator polygon for checking checking for collision with obstacles and changing the colours to simulate collision
    # Create a 2link RR manipulator, 2 links as rectangular polygons, 2 joints as circles, Given the current thetas
    link1_polygon, link2_polygon = create_manipulator_polygons(ground_x, ground_y, link_length_1, link_length_2, half_width, theta_1, theta_2)

    # Check for collisions between Link polygon and the obstacles, if it occurs then change the colour of that link to simulate collision
    if (link1_polygon.intersects(obs_1_polygon) or link1_polygon.intersects(obs_2_polygon)):
        L1_COLOUR = COLOUR_LINK1_COLLISION
    else: 
        L1_COLOUR = COLOUR_LINK1

    if (link2_polygon.intersects(obs_1_polygon) or link2_polygon.intersects(obs_2_polygon)):
        L2_COLOUR = COLOUR_LINK2_COLLISION
    else: 
        L2_COLOUR = COLOUR_LINK2

    # Check if the obstacle centres have changed position, if so then Create the configuration space grid again
    if ((obs_centre_dis_x1 != obs_centre_offset_x1) or (obs_centre_dis_y1 != obs_centre_offset_y1) or (obs_centre_dis_x2 != obs_centre_offset_x2) or (obs_centre_dis_y2 != obs_centre_offset_y2)):  
        # Create a configuration space grid given the resolution, (and global joint limits, link lengths, ground position, obstacles relative position and so on)
        # cspace_grid = asyncio.run (create_configuration_space (cs_resolution))
        cspace_grid = create_configuration_space (cs_resolution)

    # Apply the changes to the obstacles centre relative position to track current changes
    obs_centre_offset_x1, obs_centre_offset_y1, obs_centre_offset_x2, obs_centre_offset_y2 = obs_centre_dis_x1, obs_centre_dis_y1, obs_centre_dis_x2, obs_centre_dis_y2 

    # Draw an obstacle as a polygon, given its centre coordinates, width and height 
    obs_centre_x, obs_centre_y = ground_x + obs_centre_dis_x1, ground_y + obs_centre_dis_y1
    obs_1_polygon = draw_obstacle (obs_centre_x, obs_centre_y, obs_width, obs_height, COLOUR_OBSTACLE)

    # Draw an obstacle as a polygon, given its centre coordinates, width and height 
    obs_centre_x, obs_centre_y = ground_x + obs_centre_dis_x2, ground_y + obs_centre_dis_y2
    obs_2_polygon = draw_obstacle (obs_centre_x, obs_centre_y, obs_width, obs_height, COLOUR_OBSTACLE)

    # Draw a manipulator again but after checking for collision with obstacles and changing the colours to simulate collision
    # Draw a 2link RR manipulator, 2 links as rectangular polygons, 2 joints as circles, Given the current thetas
    link1_polygon, link2_polygon = draw_manipulator(ground_x, ground_y, link_length_1, link_length_2, half_width, theta_1, theta_2, L1_COLOUR, L2_COLOUR)
  
    # Draw a configuration space on pygame using the provided c-space grid map, which should be a square grid with 1s in indexes corresponding to obstacles 0 otherwise
    draw_configuration_space (cspace_grid) 

    # Draw a rectangle in the cs plot to show the goal config
    draw_goal_point_in_cs (goal_theta_1, goal_theta_2, COLOUR_GOAL, cspace_grid )

    # Draw a rectangle in the cs plot to show the Current config
    draw_current_point_in_cs (theta_1, theta_2, COLOUR_LINK1, cspace_grid )

    # Check for Quit command and Quit if yes
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False
    # Update the display with the new changes, update clock
    pygame.display.update()
    clock.tick(fps)

pygame.quit() 