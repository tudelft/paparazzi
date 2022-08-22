#!/usr/bin/env python3

# Import a library of functions called 'pygame'
import pygame
pygame.version.ver
from math import pi
 
# Initialize the game engine
pygame.init()
 
# Define the colors we will use in RGB format
BLACK = (  0,   0,   0)
WHITE = (255, 255, 255)
BLUE =  (  0,   0, 255)
GREEN = (  0, 255,   0)
RED =   (255,   0,   0)
 
# Set the height and width of the screen
size = [800, 400]
screen = pygame.display.set_mode(size)
font = pygame.font.Font(pygame.font.get_default_font(), 10)

pygame.display.set_caption("Visualisation approach moving target")
 
#Loop until the user clicks the close button.
done = False
clock = pygame.time.Clock()

i=5

def blitRotate(surf, image, pos, originPos, angle):

    # offset from pivot to center
    image_rect = image.get_rect(topleft = (pos[0] - originPos[0], pos[1]-originPos[1]))
    offset_center_to_pivot = pygame.math.Vector2(pos) - image_rect.center
    
    # roatated offset from pivot to center
    rotated_offset = offset_center_to_pivot.rotate(-angle) # original was negative angle

    # roatetd image center
    rotated_image_center = (pos[0] - rotated_offset.x, pos[1] - rotated_offset.y)

    # get a rotated image
    rotated_image = pygame.transform.rotate(image, -angle)
    rotated_image_rect = rotated_image.get_rect(center = rotated_image_center)

    # rotate and blit the image
    surf.blit(rotated_image, rotated_image_rect)
  
    # draw rectangle around the image
    #pygame.draw.rect(surf, (255, 0, 0), (*rotated_image_rect.topleft, *rotated_image.get_size()),2)
 
while not done:
 
    # This limits the while loop to a max of 5 times per second.
    # Leave this out and we will use all CPU we can.
    clock.tick(2)
     
    for event in pygame.event.get(): # User did something
        if event.type == pygame.QUIT: # If user clicked close
            done=True # Flag that we are done so we exit this loop
 
    # All drawing code happens after the for loop and but
    # inside the main while done==False loop.
     
    # Clear the screen and set the screen background
    screen.fill(WHITE)

    #Draw vertical line to seperate top and side view
    pygame.draw.line(screen, BLACK, [int(size[0]/2), 0], [int(size[0]/2),int(size[1])], 5)
    
    # Boat drawing
    boat_topview = pygame.image.load('boat_topview.png')
    boat_sideview = pygame.image.load('boat_sideview.png')

    # Scale boat png's down
    boat_topview  = pygame.transform.scale(boat_topview, (50,100))
    boat_sideview = pygame.transform.scale(boat_sideview, (100,50))
    boat_w_topv, boat_h_topv = boat_topview.get_size()
    boat_w_sidev, boat_h_sidev = boat_sideview.get_size()

    # define angles descent line
    phi = 160 # [deg] 
    slope_ref = 30 # [deg]
    heading = i
    COG_ship = heading + 15 # [deg]
    start_length_descent_line_real = 60 #[m]
    ref_dist_ship = 40 #[m] current distance to ship at descent line
    scale_ref_dot = ref_dist_ship/start_length_descent_line_real
    HDG_drone = heading # drone has same heading as ship for now

    # Ship Topview
    boat_center_top = (200, 200)
    offset_ship_from_center = pygame.math.Vector2(0, -80) # let the ship rotate arround the center, but with an offset
    boat_center_top  = boat_center_top + offset_ship_from_center.rotate(heading) # calculate new ship position
    rel_boat_rotation_point = pygame.math.Vector2(boat_w_topv/2, boat_h_topv/2)
    blitRotate(screen, boat_topview, boat_center_top, rel_boat_rotation_point, heading) # Rotate boat topview around some point and plot

    # Ship Sideview
    boat_center_side = (420, 350)
    screen.blit(boat_sideview, boat_center_side) # plot boat


    # Drone drawing 
    drone_topview = pygame.image.load('drone_topview.png')
    drone_sideview = pygame.image.load('drone_sideview.png')

    # Scale drone png's down
    drone_topview  = pygame.transform.scale(drone_topview, (30,30))
    drone_sideview = pygame.transform.scale(drone_sideview, (30,30))
    drone_w_topv, drone_h_topv = drone_topview.get_size()
    drone_w_sidev, drone_h_sidev = drone_sideview.get_size()

    i = i+1
    drone_center_top = (200, 50)
    drone_center_side = (700, 150)
    rel_drone_rotation_point = pygame.math.Vector2(drone_w_topv/2, drone_h_topv/2)
    blitRotate(screen, drone_topview, drone_center_top, rel_drone_rotation_point, HDG_drone) # plot boat
    screen.blit(drone_sideview, drone_center_side) # plot drone


    # Draw lines topview
    # Descent line
    descent_target_top = boat_center_top
    descent_length_top = pygame.math.Vector2(0, -250) # only the length of the line
    descent_start_top  = descent_target_top + descent_length_top.rotate(heading+phi)
    pygame.draw.line(screen, RED, descent_start_top, descent_target_top,1)
    # Heading ship
    heading_length_top = pygame.math.Vector2(0, -100)
    heading_point_top  = boat_center_top + heading_length_top.rotate(heading)
    pygame.draw.line(screen, BLUE, boat_center_top, heading_point_top,2)
    # Course over ground ship
    COG_length_top = pygame.math.Vector2(0, -100)
    COG_point_top  = boat_center_top + COG_length_top.rotate(COG_ship)
    pygame.draw.line(screen, BLUE, boat_center_top, COG_point_top,1)

    # Draw reference dot on descent line topview
    reference_dist_top = descent_length_top*scale_ref_dot # distance from ship
    reference_point_top = descent_target_top + reference_dist_top.rotate(heading+phi)
    pygame.draw.circle(screen, RED, reference_point_top, 4) # draw reference dot on descend line


    # Draw line sideview
    descent_target_side = pygame.math.Vector2(int(420+(boat_w_sidev/2)), int(350+(boat_h_sidev/2)))
    descent_length_side = pygame.math.Vector2(250, 0) # only the length of the line
    descent_start_side  = descent_target_side + descent_length_side.rotate(-slope_ref)
    pygame.draw.line(screen, RED, descent_start_side, descent_target_side,1)

    # Draw reference dot sideview
    reference_dist_side = descent_length_side*scale_ref_dot # distance from ship
    reference_point_side = descent_target_side + reference_dist_side.rotate(-slope_ref)
    pygame.draw.circle(screen, RED, reference_point_side, 4) # draw reference dot on descend line


    # print debug info in screen
    horizontal_error = round(abs(reference_point_top[0]-i),2)
    text_topview = font.render('horizontal error = '+ str(horizontal_error), True, (0, 0, 0))
    screen.blit(text_topview, dest=(50,350))

    # print debug info in screen
    vertical_error = round(abs(reference_point_side[1]-i),2)
    text_sideview = font.render('vertical error = '+ str(vertical_error), True, (0, 0, 0))
    screen.blit(text_sideview, dest=(650,350))

    

    # Go ahead and update the screen with what we've drawn.
    # This MUST happen after all the other drawing commands.
    pygame.display.flip()
 
# Be IDLE friendly
pygame.quit()