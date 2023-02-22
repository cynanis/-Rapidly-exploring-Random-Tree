import pygame
from rrt import RRT
from rrt_star import RRTStar
import argparse

parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('-t', "--type", type=str, default='rrt',
                    help='rrt or rrt_star')
args = parser.parse_args()
# Initializing Pygame
pygame.init()

# Initializing surface
map = pygame.display.set_mode((600,600))

map.fill((255,255,255))

#draw star and goal points
q_start={"x":100,"y":20,"theta":0.3,"delta":0.1,"beta":0.01}
q_goal={"x":400,"y":500,"theta":0,"delta":0,"beta":0}
star_color = (255,0,255,255)
goal_color = (0,0,255,255)
pygame.draw.circle(map,star_color,center=(q_start["x"],q_start["y"]),radius=5,width=5)
pygame.draw.circle(map,goal_color,center=(q_goal["x"],q_goal["y"]),radius=7,width=7)

#draw obstacles
obstacle_color = (0,255,0,255)
pygame.draw.rect(map,obstacle_color,[300,300,50,50],width=50)
pygame.draw.rect(map,obstacle_color,[100,400,50,50],width=50)
pygame.draw.circle(map,obstacle_color,center=(400,60),radius=40,width=40)
pygame.draw.circle(map,obstacle_color,center=(250,60),radius=40,width=40)
pygame.draw.rect(map,obstacle_color,[100,80,50,50],width=50)
pygame.draw.rect(map,obstacle_color,[600,400,50,50],width=50)
pygame.draw.rect(map,obstacle_color,[10,80,50,50],width=50)
pygame.draw.rect(map,obstacle_color,[300,130,50,50],width=50)

# Initializing RTT
rrt = None
if args.type == "rrt":
    rrt = RRT(map=map,q_star=q_start,q_goal=q_goal,goal_threshold=7, obstacles_color=obstacle_color)
elif args.type == "rrt_star":
    rrt = RRTStar(map=map,q_star=q_start,q_goal=q_goal,goal_threshold=7, obstacles_color=obstacle_color)

#Build RRT
rrt.build_rrt(int(1e6))

input('Press ENTER to exit')