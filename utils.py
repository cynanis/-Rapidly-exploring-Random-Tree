import pygame


def line(x1, x2):
    """denote the straight-line path from x1 to x2"""
    x_1 , y_1 = x1
    x_2, y_2  = x2
    a = (y_2 - y_1)/(x_2 - x_1+1e-7)
    start = x_1
    end = x_2+1
    step = 1
    if(x1 > x2):
        start = x_1+1
        end = x_2
        step = -1
        
    line_  = [(x, round(a*(x-x_1)+y_1)) for x in range(start,end,step)]
    
    return line_

def parent(v):
    if v.parent is not None:
        return v.parent
    return v

def cost(v):
    if v.cost is not None:
        return v.cost
    elif v.parent is None:
        return 0
    
    return cost(parent(v)) + c(line(parent(v).data,v.data))

def c(line):
    return ecludian(line[0],line[-1])

def ecludian(q_n, q_r):
    x1 , y1 = q_n
    x2, y2 = q_r
    return ((x2-x1)**2 + (y2 - y1)**2)**0.5

def near(x,q,r):
    """
    returns list of nodes lie in the cirlce centered at q with raduis r

    Args:
        x (Node): root node
        q (Typle): (x,y) circle center
        r (Int): circle raduis

    Returns:
        Xnear: list of near nodes
    """
    list = []
    dist = ecludian(x.q,q)
    if(dist < r):
        list.append(x)
    for x_ in x.children:
        list.extend(near(x_,q,r))
    return list


def is_goal_reached(line,q_goal,goal_threshold):

    for q in reversed(line):
        if ecludian(q,q_goal) < goal_threshold:
            return True
    return False


def draw_branch(map,q_start, q_end,color = (0,0,0)):
    # Drawing line
    pygame.draw.circle(map,color,center=q_end,radius=2,width=2)
    pygame.draw.line(map,color,start_pos=q_start,end_pos=q_end,width=1)
    pygame.display.update() 

def delete_branch(map,q_start, q_end,color = (255,255,255)):
    # delete line
    pygame.draw.line(map,color,start_pos=q_start,end_pos=q_end,width=1)
    pygame.display.update() 

def draw_point(map,q_point,color = (0,0,0)):
    # Drawing point
    pygame.draw.circle(map,color,center=q_point,radius = 1,width=1)
    pygame.display.update() 