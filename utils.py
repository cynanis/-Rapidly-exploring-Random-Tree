


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

def ecludian(x_n, x_r):
    x1 , y1 = x_n
    x2, y2 = x_r
    return ((x2-x1)**2 + (y2 - y1)**2)**0.5