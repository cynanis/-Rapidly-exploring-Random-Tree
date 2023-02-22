class Node:

    def __init__(self,q,u=None):
        """ q = {"x":x,"y"=y,"theta"=theta,"delta":delta,"beta":beta} """
        self.q = q
        self.u = u
        self.children = []
        self.cost = None
        self.parent = None
        self.goal_reached = False

    def add_child(self,child_x):
        child_x.parent = self
        self.children.append(child_x)
    
    def remove_child(self,child_x):
        self.children = [child for child in self.children if child is not child_x]
    
    def add_cost(self,cost):
        self.cost = cost    
        