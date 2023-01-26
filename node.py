class Node:

    def __init__(self,q):
        """ q = (x,y) """
        self.q = q
        self.children = []
        self.cost = None
        self.parent = None

    def add_child(self,child_x):
        child_x.parent = self
        self.children.append(child_x)
    
    def remove_child(self,child_x):
        self.children = [child for child in self.children if child is not child_x]
    
    def add_cost(self,cost):
        self.cost = cost    