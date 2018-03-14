class Node_(object):
    """This is the Node class, with all it's properties of a state and other side infomration needed for project 1 - ColumbiaX www.edx.org - AI. 
    February, 2018 - Ejup Hoxha"""
    def __init__(self, state_, node_depth, result_of_action, parent_, cost, max_depth):

        self.state = state_;
        self.depth = node_depth;
        self.result_of_action = result_of_action;
        self.parent = parent_
        self.cost = cost;
        self.max_search_depth = max_depth;

    #@property
    #def state_(self):
    #    return self.state

