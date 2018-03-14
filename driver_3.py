import sys
import resource
import Node_
import decimal
import gc
import queue
from queue import *
from gc import *
from decimal import *
from sys import *
from collections import *
from Node_ import *

class driver_3:
    """This class will take inputs method of search and board of 8-Puzzle Game. This code is written for Artificial Intelligence - ColumbiaX Course on edx.org
    EJUP HOXHA
    COMPUTERIZED AUTOMATION AND ROBOTICS
    FACULTY OF ELECTRICAL AND COMPUTER ENGINEERING - F.I.E.K.
    UNIVERSITY OF PRISHTINA"""

    def __init__(self, argv):	#method, board, argv):            #Initialization function. Initializes inputs.
        board_ = []
        board_ = argv[2].split(',')
        self.board = list(map(int,board_))
        self.method = argv[1]
        ############################# I didn't want to create some properties, since there is no need for them right now    
        self.path_to_goal = []      # So I created some variables that I can use.
        self.cost_of_path = 0       #
        self.explored_nodes = 0     # 
        self.search_depth = 0       #
        self.max_search_depth = 0   #
        #############################
        #print ("Method USED : " + str(self.method))
        #print ("      BOARD : " + str(self.board))

    """This function will take inputs _actualNode and _direction = {0: 'UP', 1: 'DOWN', 2: 'LEFT', 3: 'RIGHT'}
       As return we will have a list of new neighbour nodes"""
    
    def generateNodes(self, _actualNode, rvs, new_depth):
        _zero = self.findZero(_actualNode)
        _neighbour = []
        if rvs == True:
           _neighbour.insert(0,self.swapRight(_zero,_actualNode, new_depth))
           _neighbour.insert(1,self.swapLeft(_zero,_actualNode, new_depth))
           _neighbour.insert(2,self.swapDown(_zero,_actualNode, new_depth))
           _neighbour.insert(3,self.swapUp(_zero,_actualNode, new_depth))
 
        else:
           _neighbour.insert(0,self.swapUp(_zero,_actualNode, new_depth))
           _neighbour.insert(1,self.swapDown(_zero,_actualNode, new_depth))
           _neighbour.insert(2,self.swapLeft(_zero,_actualNode, new_depth))
           _neighbour.insert(3,self.swapRight(_zero,_actualNode, new_depth))
       
        self.explored_nodes +=1                                           # We use this variable to know the explored_nodes
        for i in range(len(_neighbour)):
            if 'notPossible' in _neighbour:
               _neighbour.remove('notPossible')

        return _neighbour

    """This function will take the _board and return position of '0' on the board"""
    def findZero(self, _board):
        if 0 in _board:
            i = _board.index(0);
            return i

    """This function will take direction and the board and generate the board node after calculations. _direction = { Right=1, Left=-1, Up=-3, Down=3}"""
    def swap(self, _zero, actualNode, _direction,new_depth):

        myList = actualNode[:]  
        del myList[-1]
        temp = myList[_zero]
        myList[_zero] = myList[_zero+_direction]
        myList[_zero+_direction] = temp
        
        if cl1.method == 'bfs' or cl1.method == 'dfs':
           myList.insert(len(myList)+1,new_depth)
        else:
           myList.insert(0,new_depth)               # the depth we are going to use as a high weight whenever the f(n) of different states is equal
                                                    # and the heappop compares the values of states in the priority queue.
        
        if _direction == -3:
           myList.insert(len(myList)+1,'Up')
        elif _direction == -1:
             myList.insert(len(myList)+1,'Left')
        elif _direction == 1:
             myList.insert(len(myList)+1,'Right')
        elif _direction == 3:
             myList.insert(len(myList)+1,'Down')

        return myList[:]

    #########  Swap: UP, DOWN, LEFT & RIGHT  ###########

    def swapUp(self, _zero, _actualNode, new_depth):
        if _zero not in [0,1,2]:                          # If the zero is not in the top row, swap. Else: this move is invalid
           return self.swap(_zero,_actualNode,-3,new_depth)   # Direction = -3 for UP swap.
        else:
           return 'notPossible'

    def swapDown(self, _zero, _actualNode, new_depth):
        if _zero not in [6,7,8]:                          # If the zero is not in the bottom row, swap. Else: this move is invalid
           return self.swap(_zero,_actualNode,3, new_depth)    # Direction = 3 for Down swap.
        else:
           return 'notPossible'
    
    def swapLeft(self, _zero, _actualNode, new_depth):
        if _zero not in [0,3,6]:                # We can not do left swap if we are on 0, 3 and 6 index of the board.
           return self.swap(_zero,_actualNode,-1, new_depth)    # Direction = -1 for Left swap.
        else:
           return 'notPossible'
    
    def swapRight(self, _zero, _actualNode, new_depth):
        if _zero not in [2,5,8]:                # We can not do right swap if we are on 2, 5 and 8 index of the board.
           return self.swap(_zero,_actualNode,1, new_depth)    # Direction = 1 for Right swap.
        else:
           return 'notPossible'
    ####################################################

    def BreadthFirstSearch_BFS(self, goalTest):
     
        initialState = self.board                          #we will read the initial board
        initialState_str = "".join(map(str,initialState))  #we will convert the initial board to string '012345678' for dict frontier_str
        frontier_str = {}
        frontier_v = deque();
        explored = set()
        f_v_initialState = initialState[:]                                                      # Copy the list from initialState
        f_v_initialState.insert(10,'I')                                                         # Add 'I' to the 10th elemnt of the list, as information that this state is initial state.
        f_v_initialState.insert(9,0)                                                            # At position 9 we insert depth of the node
        frontier_v.appendleft(f_v_initialState)                                                 # Insert the initial state in frontier + 'I' - meaning it's the initial state. In [0,1,2,3,4,5,6,7,8,'I'] format.
        frontier_str[initialState_str] = initialState                                           # Insert the initial state in dict. This dict is used just for checking because of O(n log n) time operation...
                                                                                                # frontier_str will contain a key in this format '012345678' and the state in this format [0,1,2,3,4,5,6,7,8]
        node = cl1.newNode(f_v_initialState, f_v_initialState[9], f_v_initialState[10], f_v_initialState, 1, 0)    # we will create a node. (state_, node_depth, result_of_action, parent, cost, max_depth) 
        fv = f_v_initialState[:]                                                                                        # E.g. ([0,1,2,3,4,5,6,7,8,'Down'],0,'Up',parentstate=[3,1,2,0,4,5,6,7,8],1, max_depth)
        del fv[9]																		        # we will remove depth, as we don't want to have it on the key
        nodes_frontier = {}                                                                     # in this dict we will keep all the nodes objects.
        nodes_frontier["".join(map(str,fv))] = node                               # The nodes object will be added with the key: of state converted to string --> '1203456780Up'
        del f_v_initialState
        del fv
        del node
        del initialState_str
        while True:
                 state_v = frontier_v.popleft();                         # Pop from left side, since we appended in the right side. state_v  format is for e.g. [0,1,2,3,4,5,6,7,8,1,'Down'] _ state_v[9] - depth
                 new_depth = (state_v.pop(9)+1)
                 exp_list = state_v[:]                                   # Copy the list from state to exp_list for manipulation
                 del exp_list[-1]                                        # Delete the last element containing move information, as we don't need it on the frontier.
                 exp_str = "".join(map(str,exp_list))                    # Now we have exp_str in the format '012345678'
                 del frontier_str[exp_str]                               # We delete the element with the key '012345678' format from frontier_str (where all the states are associated to a key of this format).
                 explored.add(exp_str)                                   # We add the state to explored states set, so we don't check and don't add that same state to the frontier ever again.
                                                                         # Explanation: We use nodes_frontier to keep all the informations for the nodes, we use frontier_v to keep the state lists,
                                                                         # we use explored set to keep the sets in '012345678' format, and we cannot add the same element two times,
                                                                         # we use frontier_str to keep the states of the frontier in the format [0,1,2,3,4,5,6,7,8] associated to a key in the format '012345678' 
                                                                         # the reason is that it's way faster to check O(n log n) time operations, if the element is in the frontier_str~formated cpy of fronter_v
                 if exp_list == goalTest:                                # we check the exp_list ([0,1,2,3,4,5,6,7,8]) which is a formated state_v, if is the goal[0,1,2,3,4,5,6,7,8].
                    return cl1.roadToSuccess(nodes_frontier, state_v)
                 del exp_list
                 del exp_str
                 if self.max_search_depth < new_depth:
                    self.max_search_depth = new_depth                    # New maximal depth.
                   
                 for newState in cl1.generateNodes(state_v, False, new_depth):                  # generateNodes will return the generated states from the current parent state, [1,2,3,4,5,6,7,8,0,'Down']
                     manipulator_state = newState[:]                                           # We will use this manipulator to copy the newState of format [0,1,2,3,4,5,6,7,8,'Down'] needed for 'roadToSuccess'
                     del manipulator_state[-1]                                                 # Delete the last element, needed for frontier_str and explored
                     del manipulator_state[-1] 
                     manipulator_str="".join(map(str,manipulator_state))                       # Calculate the string of manipulator state
                     if manipulator_str not in frontier_str:                                   # If this state is not in the frontier_str
                        if manipulator_str not in explored:                                    # If this state is not in the explored
                            nodes_frontier[manipulator_str + str(newState[10])] = cl1.newNode(newState, new_depth, state_v[9], state_v , 1, self.max_search_depth)#(newstate,depth,direction,parent_,cost,max_depth)
                            frontier_v.append(newState)
                            frontier_str[manipulator_str] = manipulator_state
                 if len(frontier_v) < 1:
                    print ("NO SOLUTION FOUND!\n" + "EXPLORED NODES : " + str(self.explored_nodes)+ "\nLast State = " + str(state_v))
                    return 'f'
                       
    ####################################################   

    ####################################################

    def DepthFirstSearch_DFS(self, goalTest):
     
        initialState = self.board                          #we will read the initial board
        initialState_str = "".join(map(str,initialState))  #we will convert the initial board to string '012345678' for dict frontier_str
        frontier_str = {}
        frontier_v = deque();
        explored = set()
        f_v_initialState = initialState[:]                                                      # Copy the list from initialState
        f_v_initialState.insert(10,'I')                                                         # Add 'I' to the 9th elemnt of the list, as information that this state is initial state.
        f_v_initialState.insert(9,0)                                                            # At position 9 we insert depth of the node
        frontier_v.appendleft(f_v_initialState)                                                 # Insert the initial state in frontier + 'I' - meaning it's the initial state. In [0,1,2,3,4,5,6,7,8,'I'] format.
        frontier_str[initialState_str] = initialState                                           # Insert the initial state in dict. This dict is used just for checking because of O(n log n) time operation...
                                                                                                # frontier_str will contain a key in this format '012345678' and the state in this format [0,1,2,3,4,5,6,7,8]
        node = cl1.newNode(f_v_initialState, f_v_initialState[9], f_v_initialState[10], f_v_initialState, 1, 0)    # we will create a node. (state_, node_depth, result_of_action, parent, cost, max_depth) 
        fv = f_v_initialState[:]                                                                                        # E.g. ([0,1,2,3,4,5,6,7,8,'Down'],0,'Up',parentstate=[3,1,2,0,4,5,6,7,8],1, max_depth)
        del fv[9]
        nodes_frontier = {}                                                                     # in this dict we will keep all the nodes objects.
        nodes_frontier["".join(map(str,fv))] = node                               # The nodes object will be added with the key: of state converted to string --> '1203456780Up'
        del f_v_initialState
        del fv
        del node
        del initialState_str
        while True:
                 state_v = frontier_v.popleft();                         # Pop from left side, since we appended in the right side. state_v  format is for e.g. [0,1,2,3,4,5,6,7,8,1,'Down'] _ state_v[9] - depth
                 new_depth = (state_v.pop(9)+1)
                 exp_list = state_v[:]                                   # Copy the list from state to exp_list for manipulation
                 del exp_list[-1]                                        # Delete the last element containing move information, as we don't need it on the frontier.
                 exp_str = "".join(map(str,exp_list))                    # Now we have exp_str in the format '012345678'
                 del frontier_str[exp_str]                               # We delete the element with the key '012345678' format from frontier_str (where all the states are associated to a key of this format).
                 explored.add(exp_str)                                   # We add the state to explored states set, so we don't check and don't add that same state to the frontier ever again.
				                                                         # Explanation: We use nodes_frontier to keep all the informations for the nodes, we use frontier_v to keep the state lists,
                                                                         # we use explored set to keep the sets in '012345678' format, and we cannot add the same element two times,
                                                                         # we use frontier_str to keep the states of the frontier in the format [0,1,2,3,4,5,6,7,8] associated to a key in the format '012345678' 
                                                                         # the reason is that it's way faster to check O(n log n) time operations, if the element is in the frontier_str~formated cpy of fronter_v
                 if exp_list == goalTest:                                # we check the exp_list ([0,1,2,3,4,5,6,7,8]) which is a formated state_v, if is the goal[0,1,2,3,4,5,6,7,8].
                    return cl1.roadToSuccess(nodes_frontier, state_v)
                 del exp_list
                 del exp_str
                 if self.max_search_depth < new_depth:
                    self.max_search_depth = new_depth                                # New maximal depth.
                 for newState in cl1.generateNodes(state_v,True, new_depth):                  # generateNodes will return the generated states from the current parent state, [1,2,3,4,5,6,7,8,0,'Down']         
                     manipulator_state = newState[:]                                          # We will use this manipulator to copy the newState of format [0,1,2,3,4,5,6,7,8,'Down'] needed for 'roadToSuccess'
                     del manipulator_state[-1]                                                # Delete the last element, needed for frontier_str and explored
                     del manipulator_state[-1] 
                     manipulator_str="".join(map(str,manipulator_state))                      # Calculate the string of manipulator state
                     if manipulator_str not in frontier_str:                                  # If this state is not in the frontier_str
                        if manipulator_str not in explored:                                   # If this state is not in the explored # The key of nodes_frontier is in format '012345678Up'
                           nodes_frontier[manipulator_str + str(newState[10])] = cl1.newNode(newState, new_depth, state_v[9], state_v , 1, self.max_search_depth)
						                                                                   #(newstate, depth, operation_direction ,parent_state, cost, max_depth)
                           frontier_v.appendleft(newState)
                           frontier_str[manipulator_str] = manipulator_state
                 if len(frontier_v) < 1:
                    print ("NO SOLUTION FOUND!\n" + "EXPLORED NODES : " + str(self.explored_nodes)+ "\nLast State = " + str(state_v))
                    return 'f'
                       
    ####################################################                 
    def A_star(self, goalTest):

        initialState = self.board                          #we will read the initial board
        initialState_str = "".join(map(str,initialState))  #we will convert the initial board to string '012345678' for dict frontier_str
        state_mn = initialState[:]
        state_mn.insert(0,0)
        state_mn.insert(10,'I')

        frontier_str = {}
        frontier = PriorityQueue()
        nodes_frontier = {}
        explored = set()

        frontier_str[initialState_str] = initialState      # Format: ['012345678': [0,1,2,3,4,5,6,7,8]]
        node = cl1.newNode(state_mn,0,'I',state_mn,1,0)    # newNode(state_of_node, depth_of_this_node, move_from which_node_is_born, parent_state, cost, max_depth)
        #heappush(frontier, (cl1.manhattan_fn(state_mn),state_mn))  
        frontier.put((cl1.manhattan_fn(state_mn),state_mn))  # Push the state with an underestimated value of steps (f(n) = g(n)+h(n)) to the solution
        cp = state_mn[:]
        del cp[0]
        nodes_frontier["".join(map(str,cp))] = node  # Fromat: Key= '01234567Down', node.state=[cost,0,1,2,3,4,5,6,7,8,'Down']
        del node
        del state_mn
        del initialState
        del initialState_str
        del cp

        while True:                          
                 x, state_v = frontier.get()               # Pop the state, corresponding with the lowest value of f(n)
                 del x
                 new_depth = (state_v.pop(0)+1)
                 exp_list = state_v[:]                               # Format: [0,1,2,3,4,5,6,7,8,'Down']
                 del exp_list[-1]
                 exp_str = "".join(map(str,exp_list))                    # Now we have exp_str in the format '012345678'
                 del frontier_str[exp_str]                               # We delete the element with the key '012345678': [0,1,2,3,4,5,6,7,8]
                 explored.add(exp_str)                                   # Format: '012345678'
                                                                        
                 if exp_list == goalTest: 
                    return cl1.roadToSuccess(nodes_frontier, state_v)

                 if self.max_search_depth < new_depth:                   # If new_depth is even deeper than actual maximum depth, make it as a new maximum depth.
                    self.max_search_depth = new_depth                    # New maximal depth.
                 del exp_list

                 for newState in cl1.generateNodes(state_v, False, new_depth):  # generateNodes will return the generated states from the current parent state, [0,1,2,3,4,5,6,7,8,cost,'Down']
                     manipulator_state = newState[:]                                           # We will use this manipulator to copy the newState of format [0,1,2,3,4,5,6,7,8,'Down'] needed for 'roadToSuccess'
                     del manipulator_state[0]                                                 # Delete the last element, needed for frontier_str and explored
                     newState_str = "".join(map(str,manipulator_state))                                 # This will be used as a key of each state in nodes_frontier, to generate the path to success.
                     del manipulator_state[-1]
                     manipulator_str="".join(map(str,manipulator_state))                       # Calculate the string of manipulator state
                     if manipulator_str not in frontier_str:               
                        if manipulator_str not in explored:       
                            new_node = cl1.newNode(newState, new_depth, newState[0], state_v, 1, self.max_search_depth) # We create new_node instance!
                            nodes_frontier[newState_str] =  new_node                    # We add the new node to the nodes_frontier
                            #heappush(frontier, (cl1.manhattan_fn(newState)+new_depth, newState))      # We add the state to the frontier heapq
                            frontier.put((cl1.manhattan_fn(newState)+new_depth, newState))
                            frontier_str[manipulator_str] = manipulator_state           # frontier_str['012345678'] = [0,1,2,3,4,5,6,7,8]

                 if frontier.empty():
                    print ("NO SOLUTION FOUND!\n" + "EXPLORED NODES : " + str(self.explored_nodes)+ "\nLast State = " + str(state_v))
                    return 'f'            
    ####################################################

    def newNode(self, state_, node_depth, result_of_action, parent, cost, max_depth):          # Since it is easier to keep track and do calc with object oriented...
        return Node_(state_, node_depth, result_of_action, parent, cost, max_depth)            # Creates a node and we have all information needed for that node...

    ####################################################

    def roadToSuccess(self, nodes_frontier, state_v):                 # We take nodes_frontier and last state_v in the format [0,1,2,3,4,5,6,7,8,'whatever']
        node = nodes_frontier.pop("".join(map(str,state_v)))          # pop the node with the format state_v key in the format '012345678whatever'
        self.path_to_goal.insert(0, node.state[9])                   # insert the move which resulted in the solution state_v
        state = node.parent                                           # now we use the parent and we will learn the information where did parent come.

        if node.state[9] == 'I':                                      # if we have the whole path_to_goal, return the array.
            return self.path_to_goal.reverse()

        while True:
              node_x = nodes_frontier.pop("".join(map(str,state)))    # node_x = pop of nodes_frontier where 
              if node_x.state[9] == 'I':                              # if we have the whole path_to_goal, return the array.
                 return self.path_to_goal.reverse(), cl1.cal_cost_depth_max(node) #reverse the sel.path_to_goal, to have the right order of actions top to goal then call cal_cost_depth_max(node).
              indx = len(self.path_to_goal)+1;                        # index of next item on path_to_goal list
              self.path_to_goal.insert(indx, node_x.state[9])         # insert the node.state[9] to the list... which is previous action that leaded to the current state of the board
              state = node_x.parent                                   # take the parent of the previous parent-checked and determine one more move...

              if len(nodes_frontier)<1:
                 return print("Failure of generating path_to_goal")
              
    def cal_cost_depth_max(self, successfulNode):         # Calculate cost_of_path, where cost_of_step = 1, which means number of steps from initialState to Goal.
         self.search_depth = successfulNode.depth
         self.cost_of_path = self.search_depth

    def manhattan_fn(self,state):
        _st = state[:]
        del _st[0]
        index_table = []
        for i in range(9):
            index_table.insert(i,_st.index(i))

        sum = one_table[index_table[1]]+two_table[index_table[2]]+three_table[index_table[3]]+four_table[index_table[4]] + five_table[index_table[5]] + six_table[index_table[6]]+seven_table[index_table[7]]+ eight_table[index_table[8]]
            
        return sum

####### Tables for manhattan function ########
# I didn't want to do math calculations, I am going to use tables. 
# If zero is on position eight, we will read the value a_0 = eight_table[0].
# If 3 of the state is in the position six, we will read a_3 = six_table[3]
one_table = [1,0,1,2,1,2,3,2,3]
two_table = [2,1,0,3,2,1,4,3,2]
three_table = [1,2,3,0,1,2,1,2,3]
four_table = [2,1,2,1,0,1,2,1,2]
five_table = [3,2,1,2,1,0,3,2,1]
six_table = [2,3,4,1,2,3,0,1,2]
seven_table = [3,2,3,2,1,2,1,0,1]
eight_table = [4,3,2,3,2,1,2,1,0]
##########################################################################
cl1 = driver_3(sys.argv)
method = argv[1]
if (method == 'bfs'):
 cl1.BreadthFirstSearch_BFS([0,1,2,3,4,5,6,7,8])
elif method== 'dfs':
 cl1.DepthFirstSearch_DFS([0,1,2,3,4,5,6,7,8])
elif method == 'ast':
 cl1.A_star([0,1,2,3,4,5,6,7,8])
##########################################################################

#Write Results to output.txt file 
 
file = open('output.txt','w')
file.write('path_to_goal: '+str(cl1.path_to_goal))
file.write('\ncost_of_path: ' + str(cl1.cost_of_path))
file.write('\nnodes_expanded: ' + str(cl1.explored_nodes))
file.write('\nsearch_depth: ' + str(cl1.search_depth))
file.write('\nmax_search_depth: ' + str(cl1.max_search_depth))
getcontext().prec = 8
time = Decimal(resource.getrusage(resource.RUSAGE_SELF).ru_utime)*Decimal(1)
file.write('\nrunning_time: ' + str(time))
getcontext().prec = 7
file.write('\nmax_ram_usage: ' + str(Decimal(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)/Decimal(1048576.0)))
file.close()
##########################################################################