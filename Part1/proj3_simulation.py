## Project 3 - Phase 2 Implementing A* algorithm on differential drive T

import time
from cv2 import waitKey
import numpy as np
import heapq as hq
import cv2 as cv
import math

start_time = time.time()

c2c_node =  np.full((250,400),np.inf)

# Initializes a numpy matrix full of np.inf values to represent the workspace
c2c_node =  np.full((250,400),np.inf)

# Initializes image file full of pixels with black color (0,0,0) RGB scale
image = np.zeros((250,400,3),np.uint8)

# Creates window showing obstacle
flipped_image=cv.flip(image,0)
cv.imshow("actual_image",flipped_image)
cv.waitKey(10)

# Robot setting loop

robot_set = False

while robot_set == False:
    print("\nPlease enter robot clearance and 2 RPM settings for wheels\n")
    print("-------------------------------------------------------\n")
    robot_clearance = float(input("Enter desired clearance, must be an integer\n"))
    if (robot_clearance < 0):
        print("\nRobot clearance must be positive value!\n")
        time.sleep(2)
        continue
    wheelspeed1 = float(input("Enter first wheel RPM setting\n"))
    wheelspeed2 = float(input("Enter second wheel RPM setting\n"))
    if (wheelspeed1 < 0 or wheelspeed2 < 0):
        print("\nRPM setting must be positive value!\n")
        time.sleep(2)
    else:
        robot_set = True

# Set by the robot's datasheet in millimeters
robot_r = float(.138) #robot physical radius
robot_t = float(.178) #robot trackwidth
robot_wheelradius = float(.033)
clearance = robot_r+robot_clearance

# Creating objects using half planes and semi-algebraic definitions, all object
# spaces have their cost set to -1
for x in range(0,400,1):
    for y in range(0,250,1):
        if x < (clearance) or x > (400-clearance) or y < clearance or y > 250-clearance:
            c2c_node[y][x] = -1
            image[y,x]= [255,0,0]
        elif y >= (100-clearance) and y <= 180 and x>math.ceil((105-clearance)-((37/45)*(y-100))) and x < math.floor((105+clearance)-(.3125*(y-100))):
            c2c_node[y][x] = -1
            image[y,x]= [255,0,0]
        elif y >= 180 and y <= 185 and x>math.floor((105-clearance)-((37/45)*(y-100))) and x < math.ceil((80+clearance)+((1)*(y-180))):
            c2c_node[y][x] = -1
            image[y,x]= [255,0,0]
        elif y >= 185 and y <= 215 and x>math.ceil((36-clearance)+(2.9666*(y-185))) and x < math.ceil((80+clearance)+((1)*(y-180))):
            c2c_node[y][x] = -1
            image[y,x]= [255,0,0]
        elif y >= (64-clearance) and y <= (82) and x > math.floor((200-clearance)-(1.94*(y-64))) and x < math.ceil((200+clearance)+(1.94*(y-(64)))):
            c2c_node[y][x] = -1
            image[y,x]= [255,0,0]
        elif y >= 82 and y <= 117 and x > math.floor(165-clearance) and x < math.ceil(235+clearance):
            c2c_node[y][x] = -1
            image[y,x]= [255,0,0]
        elif y >= 117 and y <= 136+clearance and x > math.ceil(165-clearance +(1.732*(y-117))) and x < math.floor(235+clearance -(1.732*(y-117))):
            c2c_node[y][x] = -1
            image[y,x]= [255,0,0]
        elif math.floor(((x-300)**2)+((y-185)**2)) < ((40+clearance)**2): 
            c2c_node[y][x] = -1
            image[y,x]= [255,0,0]

# Creates variables for setting goal/start locations
goal_set = False
start_set = False
start_angle_set = False
global NODEINDEX
NODEINDEX = 1

# Goal setting loop
while goal_set == False:
    print("\nPlease enter a goal location on the workspace\n")
    print("-------------------------------------------------------\n")
    goal_x = float(input("Enter X position of goal node, must be an integer between 0-400\n"))
    goal_y = float(input("Enter Y position of goal node, must be an integer between 0-250\n"))
    if goal_x < 0 or goal_x > 400 or goal_y > 250 or goal_y < 0: # Checks if goal is outside of workspace
        print("\nGoal outside of workspace! Please try again\n")
        time.sleep(2)
        continue
    elif c2c_node[int(goal_y)][int(goal_x)] == np.inf:
        goal_set = True
    else: # If goal is not equal to np.inf it's an obstacle, prompts user again for new goal
        print("\nGoal location is ontop of an obstacle! Please enter new goal location\n")
        time.sleep(2)
        
# Start setting loop

while start_set == False:
    print("\nPlease enter a starting location on the workspace\n")
    print("-------------------------------------------------------\n")
    start_x = float(input("Enter X position of start node, must be an integer between 0-400\n"))
    start_y = float(input("Enter Y position of start node, must be an integer between 0-250\n"))
    if start_x < 0 or start_x > 400 or start_y > 250 or start_y < 0: # Checks if start is outside workspace
        print("\nStarting location is outside of workspace! Please try again\n")
        time.sleep(2)
        continue
    elif c2c_node[int(start_y)][int(start_x)] == np.inf:
        start_set = True
    else: # If start is not equal to np.inf it's an obstacle, prompts user again for new start
        print("\nStarting location is ontop of an obstacle! Please enter new starting location\n")
        time.sleep(2)
while start_angle_set == False:
    start_theta = int(input("Enter orientation of robot at start node, must be in degrees between 0-360, in steps of 30 degrees\n"))
    if (start_theta > 360) or (start_theta < 0):
        print("\n Start angle out of range, please normalize within 0-360 degrees \n")
        time.sleep(2)
        continue
    elif (start_theta % 30 != 0):
        print("\n Start angle must be in steps of 30 degrees! Please try again! \n")
        time.sleep(2)
        continue
    else:
        start_angle_set = True

global goal_node
goal_node = (goal_y,goal_x)
start_node = (start_y,start_x,start_theta)

print(goal_node)
print(start_node)

step_size_set = False
global STEPSIZE 

# Defining cost calculation function
def Cost_Calc(y,x):
    dist_y = (goal_node[0]-y)
    dist_x = (goal_node[1]-x)
    dist = ((dist_y**2) + (dist_x**2))**.5
    return dist

# Finds a node_index within a list
def Find_Node(list,node_index):
    for i in range(len(list)):
        if list[i][1]==node_index:
            return list[i]

def Backtrack(current_node,ClosedList):
    goal = current_node
    print("STARTING GOAL POSITION:", goal)
    reverse = []
    waitKey(10)
    reverse.append(goal)
    waitKey(10)
    while reverse[-1][2] > 0:
        parent = Find_Node(ClosedList,reverse[-1][2])
        print(parent)
        reverse.append(parent)
    route = []
    while reverse:
        UpdateGoal(reverse[-1])
        UpdateImage()
        route.append(reverse.pop())
    return route

# List to store all between movements


# Generic Move function for calculating change in displacement, heading, and cost on R/L wheel inputs
def Move(robot_i,rpm_l,rpm_r):
    global NODEINDEX
    movement_cost = 0
    t = 0
    dt = .1
    i = 0
    backtrack_adder = []
    yn = robot_i[0]
    xn = robot_i[1]
    theta_n = 3.14*(robot_i[2]/180)
    hit_obstacle = False
    while t < 1:
        i += 1
        t += dt
        dx = (robot_wheelradius*.5)*(rpm_l+rpm_r)* math.cos(theta_n) * dt
        dy = (robot_wheelradius*.5)*(rpm_l+rpm_r)* math.sin(theta_n) * dt
        d_theta = (robot_wheelradius/robot_t)*(rpm_r-rpm_l) * dt
        theta_n += d_theta
        if (c2c_node[int(yn+dy),int(xn+dx)] == -1): #If hits an obstacle mid motion, will cancel movement and scrub from record
            hit_obstacle = True
            continue
        xn += dx
        yn += dy
        backtrack_adder.append([yn,xn])
        UpdateSearched(yn,xn)
        movement_cost=movement_cost+ math.sqrt(math.pow((0.5*robot_r * (rpm_l + rpm_r) * math.cos(theta_n) * 
        dt),2)+math.pow((0.5*robot_r * (rpm_l + rpm_r) * math.sin(theta_n) * dt),2))
    theta_n = 180 * (theta_n) / 3.14
    robot_n = [yn,xn,theta_n]
    heuristic_cost = Cost_Calc(yn,xn)
    return robot_n,heuristic_cost, movement_cost, hit_obstacle, backtrack_adder

## Action sets each function calls the actions, which results in the generic move function being called for each type of movement
def Move01(node):
    global NODEINDEX
    new_node = [0,0,0,0]
    NODEINDEX = NODEINDEX + 1
    robot = node[3]
    new_robot_stack = Move(robot,0,wheelspeed1) # returns Euclidean cost & robot position/rotation
    new_robot = new_robot_stack[0] # Robot position/rotation
    new_c2c = node[4] + new_robot_stack[2]
    heuristic_cost = new_robot_stack[1]
    new_node = [heuristic_cost,NODEINDEX,node[1],new_robot,new_c2c,new_robot_stack[4]]
    return new_node,new_robot_stack[3]

def Move10(node):
    global NODEINDEX
    new_node = [0,0,0,0]
    NODEINDEX = NODEINDEX + 1
    robot = node[3]
    new_robot_stack = Move(robot,wheelspeed1,0)
    new_robot = new_robot_stack[0] # Robot position/rotation
    new_c2c = node[4] + new_robot_stack[2]
    heuristic_cost = new_robot_stack[1]
    new_node = [heuristic_cost,NODEINDEX,node[1],new_robot,new_c2c,new_robot_stack[4]]
    return new_node,new_robot_stack[3]

def Move11(node):
    global NODEINDEX
    new_node = [0,0,0,0]
    NODEINDEX = NODEINDEX + 1
    robot = node[3]
    new_robot_stack = Move(robot,wheelspeed1,wheelspeed1)
    new_robot = new_robot_stack[0] # Robot position/rotation
    new_c2c = node[4] + new_robot_stack[2]
    heuristic_cost = new_robot_stack[1]
    new_node = [heuristic_cost,NODEINDEX,node[1],new_robot,new_c2c,new_robot_stack[4]]
    return new_node,new_robot_stack[3]

def Move02(node):
    global NODEINDEX
    new_node = [0,0,0,0]
    NODEINDEX = NODEINDEX + 1
    robot = node[3]
    new_robot_stack = Move(robot,0,wheelspeed2)
    new_robot = new_robot_stack[0] # Robot position/rotation
    new_c2c = node[4] + new_robot_stack[2]
    heuristic_cost = new_robot_stack[1]
    new_node = [heuristic_cost,NODEINDEX,node[1],new_robot,new_c2c,new_robot_stack[4]]
    return new_node,new_robot_stack[3]

def Move20(node):
    global NODEINDEX
    new_node = [0,0,0,0]
    NODEINDEX = NODEINDEX + 1
    robot = node[3]
    new_robot_stack = Move(robot,wheelspeed2,0)
    new_robot = new_robot_stack[0] # Robot position/rotation
    new_c2c = node[4] + new_robot_stack[2]
    heuristic_cost = new_robot_stack[1]
    new_node = [heuristic_cost,NODEINDEX,node[1],new_robot,new_c2c,new_robot_stack[4]]
    return new_node,new_robot_stack[3]

def Move22(node):
    global NODEINDEX
    new_node = [0,0,0,0]
    NODEINDEX = NODEINDEX + 1
    robot = node[3]
    new_robot_stack = Move(robot,wheelspeed2,wheelspeed2)
    new_robot = new_robot_stack[0] # Robot position/rotation
    new_c2c = node[4] + new_robot_stack[2]
    heuristic_cost = new_robot_stack[1]
    new_node = [heuristic_cost,NODEINDEX,node[1],new_robot,new_c2c,new_robot_stack[4]]
    return new_node,new_robot_stack[3]

def Move12(node):
    global NODEINDEX
    new_node = [0,0,0,0]
    NODEINDEX = NODEINDEX + 1
    robot = node[3]
    new_robot_stack = Move(robot,wheelspeed1,wheelspeed2)
    new_robot = new_robot_stack[0] # Robot position/rotation
    new_c2c = node[4] + new_robot_stack[2]
    heuristic_cost = new_robot_stack[1]
    new_node = [heuristic_cost,NODEINDEX,node[1],new_robot,new_c2c,new_robot_stack[4]]
    return new_node,new_robot_stack[3]

def Move21(node):
    global NODEINDEX
    new_node = [0,0,0,0]
    NODEINDEX = NODEINDEX + 1
    robot = node[3]
    new_robot_stack = Move(robot,wheelspeed2,wheelspeed1)
    new_robot = new_robot_stack[0] # Robot position/rotation
    new_c2c = node[4] + new_robot_stack[2]
    heuristic_cost = new_robot_stack[1]
    new_node = [heuristic_cost,NODEINDEX,node[1],new_robot,new_c2c,new_robot_stack[4]]
    return new_node,new_robot_stack[3]

# Creates window showing obstacle
flipped_image=cv.flip(image,0)
cv.imshow("actual_image",flipped_image)
cv.waitKey(10)

# Updates pixels to white in image for searched nodes 
def UpdateSearched(y,x):
    flipped_image[250-int(y)][int(x)]= [0,0,255]

# Updates pixels to white in image for optimal path 
def UpdateGoal(node):
    flipped_image[250-int(node[3][0])][int(node[3][1])]= [255,255,255]
    int_solution = node[5]
    print(int_solution)
    waitKey(0)
    if node[1] == 1:
        flipped_image[250-int(int_solution[0])][int(int_solution[1])]= [255,255,255]
    else:
        for i in range(0,10):
            flipped_image[250-int(int_solution[int(i)][0])][int(int_solution[int(i)][1])]= [255,255,255]
    
# Calls cv.imshow to update image, scales image up 4X
def UpdateImage():
        resized = cv.resize(flipped_image,(1600,1000))
        cv.imshow("image",resized)
        cv.waitKey(1)
        
# Checks if an input node is inside of a list
def Check_List(new_node,list):
    for i in range(len(list)):
        if (np.absolute(list[i][3][0] - new_node[3][0]) <.1) and (np.absolute(list[i][3][1] - new_node[3][1]) <.1):
            return True

def Check_Goal(node):
    if (((node[3][0]-goal_node[0])**2+(node[3][1]-goal_node[1])**2)**.5 <= 1.5):
        return True
        
# Checks if node is not in obstacle space, has been searched, if lower cost found
# updates the cost respectively
def Check_Node(new_node,ClosedList,OpenList,c2c_node):
    newnode_y = int(new_node[3][0])
    newnode_x = int(new_node[3][1])
    if c2c_node[newnode_y][newnode_x] != -1 and Check_List(new_node,ClosedList) == None:
        ClosedList.append(new_node)
        if (Check_List(new_node,OpenList) == False or Check_List(new_node,OpenList) == None) and c2c_node[newnode_y][newnode_x]<=np.inf:
            c2c_node[newnode_y][newnode_x] = new_node[4]
            hq.heappush(OpenList,new_node)
    elif Check_List(new_node,ClosedList) == True:
        if (c2c_node[newnode_y][newnode_x] >= new_node[4]):
            c2c_node[newnode_y][newnode_x] = new_node[4]
            OpenList.append(new_node)
        
    
# Algorithm Loop

goal_found = False

def a_star_algo(start_node,goal_node,c2c_node):
    global NODEINDEX
    goal_found = False
    #Open/Closed List creation
    OpenList = []
    ClosedList = []
    ClosedList.append(((Cost_Calc(start_node[0],start_node[1])),NODEINDEX,0,start_node,0,[start_node[0],start_node[1]]))
    hq.heappush(OpenList,((Cost_Calc(start_node[0],start_node[1])),NODEINDEX,0,start_node,0,[start_node[0],start_node[1]]))  #Pushes starting node into OpenList
    iterator = 0
    cv.waitKey(0)
    while OpenList and goal_found == False:
        current_node = hq.heappop(OpenList)
        if ((current_node[3][0]-goal_node[0])**2+(current_node[3][1]-goal_node[1])**2)**.5 <= 1.5:
            goal_found = True
            print("Goal Found!")
            print(current_node)
            goal_route = Backtrack(current_node,ClosedList)
            waitKey(0)
            return goal_route
        else:
            iterator = iterator + 1
            
            output01 = Move01(current_node)
            new_node01 = output01[0]
            if output01[1] == False:
                Check_Node(new_node01,ClosedList,OpenList,c2c_node)
                if Check_Goal(new_node01) == True: continue
            
            output10 = Move10(current_node)
            new_node10 = output10[0]
            if output10[1] == False:
                Check_Node(new_node10,ClosedList,OpenList,c2c_node)
                if Check_Goal(new_node10) == True: continue
                                    
            output11 = Move11(current_node)
            new_node11 = output11[0]
            if output11[1] == False:
                Check_Node(new_node11,ClosedList,OpenList,c2c_node)
                if Check_Goal(new_node11) == True: continue
                                
            output02 = Move02(current_node)
            new_node02 = output02[0]
            if output02[1] == False:
                Check_Node(new_node02,ClosedList,OpenList,c2c_node)
                if Check_Goal(new_node02) == True: continue
                                
            output20 = Move20(current_node)
            new_node20 = output20[0]
            if output20[1] == False:
                Check_Node(new_node20,ClosedList,OpenList,c2c_node)
                if Check_Goal(new_node20) == True: continue
            
            output22 = Move22(current_node)
            new_node22 = output22[0]
            if output22[1] == False:
                Check_Node(new_node22,ClosedList,OpenList,c2c_node)
                if Check_Goal(new_node22) == True: continue
            
            output12 = Move12(current_node)
            new_node12 = output12[0]
            if output12[1] == False:
                Check_Node(new_node12,ClosedList,OpenList,c2c_node)
                if Check_Goal(new_node12) == True: continue
            
            output21 = Move21(current_node)
            new_node21 = output21[0]
            if output21[1] == False:
                Check_Node(new_node21,ClosedList,OpenList,c2c_node)
                if Check_Goal(new_node21) == True: continue
            
            if iterator % 1 == 0: # Only updates image every 100 nodes for speed
                UpdateImage()

    if goal_found == False:
        print("No Goal Found!")

a_star_algo(start_node, goal_node, c2c_node)
