from collections import defaultdict
import numpy as np
from cmath import inf
from queue import PriorityQueue
from matplotlib import pyplot as plt
import math
import cv2

#Function to check valid nodes in 5 directions

def moveFront(step_size,x,y,theta,Map):
    #Initializing new variables based on trignometric calculations
    x_new = np.round((x + ((step_size)*(math.cos(np.radians(theta))))),1)
    y_new = np.round((y + ((step_size)*(math.sin(np.radians(theta))))),1)
    if theta < 0:
        theta = 360 + theta
    if theta > 360:
        theta = 360 - theta
    #Checking whether the point is within the defined map configuration space
    if (0 <= x_new < Map.shape[0]) and (0 <= (y_new) < Map.shape[1]):
        if Map[int(x_new),int(y_new)] == 0:
            return [x_new, y_new, theta]
        else:
            return None
    else:
        return None

def moveSlightRight(step_size,x,y,theta,Map):
    theta = theta-30
    #Initializing new variables based on trignometric calculations
    x_new = np.round((x + ((step_size)*(math.cos(np.radians(theta))))),1)
    y_new = np.round((y + ((step_size)*(math.sin(np.radians(theta))))),1)
    if theta < 0:
        theta = 360+theta
    if theta > 360:
        theta = 360 - theta
    #Checking whether the point is within the defined map configuration space
    if (0 <= x_new < Map.shape[0]) and (0<= (y_new) < Map.shape[1]):
        if Map[int(x_new),int(y_new)] == 0:
            return [x_new, y_new,theta]
        else:
            return None
    else:
        return None

def moveRight(step_size,x,y,theta,Map):
    theta = theta-60
     #Initializing new variables based on trignometric calculations
    x_new = np.round((x + ((step_size)*(math.cos(np.radians(theta))))),1)
    y_new = np.round((y + ((step_size)*(math.sin(np.radians(theta))))),1)
    if theta < 0:
        theta = 360+theta
    if theta > 360:
        theta = 360 - theta
    #Checking whether the point is within the defined map configuration space
    if (0 <= x_new < Map.shape[0]) and (0<= (y_new) < Map.shape[1]):
        if Map[int(x_new),int(y_new)] == 0:
            return [x_new, y_new,theta]
        else:
            return None
    else:
        return None

def moveSlightLeft(step_size,x,y,theta,Map):
    theta = theta+30
    #Initializing new variables based on trignometric calculations
    x_new = np.round((x + ((step_size)*(math.cos(np.radians(theta))))),1)
    y_new = np.round((y + ((step_size)*(math.sin(np.radians(theta))))),1)
    if theta < 0:
        theta = 360+theta
    if theta > 360:
        theta = 360 - theta
    #Checking whether the point is within the defined map configuration space
    if (0 <= x_new < Map.shape[0]) and (0<= (y_new) < Map.shape[1]):
        if Map[int(x_new),int(y_new)] == 0:
            return [x_new, y_new,theta]
        else:
            return None
    else:
        return None

def moveLeft(step_size,x,y,theta,Map):
    theta = theta + 60
    #Initializing new variables based on trignometric calculations
    x_new = np.round((x + ((step_size)*(math.cos(np.radians(theta))))),1)
    y_new = np.round((y + ((step_size)*(math.sin(np.radians(theta))))),1)
    if theta < 0:
        theta = 360+theta
    if theta > 360:
        theta = 360 - theta
    #Checking whether the point is within the defined map configuration space
    if (0 <= x_new < Map.shape[0]) and (0<= (y_new) < Map.shape[1]):
        if Map[int(x_new),int(y_new)] == 0:
            return [x_new, y_new,theta]
        else:
            return None
    else:
        return None

#Function to get new neighbours
def getNewPoints(point,step_size,Map):
    x = point[0]
    y = point[1]
    theta = point[2]
    new_points = []
    #Checking in all directions and if possible action, appending it into the neighbours list
    if moveFront(step_size,x,y,theta,Map) != None:
        new_points.append(moveFront(step_size,x,y,theta,Map))
        
    if moveSlightRight(step_size,x,y,theta,Map) != None:
        new_points.append(moveSlightRight(step_size,x,y,theta,Map))
        
    if moveRight(step_size,x,y,theta,Map) != None:
        new_points.append(moveRight(step_size,x,y,theta,Map))
        
    if moveSlightLeft(step_size,x,y,theta,Map) != None:
        new_points.append(moveSlightLeft(step_size,x,y,theta,Map))
        
    if moveLeft(step_size,x,y,theta,Map) != None:
        new_points.append(moveLeft(step_size,x,y,theta,Map))
        

    return new_points

#Function to return a line equation
def getLine(x1,y1,x2,y2):
    slope = (y2-y1)/(x2-x1)
    y_intercept = (y1 - ((slope)*x1))
    return y-(slope*x) - y_intercept

#Cost to Go
def costToGo(point, goal_point):
    #Computation of distance between two points
    dist = ((point[0]-goal_point[0])**2 + (point[1]-goal_point[1])**2)**0.5

    return dist

#rounding off
def rPos(point):
    return int(np.round(point[0]/0.5,1)), int(np.round(point[1]/0.5,1)),int(np.round(point[2]/30,1))

#Thresholding
def resPos(point):
    return int(np.round(point[0],1)), int(np.round(point[1],1)),int(np.round(point[2]/30,1))

#BackTrack function
def backTrack(dict, final_point, start_point, image):

    path = [(int(final_point[0]),int(final_point[1]),int(final_point[2]))]

    i = (int(final_point[0]), int(final_point[1]),int(final_point[2]))

    dict_keys = list(dict.keys())

    dict_values = list(dict.values())
    
    reached_start_point = False

    while not reached_start_point:
        for children in dict_values:
            if i in children:
                v = i
                image[249-i[1],i[0]] = (255,0,255)
                j = dict_values.index(children)
                path.append(dict_keys[j])
                i = dict_keys[j]
                cv2.line(image,(v[0],249-v[1]),(i[0],249-i[1]),(255,0,255),1)
            if (i[0] == start_point[0]) and (i[1] == start_point[1]) and (i[2] == start_point[2]):
                return path[::-1]

#Function to plot vectors
def plotVector(parent, child, image):

    cv2.line(image,(int(parent[0]),int(249-parent[1]) ), (int(child[0]),int(249-child[1])), (0,255,0), 1)

    return image
    
#A-Star Algorithm
def aStar(start_point,goal_point,map, C2C, C2G,totalCost,image, step_size):
    #Initializing all the variables
    open = PriorityQueue()
    C2C[rPos(start_point)] = 0
    C2G[rPos(start_point)] = costToGo(start_point, goal_point)
    totalCost[rPos(start_point)] = 0 + costToGo(start_point, goal_point)
    open.put((0 + costToGo(start_point, goal_point),(start_point)))
    parent = defaultdict(list)
    visited = np.full((800,500,13),-1)

    #To record the video 
    fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
    out = cv2.VideoWriter("Astar.mp4", fourcc, 240, (400, 250))

    #Loop through 
    while True:
        #popping from open list and appending to closed
        _, current_point = open.get()
        print("current point", current_point)
        
        #After reaching goal
        if goal_point[0] <= current_point[0] <= goal_point[0]+1.5 and goal_point[1] <= current_point[1] <= goal_point[1]+1.5 and current_point[2] == goal_point[2]:
            print("!!Goal Reached!!")
            print("Video started")
            
            dict_keys = list(parent.keys())
            i = (current_point[0], current_point[1],current_point[2])

            #start backtracking
            backTrack(parent, current_point, start_point, image)

            #Append the backtracked image multiple times so that its visible in the video
            for i in range(0,1000):
                out.write(image)
            out.release()
            cap = cv2.VideoCapture('Astar.mp4')
            while (cap.isOpened()):
                ret, frame = cap.read()
                if ret == True:
                    cv2.imshow('exploration',frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            cap.release()

            break
        
        if visited[resPos(current_point)] != 0:

            visited[resPos(current_point)] = 0

        #Loop to explore new neighbouring points of current point
            for point in getNewPoints(current_point,step_size,map):

                newC2C = C2C[rPos(current_point)] + step_size

                if newC2C < C2C[rPos(point)]:

                    C2C[rPos(point)] = newC2C

                newCost = C2C[rPos(point)] + costToGo(point, goal_point)
                
                if newCost < totalCost[rPos(point)]:
                    
                    totalCost[rPos(point)] = newCost
                    
                    parent[(int(current_point[0]), int(current_point[1]),int(current_point[2]))].append((int(point[0]),int(point[1]),int(point[2])))
                    image = plotVector(current_point,point,image)
                    out.write(image)
                    open.put((totalCost[rPos(point)],point))

def IsPointValid(x,y,map):
    if map[x,y] == 0:
        return True
    else:
        return False
    
if __name__ == '__main__':

    map = np.zeros((400,250))
    image = np.zeros((250,400,3), dtype=np.uint8)
    C2C = np.full((800,500,13), inf)
    C2G = np.full((800,500,13), inf)
    totalCost = np.full((800,500,13), inf)

    clearance = int(input("Enter the clearance value: \n"))
    # clearance = 5
    radius = int(input("Enter the robot radius: \n"))
    # radius = 5
    step_size = int(input("Enter the step size: \n"))
    

    #Define obstacle space in map
    for y in range(0,250):
        for x in range(0,400):
            #circle

            if 0 <= x < radius+clearance or 399 - radius-clearance < x < 400 or 0 <= y < radius+clearance or 249-radius-clearance < y < 250:
                map[x,y] = 1
                image[249-y,x] = (0,0,255)
                


            if ((x-300)**2) + ((y-185)**2) - ((40+clearance+radius)**2) < 0:
                map[x,y] = 1
                image[249-y,x] = (0,0,255)
               

            if ((x-300)**2) + ((y-185)**2) - ((40)**2) < 0:
                map[x,y] = 1
                image[249-y,x] = (255,255,0)
        

            # Lines for traingles with clearance 
            l1_c = getLine(36,185,115,210) - clearance - radius
            l2_c = getLine(80,180,115,210) + clearance + radius
            l3_c = getLine(36,185,80,180) 
            l4_c = getLine(36,185,105,100) + clearance + radius
            l5_c = getLine(80,180,105,100) - clearance - radius


            #Defining two traingles from lines
            lower_triangle_c = l4_c >0 and l5_c < 0 and l3_c < 0
            upper_triangle_c = l1_c<0 and l2_c > 0 and l3_c>0

            #Setting the traingles as obstacles
            if  upper_triangle_c :
                map[x,y] = 1
                image[249-y,x] = (0,0,255)
               
            if lower_triangle_c :
                map[x,y] = 1
                image[249-y,x] = (0,0,255)
            

            #lines for two trianlges without clreance
            l1 = getLine(36,185,115,210)
            l2 = getLine(80,180,115,210)
            l3 = getLine(36,185,80,180) 
            l4 = getLine(36,185,105,100)
            l5 = getLine(80,180,105,100)

            #Defining two traingles from lines without clearance
            lower_triangle = l4 > 0 and l5 < 0 and l3 < 0
            upper_triangle = l1 < 0 and l2 > 0 and l3 > 0

            # Setting the traingles to 255
            if  upper_triangle :
                map[x,y] = 1
                image[249-y,x] = (255,255,0)
                
            if lower_triangle :
                map[x,y] = 1
                image[249-y,x] = (255,255,0)
                
            #Defining the lines of hexagon with clearnace
            hexline1_c = getLine(165,120.2,200,140.4) - clearance - radius
            hexline2_c = getLine(200,140.4,235,120.2) - clearance - radius
            hexline5_c = getLine(235,79.8,200,59.6) + clearance + radius
            hexline6_c = getLine(165,79.8,200,59.6) + clearance + radius

            hex_vert_line1_c = x-165 + clearance + radius 
            hex_vert_line2_c = x-235 -clearance - radius

            hexagon_c = hexline1_c <0 and hexline2_c <0 and hexline5_c >0 and hexline6_c >0 and hex_vert_line1_c>0 and hex_vert_line2_c <0

            #Setting the hexagon with clearance as obstacle
            if hexagon_c:
                map[x,y] = 1
                image[249-y,x] = (0,0,255)


            #Defining the triangular lines of hexagon
            hexline1 = getLine(165,120.2,200,140.4) 
            hexline2 = getLine(200,140.4,235,120.2) 
            hexline5 = getLine(235,79.8,200,59.6)
            hexline6 = getLine(165,79.8,200,59.6)

            hex_vert_line1 = x-165
            hex_vert_line2 = x-235

            hexagon = hexline1 <0 and hexline2 <0 and hexline5 >0 and hexline6 >0 and hex_vert_line1>0 and hex_vert_line2 <0


            #Setting the hexagon traingles to 255
            if hexagon:
                map[x,y] = 1
                image[249-y,x] = (255,255,0)  

    start = input("Input Staring Position in format: x,y,theta\n")
    x_start,y_start,theta_s = int(start.split(',')[0]), int(start.split(',')[1]),int(start.split(',')[2])
    goal = input("Enter the Goal Position in format: x,y,theta\n")
    x_goal,y_goal,theta_g = int(goal.split(',')[0]), int(goal.split(',')[1]),int(goal.split(',')[2])


    # #Checking if the entered input is valid
    while not (IsPointValid(x_start,y_start, map) and IsPointValid(x_goal,y_goal,map)) :
        print("Entered values are in obstacle space. Please enter values that are in free space.")
        start = input("Input Staring Position in format: x,y,theta\n")
        x_start,y_start,theta_s = int(start.split(',')[0]), int(start.split(',')[1]),int(start.split(',')[2])
        goal = input("Enter the Goal Position in format: x,y,theta\n")
        x_goal,y_goal,theta_g = int(goal.split(',')[0]), int(goal.split(',')[1]),int(goal.split(',')[2])
        
    start_point = [x_start,y_start,theta_s]
    goal_point = [x_goal,y_goal,theta_g]


    #A star algorithm
    aStar(start_point, goal_point, map, C2C, C2G, totalCost,image,step_size)

    #end
