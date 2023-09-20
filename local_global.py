#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import numpy as np
from geometry_msgs.msg import Pose
import math
import matplotlib.pyplot as plt
waypoints=[(0,1),(0,2),(0,3),(0,4),(0,5),(0,6),(0,7),(0,8),(0,9),(0,10),(0,11),(0,12)]

x=0
y=0
threshold_value=0.2
curr_index=0
object_x=0
object_y=0
threshold_x=0
threshold_y=0
o_x=0
o_y=0
scale_value=0.3
path=[]
x_trajectory=[]
y_trajectory=[]
x_trajectory1=[]
y_trajectory1=[]
x_trajectory2=[]
y_trajectory2=[]
line_b=0
line_c=0
line_a=0
slope=0
dist=0
theta=0
x_axis=0
y_axis=0
scale_y=1
mirror_x=0
mirror_y=0
new_o_x=0
new_o_y=0
c1=0
scale_x=0
start_x=0
start_y=0
def callback(data):
    global x,y
    # print("changed!!!!!!!!!!!!!!!!!!!!")
    x=data.position.x
    y=data.position.y
def callback2(data):
    global object_x,object_y
    
    object_x=data.position.x
    object_y=data.position.y
def callback3(data):
    global waypoints,threshold_x,threshold_y,x_trajectory,y_trajectory,object_x,object_y,threshold_x,threshold_y,path,slope,line_b,theta,x_axis,y_axis,dist,o_x,o_y,x,y
    
    threshold_x=data.position.x
    threshold_y=data.position.y
    find_origin()
    find_slope()
    # if(theta<0):
    #     theta=3.14159+theta
    x_axis=theta
    y_axis=1.5708+theta
    if(theta>1.5708):
        y_axis=y_axis-3.14159
    # print(line_a)
    # print(line_b)
    # print(line_c)

    # find_distance()
    
    print("theta==",np.degrees(theta))
    # print("x_axis==",x_axis)
    # print("y_axis==",y_axis)
    

    find_waypoints()
    find_mirror()
    find_mirror_points()
    new_waypoints()
    
    # find_new_origin()
    # find_new_waypoint()
    plt.plot(x_trajectory,y_trajectory,'g',label='final')
    # plt.plot(x_trajectory1,y_trajectory1,'r',label='initial')
    # plt.plot(x_trajectory2,y_trajectory2,'b',label='after rotation')
    plt.plot(object_x,object_y,marker='o',markerfacecolor='green')
    plt.plot(threshold_x,threshold_y,marker='v',markerfacecolor='red')
    plt.plot(o_x,o_y,marker="D")
    plt.plot(x,y,marker='.')
    print(path)
    plt.show()
    
def new_waypoints():
    global waypoints,path,curr_index
    new_path=waypoints[0:curr_index+1]
    for i in range(len(path)):
        new_path.append(path[i])
        
    waypoints=new_path
def find_distance():
    global line_a,line_b,line_c,start_x,start_y,dist
    
    # dist= (abs((line_a*x)+(line_b*y)+line_c))/(np.sqrt(line_a**line_a+line_b**line_b))
    dist=compute_distances()/2
def find_origin():
    global o_x,o_y,object_x,object_y,threshold_x,threshold_y,start_x,start_y
    o_x= (threshold_x+start_x)/2
    o_y= (start_y+threshold_y)/2
    # o_x=threshold_xs
    # o_y=threshold_y
    print(o_x)
    print(o_y)
    
# def find_new_origin():
#     global mirror_x,mirror_y,new_o_x,new_o_y,threshold_x,threshold_y
#     new_o_x=(threshold_x+mirror_x)/2
#     new_o_y=(threshold_y+mirror_y)/2
def find_slope():
    global threshold_x,threshold_y,object_x,object_y,slope,line_a,line_b,line_c,o_x,o_y,theta
    
    y1=object_y-threshold_y
    x1=object_x-threshold_x
    
    if(x1==0):
        line_b=0
        line_a=0
        line_c=o_x
        
        theta=1.5708
        
    # elif(y1==0):
    #     line_a=0
    #     line_b=-1
    #     line_c=o_y
    else:
        slope=y1/x1
        theta=math.atan(slope)
        line_a=slope
        line_b=-1
        line_c=o_y-(slope*o_x)
        
def find_mirror():
    global line_a,line_b,line_c,object_x,object_y,threshold_x,threshold_y
    
    # line_a=vehicle_y_right-vehicle_y_left
    line_a=threshold_y-object_y
    # line_b=vehicle_x_left-vehicle_x_right
    line_b=object_x-threshold_x
    # line_c=(line_a*vehicle_x_left)+(line_b*vehicle_y_left)
    line_c=-1*((line_a*object_x)+(line_b*object_y))
    print(line_a,line_b,line_c)
def find_mirror_points():
    global mirror_x,mirror_y,x,y,x_trajectory,y_trajectory,path
    
    rev_path=path[::-1]
    
    for i in range(len(rev_path)):
        
    
    
        mirror_x=((-2*((line_a*rev_path[i][0])+(line_b*rev_path[i][1])+line_c)*line_a)/((line_a**2)+(line_b**2))) +rev_path[i][0]
        mirror_y=((-2*((line_a*rev_path[i][0])+(line_b*rev_path[i][1])+line_c)*line_b)/((line_a**2)+(line_b**2))) +rev_path[i][1]
        path.append((mirror_x,mirror_y))      
        x_trajectory.append(mirror_x)
        y_trajectory.append(mirror_y)  
    print(mirror_x,mirror_y)    
def compute_distances(object_x,object_y,start_x,start_y):
    
    
    x1=object_x-start_x
    y1=object_y-start_y
    
    distance=np.sqrt(x1**2+y1**2)
    # print(dist)
    return distance;
def compute():
    global line_a,line_b,line_c,start_x,start_y,theta,o_y
    
    if(theta==1.5708):
        distance=abs(o_x)
    else:
        
        c=np.sqrt((line_a**2)+(line_b**2))
        distance= (abs((line_a*start_x)+(line_b*start_y)+line_c))/c
        
    # print("c=" ,c)
    print("line_a=",line_a)
    print("line_b=",line_b)
    print("line_c=",line_c)
    return distance
def find_waypoints():
    global scale_value,x_trajectory,y_trajectory,slope,theta,y_axis,x_trajectory1,x_trajectory2,y_trajectory1,y_trajectory2,scale_y,c1,scale_x,path
    # y_axis=math.radians(y_axis)
    # diff=compute()
    diff=compute()
    
    print("dist==", diff)
    scale_x=compute_distances(object_x,object_y,threshold_x,threshold_y)/2
    scale_x=scale_x
    
    pi=math.pi
    multiplier=(2*scale_x)/pi
    # diff=diff/0.5
    c1=diff
    diff=-diff
    
    diff=diff+0.3
    # theta=theta/1.4
    while diff<c1:
        # print("diff & multiplier",diff,multiplier)
        x=math.atan(diff/scale_y)*multiplier
        y=diff
        # print(x,y)
        # print("!!!!!!!!!!!!!!!!!!!!!")
        x_trajectory1.append(x)
        y_trajectory1.append(y)
        
        x_dash=(x*math.cos(theta))-(y*math.sin(theta))
        y_dash=(x*math.sin(theta))+(y*math.cos(theta))
        # print(x_dash,y_dash)
        x_trajectory2.append(x_dash)
        y_trajectory2.append(y_dash)
        x_dash=x_dash+o_x
        y_dash=y_dash+o_y
        x_trajectory.append(x_dash)
        y_trajectory.append(y_dash)
        path.append((x_dash,y_dash))
        diff=diff+0.1

def compute_distance(point,waypoint):
    global x,y,curr_index
    point=(x,y)
    # print(waypoint)
    x1=point[0]-waypoint[0]
    y1=point[1]-waypoint[1]
    # print(curr_index)
    dist=np.sqrt(x1**2+y1**2)
    # print(dist)
    return dist;
def find_nearestpoint():
    global waypoints,x,y
    shortest_distance = None
    shortest_distance_coordinates = None

    point = (x,y)
    i=0
    for waypoint in waypoints:
        # print(point[0])
        # print(waypoint[0])
        i=i+1
        distance = compute_distance(point, waypoint)
        if distance < shortest_distance or shortest_distance is None:
            shortest_distance = distance
            index=i
            shortest_distance_coordinates = waypoint
    print(index)
    return index
    print(shortest_distance_coordinates)
# def dynamic_distance(point):
#     x1=x-point[0]
#     y1=y-point[1]
    
if __name__ == '__main__':
    
    try:
        
        pub = rospy.Publisher('temp', Pose, queue_size=10)
        next_waypoint=Pose()
        initial=rospy.get_param("initial", 0)
        initial=None
        print(initial)
        final=rospy.get_param("~final",10)
        rospy.init_node('waypoint_generator', anonymous=False)
        rospy.Subscriber('temp1',Pose,callback)
        rospy.Subscriber("temp2",Pose,callback2)
        rospy.Subscriber("temp3",Pose,callback3)
        rate = rospy.Rate(10)

        if(initial==None):
            initial=find_nearestpoint()
            
        for i in range(initial,final):       
            curr_index=i 

            next_waypoint.position.x=waypoints[i][0]
            next_waypoint.position.y=waypoints[i][1]
            waypoint=(waypoints[i][0],waypoints[i][1])
            start_x=waypoints[i][0]
            start_y=waypoints[i][1]
            point=(x,y)
            print(point)
            print(waypoints)
            pub.publish(next_waypoint)
            print("publishing : ", next_waypoint)
            while(compute_distance(point,waypoint)>threshold_value):
                a=1  
                

                
        # while not rospy.is_shutdown():
        rospy.spin()
            
                
        
    except rospy.ROSInterruptException:
        pass