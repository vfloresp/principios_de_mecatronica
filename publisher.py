#!/usr/bin/env python
# license removed for brevity
import rospy
import math
import time
import serial
from geometry_msgs.msg import Pose2D
from graphical_client.msg import Pose2D_Array

port = "/dev/ttyUSB0"
arduino = serial.Serial(port, 9600)

def init_pose():
    pose = Pose2D()
    pose.x = 0
    pose.y = 0
    pose.theta = 0
    return pose

#Se inicializan todas las posiciones con valor 0
miCoord = init_pose()
coordObj = init_pose()
coordObs0 = init_pose()
coordObs1 = init_pose()
coordObs2 = init_pose()
coordObs3 = init_pose()
coordObs4 = init_pose()
coordObs5 = init_pose()
coordObs6 = init_pose()
coordObs7 = init_pose()
coordObs8 = init_pose()
coordObs9 = init_pose()
coordObs10 = init_pose()




def miPos(data):
    global miCoord 
    miCoord.x = data.x+3000
    miCoord.y = data.y+3000
    theta = data.theta
    miCoord.theta = data.theta
    go_to_goal()

def objetivo(data):
    global coordObj 
    coordObj.x = data.x+3000
    coordObj.y = data.y+3000
    coordObj.theta = data.theta

def obstaculo0(data):
    global coordObs0 
    coordObs0.x = data.x +3000
    coordObs0.y = data.y +3000
    coordObs0.theta = data.theta
    
def obstaculo1(data):
    global coordObs2 
    coordObs1.x = data.x+3000
    coordObs1.y = data.y+3000
    coordObs1.theta = data.theta

def obstaculo2(data):
    global coordObs2 
    coordObs2.x = data.x+3000
    coordObs2.y = data.y+3000
    coordObs2.theta = data.theta

def obstaculo3(data):
    global coordObs3 
    coordObs3.x = data.x+3000
    coordObs3.y = data.y+3000
    coordObs3.theta = data.theta

def obstaculo4(data):
    global coordObs4 
    coordObs4.x = data.x+3000
    coordObs4.y = data.y+3000
    coordObs4.theta = data.theta

def obstaculo5(data):
    global coordObs5 
    coordObs5.x = data.x+3000
    coordObs5.y = data.y+3000
    coordObs5.theta = data.theta

def obstaculo6(data):
    global coordObs6 
    coordObs6.x = data.x+3000
    coordObs6.y = data.y+3000
    coordObs6.theta = data.theta

def obstaculo7(data):
    global coordObs7 
    coordObs7.x = data.x+3000
    coordObs7.y = data.y+3000
    coordObs7.theta = data.theta

def obstaculo8(data):
    global coordObs8 
    coordObs8.x = data.x+3000
    coordObs8.y = data.y+3000
    coordObs8.theta = data.theta

def obstaculo9(data):
    global coordObs9
    coordObs9.x = data.x+3000
    coordObs9.y = data.y+3000
    coordObs9.theta = data.theta

def obstaculo10(data):
    global coordObs10 
    coordObs10.x = data.x+3000
    coordObs10.y = data.y+3000
    coordObs10.theta = data.theta

def go_to_goal():
    global miCoord
    global coordObj
    global coordObs0 
    global coordObs1
    global coordObs2
    global coordObs3
    global coordObs4
    global coordObs5
    global coordObs6
    global coordObs7
    global coordObs8 
    global coordObs9
    global coordObs10
    arr = Pose2D_Array()
    arr.poses.append(coordObs0)
    arr.poses.append(coordObs1)
    arr.poses.append(coordObs2)
    arr.poses.append(coordObs3)
    arr.poses.append(coordObs4)
    arr.poses.append(coordObs5)
    arr.poses.append(coordObs6)
    arr.poses.append(coordObs7)
    arr.poses.append(coordObs8)
    arr.poses.append(coordObs9)
    arr.poses.append(coordObs10)

    hayObstaculos = 0
    for i in range(10):
        if(arr.poses[i].x != 0 or arr.poses[i].y != 0):
            hayObstaculos +=1

    xIni = miCoord.x
    yIni = miCoord.y
    thetaIni = miCoord.theta
    xFin = coordObj.x
    yFin = coordObj.y

    if hayObstaculos == 0:
        kv = 0.15
        distance = abs(math.sqrt(((xFin-xIni)**2)+((yFin-yIni)**2)))
        linear_speed = kv * distance

        ka = 0.3    
        thetaFin = math.atan2(yFin-yIni, xFin-xIni)
        if math.degrees(thetaFin)<90:
            thetaConv = 90 - math.degrees(thetaFin)
        else:
            thetaConv = 90 +(-1*math.degrees(thetaFin))

        angular_speed = ka * (math.radians(thetaConv)-thetaIni)

        velIzq = (linear_speed - ((115*angular_speed)/2))/21
        velDer = (linear_speed + ((115*angular_speed)/2))/21
    
        cadVel = str(velIzq)+'_'+str(velDer)
        print(cadVel)
        arduino.write(cadVel)
        time.sleep(1)
    else:
        distancia = math.sqrt(((xFin-xIni)**2)+((yFin-yIni)**2)) 
        deltaX = distancia/8
        m = (yFin - yIni)/(xFin-xIni)
        arrP = Pose2D_Array()

        punto = init_pose()
        punto.x =  xIni + deltaX
        punto.y = yIni + m*((xIni + deltaX)-xIni)
        arrP.poses.append(punto)

        punto = init_pose()
        punto.x =  xIni + 2*deltaX
        punto.y = yIni + m*((xIni + 2*deltaX)-xIni)
        arrP.poses.append(punto)

        punto = init_pose()
        punto.x =  xIni + 3*deltaX
        punto.y = yIni + m*((xIni + 3*deltaX)-xIni)
        arrP.poses.append(punto)

        punto = init_pose()
        punto.x =  xIni + 4*deltaX
        punto.y = yIni + m*((xIni + 4*deltaX)-xIni)
        arrP.poses.append(punto)

        punto = init_pose()
        punto.x =  xIni + 5*deltaX
        punto.y = yIni + m*((xIni + 5*deltaX)-xIni)
        arrP.poses.append(punto)

        punto = init_pose()
        punto.x =  xIni + 6*deltaX
        punto.y = yIni + m*((xIni + 6*deltaX)-xIni)
        arrP.poses.append(punto)

        punto = init_pose()
        punto.x =  xIni + 7*deltaX
        punto.y = yIni + m*((xIni + 7*deltaX)-xIni)
        arrP.poses.append(punto)


        for i in range(7):
            comp1 = arrP.poses[i]
            for j in range(10):
                comp2 = arr.poses[j]
                if(math.sqrt(((comp2.x-comp1.x)**2)+((comp2.y-comp1.y)**2))<200):
                    m2 = 1/m
                    y1T =  comp2.y + m2*((comp2.x + 100)-comp2.x)
                    y2T =  comp2.y + m2*((comp2.x - 100)-comp2.x)
                    D1 = math.sqrt(((xFin-(comp2.x + 100))**2)+((yFin-y1T )**2))
                    D2 = math.sqrt(((xFin-(comp2.x - 100))**2)+((yFin-y2T )**2))
                    if(D1>D2):
                        xFin = (comp2.x + 100)
                    else:
                        xFin = (comp2.x - 100)
                    yFin = min(D1,D2)
                    i = i+7
                    j = j+10

        kv = 0.15
        distance = abs(math.sqrt(((xFin-xIni)**2)+((yFin-yIni)**2)))
        linear_speed = kv * distance

        ka = 0.3    
        thetaFin = math.atan2(yFin-yIni, xFin-xIni)
        if math.degrees(thetaFin)<90 and  math.degrees(thetaFin)>0:
            thetaConv = 90 - math.degrees(thetaFin)
        elif math.degrees(thetaFin)<0:
            thetaConv = 90 +(-1*math.degrees(thetaFin))
        elif math.degrees(thetaFin)<-90:
            thetaConv = 90 +(-1*math.degrees(thetaFin))

        angular_speed = ka * (math.radians(thetaConv)-thetaIni)

        velIzq = (linear_speed - ((115*angular_speed)/2))/21
        velDer = (linear_speed + ((115*angular_speed)/2))/21
    
        cadVel = str(velIzq)+'_'+str(velDer)
        print(cadVel)
        arduino.write(cadVel)
        time.sleep(1)



def talker():
    rospy.init_node('talker', anonymous=True)

    pub = rospy.Publisher('/trajectory', Pose2D_Array, queue_size=10)
    #rospy.init_node('talker', anonymous=True)

    rospy.Subscriber("/y_r0", Pose2D, miPos)
    rospy.Subscriber("/ball", Pose2D, objetivo)
    rospy.Subscriber("/b_r0", Pose2D, obstaculo0)
    rospy.Subscriber("/b_r1", Pose2D, obstaculo1)
    rospy.Subscriber("/b_r2", Pose2D, obstaculo2)
    rospy.Subscriber("/b_r3", Pose2D, obstaculo3)
    rospy.Subscriber("/b_r4", Pose2D, obstaculo4)
    rospy.Subscriber("/b_r5", Pose2D, obstaculo5)
    rospy.Subscriber("/b_r6", Pose2D, obstaculo6)
    rospy.Subscriber("/b_r7", Pose2D, obstaculo7)
    rospy.Subscriber("/b_r8", Pose2D, obstaculo8)
    rospy.Subscriber("/b_r9", Pose2D, obstaculo9)
    rospy.Subscriber("/b_r10", Pose2D, obstaculo10)


    rate = rospy.Rate(1) # 10hz


    aux = 1
    while not rospy.is_shutdown():
        arr = Pose2D_Array()
        for i in range(10):
            pose = init_pose()
            pose.x = 100 * ( i + 1 ) 
            pose.y = 150 * ( i + 1 ) * aux
            pose.theta +=0.7853 * i
            arr.poses.append(pose)
            aux *= -1
        #print ("The array is:", arr)
        pub.publish(arr)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass