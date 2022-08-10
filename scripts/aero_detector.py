#!/usr/bin/env python
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, SetBoolResponse

from sklearn.linear_model import LinearRegression
import matplotlib.pyplot as plt

class pose_drone():
    def __init__(self):
        self.x=0.0
        self.y=0.0
        self.z=0.0

class results():
    def __init__(self):
        self.angle=0.0
        self.center=0.0
        self.correlation=0.0
        self.offset_fuste = 3.0

class aero_detector():
    def __init__(self):
        # drone pose
        self.pose_drone = pose_drone() 
        # Processed data
        self.vector = [] 
        self.output = results()
        self.init_model_flag = False
        # ROS infraestructure
        self.pose_sub = rospy.Subscriber("/mavros/global_position/local", Odometry, self.callback)
        self.laser_sub = rospy.Subscriber("/laser/scan", LaserScan, self.detect)
        self.graph_srv = rospy.Service('/get_model', SetBool, self.get_model)
        self.init_srv = rospy.Service('/init_model', SetBool, self.init_model)

    def callback(self, msg):
        self.pose_drone.x=msg.pose.pose.position.x
        self.pose_drone.y=msg.pose.pose.position.y
        self.pose_drone.z=msg.pose.pose.position.z

    def detect(self, msg):
        length_array = int(((msg.angle_max-msg.angle_min)/msg.angle_increment))
        range_array = np.array(msg.ranges[0:length_array])
        count_objects=0
        flag=False
        limits = np.zeros((3, 2))
        for i in range(1,len(range_array)):
            if range_array[i]<msg.range_max and flag==False:
                count_objects=count_objects+1
                limits[count_objects-1,0]=i
                flag=True
            if range_array[i]>msg.range_max and range_array[i-1]<msg.range_max:
                limits[count_objects-1,1]=i
                flag=False 
        #print("Count Objects: %s" %count_objects)
        if count_objects==3:
            min_index = int(np.where(msg.ranges == np.amin(msg.ranges[int(limits[0,0]):int(limits[0,1])]))[0])
            x_value = msg.ranges[min_index]*np.sin(min_index*msg.angle_increment)
            y_value = msg.ranges[min_index]*np.cos(min_index*msg.angle_increment)
            x_point = self.pose_drone.x + x_value
            y_point = self.pose_drone.y - y_value
            z_point = self.pose_drone.z
            if self.init_model_flag:
                self.vector.append([x_point, y_point, z_point])
                self.get_angle(self.vector)
        else:
            rospy.loginfo("Detected solids(%s) are not enough." %count_objects)


    def get_angle(self, vector):
        print("----------")
        data=np.asmatrix(vector)
        x_vector = data[:,1].reshape((-1, 1))
        y_vector = data[:,2]
        model = LinearRegression().fit(x_vector, y_vector)
        self.output.correlation = model.score(x_vector, y_vector)
        self.output.angle = np.rad2deg(np.arctan(model.coef_))
        self.output.center = model.intercept_ + self.output.offset_fuste
        print("Correlacion r^2: %s" %self.output.correlation)
        print("Angulo Aspa: %s" %self.output.angle)
        print("Punto de corte con Z: %s" %self.output.center)

    def init_model(self,req):
        self.init_model_flag=req.data
        if req.data:
            txt_msg = "Recording data"
        else:
            txt_msg = "No Recording"
        return SetBoolResponse(req.data, txt_msg)

    def get_model(self, req):
        aspa_len = 20.0
        base_fuste=[0,0]
        final_fuste=[0,float(self.output.center)]
        plt.plot(base_fuste,final_fuste)
        x_aspas = [float(aspa_len*np.cos(np.deg2rad(-self.output.angle))), float(aspa_len*np.cos(np.deg2rad(-self.output.angle-120))), float(aspa_len*np.cos(np.deg2rad(-self.output.angle-240)))]
        y_aspas = [float(aspa_len*np.sin(np.deg2rad(-self.output.angle))), float(aspa_len*np.sin(np.deg2rad(-self.output.angle-120))), float(aspa_len*np.sin(np.deg2rad(-self.output.angle-240)))]
        plt.plot([final_fuste[0],x_aspas[0]], [final_fuste[1],y_aspas[0]+final_fuste[1]])
        plt.plot([final_fuste[0],x_aspas[1]], [final_fuste[1],y_aspas[1]+final_fuste[1]])
        plt.plot([final_fuste[0],x_aspas[2]], [final_fuste[1],y_aspas[2]+final_fuste[1]])
        plt.xlim([-50, 50])
        plt.ylim([0, 100])
        plt.show()
        return SetBoolResponse(req.data, 'Model was drawn.')


if __name__ == '__main__':
    rospy.init_node('aero_detector')
    try:
        node = aero_detector()
        rate = rospy.Rate(5)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')