#include <ros/ros.h> 
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <iomanip>

#include <geometry_msgs/PoseStamped.h>

#include <iostream>
#include <stdlib.h>
#include <string>
#include <sstream>

using namespace std;
serial::Serial ser; //声明串口对象 

template <class Type>
Type stringToNum(const string& str)
{
  istringstream iss(str);
  Type num;
  iss >> num;
  return num;
}

int main (int argc, char** argv) 
{ 
  //初始化节点 
  ros::init(argc, argv, "pub_current_rtk"); 
  //声明节点句柄 
  ros::NodeHandle nh; 

  //发布主题 
  ros::Publisher rtk_pub = nh.advertise<geometry_msgs::PoseStamped>("local_xy_origin", 50); 
  try 
    { 
      //设置串口属性，并打开串口 
      ser.setPort("/dev/ttyUSB0"); 
      ser.setBaudrate(115200); 
      serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
      ser.setTimeout(to); 
      ser.open(); 
    } 
  catch (serial::IOException& e) 
    { 
      ROS_ERROR_STREAM("Unable to open port "); 
      return -1; 
    } 
  //检测串口是否已经打开，并给出提示信息 
  if(ser.isOpen()) 
    { 
      ROS_INFO_STREAM("Serial Port initialized "); 
    } 
  else 
    { 
      return -1; 
    } 
  //指定循环的频率 
  ros::Rate loop_rate(100); 
  while(ros::ok()) 
    { 
      if(ser.available()){
        std_msgs::String rtk_msgs;
        rtk_msgs.data = ser.read(ser.available());	

        /*rtk_msgs.data = "$GPGGA,UTCtime,latitude,N/s,longitude,W/E,
          position_type,number_of_satellite,HDOP,
          altitude,M,geoidal_separation,M,
          age_of_differential_corrections,id*checksum"
        */

        stringstream input(rtk_msgs.data);
        string tmp;
        vector<string> str_data;

        // 15 numbers in RTK systems
        for(int i=0; i<15; i++){
          getline(input, tmp, ',');
          str_data.push_back(tmp);
          }

        double latitude, longitude, altitude, HDOP;
        latitude = stringToNum<double>(str_data[2]);
        latitude *= 0.01;
        longitude = stringToNum<double>(str_data[4]);
        longitude *= 0.01;
        //HDOP = stringToNum<float>(str_data[8]);
        altitude = stringToNum<double>(str_data[9]);

        if (str_data[3] != "N")
        {
          latitude *= (-1);
        }
        if (str_data[5] != "E")
        {
          longitude *= (-1);
        }

        cout << "Latitude:" << fixed << setprecision(6) << latitude << endl;
        cout << "Longitude:" << fixed << setprecision(6) << longitude << endl;
        //cout << "HDOP:" << HDOP << endl;
        //cout << "Altitude:" << altitude << endl;

        //ROS_INFO_STREAM("RTK_GPS: " << rtk_msgs.data);

        //publish current rtk message
        geometry_msgs::PoseStamped current_rtk;
        //sensor_msgs::NavSatStatus gps_status;
        //gps_status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
        //gps_status.status = sensor_msgs::NavSatStatus::SERVICE_GPS;
        current_rtk.header.stamp = ros::Time::now();
        current_rtk.header.frame_id = "map";
        current_rtk.pose.position.x = longitude;
        current_rtk.pose.position.y = latitude;
        rtk_pub.publish(current_rtk);
      };

      //处理ROS的信息，比如订阅消息,并调用回调函数 
      ros::spinOnce(); 
      loop_rate.sleep(); 
      //ros::spin();  
    } 
}


