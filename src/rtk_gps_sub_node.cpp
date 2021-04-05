#include <ros/ros.h> 
#include <serial/serial.h>
#include <std_msgs/String.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

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
  ros::init(argc, argv, "rtk_gps_sub"); 
  //声明节点句柄 
  ros::NodeHandle nh; 

  //发布主题 
  ros::Publisher rtk_gps_pub = nh.advertise<sensor_msgs::NavSatFix>("rtk_gps", 50); 
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

        float latitude, longitude, altitude, HDOP;
        latitude = stringToNum<float>(str_data[2]);
        longitude = stringToNum<float>(str_data[4]);
        HDOP = stringToNum<float>(str_data[8]);
        altitude = stringToNum<float>(str_data[9]);

        if (str_data[3] != "N")
        {
          latitude *= (-1);
        }
        if (str_data[5] != "E")
        {
          longitude *= (-1);
        }

        cout << "Latitude:" << latitude << endl;
        cout << "Longitude:" << longitude << endl;
        cout << "HDOP:" << HDOP << endl;
        cout << "Altitude:" << altitude << endl;

        //ROS_INFO_STREAM("RTK_GPS: " << rtk_msgs.data);

        // rtk_gps msg pub
        sensor_msgs::NavSatFix nav_sat_fix;
        sensor_msgs::NavSatStatus gps_status;
        gps_status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
        gps_status.status = sensor_msgs::NavSatStatus::SERVICE_GPS;
        nav_sat_fix.header.stamp = ros::Time::now();
        nav_sat_fix.header.frame_id = "rtk_gps_link";
        nav_sat_fix.status = gps_status;
        nav_sat_fix.longitude = longitude;
        nav_sat_fix.latitude = latitude;
        nav_sat_fix.altitude = altitude;
        rtk_gps_pub.publish(nav_sat_fix);
      };

      //处理ROS的信息，比如订阅消息,并调用回调函数 
      ros::spinOnce(); 
      loop_rate.sleep(); 
      //ros::spin();  
    } 
}


