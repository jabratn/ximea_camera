/******************************************************************************

Copyright 2015  Abdelhamid El-Bably (University of Waterloo)
                      [ahelbably@uwaterloo.ca]
                Arun Das (University of Waterloo)
                      [adas@uwaterloo.ca]
                Michael Tribou (University of Waterloo)
                      [mjtribou@uwaterloo.ca]

All rights reserved.

********************************************************************************/
#ifndef XIMEA_CAMERA_XIMEA_ROS_CLUSTER_H
#define XIMEA_CAMERA_XIMEA_ROS_CLUSTER_H

#include <ximea_camera/ximea_ros_driver.h>
#include <boost/thread.hpp>
#include <string>
#include <vector>

class ximea_ros_cluster
{
public:
  explicit ximea_ros_cluster(int num_cams);
  explicit ximea_ros_cluster(std::vector < std::string > filenames, int bandwidth_mbps);
  void add_camera(ximea_ros_driver xd);
  void remove_camera(std::string serial_no);

  // cluster functions
  void clusterInit();
  void clusterAcquire();
  void clusterPublishImages();
  void clusterPublishCamInfo();
  void clusterPublishImageAndCamInfo();
  void clusterEnd();
  bool isDeviceOpen()
  {
    return devices_open_;
  }

  // individual camera functions (encapsulated for thread security)
  void setExposure(std::string serial_no, int time);
  void setImageDataFormat(std::string serial_no, std::string s);
  void setROI(std::string serial_no, int l, int t, int w, int h);

private:
  std::vector<ximea_ros_driver> cams_;
  std::vector<boost::thread*> threads_;
  bool devices_open_;
  int num_cams_;
  int getCameraIndex(std::string serial_no);  // this is private so that no one tries to be smart and open/close our cameras externally, in which case we cannot manage
  const int bandwidth_mbps;
};

#endif  // XIMEA_CAMERA_XIMEA_ROS_CLUSTER_H
