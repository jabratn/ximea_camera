/******************************************************************************

Copyright 2015  Abdelhamid El-Bably (University of Waterloo)
                      [ahelbably@uwaterloo.ca]
                Arun Das (University of Waterloo)
                      [adas@uwaterloo.ca]
                Michael Tribou (University of Waterloo)
                      [mjtribou@uwaterloo.ca]

All rights reserved.

********************************************************************************/

#include <ximea_camera/ximea_ros_cluster.h>
#include <string>
#include <vector>

std::string getCamNameFromYaml(std::string file_name)
{
  std::ifstream fin(file_name.c_str());
  if (fin.fail())
  {
    ROS_ERROR_STREAM("could not open file " << file_name.c_str() << std::endl);
    exit(-1);  // this has to be changed
  }

  YAML::Node doc = YAML::LoadFile(file_name);
  std::string ret;
  ret = doc["cam_name"].as<std::string>();
  return ret;
}

ximea_ros_cluster::ximea_ros_cluster(std::vector<std::string> filenames, int bandwidth_mbps = 0) : bandwidth_mbps(bandwidth_mbps)
{
  devices_open_ = false;
  for (int i = 0 ; i < filenames.size(); i ++)
  {
    std::string cam_name = getCamNameFromYaml(filenames[i]);
    ros::NodeHandle nh(std::string("/") + cam_name);
    add_camera(ximea_ros_driver(nh, filenames[i]));
  }
}

void ximea_ros_cluster::add_camera(ximea_ros_driver xd)
{
  if (devices_open_)
  {
    clusterEnd();
  }
  cams_.push_back(xd);
  num_cams_++;
  threads_.resize(num_cams_);
  ROS_INFO_STREAM("done camera add");
}

void ximea_ros_cluster::remove_camera(std::string serial_no)
{
  if (devices_open_)
  {
    clusterEnd();
  }
  for (int i = 0; i < cams_.size(); i++)
  {
    if (serial_no == cams_[i].getSerialNo())
    {
      cams_.erase(cams_.begin() + i);
      delete threads_[i];
      threads_.erase(threads_.begin() + i);
      break;
    }
  }
  num_cams_--;
}

void ximea_ros_cluster::clusterInit()
{
  for (int i = 0; i < cams_.size(); i++)
  {
    ROS_INFO_STREAM("opening device " << cams_[i].getSerialNo());
    cams_[i].openDevice();
    if (bandwidth_mbps)
    {
      // share bandwidth between cameras
      cams_[i].limitBandwidth(bandwidth_mbps/cams_.size());
      std::cout << "limit BANDWIDTH to: " << bandwidth_mbps/cams_.size() << std::endl;
    }
    cams_[i].startAcquisition();
    // TODO: remove this into constructor
  }
  devices_open_ = true;
}

void ximea_ros_cluster::clusterEnd()
{
  for (int i = 0; i < cams_.size(); i  ++)
  {
    cams_[i].stopAcquisition();
    cams_[i].closeDevice();
  }
  devices_open_ = false;
}

// triggered_acquire
void ximea_ros_cluster::clusterAcquire()
{
  for (int i = 0; i < cams_.size(); i  ++)
  {
    threads_[i] = new boost::thread(&ximea_driver::acquireImage, &cams_[i]);
  }
  for (int i = 0; i < cams_.size(); i  ++)
  {
    threads_[i]->join();
    delete threads_[i];
  }
}

void ximea_ros_cluster::clusterPublishImages()
{
  // TODO: might want to think as to how to multithread this
  for (int i = 0; i < cams_.size(); i  ++)
  {
    threads_[i] = new boost::thread(&ximea_ros_driver::publishImage, &cams_[i], ros::Time::now());
  }
  for (int i = 0; i < cams_.size(); i  ++)
  {
    threads_[i]->join();
    delete threads_[i];
  }
}

void ximea_ros_cluster::clusterPublishCamInfo()
{
  for (int i = 0 ; i < cams_.size(); i ++)
  {
    cams_[i].publishCamInfo(ros::Time::now());
  }
}

void ximea_ros_cluster::clusterPublishImageAndCamInfo()
{
  ros::Time curr_time = ros::Time::now();
  for (int i = 0; i < cams_.size(); i  ++)
  {
    threads_[i] = new boost::thread(&ximea_ros_driver::publishImage, &cams_[i], curr_time);
  }
  for (int i = 0; i < cams_.size(); i  ++)
  {
    threads_[i]->join();
    delete threads_[i];
  }
  for (int i = 0 ; i < cams_.size(); i ++)
  {
    cams_[i].publishCamInfo(curr_time);
  }
}

int ximea_ros_cluster::getCameraIndex(std::string serial_no)
{
  for (int i = 0; i < cams_.size(); i++)
  {
    if (serial_no == cams_[i].getSerialNo())
    {
      return i;
    }
  }
  return -1;
}

void ximea_ros_cluster::setExposure(std::string serial_no, int time)
{
  int idx = getCameraIndex(serial_no) ;
  if (idx != -1)
  {
    cams_[idx].setExposure(time);
  }
}

void ximea_ros_cluster::setImageDataFormat(std::string serial_no, std::string s)
{
  int idx = getCameraIndex(serial_no) ;
  if (idx != -1)
  {
    cams_[idx].setImageDataFormat(s);
  }
}

void ximea_ros_cluster::setROI(std::string serial_no, int l, int t, int w, int h)
{
  int idx = getCameraIndex(serial_no) ;
  if (idx != -1)
  {
    cams_[idx].setROI(l, t, w, h);
  }
}
