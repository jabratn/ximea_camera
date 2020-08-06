/******************************************************************************

Copyright 2015  Abdelhamid El-Bably (University of Waterloo)
                      [ahelbably@uwaterloo.ca]
                Arun Das (University of Waterloo)
                      [adas@uwaterloo.ca]
                Michael Tribou (University of Waterloo)
                      [mjtribou@uwaterloo.ca]

All rights reserved.

********************************************************************************/

#include <ximea_camera/ximea_driver.h>
#include <stdexcept>
#include <sstream>
#include <string>

/*ximea_driver::ximea_driver(std::string serial_no, std::string cam_name)
{
  serial_no_ = serial_no;
  cam_name_ = cam_name;
  assignDefaultValues();
}*/

void ximea_driver::assignDefaultValues()
{
  cams_on_bus_ = 1;
  bandwidth_ = 0;
  downsample_factor_ = 1;
  auto_exposure_ = true;
  auto_exposure_limit_ = 500000;
  auto_gain_limit_ = 2;
  auto_exposure_priority_ = 1.0;
  exp_level_pct_ = 50;
  gain_ = 0;
  exposure_time_ = 100000;
  auto_wb_ = true;
  image_data_format_ = "XI_RAW8";
  roi_left_ = 0;
  roi_top_ = 0;
  roi_width_ = 0;
  roi_height_ = 0;
  xiH_ = NULL;
  image_.size = sizeof(XI_IMG);
  image_.bp = NULL;
  image_.bp_size = 0;
  acquisition_active_ = false;
  image_capture_timeout_ = 1000;
  frame_id_ = "";
}

ximea_driver::ximea_driver(std::string file_name)
{
  assignDefaultValues();
  readParamsFromFile(file_name);
  ROS_INFO_STREAM("ximea_driver: reading paramter values from file: " << file_name);
}

void ximea_driver::errorHandling(XI_RETURN ret, std::string message)
{
  if (ret != XI_OK)
  {
    std::stringstream errMsg;
    std::cout << "Error after "  << message << std::endl;
    closeDevice();
  }
}

void ximea_driver::applyParameters()
{
  setImageDataFormat(image_data_format_);
  setExposure(exposure_time_);
  setGain(gain_);
  setAutoWb(auto_wb_);
  setAutoExposure(auto_exposure_);

  if (auto_exposure_) {
      setAutoExposureLimit(auto_exposure_limit_);
      setAutoGainLimit(auto_gain_limit_);
      setAutoExposurePriority(auto_exposure_priority_);
      setAutoExposureLevel(exp_level_pct_);
  }

  if (downsample_factor_ > 1)
  {
    setDownsample(downsample_factor_);
  }

  setROI(roi_left_, roi_top_, roi_width_, roi_height_);
}

void ximea_driver::openDevice()
{
  XI_RETURN stat;
  if (serial_no_ == "")
  {
    stat = xiOpenDevice(0, &xiH_);
    errorHandling(stat, "Open Device");
  }
  else
  {
    std::stringstream conv;
    conv << serial_no_;
    stat = xiOpenDeviceBy(XI_OPEN_BY_SN, conv.str().c_str(), &xiH_);
    errorHandling(stat, "Open Device");
  }
  applyParameters();
}

void ximea_driver::closeDevice()
{
  if (xiH_)
  {
    xiCloseDevice(xiH_);
    xiH_ = NULL;
  }
}

void ximea_driver::stopAcquisition()
{
  XI_RETURN stat;
  if (!hasValidHandle())
  {
    return;
  }
  stat = xiStopAcquisition(xiH_);
  errorHandling(stat, "Starting Acquisition");  // die if we cannot stop acquisition.
  acquisition_active_ = false;
}

void ximea_driver::startAcquisition()
{
  if (!hasValidHandle())
  {
    return;
  }
  XI_RETURN stat = xiStartAcquisition(xiH_);
  errorHandling(stat, "Starting Acquisition");
  acquisition_active_ = true;
}

void ximea_driver::acquireImage()
{
  if (!hasValidHandle())
  {
    return;
  }
  XI_RETURN stat = xiGetImage(xiH_, image_capture_timeout_, &image_);
  
  if (stat != 0)
  {
    std::cout << "Error on" << cam_name_ << " with error " <<  stat << std::endl;
  }else{
    // std::cout <<"width: "<< image_.width <<std::endl;
    // std::cout <<"height: "<< image_.height <<std::endl;

  }
}

void ximea_driver::setImageDataFormat(std::string image_format)
{
  XI_RETURN stat;
  int image_data_format;

  if (!hasValidHandle())
  {
    return;
  }

  if (image_format == std::string("XI_MONO16"))
  {
    image_data_format = XI_MONO16;
  }
  else if (image_format == std::string("XI_RGB24"))
  {
    image_data_format = XI_RGB24;
  }
  else if (image_format == std::string("XI_RGB32"))
  {
    image_data_format = XI_RGB32;
  }
  else if (image_format == std::string("XI_RGB_PLANAR"))
  {
    image_data_format = XI_MONO8;
    std::cout << "This is unsupported in ROS default to XI_MONO8" << std::endl;
  }
  else if (image_format == std::string("XI_RAW8"))
  {
    image_data_format = XI_RAW8;
  }
  else if (image_format == std::string("XI_RAW16"))
  {
    image_data_format = XI_RAW16;
  }
  else
  {
    image_data_format = XI_MONO8;
  }

  stat = xiSetParamInt(xiH_, XI_PRM_IMAGE_DATA_FORMAT, image_data_format);
  errorHandling(stat, "image_format");  // if we cannot set the format then there is something wrong we should probably quit then
  image_data_format_ = image_data_format;
}

void ximea_driver::setROI(int rect_left, int rect_top, int rect_width, int rect_height)
{
  XI_RETURN stat;

  if (!hasValidHandle())
  {
    return;
  }

  if (rect_width)
  {
    stat = xiSetParamInt(xiH_, XI_PRM_WIDTH, rect_width);
    errorHandling(stat, "xiSetParamInt (XI_PRM_WIDTH)");
  }

  if (rect_height)
  {
    stat = xiSetParamInt(xiH_, XI_PRM_HEIGHT, rect_height);
    errorHandling(stat, "xiSetParamInt (XI_PRM_HEIGHT)");
  }

  if (rect_left)
  {
    stat = xiSetParamInt(xiH_, XI_PRM_OFFSET_X, rect_left);
    errorHandling(stat, "xiSetParamInt (XI_PRM_OFFSET_X)");
  }

  if (rect_top)
  {
    stat = xiSetParamInt(xiH_, XI_PRM_OFFSET_Y, rect_top);
    errorHandling(stat, "xiSetParamInt (XI_PRM_OFFSET_Y)");
  }
}

void ximea_driver::setDownsample (int factor){
  
  XI_RETURN stat;
  stat = xiSetParamInt(xiH_, XI_PRM_BINNING_SELECTOR, XI_BIN_SELECT_HOST_CPU);
  errorHandling(stat, "xiSetParamInt (XI_BIN_SELECT_HOST_CPU)");
 
  // stat = xiSetParamInt(xiH_, XI_PRM_DOWNSAMPLING_TYPE, XI_SKIPPING);
  // errorHandling(stat, "xiSetParamInt (DownSampling)");
  stat = xiSetParamInt(xiH_, XI_PRM_DOWNSAMPLING, factor);
  errorHandling(stat, "xiSetParamInt (DownSampling)");
}

void ximea_driver::setExposure(int time)
{
  XI_RETURN stat = xiSetParamInt(xiH_, XI_PRM_EXPOSURE, time);
  errorHandling(stat, "xiSetParamInt (Exposure Time)");
  if (!stat)
  {
    exposure_time_ = time;
  }
}

void ximea_driver::setGain(float gain)
{
  XI_RETURN stat = xiSetParamInt(xiH_, XI_PRM_GAIN, gain);
  errorHandling(stat, "xiSetParamInt (Gain)");
}

void ximea_driver::setAutoWb(bool auto_wb)
{
  XI_RETURN stat = xiSetParamInt(xiH_, XI_PRM_AUTO_WB, auto_wb);
  errorHandling(stat, "xiSetParamInt (white balance)");
}

void ximea_driver::setAutoExposure(bool auto_exposure)
{
  XI_RETURN stat = xiSetParamInt(xiH_, XI_PRM_AEAG, auto_exposure);
  errorHandling(stat, "xiSetParamInt (AutoExposure)");
}

void ximea_driver::setAutoExposureLimit(int ae_limit)
{
  XI_RETURN stat = xiSetParamInt(xiH_, XI_PRM_AE_MAX_LIMIT, ae_limit);
  errorHandling(stat, "xiSetParamInt (AutoExposure Limit Time)");
}

void ximea_driver::setAutoGainLimit(int ag_limit)
{
  XI_RETURN stat = xiSetParamInt(xiH_, XI_PRM_AG_MAX_LIMIT, ag_limit);
  errorHandling(stat, "xiSetParamInt (AutoExposure Limit GAIN)");
}

void ximea_driver::setAutoExposurePriority(float exp_priority)
{
  XI_RETURN stat = xiSetParamFloat(xiH_, XI_PRM_EXP_PRIORITY, exp_priority);
  errorHandling(stat, "xiSetParamFloat (AutoExposure Priority)");
}

void ximea_driver::setAutoExposureLevel(int exp_level_pct)
{
  XI_RETURN stat = xiSetParamInt(xiH_, XI_PRM_AEAG_LEVEL, exp_level_pct);
  errorHandling(stat, "xiSetParamInt (AutoExposure Level)");
}

int ximea_driver::readParamsFromFile(std::string file_name)
{
  std::ifstream fin(file_name.c_str());
  if (fin.fail())
  {
    ROS_ERROR_STREAM("could not open file " << file_name.c_str() << std::endl);
    exit(-1);
  }


  YAML::Node doc = YAML::LoadFile(file_name);
  std::string tmpS;
  int tmpI1, tmpI2, tmpI3, tmpI4;
  bool tmpB;


  try
  {
    serial_no_ = doc["serial_no"].as<std::string>();
  }
  catch (std::runtime_error) {}
  try
  {
    cam_name_ =  doc["cam_name"].as<std::string>();
  }
  catch (std::runtime_error) {}
  try
  {
    frame_id_ =  doc["frame_id"].as<std::string>();
  }
  catch (std::runtime_error) {}

  try
  {
    yaml_url_ = doc["yaml_url"].as<std::string>();
  }
  catch (std::runtime_error) {}

  try
  {
    cams_on_bus_ = doc["cams_on_bus"].as<int>();
  }
  catch (std::runtime_error) {}

  try
  {
    bandwidth_ = doc["bandwidth"].as<int>();
  }
  catch (std::runtime_error) {}

  try
  {
    frame_rate_ = doc["frame_rate"].as<int>();
  }
  catch (std::runtime_error) {}

  try
  {
    exposure_time_ = doc["exposure_time"].as<int>();
  }
  catch (std::runtime_error) {}

  try
  {
    gain_ = doc["gain"].as<float>();
  }
  catch (std::runtime_error) {}

  try
  {
    auto_wb_ = doc["auto_wb"].as<bool>();
  }
  catch (std::runtime_error) {}

  try
  {
    auto_exposure_ = doc["auto_exposure"].as<bool>();
  }
  catch (std::runtime_error) {}

  try
  {
    auto_exposure_limit_ = doc["auto_exposure_limit"].as<int>();
  }
  catch (std::runtime_error) {}

  try
  {
    auto_gain_limit_ = doc["auto_gain_limit"].as<int>();
  }
  catch (std::runtime_error) {}

  try
  {
    auto_exposure_priority_ = doc["auto_exposure_priority"].as<float>();
  }
  catch (std::runtime_error) {}

  try
  {
    exp_level_pct_ = doc["auto_exposure_level_pct"].as<int>();
  }
  catch (std::runtime_error) {}

  try
  {
    downsample_factor_ = doc["downsample_factor"].as<int>();
  }
  catch (std::runtime_error) {}

  try
  {
    roi_left_ = doc["roi_left"].as<int>();
  }
  catch (std::runtime_error) {}
  try
  {
    roi_top_ = doc["roi_top"].as<int>();
  }
  catch (std::runtime_error) {}
  try
  {
    roi_width_ = doc["roi_width"].as<int>();
  }
  catch (std::runtime_error) {}

  try
  {
    roi_height_ = doc["roi_height"].as<int>();
  }
  catch (std::runtime_error) {}

  try
  {
    image_data_format_ = doc["image_data_format"].as<std::string>();
  }
  catch (std::runtime_error) {}
}
void ximea_driver::enableTrigger(unsigned char trigger_mode)
{
  if (!xiH_) return;
  trigger_mode_ = trigger_mode;
  if (trigger_mode_ > 3 || trigger_mode_ < 0)
  {
    trigger_mode_ = 0;
  }
  XI_RETURN stat;
  switch (trigger_mode_)
  {
  case (0):
    break;
  case (1):
    stat = xiSetParamInt(xiH_, XI_PRM_TRG_SOURCE, XI_TRG_SOFTWARE);
    errorHandling(stat, "Could not enable software trigger");
    break;
  case (2):
    break;
  default:
    break;
  }
}

void ximea_driver::triggerDevice()
{
  if (!xiH_) return;
  XI_RETURN stat;
  switch (trigger_mode_)
  {
  case (0):
    break;
  case (1):
    stat = xiSetParamInt(xiH_, XI_PRM_TRG_SOFTWARE, 1);
    errorHandling(stat, "Error During triggering");
    std::cout << "triggering " << cam_name_ <<  std::endl;
    break;
  case (2):
    break;
  default:
    break;
  }
}

void ximea_driver::limitBandwidth(int mbps)
{
  if (!xiH_) return;
  XI_RETURN stat;
  stat = xiSetParamInt(xiH_, XI_PRM_LIMIT_BANDWIDTH_MODE , XI_ON);
  if (!stat)
  {
    errorHandling(stat, "could not enable bandwidth");
  }
  stat = xiSetParamInt(xiH_, XI_PRM_LIMIT_BANDWIDTH , mbps);
  if (!stat)
  {
    errorHandling(stat, "could not limit bandwidth");
  }
}
