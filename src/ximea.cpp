/******************************************************************************

Copyright 2015  Arun Das (University of Waterloo) 
                      [adas@uwaterloo.ca]
                Abdelhamid El-Bably (University of Waterloo)
                      [ahelbably@uwaterloo.ca]
                Michael Tribou (University of Waterloo)
  	                  [mjtribou@uwaterloo.ca]

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

#include <ximea_camera/ximea_ros_cluster.h>

using namespace std;

int main(int argc, char ** argv){
	//string file = "/home/wavelab/WavelabRepo/projects/drivers/ximea_camera/src/cam1.yaml";
	//ximea_driver xd(file);
	ros::init(argc,argv, "ximea");
	ros::NodeHandle nh;		//standard ros nodehanlde
	ros::NodeHandle pnh("~");	//needed for parameter server
	int frame_rate_;
	std::string yaml_file1 = "/home/adas/indigo/catkin_ws/src/ximea_camera/config/cam1.yaml";
	std::string yaml_file2 = "/home/adas/indigo/catkin_ws/src/ximea_camera/config/cam2.yaml";
	std::string yaml_file3 = "/home/adas/indigo/catkin_ws/src/ximea_camera/config/cam3.yaml";
	//std::string yaml_file4 = "/home/adas/catkin_ws/src/ximea_camera/src/cam4.yaml";

  	pnh.param<int>("frame_rate", frame_rate_, 100);
	ros::Rate loop(frame_rate_);
	//ximea_ros_driver xd(nh, "camera1", 0);
	//xd.openDevice();
	//xd.setImageDataFormat("XI_RGB24");
	//xd.setExposure(5000);
	//xd.setROI(172, 274, 940, 480);
	//xd.startAcquisition();
	//ximea_ros_cluster xd(1);

	std::vector<std::string> file_names;

	file_names.push_back(yaml_file1);
	file_names.push_back(yaml_file2);
	//file_names.push_back(yaml_file3);
	//file_names.push_back(yaml_file4);

	ximea_ros_cluster xd(file_names);
	//std::cout << "we're here" << std::endl;
	xd.clusterInit();
	while (ros::ok()){	//TODO: need to robustify against replugging and cntrlc
		ros::spinOnce();
	//	xd.acquireImage();
		xd.clusterAcquire();
	//	xd.publishImageAndCamInfo();
		xd.clusterPublishImageAndCamInfo();
		loop.sleep();
	}
	xd.clusterEnd();
	return 1;
}
