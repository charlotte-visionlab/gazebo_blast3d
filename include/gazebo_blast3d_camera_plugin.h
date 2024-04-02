/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef GAZEBO_BLAST3D_CAMERA_PLUGIN_H
#define GAZEBO_BLAST3D_CAMERA_PLUGIN_H

#include <string>

#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/ImuSensor.hh"
#include "gazebo/sensors/CameraSensor.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/util/system.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"

#include "Event.pb.h"
#include "EventArray.pb.h"
#include "OpticalFlow.pb.h"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <ignition/math.hh>

//#include "flow_opencv.hpp"
//#include "flow_px4.hpp"

#define DEFAULT_RATE 20
#define HAS_GYRO true


#pragma once

#include <cmath>

//#include "optical_flow.hpp"
//#include "trackFeatures.h"
#include "utils/common.h"

#define DEFAULT_OUTPUT_RATE 15
#define DEFAULT_IMAGE_WIDTH 64
#define DEFAULT_IMAGE_HEIGHT 64
#define DEFAULT_NUMBER_OF_FEATURES 20
#define DEFAULT_CONFIDENCE_MULTIPLIER 1.645f //90% confidence interval

class OpticalFlow
{

protected:
	//params which can be set
	int image_width;
	int image_height;
	float focal_length_x; //[pixel]
	float focal_length_y; //[pixel]
	int output_rate;
	float sum_flow_x;
	float sum_flow_y;
	int sum_flow_quality;
	int valid_frame_count;

	void initLimitRate();
	int limitRate(int flow_quality, const uint32_t frame_time_us, int *dt_us,
		      float *flow_x, float *flow_y);

public:

	virtual ~OpticalFlow(){};

//	inline void setImageWidth(int img_width) { image_width = img_width; };
//	inline void setImageHeight(int img_height) { image_height = img_height; };
//	inline void setFocalLengthX(float f_lengh) { focal_length_x = f_lengh; };
//	inline void setFocalLengthY(float f_lengh) { focal_length_y = f_lengh; };
//	inline void setOutputRate(int out_rate) { output_rate = out_rate; };   //TODO check valid range 10-20?
//
//	inline int getImageWidth() { return image_width; };
//	inline int getImageHeight() { return image_height; };
//	inline int getFocalLengthX() { return focal_length_x; };
//	inline int getFocalLengthy() { return focal_length_y; };
//	inline int getOutputRate() { return output_rate; };
//
//	virtual int calcFlow(uint8_t *img_current, const uint32_t &img_time_us, int &dt_us, float &flow_x, float &flow_y) = 0;

};

class OpticalFlowOpenCV : public OpticalFlow
{

private:
	//params which can be set
	int num_features;
	float confidence_multiplier;
	cv::Mat_<float> camera_matrix;
	cv::Mat_<float> camera_distortion;
	//general
	std::vector<int> updateVector;
	std::vector<cv::Point2f> features_current, features_previous, features_tmp, useless;
	bool set_camera_matrix;
	bool set_camera_distortion;

public:

//	inline void setNumFeatures(int n_feat) { num_features = n_feat; };
//	inline void setConfMultiplier(float conf_multi) { confidence_multiplier = conf_multi; };
//	void setCameraMatrix(float focal_len_x, float focal_len_y, float principal_point_x, float principal_point_y);
//	void setCameraDistortion(float k1, float k2, float k3, float p1 = 0.0f, float p2 = 0.0f);
//
//	inline int getNumFeatures() { return num_features; };
//	inline int getConfMultiplier() { return confidence_multiplier; };
//
//	OpticalFlowOpenCV(float f_length_x, float f_length_y, int output_rate = DEFAULT_OUTPUT_RATE,
//			  int img_width = DEFAULT_IMAGE_WIDTH, int img_height = DEFAULT_IMAGE_HEIGHT, int num_feat = DEFAULT_NUMBER_OF_FEATURES,
//			  float conf_multi = DEFAULT_CONFIDENCE_MULTIPLIER);
//	~OpticalFlowOpenCV();
//
//	int calcFlow(uint8_t *img_current, const uint32_t &img_time_us, int &dt_us,
//		     float &flow_x, float &flow_y);

};

using namespace std;

namespace gazebo
{
    // Constants
    static const bool kPrintOnPluginLoad = true;
  static const std::string kDefaultGyroTopic = "/px4flow/imu";

  class GAZEBO_VISIBLE GazeboBlast3DCameraPlugin : public SensorPlugin
  {
    public:
      GazeboBlast3DCameraPlugin();
      virtual ~GazeboBlast3DCameraPlugin();
      virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
      virtual void OnNewFrameRGBCamera(const unsigned char * _image);
      virtual void OnNewFrameOpticalFlow(const unsigned char *_image,
                              unsigned int _width, unsigned int _height,
                              unsigned int _depth, const std::string &_format);
      virtual void OnNewFrameEventCamera(const unsigned char *_image,
                              unsigned int _width, unsigned int _height,
                              unsigned int _depth, const std::string &_format);
      virtual void ImuCallback(ConstIMUPtr& _imu);
      virtual void processDelta(cv::Mat &last_image, cv::Mat &curr_image, cv::Mat &last_blast_image, 
                                cv::Mat &curr_blast_image, cv::Mat &curr_blast_alaph, 
                                std::vector<sensor_msgs::msgs::Event> &events,
                                bool explosion=false);
      virtual void fillEvents(cv::Mat &diff, int polarity, vector<sensor_msgs::msgs::Event> &events);

    protected:
      unsigned int width, height, depth;
      std::string format;
      sensors::CameraSensorPtr parentSensor;
      rendering::CameraPtr camera;
      physics::WorldPtr world;

    private:
      event::ConnectionPtr newFrameConnection;
      cv::Mat last_image;
      cv::Mat last_blast_image;
      int last_blast_image_idx;
      bool has_last_image;
      bool has_last_blast_image;
      float event_threshold;
      transport::PublisherPtr opticalFlow_pub_;
      transport::PublisherPtr eventCamera_pub_;
      transport::PublisherPtr rgbCamera_pub_;
      transport::NodePtr node_handle_;
      transport::SubscriberPtr imuSub_;
      gazebo::msgs::Image rgbCamera_message;
      sensor_msgs::msgs::Event eventCameraEvent_message;
      sensor_msgs::msgs::EventArray eventCameraEventArray_message;
      sensor_msgs::msgs::OpticalFlow opticalFlow_message;
      ignition::math::Vector3d opticalFlow_rate;
      std::string namespace_;
      std::string gyro_sub_topic_;
      std::string blast3d_video_datafolder_;
      std::string blast3d_rgb_image_topic_, blast3d_event_image_topic_, blast3d_event_topic_;
      std::string camera_mode_;
      OpticalFlowOpenCV *optical_flow_;
      // OpticalFlowPX4 *optical_flow_;
      std::vector<cv::Mat> blastRGBImageVec;
      std::vector<cv::Mat> blastGrayImageVec;
      std::vector<cv::Mat> blastImageAlphaVec;

      float hfov_;
      int dt_us_;
      int output_rate_;
      float focal_length_;
      double first_frame_time_;
      uint32_t frame_time_us_;
      bool has_gyro_;
  };
}
#endif /* GAZEBO_BLAST3D_CAMERA_PLUGIN_H */

