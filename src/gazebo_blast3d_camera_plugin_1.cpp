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
#ifdef _WIN32
// Ensure that Winsock2.h is included before Windows.h, which can get
// pulled in by anybody (e.g., Boost).
#include <Winsock2.h>
#endif

#include "gazebo/sensors/DepthCameraSensor.hh"
#include "gazebo_blast3d_camera_plugin.h"
#include "Event.pb.h"
#include "EventArray.pb.h"

#include <math.h>
#include <string>
#include <iostream>
#include <boost/algorithm/string.hpp>

using namespace std;

using namespace gazebo;

GZ_REGISTER_SENSOR_PLUGIN(GazeboBlast3DCameraPlugin)

/////////////////////////////////////////////////
GazeboBlast3DCameraPlugin::GazeboBlast3DCameraPlugin(): 
    SensorPlugin(), 
    frame_id_(kDefaultFrameId),
    link_name_(kDefaultLinkName),
    blast3d_server_reglink_topic_(kDefaultBlast3dServerRegisterTopic_model),
    blast3d_server_link_topic_(kDefaultNamespace + "/" + kDefaultLinkName + "/" + kDefaultBlast3dTopic),
    pub_interval_(0.1),
    pubs_and_subs_created_(false),
    width(0), 
    height(0),
    depth(0), 
    explosion_triggered_(false),
    has_last_image(false),  
    has_last_blast_image(false){

}

/////////////////////////////////////////////////

GazeboBlast3DCameraPlugin::~GazeboBlast3DCameraPlugin() {
    this->parentSensor.reset();
    this->camera.reset();
}

/////////////////////////////////////////////////

void GazeboBlast3DCameraPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) {
    if (!_sensor)
        gzerr << "Invalid sensor pointer.\n";
    if (kPrintOnPluginLoad) {
        gzdbg << __FUNCTION__ << "() called." << std::endl;
    }

    this->parentSensor = std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);

    if (!this->parentSensor) {
        gzerr << "OpticalFlowPlugin requires a CameraSensor.\n";
        if (std::dynamic_pointer_cast<sensors::DepthCameraSensor>(_sensor))
            gzmsg << "It is a depth camera sensor\n";
    }

    if (!this->parentSensor) {
        gzerr << "OpticalFlowPlugin not attached to a camera sensor\n";
        return;
    }

    this->sensor_ = _sensor;
    this->world_ = physics::get_world(this->parentSensor->WorldName());

#if GAZEBO_MAJOR_VERSION >= 7
    this->camera = this->parentSensor->Camera();
    this->width = this->camera->ImageWidth();
    this->height = this->camera->ImageHeight();
    this->depth = this->camera->ImageDepth();
    this->format = this->camera->ImageFormat();
    hfov_ = float(this->camera->HFOV().Radian());
    first_frame_time_ = this->camera->LastRenderWallTime().Double();
    const string scopedName = _sensor->ParentName();
#else
    this->camera = this->parentSensor->GetCamera();
    this->width = this->camera->GetImageWidth();
    this->height = this->camera->GetImageHeight();
    this->depth = this->camera->GetImageDepth();
    this->format = this->camera->GetImageFormat();
    hfov_ = float(this->camera->GetHFOV().Radian());
    first_frame_time_ = this->camera->GetLastRenderWallTime().Double();
    const string scopedName = _sensor->GetParentName();
#endif
    
 #if GAZEBO_MAJOR_VERSION >= 9
        last_time_ = world_->SimTime();
#else
        last_time_ = world_->GetSimTime();
#endif

    focal_length_ = (this->width / 2) / tan(hfov_ / 2);

    //    if (this->width != 64 || this->height != 64) {
    //        gzerr << "[gazebo_optical_flow_plugin] Incorrect image size, must by 64 x 64.\n";
    //    }

    if (_sdf->HasElement("robotNamespace"))
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
        gzwarn << "[gazebo_blast3d_camera_plugin] Please specify a robotNamespace.\n";

    if (_sdf->HasElement("outputRate")) {
        output_rate_ = _sdf->GetElement("outputRate")->Get<int>();
    } else {
        output_rate_ = DEFAULT_RATE;
        gzwarn << "[gazebo_optical_flow_plugin] Using default output rate " << output_rate_ << ".";
    }

    if (_sdf->HasElement("hasGyro"))
        has_gyro_ = _sdf->GetElement("hasGyro")->Get<bool>();
    else
        has_gyro_ = HAS_GYRO;

    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init(namespace_);
    
    // topic publishing rates
    double pub_rate;
    getSdfParam<double>(_sdf, "publishRate", pub_rate, pub_rate);
    pub_interval_ = (pub_rate > 0.0) ? 1 / pub_rate : 0.0;
    gzdbg << "publish rate = " << pub_rate << std::endl;

    if (has_gyro_) {
        if (_sdf->HasElement("hasGyro"))
            gyro_sub_topic_ = _sdf->GetElement("gyroTopic")->Get<std::string>();
        else
            gyro_sub_topic_ = kDefaultGyroTopic;

        string topicName = "~/" + _sensor->ParentName() + gyro_sub_topic_;
        boost::replace_all(topicName, "::", "/");
        imuSub_ = node_handle_->Subscribe(topicName, &GazeboBlast3DCameraPlugin::ImuCallback, this);
    }

    if (_sdf->HasElement("eventThreshold"))
        this->event_threshold = _sdf->GetElement("eventThreshold")->Get<float>();
    else
        gzwarn << "[gazebo_blast3d_camera_plugin] Please specify a DVS event threshold." << endl;

    getSdfParam<std::string>(_sdf, "blast3dRGBImageTopic", blast3d_rgb_image_topic_,
            blast3d_rgb_image_topic_);
    getSdfParam<std::string>(_sdf, "blast3dEventImageTopic", blast3d_event_image_topic_,
            blast3d_event_image_topic_);
    getSdfParam<std::string>(_sdf, "blast3dEventTopic", blast3d_event_topic_,
            blast3d_event_topic_);
    getSdfParam<std::string>(_sdf, "blast3dVideoDataFolder", blast3d_video_datafolder_,
            blast3d_video_datafolder_);
    getSdfParam<std::string>(_sdf, "cameraMode", camera_mode_,
            camera_mode_);
    getSdfParam<std::string>(_sdf, "blast3dServerRegisterLinkTopic", blast3d_server_reglink_topic_,
            blast3d_server_reglink_topic_);
    getSdfParam<std::string>(_sdf, "blast3dServerLinkTopic", blast3d_server_link_topic_,
                blast3d_server_link_topic_);
    
    // Load blast images
    cv::String folder(blast3d_video_datafolder_);
    cv::String pattern = folder + "/*.png";
    std::vector<cv::String> fn;
    cv::glob(pattern, fn, true);    // recursive glob
    std::sort(fn.begin(), fn.end());    // sort file paths alphabetically
    
    for (const auto& file : fn) {        
        cv::Mat image = cv::imread(file, cv::IMREAD_UNCHANGED);
        //assert(image.channels() == 4 &&  && "[gazebo_blast_camera_plugin] Blast images have to be RGBA with an alpha channel for image overlay to work.\n");
        if (!image.empty()) {
            gzdbg << "Successfully read blast image " << file << std::endl;
            cv::Mat bgrImage;
            cv::cvtColor(image, bgrImage, cv::COLOR_BGRA2BGR);
            blastRGBImageVec.push_back(bgrImage);
            cv::Mat grayImage;
            cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
            blastGrayImageVec.push_back(grayImage);
            // Extract the alpha channel from the base image for blending
            std::vector<cv::Mat> channels;
            cv::split(image, channels);
            cv::Mat alpha = channels[3];
            blastImageAlphaVec.push_back(alpha);
        } else {
            gzerr << "Could not read image: " << file << std::endl;
        }
    }

    if (camera_mode_ == "optical") {
        std::string topicName = "~/" + scopedName + "/opticalFlow";
        boost::replace_all(topicName, "::", "/");
        opticalFlow_pub_ = node_handle_->Advertise<sensor_msgs::msgs::OpticalFlow>(topicName, 1);
        this->newFrameConnection = this->camera->ConnectNewImageFrame(
                boost::bind(&GazeboBlast3DCameraPlugin::OnNewFrameOpticalFlow,
                this, _1, this->width, this->height, this->depth, this->format));
    } 
    else {
        if (camera_mode_ == "event") {
            eventCamera_pub_ = node_handle_->Advertise<sensor_msgs::msgs::EventArray>(blast3d_event_image_topic_, 1);
        }
        else if (camera_mode_ == "RGB") {
            rgbCamera_pub_ = node_handle_->Advertise<gazebo::msgs::Image>(blast3d_rgb_image_topic_, 1);
        }
        this->newFrameConnection = this->camera->ConnectNewImageFrame(
                boost::bind(&GazeboBlast3DCameraPlugin::OnNewFrameCamera, this, _1));
    }

    string sensorName = "";
    if (_sdf->HasElement("cameraName"))
        sensorName = _sdf->GetElement("cameraName")->Get<std::string>() + "/";
    else
        gzwarn << "[gazebo_blast3d_camera_plugin] Please specify a cameraName." << endl;

    this->parentSensor->SetActive(true);

    //init flow
    //optical_flow_ = new OpticalFlowOpenCV(focal_length_, focal_length_, output_rate_);
    // _optical_flow = new OpticalFlowPX4(focal_length_, focal_length_, output_rate_, this->width);
}

/////////////////////////////////////////////////

void GazeboBlast3DCameraPlugin::OnNewFrameOpticalFlow(const unsigned char * _image,
        unsigned int _width,
        unsigned int _height,
        unsigned int _depth,
        const std::string &_format) {
    
    //get data depending on gazebo version
#if GAZEBO_MAJOR_VERSION >= 7
    _image = this->camera->ImageData(0);
    double frame_time = this->camera->LastRenderWallTime().Double();
#else
    _image = this->camera->GetImageData(0);
    double frame_time = this->camera->GetLastRenderWallTime().Double();
#endif

    frame_time_us_ = (frame_time - first_frame_time_) * 1e6; //since start
    
    float flow_x_ang = 0.0f;
    float flow_y_ang = 0.0f;
    //calculate angular flow
    // int quality = optical_flow_->calcFlow((uchar*)_image, frame_time_us_, dt_us_, flow_x_ang, flow_y_ang);
    int quality = 0;
    if (quality >= 0) { // calcFlow(...) returns -1 if data should not be published yet -> output_rate
        //prepare optical flow message
        // Get the current simulation time.
#if GAZEBO_MAJOR_VERSION >= 9
        common::Time now = world_->SimTime();
#else
        common::Time now = world_->GetSimTime();
#endif
        
        opticalFlow_message.set_time_usec(now.Double() * 1e6);
        opticalFlow_message.set_sensor_id(2.0);
        opticalFlow_message.set_integration_time_us(quality ? dt_us_ : 0);
        opticalFlow_message.set_integrated_x(quality ? flow_x_ang : 0.0f);
        opticalFlow_message.set_integrated_y(quality ? flow_y_ang : 0.0f);
        if (has_gyro_) {
            opticalFlow_message.set_integrated_xgyro(opticalFlow_rate.X());
            opticalFlow_message.set_integrated_ygyro(opticalFlow_rate.Y());
            opticalFlow_message.set_integrated_zgyro(opticalFlow_rate.Z());
            //reset gyro integral
            opticalFlow_rate.Set();
        } else {
            //no gyro
            opticalFlow_message.set_integrated_xgyro(NAN);
            opticalFlow_message.set_integrated_ygyro(NAN);
            opticalFlow_message.set_integrated_zgyro(NAN);
        }
        opticalFlow_message.set_temperature(20.0f);
        opticalFlow_message.set_quality(quality);
        opticalFlow_message.set_time_delta_distance_us(0);
        opticalFlow_message.set_distance(0.0f); //get real values in gazebo_mavlink_interface.cpp
        //send message
        opticalFlow_pub_->Publish(opticalFlow_message);
    }
}

/////////////////////////////////////////////////

void GazeboBlast3DCameraPlugin::OnNewFrameCamera(const unsigned char * _image) {
    
    input_image.create(this->height, this->width, CV_8UC3);
    // convert given frame to opencv image
    input_image.data = (uchar*) _image;
    
    if (!this->has_last_image) {
        this->last_image = input_image.clone();
        this->has_last_image = true;
    }
    
    if (!this->has_last_blast_image) {
        this->last_blast_image_idx = 10;    // blast image start index (index 0 is an empty background image)
        this->last_blast_image = blastGrayImageVec[this->last_blast_image_idx];
        this->has_last_blast_image = true;
    }
        
    if (!pubs_and_subs_created_) {
        CreatePubsAndSubs();
        pubs_and_subs_created_ = true;
    }
    
    // Get the current simulation time.
#if GAZEBO_MAJOR_VERSION >= 9
    common::Time now = world_->SimTime();
    _image = this->camera->ImageData(0);
#else
    common::Time now = world_->GetSimTime();
    _image = this->camera->GetImageData(0);
#endif
    
    double current_time_double = now.Double();    
    if ((now - last_time_).Double() < pub_interval_ || pub_interval_ == 0.0) {
        // update most recent background image for future event calculation even when not publishing the data
        this->last_image = input_image.clone();     
        return;
    }
    last_time_ = now;
    
//    gzdbg << "blastMsgList size = " << blastMsgList.size() << std::endl;
    std::vector<blast3d_msgs::msgs::Blast3d>::iterator msg_iter;
    for (msg_iter = blastMsgList.begin(); msg_iter != blastMsgList.end(); ++msg_iter) {
        if (msg_iter->time() < current_time_double) {
            explosion_triggered_ = true;    // this trigger will be reset to false when a blast is finished
            gzdbg << __FUNCTION__ << "() blending environment and blast data " 
                    << "from blast model plugin for blast at time " 
                    << msg_iter->time() << "." << std::endl;
            blastMsgList.erase(msg_iter);  // for now, only consider one blast at a time
            break;
        }
    }
    
    physics::LinkPtr currentModelLinkPtr = NULL;
    gazebo::physics::ModelPtr currentModelPtr = NULL;
    currentModelPtr = world_->ModelByName("iris");
    currentModelLinkPtr = currentModelPtr->GetLink("/base_link");
//    gzdbg << "camera sensor pose = " << currentModelLinkPtr->WorldPose() << std::endl; 
    ignition::math::Quaternion rotation = currentModelLinkPtr->WorldPose().Rot();
    ignition::math::Vector3d euler = rotation.Euler();
    double roll = euler.X();
//    gzdbg << "Vehicle roll angle (in radians) = " << roll << std::endl;
    
    if (camera_mode_ == "event") {
        blendEventOutput(roll);
    }
    else if (camera_mode_ == "RGB") {
        blendRGBOutput();
    }
    else {
        gzwarn << "Not a valid camera mode.";
    }
    this->last_image = input_image.clone();  // update most recent background image for future event calculation
}

void GazeboBlast3DCameraPlugin::blendEventOutput(double roll){
    // color to grayscale
    if (this->last_image.channels() == 3) {     // needs to do the conversion when the last image is assigned the first time
        cv::Mat last_image(this->height, this->width, CV_8UC1);
        cv::cvtColor(this->last_image, last_image, cv::COLOR_RGB2GRAY);
        this->last_image = last_image;
    }
    cv::Mat curr_image(this->height, this->width, CV_8UC1);
    cv::cvtColor(input_image, curr_image, cv::COLOR_RGB2GRAY);
    
    cv::Mat curr_blast_image;
    cv::Mat curr_blast_alpha;

    std::vector<sensor_msgs::msgs::Event> events;

    if (!explosion_triggered_) {
        this->processDelta(this->last_image, curr_image, this->last_blast_image, curr_blast_image, curr_blast_alpha, events);
        this->last_image = curr_image;
    }
    else {
        if (this->has_last_blast_image && blastGrayImageVec.size() > 1) {
            if (this->last_blast_image.size() != this->last_image.size()) {
                cv::resize(this->last_blast_image, this->last_blast_image, cv::Size(this->last_image.cols, this->last_image.rows));
            }
            curr_blast_image = blastGrayImageVec[this->last_blast_image_idx + 1];
            curr_blast_alpha = blastImageAlphaVec[this->last_blast_image_idx + 1];
            if (curr_blast_image.size() != this->last_image.size()) {
                cv::resize(curr_blast_image, curr_blast_image, this->last_image.size());
                cv::resize(curr_blast_alpha, curr_blast_alpha, cv::Size(this->last_image.cols, this->last_image.rows));
            }
            this->processDelta(this->last_image, curr_image, this->last_blast_image, curr_blast_image, curr_blast_alpha, events, roll, explosion_triggered_);
            this->last_blast_image_idx += 1;
            this->last_blast_image = curr_blast_image;
            this->last_image = curr_image;
            // reset when it exceeds the file range 
            if (this->last_blast_image_idx >= 30) { //(blastGrayImageVec.size() - 1)) {  // blast image end index (controls the duration of the blast)
                this->has_last_blast_image = false;
                explosion_triggered_ = false;   // when a blast is finished, reset the trigger
            }
        } 
    }
    
    PublishEventMessage(events);
}

/////////////////////////////////////////////////

void GazeboBlast3DCameraPlugin::PublishEventMessage(std::vector<sensor_msgs::msgs::Event> events) {
    // Get the current simulation time.
#if GAZEBO_MAJOR_VERSION >= 9
    common::Time now = world_->SimTime();
#else
    common::Time now = world_->GetSimTime();
#endif
    eventCameraEventArray_message.mutable_header()->mutable_stamp()->set_sec(
                now.Double());
    eventCameraEventArray_message.mutable_header()->mutable_stamp()->set_nsec(
                now.Double() * 1e9);
    eventCameraEventArray_message.mutable_header()->set_frame_id("drone");
    eventCameraEventArray_message.set_width(this->width); 
    eventCameraEventArray_message.set_height(this->height);
    for (auto& event_msg : events) {
    //        sensor_msgs::msgs::Event* event_msg = new sensor_msgs::msgs::Event();
        eventCameraEventArray_message.mutable_events()->AddAllocated(new sensor_msgs::msgs::Event(event_msg));
    }
    eventCamera_pub_->Publish(eventCameraEventArray_message);
}

/////////////////////////////////////////////////

void GazeboBlast3DCameraPlugin::blendRGBOutput(){
    cv::Mat curr_image = input_image;
    cv::Mat curr_blast_image;
    cv::Mat curr_blast_alpha;

    // when there is no explosion, the output image is the scene image
    cv::Mat blend_image = curr_image;
    
    if (explosion_triggered_) {
        if (this->has_last_blast_image && blastGrayImageVec.size() > 1) {
            curr_blast_image = blastRGBImageVec[this->last_blast_image_idx];
            curr_blast_alpha = blastImageAlphaVec[this->last_blast_image_idx];
            if (curr_blast_image.size() != blend_image.size()) {
                cv::resize(curr_blast_image, curr_blast_image, blend_image.size());
                cv::resize(curr_blast_alpha, curr_blast_alpha, blend_image.size());
            }
            // overlay blast image onto the scene image
            curr_blast_image.copyTo(blend_image, curr_blast_alpha);
            // For debug
//            std::cout << "alpha value = " << static_cast<int>(zeroMat.at<uchar>(25, 25)) << std::endl;
//            std::cout << "blend image channel = " << blend_image.channels() << std::endl;
//            std::cout << "curr_blast_image channel = " << curr_blast_image.channels() << std::endl;
            this->last_blast_image_idx += 1;
            this->last_blast_image = curr_blast_image;
            // reset when it exceeds the file range (currently having 61 images)
            if (this->last_blast_image_idx >= 30) { //(blastGrayImageVec.size() - 1)) {  // blast image end index (controls the duration of the blast)
                this->has_last_blast_image = false;
                explosion_triggered_ = false;   // when a blast is finished, reset the trigger
            }
        } 
    }
    
    PublishRGBMessage(blend_image);
}

/////////////////////////////////////////////////

void GazeboBlast3DCameraPlugin::PublishRGBMessage(cv::Mat image) {
    rgbCamera_message.set_width(this->width);
    rgbCamera_message.set_height(this->height);
    rgbCamera_message.set_pixel_format(2);  // RGB8: 2; Grayscale: 1
    rgbCamera_message.set_step(this->width * this->height * this->depth);
    // Convert OpenCV Mat to Gazebo message data
    const uchar* data = image.data;
    size_t dataSize = image.step[0] * image.rows;
    rgbCamera_message.set_data(data, dataSize);

    rgbCamera_pub_->Publish(rgbCamera_message);

    // For debug
    cv::imshow("blended RGB image", image);
    cv::waitKey(30);
}

/////////////////////////////////////////////////

void GazeboBlast3DCameraPlugin::processDelta(cv::Mat &last_image, cv::Mat &curr_image, 
        cv::Mat &last_blast_image, cv::Mat &curr_blast_image, cv::Mat &curr_blast_alpha,
        std::vector<sensor_msgs::msgs::Event> &events, double roll,
        bool explosion) {
    if (curr_image.size() == last_image.size()) {
        
        // camera image
        cv::Mat pos_diff = curr_image - last_image;
        cv::Mat neg_diff = last_image - curr_image;

        cv::Mat pos_mask;
        cv::Mat neg_mask;

        cv::threshold(pos_diff, pos_mask, event_threshold, 255, cv::THRESH_BINARY);
        cv::threshold(neg_diff, neg_mask, event_threshold, 255, cv::THRESH_BINARY);

        last_image += pos_mask & pos_diff;
        last_image -= neg_mask & neg_diff;
        
        // blast image
        if (explosion) {
            cv::Mat blast_pos_diff = curr_blast_image - last_blast_image;
            cv::Mat blast_neg_diff = last_blast_image - curr_blast_image;

            cv::Mat blast_pos_mask;
            cv::Mat blast_neg_mask;

            cv::threshold(blast_pos_diff, blast_pos_mask, event_threshold, 255, cv::THRESH_BINARY);
            cv::threshold(blast_neg_diff, blast_neg_mask, event_threshold, 255, cv::THRESH_BINARY);
            
            // TODO: requires double-check
            last_blast_image += blast_pos_mask & blast_pos_diff;
            last_blast_image -= blast_neg_mask & blast_neg_diff;
            
            cv::Point2f center(blast_pos_mask.cols / 2.0, blast_pos_mask.rows / 2.0);
            cv::Mat rot = cv::getRotationMatrix2D(center, roll / CV_PI * 180, 1.0);
            // rotate event mask image
            cv::Mat blast_pos_mask_rot = cv::Mat::zeros(blast_pos_mask.rows, blast_pos_mask.cols, blast_pos_mask.type());
            cv::Mat blast_neg_mask_rot = cv::Mat::zeros(blast_neg_mask.rows, blast_neg_mask.cols, blast_neg_mask.type());
            cv::warpAffine(blast_pos_mask, blast_pos_mask_rot, rot, blast_pos_mask_rot.size());
            cv::warpAffine(blast_neg_mask, blast_neg_mask_rot, rot, blast_neg_mask_rot.size());
            // rotate alpha channel image
            cv::Mat curr_blast_alpha_rot = cv::Mat::zeros(curr_blast_alpha.rows, curr_blast_alpha.cols, curr_blast_alpha.type());
            cv::warpAffine(curr_blast_alpha, curr_blast_alpha_rot, rot, curr_blast_alpha_rot.size());
    
            // Alpha blend the blast image with transparent background into the scene image
            blast_pos_mask_rot.copyTo(pos_mask, curr_blast_alpha_rot);
            blast_neg_mask_rot.copyTo(neg_mask, curr_blast_alpha_rot);
        }
        
        this->fillEvents(pos_mask, 0, events);
        this->fillEvents(neg_mask, 1, events);
        
        // For debugging
        cv::Mat eventVis(pos_mask.size(), CV_8UC3, cv::Scalar(0, 0, 0));
        std::vector<cv::Mat> channels(3);
        cv::split(eventVis, channels);
        channels[0] = neg_mask * 255;
        channels[2] = pos_mask * 255; 
        // modify channel// then merge
        cv::merge(channels, eventVis);
        cv::imshow("event image", eventVis);
        cv::waitKey(30);
    } else {
        gzwarn << "Unexpected change in image size (" << last_image.size() << " -> " << curr_image.size() << "). Publishing no events for this frame change." << endl;
    }
}

/////////////////////////////////////////////////

void GazeboBlast3DCameraPlugin::fillEvents(cv::Mat &mask, int polarity, std::vector<sensor_msgs::msgs::Event> &events) {
    // TODO: findNonZero fails when there are no zeros
    // Get the current simulation time.
#if GAZEBO_MAJOR_VERSION >= 9
    common::Time now = world_->SimTime();
#else
    common::Time now = world_->GetSimTime();
#endif
    if (cv::countNonZero(mask) != 0) {
        std::vector<cv::Point> locs;
        cv::findNonZero(mask, locs);

        for (int i = 0; i < locs.size(); i++) {
            sensor_msgs::msgs::Event event;
            event.set_x(locs[i].x);
            event.set_y(locs[i].y);
            event.set_time(now.Double() * 1e6);
            event.set_polarity(polarity);
            events.push_back(event);
        }
    }
}

/////////////////////////////////////////////////

void GazeboBlast3DCameraPlugin::ImuCallback(ConstIMUPtr& _imu) {
    //accumulate gyro measurements that are needed for the optical flow message
#if GAZEBO_MAJOR_VERSION >= 9
    common::Time now = world_->SimTime();
#else
    common::Time now = world_->GetSimTime();
#endif

    uint32_t now_us = now.Double() * 1e6;
    ignition::math::Vector3d px4flow_gyro = ignition::math::Vector3d(_imu->angular_velocity().x(),
            _imu->angular_velocity().y(),
            _imu->angular_velocity().z());

    static uint32_t last_dt_us = now_us;
    uint32_t dt_us = now_us - last_dt_us;

    if (dt_us > 1000) {
        opticalFlow_rate += px4flow_gyro * (dt_us / 1000000.0f);
        last_dt_us = now_us;
    }
}

/////////////////////////////////////////////////

void GazeboBlast3DCameraPlugin::Blast3DCallback(Blast3dMsgPtr & blast3d_msg) {
    if (kPrintOnMsgCallback) {
        gzdbg << __FUNCTION__ << "() blast message received by camera plugin." << std::endl;
    }
    blast3d_msgs::msgs::Blast3d msg_copy;
    msg_copy.set_x(blast3d_msg->x());
    msg_copy.set_y(blast3d_msg->y());
    msg_copy.set_z(blast3d_msg->z());
    msg_copy.set_weight_tnt_kg(blast3d_msg->weight_tnt_kg());
    msg_copy.set_time(blast3d_msg->time());
    //blastMsgQueue.push_back(*blast3d_msg);
    blastMsgList.push_back(msg_copy);
}

/////////////////////////////////////////////////

void GazeboBlast3DCameraPlugin::CreatePubsAndSubs() {
    // Gazebo publishers and subscribers

    // ======================================= //
    // ====== BLAST3D SERVER MSG SETUP ======= //
    // ======================================= //
    blast3d_server_register_pub_ = node_handle_->Advertise<blast3d_msgs::msgs::Blast3dServerRegistration>(
            blast3d_server_reglink_topic_, 1);
    blast3d_server_msg_sub_ = node_handle_->Subscribe<blast3d_msgs::msgs::Blast3d>(blast3d_server_link_topic_,
            &GazeboBlast3DCameraPlugin::Blast3DCallback, this);
    
    // Register this plugin with the world dynamic wind server
//    blast3d_msgs::msgs::Blast3dServerRegistration register_msg;
//    register_msg.set_link_name("/base_link");
//    register_msg.set_model_name("iris");
//    register_msg.set_namespace_(namespace_);
//    register_msg.set_link_wind_topic(blast3d_server_link_topic_);
//    blast3d_server_register_pub_->Publish(register_msg);
    gzdbg << __FUNCTION__ << "() camera plugin registering to world blast plugin server on topic " <<
            blast3d_server_reglink_topic_ << "." << std::endl;

}
/* vim: set et fenc=utf-8 ff=unix sts=0 sw=2 ts=2 : */