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
GazeboBlast3DCameraPlugin::GazeboBlast3DCameraPlugin()
: SensorPlugin(), width(0), height(0), depth(0), has_last_image(false),  has_last_blast_image(false){

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

    this->world = physics::get_world(this->parentSensor->WorldName());

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

    focal_length_ = (this->width / 2) / tan(hfov_ / 2);

    //    if (this->width != 64 || this->height != 64) {
    //        gzerr << "[gazebo_optical_flow_plugin] Incorrect image size, must by 64 x 64.\n";
    //    }

    if (_sdf->HasElement("robotNamespace"))
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
        gzwarn << "[gazebo_optical_flow_plugin] Please specify a robotNamespace.\n";

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

    if (has_gyro_) {
        if (_sdf->HasElement("hasGyro"))
            gyro_sub_topic_ = _sdf->GetElement("gyroTopic")->Get<std::string>();
        else
            gyro_sub_topic_ = kDefaultGyroTopic;

        string topicName = "~/" + _sensor->ParentName() + gyro_sub_topic_;
        boost::replace_all(topicName, "::", "/");
        imuSub_ = node_handle_->Subscribe(topicName, &GazeboBlast3DCameraPlugin::ImuCallback, this);
    }

    getSdfParam<std::string>(_sdf, "blast3dImageTopic", blast3d_image_topic_,
            blast3d_image_topic_);
    getSdfParam<std::string>(_sdf, "blast3dEventTopic", blast3d_event_topic_,
            blast3d_event_topic_);
    getSdfParam<std::string>(_sdf, "blast3dVideoDataFolder", blast3d_video_datafolder_,
            blast3d_video_datafolder_);

    string topicName;

    if (false) {
        topicName = "~/" + scopedName + "/opticalFlow";
        boost::replace_all(topicName, "::", "/");
        opticalFlow_pub_ = node_handle_->Advertise<sensor_msgs::msgs::OpticalFlow>(topicName, 10);
        this->newFrameConnection = this->camera->ConnectNewImageFrame(
                boost::bind(&GazeboBlast3DCameraPlugin::OnNewFrameOpticalFlow,
                this, _1, this->width, this->height, this->depth, this->format));
    } else {
        topicName = blast3d_image_topic_;
        boost::replace_all(topicName, "::", "/");
//        eventCamera_pub_ = node_handle_->Advertise<sensor_msgs::msgs::Event>(topicName, 10);
//        this->newFrameConnection = this->camera->ConnectNewImageFrame(
//                boost::bind(&GazeboBlast3DCameraPlugin::OnNewFrameEventCamera,
//                this, _1, this->width, this->height, this->depth, this->format));
        eventCamera_pub_ = node_handle_->Advertise<sensor_msgs::msgs::EventArray>(topicName, 10);
        this->newFrameConnection = this->camera->ConnectNewImageFrame(
                boost::bind(&GazeboBlast3DCameraPlugin::OnNewFrameEventCamera,
                this, _1, this->width, this->height, this->depth, this->format, blast3d_video_datafolder_));
    }

    string sensorName = "";
    if (_sdf->HasElement("cameraName"))
        sensorName = _sdf->GetElement("cameraName")->Get<std::string>() + "/";
    else
        gzwarn << "[gazebo_blast3d_camera_plugin] Please specify a cameraName." << endl;

    //    string topicName = "events";
    //    if (_sdf->HasElement("eventsTopicName"))
    //      topicName = _sdf->GetElement("eventsTopicName")->Get<std::string>();

    const string topic = sensorName + topicName;

    if (_sdf->HasElement("eventThreshold"))
        this->event_threshold = _sdf->GetElement("eventThreshold")->Get<float>();
    else
        gzwarn << "[gazebo_blast3d_camera_plugin] Please specify a DVS threshold." << endl;

    //    event_pub_ = node_handle_.advertise<sensor_msgs::msgs::EventArray>(topic, 10, 10.0);


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
        common::Time now = world->SimTime();
#else
        common::Time now = world->GetSimTime();
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

void GazeboBlast3DCameraPlugin::OnNewFrameEventCamera(const unsigned char * _image,
        unsigned int _width,
        unsigned int _height,
        unsigned int _depth,
        const std::string &_format,
        const std::string &blast3d_video_datafolder_) {

    //get data depending on gazebo version
#if GAZEBO_MAJOR_VERSION >= 7
    _image = this->camera->ImageData(0);
    double frame_time = this->camera->LastRenderWallTime().Double();
#else
    _image = this->camera->GetImageData(0);
    double frame_time = this->camera->GetLastRenderWallTime().Double();
#endif

    frame_time_us_ = (frame_time - first_frame_time_) * 1e6; //since start

    //calculate angular flow
    //  int quality = optical_flow_->calcFlow((uchar*)_image, frame_time_us_, dt_us_, flow_x_ang, flow_y_ang);
    int quality = 0;

    // convert given frame to opencv image
    cv::Mat input_image(_height, _width, CV_8UC3);
    input_image.data = (uchar*) _image;

    // color to grayscale
    cv::Mat curr_image_rgb(_height, _width, CV_8UC3);
    cv::cvtColor(input_image, curr_image_rgb, cv::COLOR_RGB2BGR);
    cv::cvtColor(curr_image_rgb, input_image, cv::COLOR_BGR2GRAY);
    //
    cv::Mat curr_image = input_image;
    cv::Mat curr_blast_image;

    assert(_height == height && _width == width);
    std::vector<sensor_msgs::msgs::Event> events;
    
    bool explosion = true;
    if (this->has_last_image) {
        if (!explosion) {
            this->processDelta(&this->last_image, &curr_image, &this->last_blast_image, &curr_blast_image, &events);
        }
        else {
            if (this->has_last_blast_image) {
                int zero_padding_length = 4 - std::to_string(this->last_blast_image_idx).length();
                int curr_blast_image_idx = this->last_blast_image_idx + 1;
                std::string curr_blast_image_name = std::string(zero_padding_length, '0')
                                                            + std::to_string(curr_blast_image_idx);
                std::string curr_blast_image_path = blast3d_video_datafolder_ + "/" + curr_blast_image_name + ".png";
                cv::Mat tmp = cv::imread(curr_blast_image_path, cv::IMREAD_GRAYSCALE);
                
                cv::resize(tmp, curr_blast_image, this->last_image.size());
                this->processDelta(&this->last_image, &curr_image, &this->last_blast_image, &curr_blast_image, &events, explosion);
                this->last_blast_image_idx += 1;
                this->last_blast_image = curr_blast_image;
                // reset last image when it exceeds the file range (currently having 61 images)
                if (this->last_blast_image_idx >= 60) {
                    this->last_blast_image_idx = 10;
                }
            } else {
                std::string last_blast_image_filename = "/0010.png";
                this->last_blast_image_idx = 10;
                cv::Mat blast_image = cv::imread(blast3d_video_datafolder_ + last_blast_image_filename, cv::IMREAD_GRAYSCALE);
                cv::resize(blast_image, this->last_blast_image, this->last_image.size());
                this->has_last_blast_image = true;
            } 
        }
    } else if (curr_image.size().area() > 0) {
        this->last_image = curr_image;
        this->has_last_image = true;
    } else {
        gzwarn << "Ignoring empty image." << endl;
    }

    if (quality >= 0) { // calcFlow(...) returns -1 if data should not be published yet -> output_rate
        //prepare optical flow message
        // Get the current simulation time.
#if GAZEBO_MAJOR_VERSION >= 9
        common::Time now = world->SimTime();
#else
        common::Time now = world->GetSimTime();
#endif

        for (auto& event_msg : events) {
            eventCameraEventArray_message.mutable_header()->mutable_stamp()->set_sec(
                    now.Double());
            eventCameraEventArray_message.mutable_header()->mutable_stamp()->set_nsec(
                    now.Double() * 1e9);
            eventCameraEventArray_message.mutable_header()->set_frame_id("drone");
            eventCameraEventArray_message.set_width(_width); 
            eventCameraEventArray_message.set_height(_height);
    //        sensor_msgs::msgs::Event* event_msg = new sensor_msgs::msgs::Event();
            eventCameraEventArray_message.mutable_events()->AddAllocated(new sensor_msgs::msgs::Event(event_msg));
            //send message
            eventCamera_pub_->Publish(eventCameraEventArray_message);
        }
    }
}

void GazeboBlast3DCameraPlugin::processDelta(cv::Mat *last_image, cv::Mat *curr_image, 
        cv::Mat *last_blast_image, cv::Mat *curr_blast_image,
        std::vector<sensor_msgs::msgs::Event> *events,
        bool explosion) {
    if (curr_image->size() == last_image->size()) {
        
        // camera image
        cv::Mat pos_diff = *curr_image - *last_image;
        cv::Mat neg_diff = *last_image - *curr_image;

        cv::Mat pos_mask;
        cv::Mat neg_mask;

        cv::threshold(pos_diff, pos_mask, event_threshold, 255, cv::THRESH_BINARY);
        cv::threshold(neg_diff, neg_mask, event_threshold, 255, cv::THRESH_BINARY);

        *last_image += pos_mask & pos_diff;
        *last_image -= neg_mask & neg_diff;
        
        cv::imshow("scene image", *curr_image);
        cv::waitKey(200);
        
        
        // blast image
        if (explosion) {
            cv::Mat blast_pos_diff = *curr_blast_image - *last_blast_image;
            cv::Mat blast_neg_diff = *last_blast_image - *curr_blast_image;

            cv::Mat blast_pos_mask;
            cv::Mat blast_neg_mask;

            cv::threshold(blast_pos_diff, blast_pos_mask, event_threshold, 255, cv::THRESH_BINARY);
            cv::threshold(blast_neg_diff, blast_neg_mask, event_threshold, 255, cv::THRESH_BINARY);
            
            // TODO: requires double-check
            *last_blast_image += blast_pos_mask & blast_pos_diff;
            *last_blast_image -= blast_neg_mask & blast_neg_diff;
            
            // update the pos_mask and neg_mask with blast
            cv::bitwise_or(pos_mask, blast_pos_mask, pos_mask);
            cv::bitwise_or(neg_mask, blast_neg_mask, neg_mask);
            
        }
        
        this->fillEvents(&pos_mask, 0, events);
        this->fillEvents(&neg_mask, 1, events);
        
        cv::Mat eventVis(pos_mask.size(), CV_8UC3, cv::Scalar(0, 0, 0));
        cv::Mat ch1, ch2, ch3; // declare three matrices 
        std::vector<cv::Mat> channels(3);
        cv::split(eventVis, channels);
        // get the channels (follow BGR order in OpenCV)
        channels[0] = neg_mask * 255;
        channels[2] = pos_mask * 255; 
        // modify channel// then merge
        cv::merge(channels, eventVis);
        cv::imshow("event image", eventVis);
        cv::waitKey(100);
        
        cv::imshow("blast image", *curr_blast_image);
        cv::waitKey(200);
        
    } else {
        gzwarn << "Unexpected change in image size (" << last_image->size() << " -> " << curr_image->size() << "). Publishing no events for this frame change." << endl;
    }
}

void GazeboBlast3DCameraPlugin::fillEvents(cv::Mat *mask, int polarity, std::vector<sensor_msgs::msgs::Event> *events) {
    // findNonZero fails when there are no zeros
    // TODO is there a better workaround then iterating the binary image twice?
    if (cv::countNonZero(*mask) != 0) {
        std::vector<cv::Point> locs;
        cv::findNonZero(*mask, locs);

        for (int i = 0; i < locs.size(); i++) {
            sensor_msgs::msgs::Event event;
            event.set_x(locs[i].x);
            event.set_y(locs[i].y);
            event.set_time(frame_time_us_);
            event.set_polarity(polarity);
            events->push_back(event);
        }
    }
}

void GazeboBlast3DCameraPlugin::ImuCallback(ConstIMUPtr& _imu) {
    //accumulate gyro measurements that are needed for the optical flow message
#if GAZEBO_MAJOR_VERSION >= 9
    common::Time now = world->SimTime();
#else
    common::Time now = world->GetSimTime();
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
/* vim: set et fenc=utf-8 ff=unix sts=0 sw=2 ts=2 : */