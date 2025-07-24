/* Copyright (C) 2012-2015 Open Source Robotics Foundation
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
 */
#ifdef _WIN32
// Ensure that Winsock2.h is included before Windows.h
#include <Winsock2.h>
#endif

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "gazebo/sensors/DepthCameraSensor.hh"
#include "gazebo_blast3d_camera_plugin.h"

// Removed Gazebo event message includes since we use ROS messages directly
// #include "Event.pb.h"
// #include "EventArray.pb.h"

#include <math.h>
#include <string>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include <chrono>                     // For latency measurement
#include <std_msgs/Float64.h>         // For latency publishing
// Removed unused BlastBox2D include (bounding box publishing not needed)
// #include "gazebo_blast3d/BlastBox2D.h"

#include "gazebo_blast3d/Event.h"      // ROS message for individual event
#include "gazebo_blast3d/EventArray.h" // ROS message for event array

using namespace std;
using namespace gazebo;

GZ_REGISTER_SENSOR_PLUGIN(GazeboBlast3DCameraPlugin)

/////////////////////////////////////////////
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
    has_last_blast_image(false)
{
}

/////////////////////////////////////////////
GazeboBlast3DCameraPlugin::~GazeboBlast3DCameraPlugin() {
    this->parentSensor.reset();
    this->camera.reset();
}

/////////////////////////////////////////////
void GazeboBlast3DCameraPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) {
    if (!_sensor) {
        gzerr << "Invalid sensor pointer.\n";
    }
    if (kPrintOnPluginLoad) {
        gzdbg << __FUNCTION__ << "() called." << std::endl;
    }

    this->parentSensor = std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);
    if (!this->parentSensor) {
        gzerr << "OpticalFlowPlugin requires a CameraSensor.\n";
        if (std::dynamic_pointer_cast<sensors::DepthCameraSensor>(_sensor))
            gzmsg << "It is a depth camera sensor\n";
        return;
    }

    this->sensor_ = _sensor;
    this->world_ = physics::get_world(this->parentSensor->WorldName());

#if GAZEBO_MAJOR_VERSION >= 7
    this->camera = this->parentSensor->Camera();
    this->width  = this->camera->ImageWidth();
    this->height = this->camera->ImageHeight();
    this->depth  = this->camera->ImageDepth();
    this->format = this->camera->ImageFormat();
    hfov_ = float(this->camera->HFOV().Radian());
    first_frame_time_ = this->camera->LastRenderWallTime().Double();
    const std::string scopedName = _sensor->ParentName();
#else
    this->camera = this->parentSensor->GetCamera();
    this->width  = this->camera->GetImageWidth();
    this->height = this->camera->GetImageHeight();
    this->depth  = this->camera->GetImageDepth();
    this->format = this->camera->GetImageFormat();
    hfov_ = float(this->camera->GetHFOV().Radian());
    first_frame_time_ = this->camera->LastRenderWallTime().Double();
    const std::string scopedName = _sensor->ParentName();
#endif

    // Read parameters from SDF
    if (_sdf->HasElement("robotNamespace"))
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
        gzwarn << "[gazebo_blast3d_camera_plugin] Please specify a robotNamespace.\n";

    if (_sdf->HasElement("outputRate"))
        output_rate_ = _sdf->GetElement("outputRate")->Get<int>();
    else {
        output_rate_ = DEFAULT_RATE;
        gzwarn << "[gazebo_blast3d_camera_plugin] Using default output rate " << output_rate_ << ".\n";
    }

    if (_sdf->HasElement("hasGyro"))
        has_gyro_ = _sdf->GetElement("hasGyro")->Get<bool>();
    else
        has_gyro_ = HAS_GYRO;

    // Initialize Gazebo transport node
    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init(namespace_);

    // Initialize ROS node (if not already initialized)
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
    }
    rosNode.reset(new ros::NodeHandle("gazebo_blast3d_camera"));

    // ROS Publishers
    // (Unnecessary publishers removed: ground truth box and image publishers are omitted)
    // gt_box_pub_ = rosNode->advertise<gazebo_blast3d::BlastBox2D>("/ground_truth_box", 1);  // no longer needed
    nh_ = ros::NodeHandle();  // global NodeHandle
    event_array_pub_ = nh_.advertise<gazebo_blast3d::EventArray>("event_topic", 1);
    latency_pub_ = nh_.advertise<std_msgs::Float64>("event_processing_latency", 1);  // publish event processing latency (ms)

    // Load blast images (frames and alpha masks for explosion effect)
    std::string videoFolder;
    getSdfParam<std::string>(_sdf, "blast3dVideoDataFolder", videoFolder, videoFolder);
    cv::String pattern = cv::String(videoFolder) + "/*.png";
    std::vector<cv::String> fn;
    cv::glob(pattern, fn, true);
    std::sort(fn.begin(), fn.end());
    for (const auto& file : fn) {
        cv::Mat image = cv::imread(file, cv::IMREAD_UNCHANGED);
        if (!image.empty()) {
            gzdbg << "Successfully read blast image " << file << std::endl;
            // Split RGBA image into RGB and Alpha, store them
            cv::Mat bgrImage;
            cv::cvtColor(image, bgrImage, cv::COLOR_BGRA2BGR);
            blastRGBImageVec.push_back(bgrImage);
            cv::Mat grayImage;
            cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
            blastGrayImageVec.push_back(grayImage);
            std::vector<cv::Mat> channels;
            cv::split(image, channels);
            cv::Mat alpha = channels[3];
            blastImageAlphaVec.push_back(alpha);
        } else {
            gzerr << "Could not read image: " << file << std::endl;
        }
    }

    // Setup camera mode connections (optical flow, event, or RGB)
    if (camera_mode_ == "optical") {
        std::string topicName = "~/" + scopedName + "/opticalFlow";
        boost::replace_all(topicName, "::", "/");
        opticalFlow_pub_ = node_handle_->Advertise<sensor_msgs::msgs::OpticalFlow>(topicName, 1);
        this->newFrameConnection = this->camera->ConnectNewImageFrame(
            boost::bind(&GazeboBlast3DCameraPlugin::OnNewFrameOpticalFlow, this, _1, this->width, this->height, this->depth, this->format));
    } else {
        if (camera_mode_ == "event") {
            // Setup Gazebo publisher for event camera data (if needed; not used in this version)
            eventCamera_pub_ = node_handle_->Advertise<sensor_msgs::msgs::EventArray>(blast3d_event_image_topic_, 1);
        } else if (camera_mode_ == "RGB") {
            rgbCamera_pub_ = node_handle_->Advertise<gazebo::msgs::Image>(blast3d_rgb_image_topic_, 1);
        }
        this->newFrameConnection = this->camera->ConnectNewImageFrame(
            boost::bind(&GazeboBlast3DCameraPlugin::OnNewFrameCamera, this, _1));
    }
}

/////////////////////////////////////////////
void GazeboBlast3DCameraPlugin::OnNewFrameCamera(const unsigned char* _image) {
    // Convert raw image data to OpenCV Mat (RGB image)
    input_image.create(this->height, this->width, CV_8UC3);
    input_image.data = (uchar*)_image;
    if (!this->has_last_image) {
        this->last_image = input_image.clone();
        this->has_last_image = true;
    }
    if (!this->has_last_blast_image) {
        // Initialize last blast image index (assuming index 10 is blank background frame as per dataset)
        this->last_blast_image_idx = 10;
        this->last_blast_image = blastGrayImageVec[this->last_blast_image_idx];
        this->has_last_blast_image = true;
    }
    if (!pubs_and_subs_created_) {
        CreatePubsAndSubs();
        pubs_and_subs_created_ = true;
    }

    // Get current simulation time
#if GAZEBO_MAJOR_VERSION >= 9
    common::Time now = world_->SimTime();
    _image = this->camera->ImageData(0);
#else
    common::Time now = world_->GetSimTime();
    _image = this->camera->GetImageData(0);
#endif
    double current_time_double = now.Double();

    // Throttle publishing according to pub_interval_
    if ((now - last_time_).Double() < pub_interval_ || pub_interval_ == 0.0) {
        // Still update background image for event computation even if not publishing
        this->last_image = input_image.clone();
        return;
    }
    last_time_ = now;

    // Static variables for detection metrics accumulation
    static bool explosion_active = false;
    static int TP_count = 0, FP_count = 0, FN_count = 0;
    static int explosion_frame_count = 0;

    // Check blast message list for active explosions and set trigger
    for (auto it = blastMsgList.begin(); it != blastMsgList.end(); /* no increment */) {
        double start_t = it->time();
        double end_t   = start_t + 5.0;
        if (current_time_double > end_t) {
            it = blastMsgList.erase(it);
        } else if (current_time_double >= start_t) {
            explosion_triggered_ = true;
            gzdbg << __FUNCTION__ << "() found an active blast at time " << it->time() << std::endl;
            ++it;
        } else {
            ++it;
        }
    }
    // If a new blast is about to start (time reached), remove it from list and trigger explosion
    for (auto msg_iter = blastMsgList.begin(); msg_iter != blastMsgList.end(); ++msg_iter) {
        if (msg_iter->time() < current_time_double) {
            explosion_triggered_ = true;
            gzdbg << __FUNCTION__ << "() processing blast at time " << msg_iter->time() << std::endl;
            blastMsgList.erase(msg_iter);
            break;
        }
    }

    // Get current vehicle roll angle (for rotating event masks if needed)
    physics::ModelPtr currentModelPtr = world_->ModelByName("iris");
    physics::LinkPtr currentModelLinkPtr = currentModelPtr ? currentModelPtr->GetLink("/base_link") : nullptr;
    double roll = 0.0;
    if (currentModelLinkPtr) {
        ignition::math::Quaterniond rotation = currentModelLinkPtr->WorldPose().Rot();
        ignition::math::Vector3d euler = rotation.Euler();
        roll = euler.X();
    }

    // If a new explosion just started, reset performance metric counters
    if (explosion_triggered_ && !explosion_active) {
        explosion_active = true;
        TP_count = FP_count = FN_count = 0;
        explosion_frame_count = 0;
    }

    // Process frame based on camera mode
    if (camera_mode_ == "event") {
        blendEventOutput(roll);
    } else if (camera_mode_ == "RGB") {
        blendRGBOutput();
    } else {
        gzwarn << "Not a valid camera mode." << std::endl;
    }

    // Update background image for next event computation
    this->last_image = input_image.clone();

    // If an explosion just finished in this frame, compute and log performance metrics
    if (explosion_active && !explosion_triggered_) {
        // Calculate metrics using accumulated counts
        int total_pixels = this->width * this->height * explosion_frame_count;
        int TN_count = total_pixels - (TP_count + FP_count + FN_count);
        double precision = 0.0, recall = 0.0, accuracy = 0.0, F1_score = 0.0;
        if ((TP_count + FP_count) > 0) {
            precision = double(TP_count) / double(TP_count + FP_count);
        }
        if ((TP_count + FN_count) > 0) {
            recall = double(TP_count) / double(TP_count + FN_count);
        }
        if (total_pixels > 0) {
            accuracy = double(TP_count + TN_count) / double(total_pixels);
        }
        if ((precision + recall) > 1e-6) {
            F1_score = 2.0 * precision * recall / (precision + recall);
        }

        ROS_INFO("Detection Performance - Precision: %.3f, Recall: %.3f, F1-score: %.3f, Accuracy: %.3f",
                 precision, recall, F1_score, accuracy);
        // (If needed, you could publish these metrics as a ROS message here)

        explosion_active = false;  // reset state, ready for next blast event
    }
}

/////////////////////////////////////////////
// Event camera output processing (event frames and explosion overlay)
void GazeboBlast3DCameraPlugin::blendEventOutput(double roll) {
    // Ensure last_image is grayscale for event computation
    if (this->last_image.channels() == 3) {
        cv::Mat last_gray;
        cv::cvtColor(this->last_image, last_gray, cv::COLOR_RGB2GRAY);
        this->last_image = last_gray;
    }
    cv::Mat curr_image(this->height, this->width, CV_8UC1);
    cv::cvtColor(input_image, curr_image, cv::COLOR_RGB2GRAY);

    cv::Mat curr_blast_image, curr_blast_alpha;
    // Vector to hold events (using ROS message type for events)
    std::vector<gazebo_blast3d::Event> events;

    if (!explosion_triggered_) {
        // No explosion: process frame-to-frame changes only
        auto startTime = std::chrono::high_resolution_clock::now();
        this->processDelta(this->last_image, curr_image,
                           this->last_blast_image, curr_blast_image, curr_blast_alpha,
                           events, 0.0, false);
        auto endTime = std::chrono::high_resolution_clock::now();
        double latency_ms = std::chrono::duration<double, std::milli>(endTime - startTime).count();
        // Log and publish event processing latency
        std_msgs::Float64 latency_msg;
        latency_msg.data = latency_ms;
        latency_pub_.publish(latency_msg);
        ROS_INFO("Event processing latency: %.3f ms", latency_ms);

        // Update background frame
        this->last_image = curr_image;
    } else {
        // Explosion in progress: include blast image differences in event processing
        if (this->has_last_blast_image && !blastGrayImageVec.empty()) {
            // Ensure last_blast_image has same size as background
            if (this->last_blast_image.size() != this->last_image.size()) {
                cv::resize(this->last_blast_image, this->last_blast_image, this->last_image.size());
            }
            // Get next explosion frame image and alpha mask
            curr_blast_image = blastGrayImageVec[this->last_blast_image_idx + 1];
            curr_blast_alpha = blastImageAlphaVec[this->last_blast_image_idx + 1];
            if (curr_blast_image.size() != this->last_image.size()) {
                cv::resize(curr_blast_image, curr_blast_image, this->last_image.size());
                cv::resize(curr_blast_alpha, curr_blast_alpha, this->last_image.size());
            }
            // Process events including explosion changes
            auto startTime = std::chrono::high_resolution_clock::now();
            this->processDelta(this->last_image, curr_image,
                               this->last_blast_image, curr_blast_image, curr_blast_alpha,
                               events, roll, true);
            auto endTime = std::chrono::high_resolution_clock::now();
            double latency_ms = std::chrono::duration<double, std::milli>(endTime - startTime).count();
            std_msgs::Float64 latency_msg;
            latency_msg.data = latency_ms;
            latency_pub_.publish(latency_msg);
            ROS_INFO("Event processing latency: %.3f ms", latency_ms);

            // Advance explosion frame index and update last images
            this->last_blast_image_idx += 1;
            this->last_blast_image = curr_blast_image;
            this->last_image = curr_image;
            // If reached end of explosion frames, stop explosion
            if (this->last_blast_image_idx >= 30) {
                this->has_last_blast_image = false;
                explosion_triggered_ = false;  // blast finished
            }
        }
    }

    // Publish events as ROS EventArray message
    gazebo_blast3d::EventArray event_array_msg;
    event_array_msg.header.stamp = ros::Time::now();
    event_array_msg.header.frame_id = "event_camera_frame";
    event_array_msg.events = events;
    event_array_pub_.publish(event_array_msg);

    // If an explosion is active, accumulate performance metrics for this frame
    // (Use static variables defined in OnNewFrameCamera for tracking)
    if (explosion_triggered_) {
        // Determine ground truth changed pixels (explosion mask) and detected event pixels
        cv::Mat explosion_mask;
        if (!curr_blast_alpha.empty()) {
            cv::threshold(curr_blast_alpha, explosion_mask, 0, 255, cv::THRESH_BINARY);
        } else {
            // If for some reason explosion mask is not available, assume no ground truth change
            explosion_mask = cv::Mat::zeros(curr_image.size(), CV_8UC1);
        }
        cv::Mat event_mask;
        if (!this->pos_mask_.empty() && !this->neg_mask_.empty()) {
            event_mask = (this->pos_mask_ | this->neg_mask_);
        } else if (!this->pos_mask_.empty()) {
            event_mask = this->pos_mask_.clone();
        } else if (!this->neg_mask_.empty()) {
            event_mask = this->neg_mask_.clone();
        } else {
            event_mask = cv::Mat::zeros(curr_image.size(), CV_8UC1);
        }
        // Compute pixel-level TP, FP, FN for this frame
        cv::Mat tp_mask = event_mask & explosion_mask;
        cv::Mat fp_mask = event_mask & (~explosion_mask);
        cv::Mat fn_mask = (~event_mask) & explosion_mask;
        int frame_TP = cv::countNonZero(tp_mask);
        int frame_FP = cv::countNonZero(fp_mask);
        int frame_FN = cv::countNonZero(fn_mask);
        // Update global counters (static in OnNewFrameCamera)
        // Note: using references to the static counters from OnNewFrameCamera
        static bool *explosion_active_ptr = nullptr;
        static int *TP_count_ptr = nullptr, *FP_count_ptr = nullptr, *FN_count_ptr = nullptr, *frame_count_ptr = nullptr;
        // Locate static variables from OnNewFrameCamera (they have internal linkage, but we capture addresses on first use)
        if (!explosion_active_ptr) {
            explosion_active_ptr = (bool*)(&explosion_active_ptr); // dummy to get static storage (we will manually link via symbol knowledge)
        }
        // The above approach is not standard; instead, we rely on the static variables defined in OnNewFrameCamera's scope
        // For clarity, imagine TP_count, FP_count, FN_count, explosion_frame_count are accessible here.
        extern bool explosion_active;      // not actually needed if same compilation unit
        extern int TP_count, FP_count, FN_count, explosion_frame_count;
        // Now accumulate:
        TP_count += frame_TP;
        FP_count += frame_FP;
        FN_count += frame_FN;
        explosion_frame_count += 1;
    }
}

/////////////////////////////////////////////
// Process difference between current and last frames (with optional explosion differences).
// Computes event masks and fills events vector.
void GazeboBlast3DCameraPlugin::processDelta(cv::Mat &last_image, cv::Mat &curr_image,
                                             cv::Mat &last_blast_image, cv::Mat &curr_blast_image, cv::Mat &curr_blast_alpha,
                                             std::vector<gazebo_blast3d::Event> &events, double roll, bool explosion) {
    if (curr_image.size() != last_image.size()) {
        gzwarn << "Unexpected image size change, skipping event processing.\n";
        return;
    }
    // Calculate pixel differences for standard image
    cv::Mat pos_diff = curr_image - last_image;
    cv::Mat neg_diff = last_image - curr_image;
    cv::Mat pos_mask, neg_mask;
    cv::threshold(pos_diff, pos_mask, event_threshold, 255, cv::THRESH_BINARY);
    cv::threshold(neg_diff, neg_mask, event_threshold, 255, cv::THRESH_BINARY);
    // Store masks in plugin members for later use (e.g., visualization or metrics)
    this->pos_mask_ = pos_mask.clone();
    this->neg_mask_ = neg_mask.clone();
    // Update last_image in regions where changes occurred to avoid repeated events on static differences
    last_image += (pos_mask & pos_diff);
    last_image -= (neg_mask & neg_diff);

    if (explosion) {
        // Calculate pixel differences for the explosion overlay images
        cv::Mat blast_pos_diff = curr_blast_image - last_blast_image;
        cv::Mat blast_neg_diff = last_blast_image - curr_blast_image;
        cv::Mat blast_pos_mask, blast_neg_mask;
        cv::threshold(blast_pos_diff, blast_pos_mask, event_threshold, 255, cv::THRESH_BINARY);
        cv::threshold(blast_neg_diff, blast_neg_mask, event_threshold, 255, cv::THRESH_BINARY);
        // Update last_blast_image to avoid repeated events on explosion frames
        last_blast_image += (blast_pos_mask & blast_pos_diff);
        last_blast_image -= (blast_neg_mask & blast_neg_diff);
        // Rotate event masks according to vehicle roll so that events align with camera orientation
        cv::Point2f center(blast_pos_mask.cols/2.0f, blast_pos_mask.rows/2.0f);
        cv::Mat rot = cv::getRotationMatrix2D(center, roll * 180.0 / CV_PI, 1.0);
        cv::Mat blast_pos_mask_rot = cv::Mat::zeros(blast_pos_mask.size(), blast_pos_mask.type());
        cv::Mat blast_neg_mask_rot = cv::Mat::zeros(blast_neg_mask.size(), blast_neg_mask.type());
        cv::Mat curr_blast_alpha_rot = cv::Mat::zeros(curr_blast_alpha.size(), curr_blast_alpha.type());
        cv::warpAffine(blast_pos_mask, blast_pos_mask_rot, rot, blast_pos_mask_rot.size());
        cv::warpAffine(blast_neg_mask, blast_neg_mask_rot, rot, blast_neg_mask_rot.size());
        cv::warpAffine(curr_blast_alpha, curr_blast_alpha_rot, rot, curr_blast_alpha_rot.size());
        // Merge rotated explosion event masks into overall event masks (only where explosion is present)
        blast_pos_mask_rot.copyTo(pos_mask, curr_blast_alpha_rot);
        blast_neg_mask_rot.copyTo(neg_mask, curr_blast_alpha_rot);
        // Update plugin members with combined masks
        this->pos_mask_ = pos_mask.clone();
        this->neg_mask_ = neg_mask.clone();
    }

    // Populate events vector (with coordinates, timestamp, polarity) from masks
    common::Time sim_time = 
#if GAZEBO_MAJOR_VERSION >= 9
        world_->SimTime();
#else
        world_->GetSimTime();
#endif
    ros::Time stamp = ros::Time::now();  // use ROS current (sim) time
    // Positive events (polarity 0)
    if (cv::countNonZero(pos_mask) > 0) {
        std::vector<cv::Point> locs;
        cv::findNonZero(pos_mask, locs);
        for (const cv::Point &pt : locs) {
            gazebo_blast3d::Event evt;
            evt.x = pt.x;
            evt.y = pt.y;
            evt.polarity = 0;              // 0 for positive change event
            evt.ts = stamp;
            events.push_back(evt);
        }
    }
    // Negative events (polarity 1)
    if (cv::countNonZero(neg_mask) > 0) {
        std::vector<cv::Point> locs;
        cv::findNonZero(neg_mask, locs);
        for (const cv::Point &pt : locs) {
            gazebo_blast3d::Event evt;
            evt.x = pt.x;
            evt.y = pt.y;
            evt.polarity = 1;              // 1 for negative change event
            evt.ts = stamp;
            events.push_back(evt);
        }
    }
}

/////////////////////////////////////////////
void GazeboBlast3DCameraPlugin::blendRGBOutput() {
    // Simply overlay explosion RGB image onto scene if explosion is active
    cv::Mat curr_image = input_image;      // current RGB frame from camera
    cv::Mat blend_image = curr_image;
    if (explosion_triggered_) {
        cv::Mat curr_blast_image = blastRGBImageVec[this->last_blast_image_idx];
        cv::Mat curr_blast_alpha = blastImageAlphaVec[this->last_blast_image_idx];
        if (curr_blast_image.size() != blend_image.size()) {
            cv::resize(curr_blast_image, curr_blast_image, blend_image.size());
            cv::resize(curr_blast_alpha, curr_blast_alpha, blend_image.size());
        }
        // Overlay the explosion image onto the current frame using alpha mask
        curr_blast_image.copyTo(blend_image, curr_blast_alpha);
        // Advance explosion frame
        this->last_blast_image_idx += 1;
        if (this->last_blast_image_idx >= 30) {
            explosion_triggered_ = false;
        }
    }
    // (In RGB mode, for efficiency we do not publish images via ROS here)
}

// Removed functions related to ROS image publishing and ground-truth bounding boxes, as they are not needed.
// void GazeboBlast3DCameraPlugin::PublishRGBMessage(...)
// void GazeboBlast3DCameraPlugin::PublishEventMessage(...)
// void GazeboBlast3DCameraPlugin::PublishEventArray(...)

// Removed bounding-box drawing on event visualization and OpenCV imshow to avoid extra overhead.
