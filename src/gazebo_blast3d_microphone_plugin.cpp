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

#include "gazebo_blast3d_microphone_plugin.h"

#include <algorithm>
#include <math.h>
#include <string>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include <vector>
#include <chrono>
#include <ros/ros.h>

#include "sync_utils.h"

//#include "gazebo_blast3d/BlastSync.h"

using namespace std;

namespace gazebo {

    GZ_REGISTER_MODEL_PLUGIN(GazeboBlast3DMicrophonePlugin)

    /////////////////////////////////////////////////
    GazeboBlast3DMicrophonePlugin::GazeboBlast3DMicrophonePlugin() :
    frame_id_(kDefaultFrameId),
    link_name_(kDefaultLinkName),
    blast3d_server_reglink_topic_(kDefaultBlast3dServerRegisterTopic_model),
    blast3d_server_link_topic_(kDefaultNamespace + "/" + kDefaultLinkName + "/" + kDefaultBlast3dTopic),
    pub_interval_(0.1),
    pubs_and_subs_created_(false),
    background_audio_index_(0),
    explosion_triggered_(false),
    ModelPlugin() {
    }

    /////////////////////////////////////////////////

    GazeboBlast3DMicrophonePlugin::~GazeboBlast3DMicrophonePlugin() {
        updateConnection_->~Connection();
    }

    /////////////////////////////////////////////////

    void GazeboBlast3DMicrophonePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
        if (!_model)
            gzerr << "Invalid sensor pointer.\n";
        if (kPrintOnPluginLoad) {
            gzdbg << __FUNCTION__ << "() called." << std::endl;
        }

        // Store the pointer to the model and the world.
        model_ = _model;
        world_ = model_->GetWorld();
        nh_ = ros::NodeHandle("~blast3d_mic");
        sync_pub_ = nh_.advertise<gazebo_blast3d::BlastSync>("blast_sync_log",50,false);
        
        audio_ros_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("blast_audio", 1);
        vehicle_id_ = _sdf->HasElement("vehicleName") ? _sdf->Get<std::string>("vehicleName") : model_->GetName();
        std::unordered_map<double,uint32_t> event_id_map_;
        uint32_t last_eid_ = 0;


        //==============================================//
        //========== READ IN PARAMS FROM SDF ===========//
        //==============================================//

        // Use the robot namespace to create the node handle.
        if (_sdf->HasElement("robotNamespace"))
            namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
        else
            gzerr << "[gazebo_microphone_plugin] Please specify a robotNamespace.\n";

        // Get node handle.
        node_handle_ = transport::NodePtr(new transport::Node());

        // Initialize with default namespace (typically /gazebo/default/).
        node_handle_->Init(namespace_);

        if (_sdf->HasElement("linkName"))
            link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
        else
            gzerr << "[gazebo_microphone_plugin] Please specify a linkName.\n";

        // Get the pointer to the link.
        link_ = model_->GetLink(link_name_);
        if (link_ == NULL)
            gzthrow("[gazebo_microphone_plugin] Couldn't find specified link \"" << link_name_ << "\".");

        frame_id_ = link_name_;

        node_handle_ = transport::NodePtr(new transport::Node());
        node_handle_->Init(namespace_);

        // Blast topic publishing rates
        double pub_rate;
        getSdfParam<double>(_sdf, "publishRate", pub_rate, pub_rate);
        pub_interval_ = (pub_rate > 0.0) ? 1 / pub_rate : 0.0;

        getSdfParam<std::string>(_sdf, "audioLinkTopic", blast3d_audio_topic_,
                blast3d_audio_topic_);
        getSdfParam<std::string>(_sdf, "customAudioDataFolder", blast3d_audio_datafolder_,
                blast3d_audio_datafolder_);
        // READ THE AUDIO FILE FOR BACKGROUND
        std::string background_file = blast3d_audio_datafolder_ + "/background_loop.wav";
        // READ THE AUDIO FILE FOR BOOM
        std::string blast_file = blast3d_audio_datafolder_ + "/bomb.wav";

        bool loadedOK = background_audio_.load(background_file);
        if (!loadedOK) {
            gzerr << "Failed to load background audio file: " << background_file << std::endl;
        }

        loadedOK = blast_audio_.load(blast_file);
        if (!loadedOK) {
            gzerr << "Failed to load blast audio file: " << blast_file << std::endl;
        }
        
        seismicAudio = blast_audio_;
        airBlastAudio = blast_audio_;
        
        pubBitDepth = background_audio_.getBitDepth();
        pubSampleRate = background_audio_.getSampleRate();
        
        // Apply the average filter to the seismic audio
        int windowSize = 6;
        float samplingFreq = seismicAudio.getSampleRate();
        float cutoffFreq = 25; // Hz

        // Apply the average filter with cutoff frequency
        std::vector<float> filteredSignal;
        std::vector<float> seismicSignal = seismicAudio.samples[0];
        filteredSignal = averageFilterWithCutoff(seismicSignal, cutoffFreq, samplingFreq);
        for (int channel = 0; channel < seismicAudio.samples.size(); channel++) {
            seismicAudio.samples[channel] = filteredSignal;
        }
        // For debugging
//        AudioFile<float> a;
//        a.setNumChannels (1);
//        a.setNumSamplesPerChannel (filteredSignal.size());
//        a.setBitDepth (this->pubBitDepth);
//        a.samples[0] = filteredSignal;
//        std::string filename = "filtered_seismic";
//        filename = filename + ".wav";
//        std::string filePath = blast3d_audio_datafolder_ + "/../" + filename; 
//        a.save(filePath, AudioFileFormat::Wave);
        
        // temperature in Kelvin
        double T = 313; // 40 degree Celsius (close to the blast)
        // dynamic viscosity coefficient of the fluid
        // https://en.wikipedia.org/wiki/Viscosity#Dynamic_viscosity
        double etaAir = 2.791 * 0.0000001 * std::pow(T, 0.7355); 
        // angular frequency (JUST AN APPROXIMATION)
        double omegaAir = 500;
        // medium density
        // https://en.wikipedia.org/wiki/Density
        double rhoAir = 1.2; 
        // https://en.wikipedia.org/wiki/Stokes%27s_law_of_sound_attenuation
        airAttenuationCoeff = 2 * etaAir * omegaAir * omegaAir / (3 * rhoAir * 340 * 340 * 340);
        
        
        int NUM_COPIES = 4; // blast future time is randomized from 1 to 3 seconds and blast signal is 0.99 second
        pubBufSize = background_audio_.samples[0].size() * NUM_COPIES;
        output_buffer_background.resize(background_audio_.samples.size());
        output_buffer_pub.resize(background_audio_.samples.size());

        for (int channel = 0; channel < background_audio_.samples.size(); channel++) {
            for (int copyIdx = 0; copyIdx < NUM_COPIES; copyIdx++) {
                std::copy(background_audio_.samples[channel].begin(), background_audio_.samples[channel].end(),
                        std::back_inserter(output_buffer_background[channel]));
                std::copy(background_audio_.samples[channel].begin(), background_audio_.samples[channel].end(),
                        std::back_inserter(output_buffer_pub[channel]));
            }
        }
        
        getSdfParam<std::string>(_sdf, "blast3dServerRegisterLinkTopic", blast3d_server_reglink_topic_,
                blast3d_server_reglink_topic_);
        getSdfParam<std::string>(_sdf, "blast3dServerLinkTopic", blast3d_server_link_topic_,
                blast3d_server_link_topic_);

        audio_pub_ = node_handle_->Advertise<sensor_msgs::msgs::Audio>(blast3d_audio_topic_, 1);
        updateConnection_ = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&GazeboBlast3DMicrophonePlugin::OnUpdate, this, _1));
        
#if GAZEBO_MAJOR_VERSION >= 9
        last_time_ = world_->SimTime();
#else
        last_time_ = world_->GetSimTime();
#endif
    }

    /////////////////////////////////////////////////

    void GazeboBlast3DMicrophonePlugin::OnUpdate(const common::UpdateInfo& _info) {
        
        plotEverySteps++;
        
#if GAZEBO_MAJOR_VERSION >= 9
        common::Time now = world_->SimTime();
#else
        common::Time now = world_->GetSimTime();
#endif
       
        if (!pubs_and_subs_created_) {
            CreatePubsAndSubs();
            pubs_and_subs_created_ = true;
        }

        if ((now - last_time_).Double() < pub_interval_ || pub_interval_ == 0.0) {
            return;
        }
        last_time_ = now;
      
//         Packetize the audio file
        size_t packetSize = std::floor(pub_interval_ * this->pubSampleRate);
        size_t numSamples = output_buffer_pub[0].size(); // Assuming samples[0] contains the audio data for one channel
        size_t packetEnd = std::min(packetSize, numSamples);
        std::vector<float> packet(output_buffer_pub[0].begin(), output_buffer_pub[0].begin() + packetEnd);
        PublishAudioMessage(packet);
        double sim_t = world_->SimTime().Double();
        if (last_eid_ != 0)
            blast3d_sync::publishSyncLog(sync_pub_, "acoustic", last_eid_, now.Double(), vehicle_id_);



        output_buffer_pub[0].erase(output_buffer_pub[0].begin(), output_buffer_pub[0].begin() + packetEnd);
        if (output_buffer_pub[0].size() < this->pubBufSize) {
            std::copy(background_audio_.samples[0].begin(), background_audio_.samples[0].begin() + packetEnd,
                    std::back_inserter(output_buffer_pub[0]));
        }
        
        if (plotEverySteps % 100 == 0) {
        
            AudioFile<float> a;
            a.setNumChannels (1);
            a.setNumSamplesPerChannel (packet.size());
            a.setBitDepth (this->pubBitDepth);
            a.samples[0] = packet;

            std::string filename = "audioPacket.wav";
            std::string filePath = blast3d_audio_datafolder_ + "/../" + filename; 
            a.save(filePath, AudioFileFormat::Wave);
        }
    }

    void GazeboBlast3DMicrophonePlugin::PublishAudioMessage(std::vector<float>& sampleData) {

#if GAZEBO_MAJOR_VERSION >= 9
        common::Time now = world_->SimTime();
#else
        common::Time now = world_->GetSimTime();
#endif

        audio_message.mutable_header()->mutable_stamp()->set_sec(
                now.Double());
        audio_message.mutable_header()->mutable_stamp()->set_nsec(
                now.Double() * 1e9);
        audio_message.mutable_header()->set_frame_id("drone");
        audio_message.set_samplerate(this->pubSampleRate);
        audio_message.add_sampledata(reinterpret_cast<const char*> (sampleData.data()), sampleData.size() * sizeof (float));
        audio_message.set_bitspersample(this->pubBitDepth);

        audio_pub_->Publish(audio_message);
        
        // --- ROS mirror without creating a new msg type ---
        std_msgs::Float32MultiArray ros_msg;
        ros_msg.layout.dim.resize(1);
        ros_msg.layout.dim[0].label = "samples";
        ros_msg.layout.dim[0].size  = sampleData.size();
        ros_msg.layout.dim[0].stride= sampleData.size();
        ros_msg.layout.data_offset  = 0;
        ros_msg.data = sampleData;   // copies floats
        audio_ros_pub_.publish(ros_msg);

        
        // For Debugging, by saving a newly generated audio file with gain:
//        AudioFile<float> a;
//        a.setNumChannels (1);
//        a.setNumSamplesPerChannel (sampleData.size());
//        a.setBitDepth (this->pubBitDepth);
//        a.samples[0] = sampleData;
//            
//        auto nowTime = std::chrono::system_clock::now();
//        auto ns_since_epoch = std::chrono::duration_cast<std::chrono::nanoseconds>(nowTime.time_since_epoch()).count();
//        std::string filename = std::to_string(ns_since_epoch);
//        filename = filename + ".wav";
//        std::string filePath = blast3d_audio_datafolder_ + "/../" + filename; 
//        a.save(filePath, AudioFileFormat::Wave);
    }

    void GazeboBlast3DMicrophonePlugin::Blast3DCallback(Blast3dMsgPtr & blast3d_msg) {
        if (kPrintOnMsgCallback) {
            gzdbg << __FUNCTION__ << "() blast message received by model plugin." << std::endl;
        }

#if GAZEBO_MAJOR_VERSION >= 9
        common::Time now = world_->SimTime();
#else
        common::Time now = world_->GetSimTime();
#endif
        double current_time_double = now.Double();
        
        // COMPUTE INDEX AND BUFFER(S) TO INSERT DATA 
        // DO EFFECT OF BLAST HERE
        gzdbg << "Blast is happening." << std::endl;
        float Q = blast3d_msg->weight_tnt_kg();
        float Ca = 340; //air speed
        float Cs = 6000; //solid speed
         // calculate the time delay of the blast from now. The blast3d_msg->time() is the timestamp from the start time when timestamp is 0.
        double future_time = blast3d_msg->time() - current_time_double;
        if (future_time < 0) {
            gzdbg << "Blast happened in the past time. " << std::endl;
            return;
        }
        ignition::math::Vector3d blastPosRelative(blast3d_msg->x(), blast3d_msg->y(), blast3d_msg->z());
        float free_space_distance = blastPosRelative.Length();
        float ground_distance = std::sqrt(blast3d_msg->x() * blast3d_msg->x() + blast3d_msg->y() * blast3d_msg->y());
        float ground_2_uav_distance = std::abs(blast3d_msg->z());
        double time_of_arrival_free_space = 0.34 * pow(free_space_distance, (1.4)) * pow(Q, (-0.2)) / Ca;
        double avg_speed_free_space = free_space_distance / time_of_arrival_free_space;
        double time_of_arrival_seismic = 0.91 * pow(ground_distance, (1.03)) * pow(Q, (-0.02)) / Cs + ground_2_uav_distance / avg_speed_free_space;

        float backgroundAudioSampleRate = background_audio_.getSampleRate();
        
        uint32_t eid = blast3d_sync::nextEventId();
        event_id_map_[blast3d_msg->time()] = eid;
        last_eid_ = eid;
        
        // Apply sound attenuation based on the distance
        // https://en.wikipedia.org/wiki/Stokes%27s_law_of_sound_attenuation
//        gzdbg << "airAttenuationCoeff = " << airAttenuationCoeff << std::endl;
        double airAttenuationRate = std::exp(-airAttenuationCoeff * free_space_distance);
//        gzdbg << "airAttenuationRate = " << airAttenuationRate << std::endl;
        // Multiply each element of the vector by the rate
        std::transform(airBlastAudio.samples[0].begin(), airBlastAudio.samples[0].end(), airBlastAudio.samples[0].begin(),
                   [airAttenuationRate](float value) { return value * airAttenuationRate; });

        // COMPUTE TIME INDEX IN BACKGROUND
        size_t seismicSignalStartIdx = 0, seismicSignalEndIdx = 0, airSignalStartIdx = 0, airSignalEndIdx = 0;
        seismicSignalStartIdx = backgroundAudioSampleRate * (future_time + time_of_arrival_seismic);
        seismicSignalEndIdx = seismicSignalStartIdx + seismicAudio.samples[0].size();
        if (avg_speed_free_space > Ca) {    
            // when the average speed is less than 340 m/s, the blast is approximately considered to be "too far to be heard"
            airSignalStartIdx = backgroundAudioSampleRate * (future_time + time_of_arrival_free_space);
            airSignalEndIdx = airSignalStartIdx + airBlastAudio.samples[0].size();
        }
        // extend the buffer before adding the blast audios
        size_t maxEndIdx = std::max(seismicSignalEndIdx, airSignalEndIdx);
        if (maxEndIdx > output_buffer_pub[0].size()) {
            if (maxEndIdx - output_buffer_pub[0].size() < background_audio_.samples[0].size()) { // safety check
//                gzdbg << "Increasing the output buffer." << std::endl;
                std::copy(background_audio_.samples[0].begin(), background_audio_.samples[0].begin() + maxEndIdx - output_buffer_pub[0].size(),
                            std::back_inserter(output_buffer_pub[0]));
            }
            else{
                gzwarn << "Blast signal end index exceeds the length of buffer. Increase the buffer size or reduce the future time of the blast." << std::endl;
                return;
            }
        }

        // Insert audio signals
        for (size_t i = 0; i < seismicAudio.samples[0].size(); ++i) {
            output_buffer_pub[0][i + seismicSignalStartIdx] += seismicAudio.samples[0][i];
        }
        gzdbg << "Seismic audio inserted to output buffer." << std::endl;

        if (avg_speed_free_space > Ca) {    // when the average speed is below 340 m/s, the blast is approximately considered to be "too far to be heard"
            for (size_t i = 0; i < blast_audio_.samples[0].size(); ++i) {
                output_buffer_pub[0][i + airSignalStartIdx] += airBlastAudio.samples[0][i];
            }
            gzdbg << "Air blast audio inserted to output buffer." << std::endl;
        }
        else {
            gzdbg << "Air blast audio is too far to be heard due to attenuation." << std::endl;
        }
        
        // For debug
//        int len = this->pubSampleRate * 15;
//        std::vector<float> packet(output_buffer_pub[0].begin(), output_buffer_pub[0].begin() + len);
//        AudioFile<float> a;
//        a.setNumChannels (1);
//        a.setNumSamplesPerChannel (packet.size());
//        a.setBitDepth (this->pubBitDepth);
//        a.samples[0] = packet;
//        auto nowTime = std::chrono::system_clock::now();
//        auto ns_since_epoch = std::chrono::duration_cast<std::chrono::nanoseconds>(nowTime.time_since_epoch()).count();
//        std::string filename = std::to_string(ns_since_epoch);
//        filename = filename + ".wav";
//        std::string filePath = blast3d_audio_datafolder_ + "/../" + filename;
//        a.save(filePath, AudioFileFormat::Wave);
//        
//        output_buffer_pub[0].erase(output_buffer_pub[0].begin(), output_buffer_pub[0].begin() + len);
//        if (output_buffer_pub[0].size() < this->pubBufSize) {
//            int recoverSize = this->pubBufSize - output_buffer_pub[0].size();
//            std::copy(background_audio_.samples[0].begin(), background_audio_.samples[0].begin() + len,
//                    std::back_inserter(output_buffer_pub[0]));
//        }
        }

    void GazeboBlast3DMicrophonePlugin::CreatePubsAndSubs() {
        // Gazebo publishers and subscribers

        // ======================================= //
        // ====== BLAST3D SERVER MSG SETUP ======= //
        // ======================================= //
        blast3d_server_register_pub_ = node_handle_->Advertise<blast3d_msgs::msgs::Blast3dServerRegistration>(
                blast3d_server_reglink_topic_, 1);
        blast3d_server_msg_sub_ = node_handle_->Subscribe<blast3d_msgs::msgs::Blast3d>(blast3d_server_link_topic_,
                &GazeboBlast3DMicrophonePlugin::Blast3DCallback, this);

        // Register this plugin with the world dynamic wind server
//        blast3d_msgs::msgs::Blast3dServerRegistration register_msg;
//        register_msg.set_link_name(link_name_);
//        register_msg.set_model_name(model_->GetName());
//        register_msg.set_namespace_(namespace_);
//        register_msg.set_link_wind_topic(blast3d_server_link_topic_);
//        blast3d_server_register_pub_->Publish(register_msg);
        gzdbg << __FUNCTION__ << "() model plugin registering to world blast plugin server on topic " <<
                blast3d_server_reglink_topic_ << "." << std::endl;

    }
    
    // Function to apply an average filter to a signal
    std::vector<float> GazeboBlast3DMicrophonePlugin::averageFilterWithCutoff(std::vector<float>& signal, float cutoffFreq, float samplingFreq) {
        std::vector<float> filteredSignal;
        filteredSignal.reserve(signal.size());

        // Calculate the window size based on the cutoff frequency
        float windowSize = samplingFreq / (2.0 * cutoffFreq);

        // Ensure the window size is odd
        int windowSizeInt = static_cast<int>(windowSize);
        if (windowSizeInt % 2 == 0) {
            windowSizeInt++; // Make it odd
        }

        // Apply the filter
        for (size_t i = 0; i < signal.size(); ++i) {
            float sum = 0.0;
            int count = 0;
            for (int j = i - windowSizeInt/2; j <= i + windowSizeInt/2; ++j) {
                if (j >= 0 && j < signal.size()) {
                    sum += signal[j];
                    ++count;
                }
            }
            filteredSignal.push_back(sum / count);
        }

        return filteredSignal;
    }
}