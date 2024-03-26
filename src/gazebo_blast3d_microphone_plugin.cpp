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

#include <math.h>
#include <string>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include <vector>

using namespace std;

namespace gazebo {

    GZ_REGISTER_MODEL_PLUGIN(GazeboBlast3DMicrophonePlugin)

    /////////////////////////////////////////////////
    GazeboBlast3DMicrophonePlugin::GazeboBlast3DMicrophonePlugin() :
    frame_id_(kDefaultFrameId), link_name_(kDefaultLinkName), ModelPlugin(),
            background_audio_index_(0), explosion_triggered_(false) {

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

        getSdfParam<std::string>(_sdf, "audioLinkTopic", blast3d_audio_topic_,
                blast3d_audio_topic_);
        getSdfParam<std::string>(_sdf, "customAudioDataFolder", blast3d_audio_datafolder_,
                blast3d_audio_datafolder_);
        // READ THE AUDIO FILE FOR BACKGROUND
        std::string background_file = blast3d_audio_datafolder_ + "/background_loop.wav";
        // READ THE AUDIO FILE FOR BOOM
        std::string bomb_file = blast3d_audio_datafolder_ + "/bomb.wav";
        
//        bool load (std::string filePath);
        LoadAudioFiles(background_file, bomb_file);
        
//        std::string boom_file = "boom.wav";
//        std::string background_file = "background_loop.wav";
//        AudioFile<float> a;
//        bool loadedOK = a.load(boom_file);

        /** If you hit this assert then the file path above
         probably doesn't refer to a valid audio file */
//        assert(loadedOK);

        //---------------------------------------------------------------
        // 3. Let's apply a gain to every audio sample

//        float gain = 0.5f;
//
//        for (int i = 0; i < a.getNumSamplesPerChannel(); i++) {
//            for (int channel = 0; channel < a.getNumChannels(); channel++) {
//                a.samples[channel][i] = a.samples[channel][i] * gain;
//            }
//        }
        audio_pub_ = node_handle_->Advertise<sensor_msgs::msgs::Audio>(blast3d_audio_topic_, 10);
        updateConnection_ = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&GazeboBlast3DMicrophonePlugin::OnUpdate, this, _1));
    }

    /////////////////////////////////////////////////

    void GazeboBlast3DMicrophonePlugin::OnUpdate(const common::UpdateInfo& _info) {

        // Get the current simulation time.
#if GAZEBO_MAJOR_VERSION >= 9
        common::Time now = world_->SimTime();
#else
        common::Time now = world_->GetSimTime();
#endif
        // COMPUTE TIME INDEX IN BACKGROUND
        // ADD NEXT CHUNK OF BACKGROUND AUDIO
        // Play the background audio.
        
        /** If you hit this assert then the file path above
         probably doesn't refer to a valid audio file */
//        assert (loadedOK);
        
        PublishAudioMessage(background_audio_);
        
        
        // Check for explosion trigger and play bomb audio if triggered.
        if (explosion_triggered_) {
            // IF SEISMIC_BOOM ADD SEISMIC_BOOM AUDIO
//                PublishAudioMessage(bomb_audio_.samples[0], bomb_audio_.getNumChannels(), bomb_audio_.getSampleRate());
    
            // IF AIR_BOOM ADD AIR_BOOM AUDIO
            //audio_message.set_time_usec(now.Double() * 1e6);
            //send message
            //audio_pub_->Publish(audio_message);
//             PublishAudioMessage(bomb_audio_.samples[0], bomb_audio_.getNumChannels(), bomb_audio_.getSampleRate());
         
            explosion_triggered_ = false; // Reset the trigger after playing the bomb audio.
        }
    }
//}
    
    /////////////////////////////////////////////////

    void GazeboBlast3DMicrophonePlugin::LoadAudioFiles(const std::string& background_file, const std::string& bomb_file) {
        
        bool loadedOK = background_audio_.load(background_file);
        if (!loadedOK) {
            gzerr << "Failed to load background audio file: " << background_file << std::endl;
        }

        loadedOK = bomb_audio_.load(bomb_file);
        if (!loadedOK) {
            gzerr << "Failed to load bomb audio file: " << bomb_file << std::endl;
        }
    }

    /////////////////////////////////////////////////

    void GazeboBlast3DMicrophonePlugin::PublishAudioMessage(AudioFile<float> background_audio_) {
        
        sensor_msgs::msgs::Audio audio_msg;
        
        int bitDepth = background_audio_.getBitDepth();
        float sampleRate = background_audio_.getSampleRate();
        std::vector<float> sampleData = background_audio_.samples[0]; // in the channel 0
        int numChannels = background_audio_.getNumChannels();
        
        // Set the header information
//        gz_std_msgs::Header* header = new gz_std_msgs::Header();
//        header->set_frame_id(frame_id_);
//        audio_msg.set_allocated_header(header);
//        audio_msg.set_frame_id(frame_id_);
//        audio_msg.set_samplerate(samplerate);
//        audio_msg.set_channels(num_channels);
//        audio_pub_->Publish(audio_msg);

//        bool load (std::string filePath);
//        // save the audio in AudioFile format into disk for verification
//        audioFile.save (blast3d_audio_datafolder_ + "audioFile.wav", AudioFileFormat::Wave);
//        bool save (std::string filePath, AudioFileFormat format = AudioFileFormat::Wave);
        
        // 1. Let's setup our AudioFile instance
        
        AudioFile<float> a;
        a.setNumChannels (numChannels);
        a.setNumSamplesPerChannel (sampleData.size());
        a.setBitDepth (bitDepth);

        //---------------------------------------------------------------
        // 2. Create some variables to help us generate a sine wave
        
//        const float sampleRate = sampleRate;
//        const float frequencyInHz = sampleRate / 1000.f;
        
        //---------------------------------------------------------------
        // 3. Write the samples to the AudioFile sample buffer
        
        for (int i = 0; i < a.getNumSamplesPerChannel(); i++)
        {
            for (int channel = 0; channel < a.getNumChannels(); channel++)
            {
                a.samples[channel][i] = background_audio_.samples[channel][i] * 2;
            }
        }
        
        std::string filePath = blast3d_audio_datafolder_ + "audioFile.wav"; // change this to somewhere useful for you
        a.save ("audioFile.wav", AudioFileFormat::Wave);
    }

}
/* vim: set et fenc=utf-8 ff=unix sts=0 sw=2 ts=2 : */