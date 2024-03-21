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

using namespace std;

namespace gazebo {

    GZ_REGISTER_SENSOR_PLUGIN(GazeboBlast3DMicrophonePlugin)

    /////////////////////////////////////////////////
    GazeboBlast3DMicrophonePlugin::GazeboBlast3DMicrophonePlugin() : SensorPlugin() {

    }

    /////////////////////////////////////////////////

    GazeboBlast3DMicrophonePlugin::~GazeboBlast3DMicrophonePlugin() {
        updateConnection_->~Connection();
    }

    /////////////////////////////////////////////////

    void GazeboBlast3DMicrophonePlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) {
        if (!_sensor)
            gzerr << "Invalid sensor pointer.\n";

        if (_sdf->HasElement("robotNamespace"))
            namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
        else
            gzwarn << "[gazebo_optical_flow_plugin] Please specify a robotNamespace.\n";

        node_handle_ = transport::NodePtr(new transport::Node());
        node_handle_->Init(namespace_);

        string topicName = "~/audio";
        boost::replace_all(topicName, "::", "/");

        // READ THE AUDIO FILE FOR BACKGROUND
        // READ THE AUDIO FILE FOR BOOM
        std::string boom_file = "boom.wav";
        std::string background_file = "background_loop.wav";
        AudioFile<float> a;
        bool loadedOK = a.load (boom_file);
        
        /** If you hit this assert then the file path above
         probably doesn't refer to a valid audio file */
        assert (loadedOK);
        
        //---------------------------------------------------------------
        // 3. Let's apply a gain to every audio sample
        
        float gain = 0.5f;

        for (int i = 0; i < a.getNumSamplesPerChannel(); i++)
        {
            for (int channel = 0; channel < a.getNumChannels(); channel++)
            {
                a.samples[channel][i] = a.samples[channel][i] * gain;
            }
        }        
        audio_pub_ = node_handle_->Advertise<sensor_msgs::msgs::Audio>(topicName, 10);
        updateConnection_ = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&GazeboBlast3DMicrophonePlugin::OnUpdate, this, _1));
    }

    /////////////////////////////////////////////////

    void GazeboBlast3DMicrophonePlugin::OnUpdate(const common::UpdateInfo& _info) {

        // Get the current simulation time.
//#if GAZEBO_MAJOR_VERSION >= 9
//        common::Time now = world->SimTime();
//#else
//        common::Time now = world->GetSimTime();
//#endif
        // COMPUTE TIME INDEX IN BACKGROUND
        // ADD NEXT CHUNK OF BACKGROUND AUDIO
                
        // IF SEISMIC_BOOM ADD SEISMIC_BOOM AUDIO
        // IF AIR_BOOM ADD AIR_BOOM AUDIO
        
        //audio_message.set_time_usec(now.Double() * 1e6);
        //send message
        audio_pub_->Publish(audio_message);
    }
}
/* vim: set et fenc=utf-8 ff=unix sts=0 sw=2 ts=2 : */