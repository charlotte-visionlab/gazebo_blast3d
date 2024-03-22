#ifndef GAZEBO_BLAST3D_MICROPHONE_PLUGIN_H
#define GAZEBO_BLAST3D_MICROPHONE_PLUGIN_H

#include <string>

#include "gazebo/common/Plugin.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/util/system.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"

#include "Audio.pb.h"

#include <iostream>
#include <ignition/math.hh>

#include "utils/AudioFile.h"
#include "utils/common.h"

using namespace std;

namespace gazebo {
    // Constants and Defaults
    static const bool kPrintOnPluginLoad = true;
    static const std::string kDefaultFrameId = "world";
    static const std::string kDefaultLinkName = "base_link";

    class GAZEBO_VISIBLE GazeboBlast3DMicrophonePlugin : public ModelPlugin {
    public:
        GazeboBlast3DMicrophonePlugin();
        virtual ~GazeboBlast3DMicrophonePlugin();
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        virtual void OnUpdate(const common::UpdateInfo& _info);

    private:
        /// \brief    Pointer to the update event connection.
        event::ConnectionPtr updateConnection_;
        sensor_msgs::msgs::Audio audio_message;
        transport::PublisherPtr audio_pub_;
        transport::NodePtr node_handle_;

        /// \brief    Frame ID for Blast3d messages.
        std::string frame_id_;
        std::string link_name_;

        /// \brief    Pointer to the world.
        physics::WorldPtr world_;

        /// \brief    Pointer to the model.
        physics::ModelPtr model_;

        /// \brief    Pointer to the link.
        physics::LinkPtr link_;

        std::string namespace_;
        std::string blast3d_audio_topic_;
        std::string blast3d_audio_datafolder_;

    };
}
#endif /* GAZEBO_BLAST3D_MICROPHONE_PLUGIN_H */

