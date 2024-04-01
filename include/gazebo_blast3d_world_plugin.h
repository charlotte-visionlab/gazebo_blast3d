/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Anton Matosov
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef GAZEBO_BLAST3D_WORLD_PLUGIN_H
#define GAZEBO_BLAST3D_WORLD_PLUGIN_H

#include <string>
#include <random>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsTypes.hh>

#include "Blast3d.pb.h"
#include "Blast3dServerRegistration.pb.h"

#include "utils/common.h"

namespace gazebo {
    // Constants and Default Values
    static const bool kPrintOnPluginLoad = true;    
    static constexpr char TEST[] = "/gazebo/default/blast3d_register_link";
    static const std::string kDefaultBlast3dServerRegisterTopic = "/gazebo/default/blast3d_register_link";

    typedef const boost::shared_ptr<const blast3d_msgs::msgs::Blast3dServerRegistration>& Blast3dServerRegistrationPtr;

    /// \brief This gazebo plugin simulates blasts acting on a model.

    class GazeboBlast3DWorldPlugin : public WorldPlugin {
    public:

        GazeboBlast3DWorldPlugin()
        : WorldPlugin(),
        blast3d_server_reglink_topic_(kDefaultBlast3dServerRegisterTopic),
        pub_interval_(0.5),                
        node_handle_(NULL) {
        }

        virtual ~GazeboBlast3DWorldPlugin();

    protected:
        /// \brief Load the plugin.
        /// \param[in] world Pointer to the Gazebo world that loaded this plugin.
        /// \param[in] sdf SDF element that describes the plugin.
        void Load(physics::WorldPtr world, sdf::ElementPtr sdf);

        /// \brief Called when the world is updated.
        /// \param[in] _info Update timing information.
        void OnUpdate(const common::UpdateInfo & /*_info*/);

    private:
        /// \brief    Flag that is set to true once CreatePubsAndSubs() is called, used
        ///           to prevent CreatePubsAndSubs() from be called on every OnUpdate().
//        bool pubs_and_subs_created_;

        /// \brief    Creates all required publishers and subscribers, incl. routing of messages to/from ROS if required.
        /// \details  Call this once the first time OnUpdate() is called (can't
        ///           be called from Load() because there is no guarantee GazeboRosInterfacePlugin has
        ///           has loaded and listening to ConnectGazeboToRosTopic and ConnectRosToGazeboTopic messages).
//        void CreatePubsAndSubs();

        void RegisterLinkCallback(Blast3dServerRegistrationPtr msg);

        /// \brief Pointer to the update event connection.
        event::ConnectionPtr update_connection_;

        physics::WorldPtr world_;

        std::string blast3d_server_reglink_topic_;

        common::Time last_time_;
        double pub_interval_;

        transport::NodePtr node_handle_;

        gazebo::transport::SubscriberPtr blast3d_register_sub_;
        std::vector<transport::PublisherPtr> registered_link_blast3d_publisher_list_;

        std::vector<std::string> registered_link_name_list_;
        std::vector<physics::LinkPtr> registered_link_list_;
        std::vector<physics::ModelPtr> registered_model_list_;
        std::vector<std::string> registered_namespace_list_;
        std::vector<std::string> registered_client_blast3d_topic_list_;

        blast3d_msgs::msgs::Blast3d blast3d_message_;
    };
}

#endif // GAZEBO_BLAST3D_WORLD_PLUGIN_H
