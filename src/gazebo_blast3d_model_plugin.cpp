/*
 * Copyright 2024 Andrew Willis, UNC Charlotte
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

// MODULE HEADER
#include "gazebo_blast3d_model_plugin.h"

// USER HEADERS
//#include "ConnectGazeboToRosTopic.pb.h"

namespace gazebo {

    GazeboBlast3DModelPlugin::~GazeboBlast3DModelPlugin() {
    }

    void GazeboBlast3DModelPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
        if (kPrintOnPluginLoad) {
            gzdbg << __FUNCTION__ << "() called." << std::endl;
        }

        gzdbg << "_model = " << _model->GetName() << std::endl;

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
            gzerr << "[gazebo_anemometer_plugin] Please specify a robotNamespace.\n";

        // Get node handle.
        node_handle_ = transport::NodePtr(new transport::Node());

        // Initialize with default namespace (typically /gazebo/default/).
        node_handle_->Init(namespace_);

        if (_sdf->HasElement("linkName"))
            link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
        else
            gzerr << "[gazebo_anemometer_plugin] Please specify a linkName.\n";

        // Get the pointer to the link.
        link_ = model_->GetLink(link_name_);
        if (link_ == NULL)
            gzthrow("[gazebo_anemometer_plugin] Couldn't find specified link \"" << link_name_ << "\".");

        frame_id_ = link_name_;

        getSdfParam<std::string>(_sdf, "blast3dPressureDataFolder", blast3d_pressure_datafolder_,
                blast3d_pressure_datafolder_);
        getSdfParam<std::string>(_sdf, "blast3dServerRegisterLinkTopic", blast3d_server_reglink_topic_,
                blast3d_server_reglink_topic_);
        getSdfParam<std::string>(_sdf, "blast3dServerLinkTopic", blast3d_server_link_topic_,
                blast3d_server_link_topic_);

        // Check if a custom static wind field should be used.
//        getSdfParam<bool>(sdf, "useCustomBlastData", use_custom_blastdata_,
//                          use_custom_blastdata_);
//
//        if (!use_custom_blastdata_) {
//            gzdbg << "[gazebo_blast3d_world_plugin] Using standard blast models.\n";
//        } else {
//            gzdbg << "[gazebo_blast3d_world_plugin] Using custom blast models from data file.\n";
            // Get the wind field text file path, read it and save data.
//            std::string custom_blastdata_path;
//            getSdfParam<std::string>(sdf, "customBlastDataFile", custom_blastdata_path,
//                                     custom_blastdata_path);
//
//            ReadBlast3DData(custom_blastdata_path);
//        }

        // Listen to the update event. This event is broadcast every simulation
        // iteration.
        updateConnection_ = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&GazeboBlast3DModelPlugin::OnUpdate, this, _1));

    }

    void GazeboBlast3DModelPlugin::OnUpdate(const common::UpdateInfo& _info) {
        if (kPrintOnUpdates) {
            gzdbg << __FUNCTION__ << "() called." << std::endl;
        }

        if (!pubs_and_subs_created_) {
            CreatePubsAndSubs();
            pubs_and_subs_created_ = true;
        }

        common::Time current_time = world_->SimTime();

        // Get the current geometric height.
        //double height_geometric_m = model_->WorldPose().Pos().Z();

        // Compute the anemometer_reading   
        // actual sensor output = wind velocity + vehicle translational velocity +  velocity due to vehicle rotation + noise

    }

    void GazeboBlast3DModelPlugin::Blast3DCallback(Blast3dMsgPtr & blast3d_msg) {
        //if (kPrintOnMsgCallback) {
        //  gzdbg << __FUNCTION__ << "() called." << std::endl;
        //}
        //gzdbg << __FUNCTION__ << "() called." << std::endl;


    }

    void GazeboBlast3DModelPlugin::CreatePubsAndSubs() {
        // Gazebo publishers and subscribers

        // ==================================== //
        // ====== BLAST3D SERVER MSG SETUP ======= //
        // ==================================== //
        blast3d_server_register_pub_ = node_handle_->Advertise<blast3d_msgs::msgs::Blast3dServerRegistration>(
                blast3d_server_reglink_topic_, 1);
        blast3d_server_msg_sub_ = node_handle_->Subscribe<blast3d_msgs::msgs::Blast3d>(blast3d_server_link_topic_,
                &GazeboBlast3DModelPlugin::Blast3DCallback, this);

        // Register this plugin with the world dynamic wind server
        blast3d_msgs::msgs::Blast3dServerRegistration register_msg;
        register_msg.set_link_name(link_name_);
        register_msg.set_model_name(model_->GetName());
        register_msg.set_namespace_(namespace_);
        register_msg.set_link_wind_topic(blast3d_server_link_topic_);
        blast3d_server_register_pub_->Publish(register_msg);

        // ================================================= //
        // ====== ANEMOMETER GAZEBO -> ROS MSG SETUP ======= //
        // ================================================= //

        //        anemometer_pub_ = node_handle_->Advertise<gz_sensor_msgs::msgs::Anemometer>(
        //                "~/" + namespace_ + "/" + anemometer_topic_, 1);

        // Create temporary "ConnectGazeboToRosTopic" publisher and message.
        //        gazebo::transport::PublisherPtr connect_gazebo_to_ros_topic_pub =
        //                node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
        //                "~/" + kConnectGazeboToRosSubtopic, 1);
        //        gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;
        //        connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
        //                anemometer_topic_);
        //        connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
        //                anemometer_topic_);
        //        connect_gazebo_to_ros_topic_msg.set_msgtype(
        //                gz_std_msgs::ConnectGazeboToRosTopic::ANEMOMETER);
        //        connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
        //                true);
    }

    void GazeboBlast3DModelPlugin::ReadBlast3DData(std::string &custom_blastdata_path) {
        std::vector<std::vector<double>> data;
        bool csvReadOK = readCSV(custom_blastdata_path, data);
        if (!csvReadOK) {
            gzerr << __FUNCTION__ << "[gazebo_blast3d_world_plugin] Could not open custom blast data CSV file." << std::endl;
            return;
        }
        gzdbg << "[gazebo_blast3d_world_plugin] Successfully read custom blast data from text file.\n";
    }
    
    GZ_REGISTER_MODEL_PLUGIN(GazeboBlast3DModelPlugin);

} // namespace gazebo