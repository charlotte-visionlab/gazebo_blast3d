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

#include "gazebo_blast3d_model_plugin.h"

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
            gzerr << "[gazebo_blast3d_model_plugin] Please specify a robotNamespace.\n";

        // Get node handle.
        node_handle_ = transport::NodePtr(new transport::Node());

        // Initialize with default namespace (typically /gazebo/default/).
        node_handle_->Init(namespace_);

        if (_sdf->HasElement("linkName"))
            link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
        else
            gzerr << "[gazebo_blast3d_model_plugin] Please specify a linkName.\n";

        // Get the pointer to the link.
        link_ = model_->GetLink(link_name_);
        if (link_ == NULL)
            gzthrow("[gazebo_blast3d_model_plugin] Couldn't find specified link \"" << link_name_ << "\".");

        frame_id_ = link_name_;

        getSdfParam<std::string>(_sdf, "blast3dPressureDataFolder", blast3d_pressure_datafolder_,
                blast3d_pressure_datafolder_);
        getSdfParam<std::string>(_sdf, "blast3dServerRegisterLinkTopic", blast3d_server_reglink_topic_,
                blast3d_server_reglink_topic_);
        getSdfParam<std::string>(_sdf, "blast3dServerLinkTopic", blast3d_server_link_topic_,
                blast3d_server_link_topic_);
        getSdfParam<double>(_sdf, "blastForceTorqueMax", blast_force_torque_max, blast_force_torque_max);
        getSdfParam<double>(_sdf, "blastForceLinearMax", blast_force_linear_max, blast_force_linear_max);

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
        double current_time_double = current_time.Double();

        std::vector<blast3d_msgs::msgs::Blast3d>::iterator msg_iter;
        for (msg_iter = blastMsgList.begin(); msg_iter != blastMsgList.end(); ++msg_iter) {
            if (msg_iter->time() < current_time_double) {
                // DO EFFECT OF BLAST HERE
                ignition::math::Vector3d blastPosRelative(msg_iter->x(), msg_iter->y(), msg_iter->z());
                float distance = blastPosRelative.Length();
                double time_of_arrival = distance / 300.0;
                if (msg_iter->time() + time_of_arrival < current_time_double) {
                    //double time_elapsed_since_blast_start = current_time_double - msg_iter->time();
                    double weight_TNT_kg = msg_iter->weight_tnt_kg();
                    ignition::math::Vector3d linkPos = link_->WorldPose().Pos();
                    ignition::math::Vector3d blastPos = linkPos + blastPosRelative;
                    // FORCE 
                    double r = blastPosRelative.Length();
                    double az = std::atan2(msg_iter->y(), msg_iter->x());   // phi
                    double el = std::acos(msg_iter->z() / r);   // theta
                    double x = sin(az + M_PI) * cos(M_PI - el);
                    double y = sin(az + M_PI) * sin(M_PI - el);
                    double z = cos(az + M_PI);
                    ignition::math::Vector3d dir = {x, y, z};
                    double forceImpulseNs = 2.2116 * std::pow(r, -2.1604) * 2.2116 * weight_TNT_kg + 0.0196;
                    ignition::math::Vector3d forceImpulseOnLink = forceImpulseNs * dir; 
                    // convert the impulses into ~1ms long step functions 
                    ignition::math::Vector3d forceOnLink = forceImpulseOnLink / 0.001;
                    // ignition::math::Vector3d forceOnLink = 1.0e5 * (weight_TNT_kg / (distance * distance)) * (blastPosRelative / blastPosRelative.Length());
                    double forceStrength = forceOnLink.Length();
                    if (forceStrength > blast_force_linear_max) {
                        forceOnLink *= blast_force_linear_max / forceStrength;
                    }
                    // TORQUE
                    std::vector<double> blastCoords = {sin(az) * cos(el), sin(el) * sin(el), cos(az)};
                    // cross product
                    ignition::math::Vector3d momentDir = {blastCoords[1] * forceImpulseOnLink[2] - blastCoords[2] * forceImpulseOnLink[1], 
                                                    blastCoords[2] * forceImpulseOnLink[0] - blastCoords[0] * forceImpulseOnLink[2], 
                                                    blastCoords[0] * forceImpulseOnLink[1] - blastCoords[1] * forceImpulseOnLink[0]};
                    double momentImpulseNms = 4.8894 * std::pow(r, -0.0262) * 4.8894 * weight_TNT_kg + 5.4295;
                    ignition::math::Vector3d momentImpulseOnLink = momentImpulseNms * momentDir; 
                    // convert the impulses into ~100ms long step functions (TODO: MODEL NEEDS TO BE IMPROVED)
                    ignition::math::Vector3d torqueOnLink = momentImpulseOnLink / 0.1;
                    double torqueStrength = torqueOnLink.Length();
                    if (torqueStrength > blast_force_torque_max) {
                        torqueOnLink *= blast_force_torque_max / torqueStrength;
                    }
                    link_->AddForce(forceOnLink);
                    link_->AddTorque(torqueOnLink);
                    gzdbg << __FUNCTION__ << "() exerting force (X,Y,Z)=(" <<
                            forceOnLink.X() << ", " << forceOnLink.Y() << ", " <<
                            forceOnLink.Z() << ") from blast model plugin for blast at time " <<
                            msg_iter->time() << "." << std::endl;
                    gzdbg << __FUNCTION__ << "() exerting torque (X,Y,Z)=(" <<
                            torqueOnLink.X() << ", " << torqueOnLink.Y() << ", " <<
                            torqueOnLink.Z() << ") from blast model plugin for blast at time " <<
                            msg_iter->time() << "." << std::endl;
                    // mark for deletion
                    msg_iter->set_time(-1.0);
                }
            } else {
                //gzdbg << __FUNCTION__ << "() this blast will occur " <<
                //        msg_iter->time() - current_time_double << " seconds from now." << std::endl;
            }
        }
        
        // delete those messages marked to occur at time == -1.0
        blastMsgList.erase(
                std::remove_if(blastMsgList.begin(), blastMsgList.end(),
                [](const blast3d_msgs::msgs::Blast3d & msg) {
                    return msg.time() == -1.0; }),
        blastMsgList.end());
    }

    void GazeboBlast3DModelPlugin::Blast3DCallback(Blast3dMsgPtr & blast3d_msg) {
        if (kPrintOnMsgCallback) {
            gzdbg << __FUNCTION__ << "() blast message received by model plugin." << std::endl;
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

    void GazeboBlast3DModelPlugin::CreatePubsAndSubs() {
        // Gazebo publishers and subscribers

        // ======================================= //
        // ====== BLAST3D SERVER MSG SETUP ======= //
        // ======================================= //
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
        gzdbg << __FUNCTION__ << "() model plugin registering to world blast plugin server on topic " <<
                blast3d_server_reglink_topic_ << "." << std::endl;

    }

    void GazeboBlast3DModelPlugin::ReadBlast3DData(std::string & custom_blastdata_path) {
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