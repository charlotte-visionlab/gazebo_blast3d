/*
 * Copyright 2024 Andrew Willis <arwillis@charlotte.edu> UNC Charlotte
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
#include <limits>
#include <random>
#include <stdint.h>

#include "gazebo_blast3d_world_plugin.h"

namespace gazebo {

    GazeboBlast3DWorldPlugin::~GazeboBlast3DWorldPlugin() {
        update_connection_->~Connection();
    }

    void GazeboBlast3DWorldPlugin::Load(physics::WorldPtr world, sdf::ElementPtr sdf) {
        if (kPrintOnPluginLoad) {
            gzdbg << __FUNCTION__ << "() called." << std::endl;
        }
        world_ = world;

        node_handle_ = transport::NodePtr(new transport::Node());
        node_handle_->Init();

        getSdfParam<std::string>(sdf, "blast3dServerRegisterLinkTopic", blast3d_server_reglink_topic_, blast3d_server_reglink_topic_);

        // Blast topic publishing rates
        double pub_rate = 2.0;
        getSdfParam<double>(sdf, "publishRate", pub_rate, pub_rate);
        pub_interval_ = (pub_rate > 0.0) ? 1 / pub_rate : 0.0;

        //CreatePubsAndSubs();
        //pubs_and_subs_created_ = true;

        // Create subscriber to receive blast client requests
        blast3d_register_sub_ = node_handle_->Subscribe<blast3d_msgs::msgs::Blast3dServerRegistration>(blast3d_server_reglink_topic_,
                &GazeboBlast3DWorldPlugin::RegisterLinkCallback, this);
        gzdbg << __FUNCTION__ << "() world plugin serving plugin notification requests published to topic " << 
                blast3d_server_reglink_topic_ << "." << std::endl;
        
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboBlast3DWorldPlugin::OnUpdate, this, _1));

#if GAZEBO_MAJOR_VERSION >= 9
        last_time_ = world_->SimTime();
#else
        last_time_ = world_->GetSimTime();
#endif
    }

    void GazeboBlast3DWorldPlugin::RegisterLinkCallback(Blast3dServerRegistrationPtr msg) {
        const std::string& model_name = msg->model_name();
        const std::string& link_name = msg->link_name();
        const std::string& namespace_val = msg->namespace_();
        const std::string& link_blast3d_topic = msg->link_wind_topic();
        physics::LinkPtr currentModelLinkPtr = NULL;
        gazebo::physics::ModelPtr currentModelPtr = NULL;

        currentModelPtr = world_->ModelByName(model_name);
        if (currentModelPtr == NULL) {
            gzdbg << "[gazebo_wind3d_world_plugin] Model: " << model_name << "does not exist ." << std::endl;
            return;
        }
        currentModelLinkPtr = currentModelPtr->GetLink(link_name);

        if (currentModelLinkPtr != NULL) {
            registered_link_name_list_.push_back(link_name);
            registered_link_list_.push_back(currentModelLinkPtr);
            registered_namespace_list_.push_back(namespace_val);
            registered_client_blast3d_topic_list_.push_back(link_blast3d_topic);
            //transport::PublisherPtr link_blast3d_pub_ = node_handle_->Advertise<blast3d_msgs::msgs::Blast3d>(link_blast3d_topic, 10);
            registered_link_blast3d_publisher_list_.push_back(
                    node_handle_->Advertise<blast3d_msgs::msgs::Blast3d>(link_blast3d_topic, 10));
            gzdbg << __FUNCTION__ << "() Registered (Model, Namespace, Link, Topic) = (" << currentModelPtr->GetName()
                    << ", " << namespace_val << ", " << link_name << ", " << link_blast3d_topic << ") to the world wind server." << std::endl;
        } else {
            gzdbg << __FUNCTION__ << "() Model: " << currentModelPtr->GetName() <<
                    "does not have link " << link_name << "." << std::endl;
        }
    }

    // This gets called by the world update start event.

    void GazeboBlast3DWorldPlugin::OnUpdate(const common::UpdateInfo &_info) {
        // Get the current simulation time.
#if GAZEBO_MAJOR_VERSION >= 9
        common::Time now = world_->SimTime();
#else
        common::Time now = world_->GetSimTime();
#endif
//        if (!pubs_and_subs_created_) {
//            CreatePubsAndSubs();
//            pubs_and_subs_created_ = true;
//        }

        if ((now - last_time_).Double() < pub_interval_ || pub_interval_ == 0.0) {
            return;
        }
        last_time_ = now;
        std::random_device rand_dev;
        std::mt19937 generator(rand_dev());
        std::uniform_real_distribution<double> distrZ1(0.0, 1.0);
        float blast_occurs = distrZ1(generator);
        float blastLikelihood = 0.15;
        if (blast_occurs > blastLikelihood)
            return;
        std::uniform_real_distribution<double> distrFF(5.0, 15.0);
        float weight_TNT_kg = distrFF(generator);
        std::uniform_real_distribution<double> distrTime(1.0, 3.0);
        double futureTime = distrTime(generator);
        std::uniform_real_distribution<double> distr(-20.0, 20.0);
        float x = distr(generator);
        float y = distr(generator);
        float z = distr(generator);
        gzdbg << "PUBLISHED BLAST @ (X,Y,Z)=(" <<
                x << ", " << y << ", " << z << ")" <<
                " futureTime=" << futureTime <<
                " weight_TNT_kg=" << weight_TNT_kg << std::endl;
        double blastTime = futureTime + now.Double();
        blast3d_message_.set_x(x);
        blast3d_message_.set_y(y);
        blast3d_message_.set_z(z);
        blast3d_message_.set_weight_tnt_kg(weight_TNT_kg);
        blast3d_message_.set_time(blastTime);
        for (int index = 0; index < registered_link_blast3d_publisher_list_.size(); index++) {
            registered_link_blast3d_publisher_list_[index++]->Publish(blast3d_message_);
        }
    }

//    void GazeboBlast3DWorldPlugin::CreatePubsAndSubs() {
//        // Create subscriber to receive blast client requests
//        blast3d_register_sub_ = node_handle_->Subscribe<blast3d_msgs::msgs::Blast3dServerRegistration>(blast3d_server_reglink_topic_,
//                &GazeboBlast3DWorldPlugin::RegisterLinkCallback, this);
//
//    }

    GZ_REGISTER_WORLD_PLUGIN(GazeboBlast3DWorldPlugin);
}
