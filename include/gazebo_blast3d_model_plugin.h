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

#ifndef GAZEBO_BLAST3D_MODEL_PLUGIN_H
#define GAZEBO_BLAST3D_MODEL_PLUGIN_H

#include <random>

#include <glog/logging.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

//#include <mav_msgs/default_topics.h>  // This comes from the mav_comm repo

#include "Blast3d.pb.h"
#include "Blast3dServerRegistration.pb.h"
//#include "ConnectGazeboToRosTopic.pb.h"

#include "utils/common.h"

namespace gazebo {

    // Constants
    static const bool kPrintOnPluginLoad = true;
    static const bool kPrintOnUpdates = false;
    static const std::string kDefaultNamespace = "";
    static const std::string kDefaultFrameId = "world";
    static const std::string kDefaultBlast3dServerRegisterTopic_model = "/gazebo/default/blast3d_register_link";
    static const std::string kDefaultBlast3dTopic = "blast3d";
    static const std::string kDefaultLinkName = "base_link";

    static const std::string kConnectGazeboToRosSubtopic = "connect_gazebo_to_ros_subtopic";
    static const std::string kConnectRosToGazeboSubtopic = "connect_ros_to_gazebo_subtopic";

    typedef const boost::shared_ptr<const blast3d_msgs::msgs::Blast3d>& Blast3dMsgPtr;

    class GazeboBlast3DModelPlugin : public ModelPlugin {
    public:
        /// \brief    Constructor.

        GazeboBlast3DModelPlugin()
        : ModelPlugin(),
        namespace_(kDefaultNamespace),
        blast3d_server_reglink_topic_(kDefaultBlast3dServerRegisterTopic_model),
        blast3d_server_link_topic_(kDefaultNamespace + "/" + kDefaultLinkName + "/" + kDefaultBlast3dTopic),
        frame_id_(kDefaultFrameId),
        link_name_(kDefaultLinkName),
        pub_interval_(0.5),
        node_handle_(nullptr),
        pubs_and_subs_created_(false) {
        }

        /// \brief    Destructor.
        virtual ~GazeboBlast3DModelPlugin();

        //        typedef std::normal_distribution<> NormalDistribution;

    protected:
        /// \brief    Called when the plugin is first created, and after the world
        ///           has been loaded. This function should not be blocking.
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

        /// \brief  	This gets called by the world update start event.
        void OnUpdate(const common::UpdateInfo&);

    private:
        /// \brief    Flag that is set to true once CreatePubsAndSubs() is called, used
        ///           to prevent CreatePubsAndSubs() from be called on every OnUpdate().
        bool pubs_and_subs_created_;

        /// \brief    Creates all required publishers and subscribers, incl. routing of messages to/from ROS if required.
        /// \details  Call this once the first time OnUpdate() is called (can't
        ///           be called from Load() because there is no guarantee GazeboRosInterfacePlugin has
        ///           has loaded and listening to ConnectGazeboToRosTopic and ConnectRosToGazeboTopic messages).
        void CreatePubsAndSubs();

        void Blast3DCallback(Blast3dMsgPtr& blast3d_msg);

        /// \brief    Handle for the Gazebo node.
        gazebo::transport::NodePtr node_handle_;

        double pub_interval_;

        /// \brief    Topic name for blast3d messages.
        std::string blast3d_server_reglink_topic_;
        std::string blast3d_server_link_topic_;
        std::string blast3d_pressure_datafolder_;
        
        /// \brief    Blast3d model plugin publishers and subscribers
        gazebo::transport::PublisherPtr blast3d_server_register_pub_;
        gazebo::transport::SubscriberPtr blast3d_server_msg_sub_;

        /// \brief    Transport namespace.
        std::string namespace_;

        /// \brief    Frame ID for Blast3d messages.
        std::string frame_id_;
        std::string link_name_;

        /// \brief    Pointer to the world.
        physics::WorldPtr world_;

        /// \brief    Pointer to the model.
        physics::ModelPtr model_;

        /// \brief    Pointer to the link.
        physics::LinkPtr link_;

        /// \brief    Pointer to the update event connection.
        event::ConnectionPtr updateConnection_;
        
        /// \brief    Variables for custom wind field generation.
        bool use_custom_blastdata_;

        /// \brief  Reads blast data from a text file and saves it.
        /// \param[in] custom_blast3d_field_path Path to the wind field from ~/.ros.
        void ReadBlast3DData(std::string &custom_blastdata_path);

        template <typename T>
        bool readCSV(std::string datafile_path, std::vector<std::vector<T>> &data) {
            std::ifstream fin;
            fin.open(datafile_path);
            if (!fin.is_open()) {
                gzerr << __FUNCTION__ << "Error reading file " << datafile_path << std::endl;
                return false;
            }
            // Read the data from the file
            std::string line;
            double dvalue;
            int linenum = 0;
            while (getline(fin, line)) {
                linenum++;
                //std::cout << "read line[" << linenum << "]: " << line << std::endl;
                std::vector<double> row;
                std::string value;
                std::stringstream ss(line);
                bool lineOK = true;

                //std::cout << "line[" << linenum << "] = {";
                while (getline(ss, value, ',')) {
                    double dvalue;
                    try {
                        dvalue = std::stod(value);
                    } catch (...) {
                        //                        gzdbg << "[gazebo_wind3d_world_plugin] Could not convert string to double, skipping value on line " << linenum
                        //                                << std::endl;
                        //                        gzdbg << "[gazebo_wind3d_world_plugin] line[" << linenum << "]=" << line << std::endl;
                        lineOK = false;
                        break;
                    }
                    row.push_back(dvalue);
                    //std::cout << dvalue << ", ";
                }
                //std::cout << std::endl;
                if (lineOK) {
                    //std::cout << "(X,Y,Z), (U,V,W) = " <<
                    //          "(" << row[0] << ", " << row[1] << ", " << row[2] << "), " <<
                    //          "(" << row[3] << ", " << row[4] << ", " << row[5] << ")" << std::endl;
                    data.push_back(row);
                }
            }
            fin.close();
            return true;
        }

    };
}

#endif // GAZEBO_BLAST3D_MODEL_PLUGIN_H