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
#include "Blast3d.pb.h"
#include "Blast3dServerRegistration.pb.h"

#include <iostream>
#include <ignition/math.hh>

#include "utils/AudioFile.h"
#include "utils/common.h"

#include "matplotlib-cpp/matplotlibcpp.h"
#include <chrono>
#include <thread>
#include <vector>

#include <ros/ros.h>
#include <ros/publisher.h>   // optional, ros/ros.h already pulls it
#include <std_msgs/Float32MultiArray.h>
#include "HighResSimTimer.h"
#include <deque>

#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <cmath>


using namespace std;

namespace gazebo {
    // Constants and Defaults
    static const bool kPrintOnMsgCallback = false;
    static const bool kPrintOnPluginLoad = true;
    static const std::string kDefaultNamespace = "";
    static const std::string kDefaultFrameId = "world";
    static const std::string kDefaultBlast3dServerRegisterTopic_model = "/gazebo/default/blast3d_register_link";
    static const std::string kDefaultBlast3dTopic = "blast3d";
    static const std::string kDefaultLinkName = "base_link";

    typedef const boost::shared_ptr<const blast3d_msgs::msgs::Blast3d>& Blast3dMsgPtr;

    class GAZEBO_VISIBLE GazeboBlast3DMicrophonePlugin : public ModelPlugin {
    public:
        GazeboBlast3DMicrophonePlugin();
        virtual ~GazeboBlast3DMicrophonePlugin();
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        virtual void OnUpdate(const common::UpdateInfo& _info);
        std::vector<float> averageFilterWithCutoff(std::vector<float>& signal, float cutoffFreq, float samplingFreq);

    private:
        void PublishAudioMessage(std::vector<float>& sampleData);

        /// \brief    Flag that is set to true once CreatePubsAndSubs() is called, used
        ///           to prevent CreatePubsAndSubs() from be called on every OnUpdate().
        bool pubs_and_subs_created_;

        /// \brief    Creates all required publishers and subscribers, incl. routing of messages to/from ROS if required.
        /// \details  Call this once the first time OnUpdate() is called (can't
        ///           be called from Load() because there is no guarantee GazeboRosInterfacePlugin has
        ///           has loaded and listening to ConnectGazeboToRosTopic and ConnectRosToGazeboTopic messages).
        void CreatePubsAndSubs();

        void Blast3DCallback(Blast3dMsgPtr& blast3d_msg);

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

        common::Time last_time_;
        double pub_interval_;

        std::string blast3d_server_reglink_topic_;
        std::string blast3d_server_link_topic_;

        /// \brief    Blast3d model plugin publishers and subscribers
        gazebo::transport::PublisherPtr blast3d_server_register_pub_;
        gazebo::transport::SubscriberPtr blast3d_server_msg_sub_;

        std::string namespace_;
        std::string blast3d_audio_topic_;
        std::string blast3d_audio_datafolder_;

        AudioFile<float> background_audio_;
        AudioFile<float> blast_audio_;
        
        AudioFile<float> seismicAudio;
        AudioFile<float> airBlastAudio;
        
        int pubBitDepth;
        float pubSampleRate;
        int pubBufSize;
        int plotEverySteps = 0;

        std::vector<std::vector<float>> output_buffer_background, output_buffer_pub;

        int background_audio_index_; // Index for the current position in the background audio.
        bool explosion_triggered_; // Flag to indicate whether the explosion has been triggered.
        
        double airAttenuationCoeff;
        
        ros::NodeHandle nh_;
        ros::Publisher  sync_pub_;
        std::string     vehicle_id_;
        uint32_t        next_event_id_ = 0;   // world plugin only
        int             current_event_id_ = -1; // model/mic if you need it
        std::unordered_map<double, uint32_t> event_id_map_;
        uint32_t last_eid_ = 0;
        ros::Publisher audio_ros_pub_;
        
        // precise sim-time trigger for seismic/air onsets
        HighResSimTimer timer_{[this](){ return this->world_->SimTime().Double(); }};

        struct PendingOnset {
          uint32_t eid;
          size_t   onset_sample; // absolute sample index in the stream where path begins
          double   sim_time;     // nominal sim time of onset
          std::string path;      // "acoustic_seismic" or "acoustic_air"
        };
        std::deque<PendingOnset> pending_onsets_;
        size_t samples_published_{0}; // increase by packet size after each PublishAudioMessage
        
        void injectSeismicWaveform(size_t startIdx);
        void injectAirWaveformWithAttenuation(size_t startIdx, float distance_m);
        void enqueueOnset(uint32_t eid, size_t onset_sample, double sim_time, const std::string& path);
        
        // Running sample counter for channel 0 (total samples published so far)
        size_t samples_sent_ = 0;

        // For each EID, the sample index where the air-blast packet begins
        std::unordered_map<uint32_t, size_t> mic_pending_idx_;

        // For each EID, the free-space standoff distance (for logging)
        std::unordered_map<uint32_t, double> mic_pending_range_;

        // Ensure we only log once per EID
        // Targets and “already logged” sets for each arrival
        std::unordered_map<uint32_t,double> mic_target_time_air_,  mic_target_time_seis_;
        std::unordered_map<uint32_t,double> mic_target_range_air_, mic_target_range_seis_;
        std::unordered_set<uint32_t>        mic_air_logged_,       mic_seis_logged_;

        
        // store blast source world position to recompute R at detection
        std::unordered_map<uint32_t, ignition::math::Vector3d> blast_src_world_;


    };
}
#endif /* GAZEBO_BLAST3D_MICROPHONE_PLUGIN_H */

