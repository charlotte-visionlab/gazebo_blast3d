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

using namespace std;

namespace gazebo
{
  static const std::string kDefaultGyroTopic = "/px4flow/imu";

  class GAZEBO_VISIBLE GazeboBlast3DMicrophonePlugin : public SensorPlugin
  {
    public:
      GazeboBlast3DMicrophonePlugin();
      virtual ~GazeboBlast3DMicrophonePlugin();
      virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
      virtual void OnUpdate();

    protected:
      unsigned int width, height, depth;
      std::string format;
      physics::WorldPtr world;

    private:
      event::ConnectionPtr newFrameConnection;
      sensor_msgs::msgs::Audio audio_message;
      transport::PublisherPtr audio_pub_;
      transport::NodePtr node_handle_;

      std::string namespace_;

  };
}
#endif /* GAZEBO_BLAST3D_MICROPHONE_PLUGIN_H */

