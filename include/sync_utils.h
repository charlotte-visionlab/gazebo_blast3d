#pragma once
#include <ros/ros.h>
#include <gazebo_blast3d/BlastSync.h>
#include <atomic>
#include <fstream>
#include <mutex>
#include <string>
inline uint32_t nextEventId() {
    static std::atomic<uint32_t> c{1};
    return c++;
}

#ifndef SYNC_CSV_PATH
#define SYNC_CSV_PATH "/home/md2/sparab2/wind/uncc_wind_control/ros_image/ros_ws/sync_log.csv"
#endif

namespace blast3d_sync {

inline uint32_t nextEventId() {
    static std::atomic<uint32_t> id{1};
    return id++;
}

inline std::ofstream& csvStream() {
    static std::ofstream f;
    if (!f.is_open()) f.open(SYNC_CSV_PATH, std::ios::app);
    return f;
}

inline std::mutex& csvMutex() {
    static std::mutex m;
    return m;
}

inline void publishSyncLog(ros::Publisher& pub,
                           const std::string& source,
                           uint32_t event_id,
                           double sim_time,
                           const std::string& vehicle = "",
                           double standoff_dist = 0.0)
{
    gazebo_blast3d::BlastSync msg;
    msg.header.stamp   = ros::Time::now();
    msg.event_id       = event_id;
    msg.source         = source;
    msg.vehicle        = vehicle;
    msg.sim_time       = sim_time;
    msg.ros_time       = msg.header.stamp.toSec();
    msg.standoff_dist  = standoff_dist;

    pub.publish(msg);

    std::lock_guard<std::mutex> lk(csvMutex());
    static bool wrote_header = false;
    auto& f = csvStream();
    if (!wrote_header) {
        f << "event_id,source,sim_time,ros_time,latency_ms,vehicle,standoff_dist,seq\n";
        wrote_header = true;
    }
    const double latency_ms = (msg.ros_time - msg.sim_time) * 1e3;
    f << msg.event_id << ',' << msg.source << ',' << msg.sim_time << ','
      << msg.ros_time << ',' << latency_ms << ',' << msg.vehicle << ','
      << msg.standoff_dist << ',' << msg.header.seq << '\n';
    f.flush();
}

} // namespace blast3d_sync
