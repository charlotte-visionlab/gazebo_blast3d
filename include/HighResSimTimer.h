#pragma once
#include <atomic>
#include <thread>
#include <functional>
#include <chrono>

class HighResSimTimer {
public:
  explicit HighResSimTimer(std::function<double()> worldSimTimeSec)
  : nowSimSec_(std::move(worldSimTimeSec)) {}

  // Fire when SIM time >= t_target (seconds). Runs cb() on a short-lived worker.
  void arm(double t_target, std::function<void()> cb,
           double arm_window_sec = 1.0,   // start thread when within 1s of target
           int    poll_us        = 50) {  // wall-clock sleep between polls
    cancel();
    cancelled_.store(false);
    worker_ = std::thread([=, this]() {
      while (!cancelled_.load() && nowSimSec_() < (t_target - arm_window_sec)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      while (!cancelled_.load()) {
        if (nowSimSec_() + 1e-9 >= t_target) break;
        std::this_thread::sleep_for(std::chrono::microseconds(poll_us));
      }
      if (!cancelled_.load()) cb();
    });
  }

  void cancel() {
    if (worker_.joinable()) {
      cancelled_.store(true);
      worker_.join();
    }
  }

  ~HighResSimTimer() { cancel(); }

private:
  std::function<double()> nowSimSec_;
  std::atomic<bool> cancelled_{false};
  std::thread worker_;
};