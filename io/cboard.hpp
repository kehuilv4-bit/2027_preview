#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <vector>
#include <thread>
#include <atomic>

// 必须确保 io/serial 子目录下有这个头文件
#include "serial/serial.h"

#include "io/command.hpp"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
enum Mode
{
  idle,
  auto_aim,
  small_buff,
  big_buff,
  outpost
};
const std::vector<std::string> MODES = {"idle", "auto_aim", "small_buff", "big_buff", "outpost"};

enum ShootMode
{
  left_shoot,
  right_shoot,
  both_shoot
};
const std::vector<std::string> SHOOT_MODES = {"left_shoot", "right_shoot", "both_shoot"};

class CBoard
{
public:
  double bullet_speed;
  Mode mode;
  ShootMode shoot_mode;
  double ft_angle;

  CBoard(const std::string & config_path);
  ~CBoard(); 

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  // 注意：串口写入会改变类状态，所以去掉了 const
  void send(Command command);

private:
  struct IMUData
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  tools::ThreadSafeQueue<IMUData> queue_;
  
  serial::Serial serial_;
  std::thread receive_thread_;
  std::atomic<bool> running_;

  IMUData data_ahead_;
  IMUData data_behind_;

  int quaternion_canid_, bullet_speed_canid_, send_canid_;

  void receive_loop();
  void process_packet(uint8_t id, const uint8_t* data);
  void read_yaml(const std::string & config_path);
};

}  // namespace io

#endif  // IO__CBOARD_HPP