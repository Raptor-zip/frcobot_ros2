#ifndef _FR_HARDWARE_INTERFACE_
#define _FR_HARDWARE_INTERFACE_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "visibility_control.h"
#include <vector>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include "libfairino/include/robot.h"
#include "fairino_msgs/srv/remote_cmd_interface.hpp"


#define CONTROLLER_IP_ADDRESS "192.168.58.2"

namespace fairino_hardware
{

class FairinoHardwareInterface: public hardware_interface::SystemInterface{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FairinoHardwareInterface)

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  //FAIRINO_HARDWARE_PUBLIC
  //hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  
  FAIRINO_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  
  FAIRINO_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  // hardware_interface::return_type prepare_command_mode_switch(
  //   const std::vector<std::string> & start_interfaces,
  //   const std::vector<std::string> & stop_interfaces) override;
  // hardware_interface::return_type perform_command_mode_switch(
  //   const std::vector<std::string>& start_interfaces,
  //   const std::vector<std::string>& stop_interfaces) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
private:
  double _jnt_position_command[6];
  double _jnt_velocity_command[6];
  double _jnt_torque_command[6];
  double _jnt_position_state[6];
  double _jnt_velocity_state[6];
  double _jnt_torque_state[6];
  int _control_mode;
  std::string _controller_ip = CONTROLLER_IP_ADDRESS;
  std::unique_ptr<FRRobot> _ptr_robot;

  // ServoJ バックグラウンドスレッド
  // 問題: ServoJ は XML-RPC 呼び出しで ~4.7ms かかる。
  //       write() (500Hz=2ms周期) がブロックされ、実際の送信レートは ~213Hz になる。
  //       cmdT=2ms に対してコマンドが 4.7ms おきに届くのでコントローラがタイムアウト
  //       → 毎回「停止→運動」サイクル → 振動・ゴリゴリ音の原因。
  // 修正: write() は共有変数の更新だけ行い、ServoJ 送信は専用スレッドに委ねる。
  //       cmdT を実際の送信間隔(~6ms)に合わせてコントローラのタイムアウトを防ぐ。
  void _servo_loop();
  std::thread           _servo_thread;
  std::mutex            _cmd_mutex;
  std::atomic<bool>     _servo_running{false};
  double                _shared_cmd_deg[6];  // 単位: 度 (write→スレッド共有)

  // info_.joints のインデックス → ロボットAPI のインデックス(j1=0,...,j6=5)のマッピング
  // ros2_control の ResourceManager が info_.joints を URDF と異なる順序で返す場合がある
  // (例: j1,j2,j4,j5,j3,j6)。ロボットAPI の jPos[] は常に j1-j6 順なので変換が必要。
  int _joint_map[6];

  // --- DO/IO制御サービス ---
  // Fairinoコントローラは同時に1つのRPC接続しか受け付けないため、MoveIt実機制御中は
  // 別プロセス(ros2_cmd_server)からSetDOを呼べない。そこでハードウェアIFが既に保持している
  // RPC接続を共有し、デジタル出力(電磁弁・グリッパ等)を制御するサービスを提供する。
  // サービス名/型は公式 ros2_cmd_server と同一(fairino_remote_command_service /
  // fairino_msgs/srv/RemoteCmdInterface)とし、cmd_str="SetDO(id,status)" 等を受け付ける。
  void _handle_remote_command(
      const std::shared_ptr<fairino_msgs::srv::RemoteCmdInterface::Request> req,
      std::shared_ptr<fairino_msgs::srv::RemoteCmdInterface::Response> res);

  rclcpp::Node::SharedPtr _cmd_node;
  rclcpp::Service<fairino_msgs::srv::RemoteCmdInterface>::SharedPtr _cmd_service;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> _cmd_executor;
  std::thread _cmd_spin_thread;
  std::mutex  _rpc_mutex;  // _ptr_robot へのRPC呼び出しを直列化(ServoJ/read/SetDOの競合防止)
};

} //end namespace


#endif