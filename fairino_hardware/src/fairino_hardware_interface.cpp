#include "fairino_hardware/fairino_hardware_interface.hpp"

#include <sstream>
#include <vector>

namespace fairino_hardware{

hardware_interface::CallbackReturn FairinoHardwareInterface::on_init(const hardware_interface::HardwareInfo& sysinfo){
    if (hardware_interface::SystemInterface::on_init(sysinfo) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    info_ = sysinfo;//info_是父类中定义的变量
    
    for (const hardware_interface::ComponentInfo& joint : info_.joints) {

        //指令部分
        if (joint.command_interfaces.size() != 1) {//开放servoJ
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                        joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                   "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        // if (joint.command_interfaces[1].name != hardware_interface::HW_IF_EFFORT){//预留，用于关节扭矩直接控制
        //     RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
        //            "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
        //            joint.name.c_str(), joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_EFFORT);
        //     return hardware_interface::CallbackReturn::ERROR;
        // }

        //关节状态部分
        if (joint.state_interfaces.size() != 1) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"), "Joint '%s' has %zu state interface. 3 expected.",
                        joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                        "Joint '%s' have %s state interface as first state interface. '%s' expected.", joint.name.c_str(),
                        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        // if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
        //     RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
        //                 "Joint '%s' have %s state interface as second state interface. '%s' expected.", joint.name.c_str(),
        //                 joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        //     return hardware_interface::CallbackReturn::ERROR;
        // }

        // if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
        //     RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
        //                 "Joint '%s' have %s state interface as third state interface. '%s' expected.", joint.name.c_str(),
        //                 joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT);
        //     return hardware_interface::CallbackReturn::ERROR;
        // }

    }

    // info_.joints のインデックスからロボットAPIのインデックスへのマッピングを構築
    // ロボットAPIは常に jPos[0]=j1, jPos[1]=j2, ..., jPos[5]=j6 の順序
    // info_.joints は ros2_control の内部処理により異なる順序になりうる
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        const std::string& name = info_.joints[i].name;
        if (name.size() >= 2 && name[0] == 'j') {
            _joint_map[i] = std::stoi(name.substr(1)) - 1;  // "j1"→0, "j2"→1, ...
        } else {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                "予期しない関節名: '%s'", name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
        "関節マッピング: info_joints=[%s,%s,%s,%s,%s,%s] → api=[%d,%d,%d,%d,%d,%d]",
        info_.joints[0].name.c_str(), info_.joints[1].name.c_str(),
        info_.joints[2].name.c_str(), info_.joints[3].name.c_str(),
        info_.joints[4].name.c_str(), info_.joints[5].name.c_str(),
        _joint_map[0], _joint_map[1], _joint_map[2],
        _joint_map[3], _joint_map[4], _joint_map[5]);

    return hardware_interface::CallbackReturn::SUCCESS;
}//end on_init



std::vector<hardware_interface::StateInterface> FairinoHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  //导出关节相关的状态接口(位置，速度，扭矩)
  for (size_t i = 0; i < info_.joints.size(); ++i){
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_state[i]));

    // state_interfaces.emplace_back(hardware_interface::StateInterface(
    //     info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &_jnt_velocity_state.at(i)));

    // state_interfaces.emplace_back(hardware_interface::StateInterface(
    //     info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &_jnt_torque_state.at(i)));
  }

  //导出
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FairinoHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_command[i]));

//     command_interfaces.emplace_back(hardware_interface::CommandInterface(//预留的扭矩控制接口
//         info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &_jnt_torque_command.at(i)));
  }

  return command_interfaces;
}



hardware_interface::CallbackReturn FairinoHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
    using namespace std::chrono_literals;
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Starting ...please wait...");
    //做变量的初始化工作
    _ptr_robot = std::make_unique<FRRobot>();//创建机器人实例
    for(int i=0;i<6;i++){//初始化变量
        _jnt_position_command[i] = 0;
        _jnt_velocity_command[i] = 0;
        _jnt_torque_command[i] = 0;
        _jnt_position_state[i] = 0;
        _jnt_velocity_state[i] = 0;
        _jnt_torque_state[i] = 0;
    }
    _control_mode = 0;//默认是位置控制,0-位置控制，1-扭矩控制 2-速度控制
    errno_t returncode = _ptr_robot->RPC(_controller_ip.c_str());//建立xmlrpc连接
    rclcpp::sleep_for(200ms);//等待一段时间让控制器的rpc连接建立完毕
    if(returncode != 0){
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "机械臂SDK连接失败！请检查端口时候被占用");
        return hardware_interface::CallbackReturn::ERROR;
    }else{
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "机械臂SDK连接成功！");
    }
    //做第一步的工作，读取当前状态数据
    JointPos jntpos;
    returncode = _ptr_robot->GetActualJointPosDegree(0,&jntpos);
    /*
    获取反馈位置后同步到指令位置以维持当前状态，如果发现读取失败，那么就无法激活插件，
    因为错误的反馈位置会导致初始指令位置下发出现严重偏差导致事故
    */
    if(returncode == 0){
        for(int j=0;j<6;j++){
            int api = _joint_map[j];  // info_.joints[j] に対応するロボットAPI上の関節インデックス
            _jnt_position_command[j] = jntpos.jPos[api]/180.0*M_PI;
            _shared_cmd_deg[api]     = jntpos.jPos[api];  // 度単位で共有変数を初期化（API順）
        }
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),"初始指令位置: %f,%f,%f,%f,%f,%f",_jnt_position_command[0],\
        _jnt_position_command[1],_jnt_position_command[2],_jnt_position_command[3],_jnt_position_command[4],_jnt_position_command[5]);
        // ロボット初期化シーケンス: エラーリセット → 自動モード → 使能 → ServoMoveStart
        _ptr_robot->ResetAllError();
        rclcpp::sleep_for(500ms);
        _ptr_robot->Mode(0);
        rclcpp::sleep_for(500ms);
        _ptr_robot->RobotEnable(1);
        rclcpp::sleep_for(500ms);
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
            "ロボット初期化完了(ResetAllError→Mode(0)→RobotEnable(1))");

        // ServoMoveStart をリトライ付きで実行
        errno_t servo_ret = -1;
        for(int attempt = 0; attempt < 3; attempt++){
            servo_ret = _ptr_robot->ServoMoveStart();
            if(servo_ret == 0){
                RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                    "ServoMoveStart成功 (試行%d)", attempt + 1);
                break;
            }
            RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                "ServoMoveStart失敗(エラーコード:%d, 試行%d/3)", servo_ret, attempt + 1);
            // リトライ前にエラーリセットと再初期化
            _ptr_robot->ResetAllError();
            rclcpp::sleep_for(500ms);
            _ptr_robot->Mode(0);
            rclcpp::sleep_for(500ms);
            _ptr_robot->RobotEnable(1);
            rclcpp::sleep_for(500ms);
        }
        if(servo_ret != 0){
            RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                "ServoMoveStart 3回リトライ後も失敗。ロボットの状態を確認してください。");
        }
        _servo_running = true;
        _servo_thread = std::thread(&FairinoHardwareInterface::_servo_loop, this);

        // DO/IO制御サービスを起動(既存のRPC接続を共有してSetDOを実行)
        _cmd_node = std::make_shared<rclcpp::Node>("fairino_hw_command_server");
        _cmd_service = _cmd_node->create_service<fairino_msgs::srv::RemoteCmdInterface>(
            "fairino_remote_command_service",
            std::bind(&FairinoHardwareInterface::_handle_remote_command, this,
                      std::placeholders::_1, std::placeholders::_2));
        _cmd_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        _cmd_executor->add_node(_cmd_node);
        _cmd_spin_thread = std::thread([this]{ _cmd_executor->spin(); });
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
            "DO制御サービス起動: /fairino_remote_command_service (SetDO/SetToolDO対応)");

        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "机械臂硬件启动成功!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }else{
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "读取初始关节角度错误，硬件无法启动！请检查通讯内容");
        return hardware_interface::CallbackReturn::ERROR;
    }
}



hardware_interface::CallbackReturn FairinoHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Stopping ...please wait...");
    // DO制御サービスを先に停止
    if(_cmd_executor) _cmd_executor->cancel();
    if(_cmd_spin_thread.joinable()) _cmd_spin_thread.join();
    if(_cmd_executor && _cmd_node) _cmd_executor->remove_node(_cmd_node);
    _cmd_service.reset();
    _cmd_node.reset();
    _cmd_executor.reset();

    // まずバックグラウンドスレッドを停止してからロボットAPIを呼ぶ
    _servo_running = false;
    if(_servo_thread.joinable()) _servo_thread.join();
    _ptr_robot->ServoMoveEnd();
    _ptr_robot->StopMotion();//停止机器人
    _ptr_robot->CloseRPC();//销毁实例，连接断开
    _ptr_robot.release();
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "System successfully stopped!");
    return hardware_interface::CallbackReturn::SUCCESS;
}



hardware_interface::return_type FairinoHardwareInterface::read(const rclcpp::Time& time,const rclcpp::Duration& period)
{//从RTDE反馈数据中获取所需的位置，速度和扭矩信息
    JointPos state_data;
    error_t returncode;
    {
        std::lock_guard<std::mutex> rpc_lock(_rpc_mutex);
        returncode = _ptr_robot->GetActualJointPosDegree(1,&state_data);
    }
    if(returncode == 0){
        for(int i=0;i<6;i++){
            int api = _joint_map[i];  // info_.joints[i] に対応するロボットAPIインデックス
            _jnt_position_state[i] = state_data.jPos[api]/180.0*M_PI;//注意单位转换，moveit统一用弧度
            //_jnt_torque_state[i] = state_data.jt_cur_tor[api];//注意单位转换
        }
    }else{
        hardware_interface::return_type::ERROR;
    }
    //RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "System successfully read: %f,%f,%f,%f,%f,%f",_jnt_position_state[0],\
    _jnt_position_state[1],_jnt_position_state[2],_jnt_position_state[3],_jnt_position_state[4],_jnt_position_state[5]);

  return hardware_interface::return_type::OK;

}

hardware_interface::return_type FairinoHardwareInterface::write(const rclcpp::Time& time,const rclcpp::Duration& period)
{
    if(_control_mode == 0){//位置控制模式
        if (std::any_of(&_jnt_position_command[0], &_jnt_position_command[5],\
            [](double c) { return not std::isfinite(c); })) {
            return hardware_interface::return_type::ERROR;
        }
        // 度単位に変換して共有変数を更新するだけ。ServoJ 送信は _servo_loop() が行う。
        // _jnt_position_command は info_.joints 順、_shared_cmd_deg はロボットAPI順(j1-j6)
        std::lock_guard<std::mutex> lock(_cmd_mutex);
        for(auto j=0;j<6;j++){
            int api = _joint_map[j];
            _shared_cmd_deg[api] = _jnt_position_command[j]/M_PI*180;
        }
    }else if(_control_mode == 1){//扭矩控制模式
        if (std::any_of(&_jnt_torque_command[0], &_jnt_torque_command[5],\
            [](double c) { return not std::isfinite(c); })) {
            return hardware_interface::return_type::ERROR;
        }
    }else{
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "指令发送错误:未识别当前所处控制模式");
        return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
}

void FairinoHardwareInterface::_servo_loop()
{
    // バックグラウンドスレッドから ServoJ を連続送信する。
    //
    // 根本原因の解決:
    //   ServoJ は XML-RPC 呼び出しで実測 ~4.7ms かかる。
    //   write() (500Hz=2ms周期) 内で呼ぶと制御ループがブロックされ、
    //   実際の送信レートが ~213Hz に落ちる。
    //   cmdT=0.002 (2ms) に対してコマンドが 4.7ms おきにしか届かないため
    //   コントローラが毎回タイムアウト → 「停止→運動」サイクル → ゴリゴリ音。
    //
    // 解決策:
    //   write() はメモリコピーだけ行い (非常に高速)、
    //   このスレッドが ServoJ を可能な限り高速に送り続ける。
    //   cmdT=0.006 (6ms) は実際の送信間隔 (~4.7ms) より大きいため
    //   コントローラはタイムアウトする前に次のコマンドを受け取り
    //   「運動」状態を維持できる。
    ExaxisPos extcmd{0,0,0,0};
    int call_count = 0, error_count = 0;
    double total_call_ms = 0.0;

    // 目標送信間隔 = cmdT にすることで fill_rate = drain_rate → キュー安定
    // ServoJ が ~5ms かかるため、残り時間をスリープで補完して合計を TARGET_MS にする。
    // TARGET_MS と CMDT は一致させる → fill_rate = drain_rate → mc_queue_len ≈ 0
    const double TARGET_MS = 10.0;
    const float  CMDT      = 0.010f;

    while(_servo_running){
        auto loop_start = std::chrono::steady_clock::now();

        JointPos cmd;
        {
            std::lock_guard<std::mutex> lock(_cmd_mutex);
            for(int j=0;j<6;j++) cmd.jPos[j] = _shared_cmd_deg[j];
        }

        auto t0 = std::chrono::steady_clock::now();
        int ret;
        {
            std::lock_guard<std::mutex> rpc_lock(_rpc_mutex);
            ret = _ptr_robot->ServoJ(&cmd, &extcmd, 0, 0, CMDT, 0, 0);
        }
        auto t1 = std::chrono::steady_clock::now();

        double call_ms = std::chrono::duration<double,std::milli>(t1 - t0).count();
        call_count++;
        total_call_ms += call_ms;
        if(ret != 0) error_count++;

        // ループ全体を TARGET_MS に合わせるためスリープ
        // → 送信間隔 ≈ cmdT なのでコントローラがタイムアウトせず、かつキューも膨らまない
        double elapsed_ms = std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now() - loop_start).count();
        double sleep_ms = TARGET_MS - elapsed_ms;
        if(sleep_ms > 0.5){
            std::this_thread::sleep_for(
                std::chrono::microseconds(static_cast<int>(sleep_ms * 1000)));
        }

        if(call_count % 100 == 0){
            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                "[ServoThread] 呼び出し=%d, ServoJ平均=%.2fms, エラー=%d",
                call_count, total_call_ms/call_count, error_count);
            ROBOT_STATE_PKG pkg;
            int st_ret;
            {
                std::lock_guard<std::mutex> rpc_lock(_rpc_mutex);
                st_ret = _ptr_robot->GetRobotRealTimeState(&pkg);
            }
            if(st_ret == 0){
                RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                    "[ServoThread] robot_state=%d(1=停止,2=運動), mc_queue_len=%d, main_code=%d, sub_code=%d",
                    pkg.robot_state, pkg.mc_queue_len, pkg.main_code, pkg.sub_code);
            }
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
        "[ServoThread] 終了: 総呼び出し=%d, 平均=%.2fms, エラー=%d",
        call_count, call_count > 0 ? total_call_ms/call_count : 0.0, error_count);
}

void FairinoHardwareInterface::_handle_remote_command(
    const std::shared_ptr<fairino_msgs::srv::RemoteCmdInterface::Request> req,
    std::shared_ptr<fairino_msgs::srv::RemoteCmdInterface::Response> res)
{
    // cmd_str 例: "SetDO(0,1)" / "SetToolDO(0,1)"
    // func(arg0,arg1,...) を分解する
    const std::string& cmd = req->cmd_str;
    auto lpar = cmd.find('(');
    auto rpar = cmd.rfind(')');
    if(lpar == std::string::npos || rpar == std::string::npos || rpar <= lpar){
        res->cmd_res = "-1";
        RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
            "[DOサービス] 不正なコマンド形式: %s", cmd.c_str());
        return;
    }
    const std::string func = cmd.substr(0, lpar);
    const std::string argstr = cmd.substr(lpar + 1, rpar - lpar - 1);

    std::vector<int> args;
    std::stringstream ss(argstr);
    std::string item;
    try {
        while(std::getline(ss, item, ',')){
            if(item.empty()) continue;
            args.push_back(std::stoi(item));
        }
    } catch(const std::exception& e){
        res->cmd_res = "-1";
        RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
            "[DOサービス] 引数解析失敗: %s", cmd.c_str());
        return;
    }

    if((func == "SetDO" || func == "SetToolDO") && args.size() >= 2){
        const int id = args[0];
        const uint8_t status = (args[1] != 0) ? 1 : 0;
        // smooth/block はデフォルト(平滑なし/非阻塞)。3,4番目があれば使用。
        const uint8_t smooth = (args.size() >= 3) ? static_cast<uint8_t>(args[2]) : 0;
        const uint8_t block  = (args.size() >= 4) ? static_cast<uint8_t>(args[3]) : 1;

        errno_t ret;
        {
            std::lock_guard<std::mutex> rpc_lock(_rpc_mutex);
            if(func == "SetDO"){
                ret = _ptr_robot->SetDO(id, status, smooth, block);
            }else{
                ret = _ptr_robot->SetToolDO(id, status, smooth, block);
            }
        }
        // 公式command_serverと同様、ロボットの実エラーコードをそのまま返す(0=成功)
        res->cmd_res = std::to_string(ret);
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
            "[DOサービス] %s(id=%d, status=%d) → ret=%d", func.c_str(), id, status, ret);
        return;
    }

    res->cmd_res = "-1";
    RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
        "[DOサービス] 未対応コマンド(SetDO/SetToolDOのみ対応): %s", cmd.c_str());
}


}//end namesapce

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fairino_hardware::FairinoHardwareInterface, hardware_interface::SystemInterface)
