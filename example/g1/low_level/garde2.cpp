#include <cmath>
#include <memory>
#include <mutex>
#include <shared_mutex>

#include "gamepad.hpp"

// DDS
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

// IDL
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

static const std::string HG_CMD_TOPIC = "rt/lowcmd";
static const std::string HG_IMU_TORSO = "rt/secondary_imu";
static const std::string HG_STATE_TOPIC = "rt/lowstate";

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

template <typename T>
class DataBuffer {
 public:
  void SetData(const T &newData) {
    std::unique_lock<std::shared_mutex> lock(mutex);
    data = std::make_shared<T>(newData);
  }

  std::shared_ptr<const T> GetData() {
    std::shared_lock<std::shared_mutex> lock(mutex);
    return data ? data : nullptr;
  }

  void Clear() {
    std::unique_lock<std::shared_mutex> lock(mutex);
    data = nullptr;
  }

 private:
  std::shared_ptr<T> data;
  std::shared_mutex mutex;
};

const int G1_NUM_MOTOR = 29;
struct ImuState {
  std::array<float, 3> rpy = {};
  std::array<float, 3> omega = {};
};
struct MotorCommand {
  std::array<float, G1_NUM_MOTOR> q_target = {};
  std::array<float, G1_NUM_MOTOR> dq_target = {};
  std::array<float, G1_NUM_MOTOR> kp = {};
  std::array<float, G1_NUM_MOTOR> kd = {};
  std::array<float, G1_NUM_MOTOR> tau_ff = {};
};
struct MotorState {
  std::array<float, G1_NUM_MOTOR> q = {};
  std::array<float, G1_NUM_MOTOR> dq = {};
};

// Stiffness for all G1 Joints
std::array<float, G1_NUM_MOTOR> Kp{
    60, 60, 60, 100, 40, 40,      // legs
    60, 60, 60, 100, 40, 40,      // legs
    60, 40, 40,                   // waist
    60, 60, 60, 60, 60, 60, 60,   // left arm (higher stiffness for boxing guard)
    60, 60, 60, 60, 60, 60, 60    // right arm (higher stiffness for boxing guard)
};

// Damping for all G1 Joints
std::array<float, G1_NUM_MOTOR> Kd{
    1, 1, 1, 2, 1, 1,     // legs
    1, 1, 1, 2, 1, 1,     // legs
    1, 1, 1,              // waist
    2, 2, 2, 2, 2, 2, 2,  // left arm (higher damping for stability)
    2, 2, 2, 2, 2, 2, 2   // right arm (higher damping for stability)
};

enum class Mode {
  PR = 0,  // Series Control for Ptich/Roll Joints
  AB = 1   // Parallel Control for A/B Joints
};

enum G1JointIndex {
  // Legs (unchanged)
  LeftHipPitch = 0,
  LeftHipRoll = 1,
  LeftHipYaw = 2,
  LeftKnee = 3,
  LeftAnklePitch = 4,
  LeftAnkleRoll = 5,
  RightHipPitch = 6,
  RightHipRoll = 7,
  RightHipYaw = 8,
  RightKnee = 9,
  RightAnklePitch = 10,
  RightAnkleRoll = 11,
  
  // Waist
  WaistYaw = 12,
  WaistRoll = 13,
  WaistPitch = 14,
  
  // Left Arm
  LeftShoulderPitch = 15,
  LeftShoulderRoll = 16,
  LeftShoulderYaw = 17,
  LeftElbow = 18,
  LeftWristRoll = 19,
  LeftWristPitch = 20,
  LeftWristYaw = 21,
  
  // Right Arm
  RightShoulderPitch = 22,
  RightShoulderRoll = 23,
  RightShoulderYaw = 24,
  RightElbow = 25,
  RightWristRoll = 26,
  RightWristPitch = 27,
  RightWristYaw = 28
};

inline uint32_t Crc32Core(uint32_t *ptr, uint32_t len) {
  uint32_t xbit = 0;
  uint32_t data = 0;
  uint32_t CRC32 = 0xFFFFFFFF;
  const uint32_t dwPolynomial = 0x04c11db7;
  for (uint32_t i = 0; i < len; i++) {
    xbit = 1 << 31;
    data = ptr[i];
    for (uint32_t bits = 0; bits < 32; bits++) {
      if (CRC32 & 0x80000000) {
        CRC32 <<= 1;
        CRC32 ^= dwPolynomial;
      } else
        CRC32 <<= 1;
      if (data & xbit) CRC32 ^= dwPolynomial;

      xbit >>= 1;
    }
  }
  return CRC32;
};

class G1BoxingGuard {
 private:
  double time_;
  double control_dt_;  // [2ms]
  int counter_;
  Mode mode_pr_;
  uint8_t mode_machine_;

  Gamepad gamepad_;
  REMOTE_DATA_RX rx_;

  DataBuffer<MotorState> motor_state_buffer_;
  DataBuffer<MotorCommand> motor_command_buffer_;
  DataBuffer<ImuState> imu_state_buffer_;

  ChannelPublisherPtr<LowCmd_> lowcmd_publisher_;
  ChannelSubscriberPtr<LowState_> lowstate_subscriber_;
  ChannelSubscriberPtr<IMUState_> imutorso_subscriber_;
  ThreadPtr command_writer_ptr_, control_thread_ptr_;

  std::shared_ptr<unitree::robot::b2::MotionSwitcherClient> msc_;

 public:
  G1BoxingGuard(std::string networkInterface)
      : time_(0.0),
        control_dt_(0.002),
        counter_(0),
        mode_pr_(Mode::PR),
        mode_machine_(0) {
    ChannelFactory::Instance()->Init(0, networkInterface);

    // try to shutdown motion control-related service
    msc_ = std::make_shared<unitree::robot::b2::MotionSwitcherClient>();
    msc_->SetTimeout(5.0f);
    msc_->Init();
    std::string form, name;
    while (msc_->CheckMode(form, name), !name.empty()) {
      if (msc_->ReleaseMode())
        std::cout << "Failed to switch to Release Mode\n";
      sleep(5);
    }

    // create publisher
    lowcmd_publisher_.reset(new ChannelPublisher<LowCmd_>(HG_CMD_TOPIC));
    lowcmd_publisher_->InitChannel();
    // create subscriber
    lowstate_subscriber_.reset(new ChannelSubscriber<LowState_>(HG_STATE_TOPIC));
    lowstate_subscriber_->InitChannel(std::bind(&G1BoxingGuard::LowStateHandler, this, std::placeholders::_1), 1);
    imutorso_subscriber_.reset(new ChannelSubscriber<IMUState_>(HG_IMU_TORSO));
    imutorso_subscriber_->InitChannel(std::bind(&G1BoxingGuard::imuTorsoHandler, this, std::placeholders::_1), 1);
    // create threads
    command_writer_ptr_ = CreateRecurrentThreadEx("command_writer", UT_CPU_ID_NONE, 2000, &G1BoxingGuard::LowCommandWriter, this);
    control_thread_ptr_ = CreateRecurrentThreadEx("control", UT_CPU_ID_NONE, 2000, &G1BoxingGuard::Control, this);
  }

  void imuTorsoHandler(const void *message) {
    IMUState_ imu_torso = *(const IMUState_ *)message;
    auto &rpy = imu_torso.rpy();
    if (counter_ % 500 == 0)
      printf("IMU.torso.rpy: %.2f %.2f %.2f\n", rpy[0], rpy[1], rpy[2]);
  }

  void LowStateHandler(const void *message) {
    LowState_ low_state = *(const LowState_ *)message;
    if (low_state.crc() != Crc32Core((uint32_t *)&low_state, (sizeof(LowState_) >> 2) - 1)) {
      std::cout << "[ERROR] CRC Error" << std::endl;
      return;
    }

    // get motor state
    MotorState ms_tmp;
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
      ms_tmp.q.at(i) = low_state.motor_state()[i].q();
      ms_tmp.dq.at(i) = low_state.motor_state()[i].dq();
      if (low_state.motor_state()[i].motorstate() && i <= RightAnkleRoll)
        std::cout << "[ERROR] motor " << i << " with code " << low_state.motor_state()[i].motorstate() << "\n";
    }
    motor_state_buffer_.SetData(ms_tmp);

    // get imu state
    ImuState imu_tmp;
    imu_tmp.omega = low_state.imu_state().gyroscope();
    imu_tmp.rpy = low_state.imu_state().rpy();
    imu_state_buffer_.SetData(imu_tmp);

    // update gamepad
    memcpy(rx_.buff, &low_state.wireless_remote()[0], 40);
    gamepad_.update(rx_.RF_RX);

    // update mode machine
    if (mode_machine_ != low_state.mode_machine()) {
      if (mode_machine_ == 0) std::cout << "G1 type: " << unsigned(low_state.mode_machine()) << std::endl;
      mode_machine_ = low_state.mode_machine();
    }

    if (++counter_ % 500 == 0) {
      counter_ = 0;
      auto &rpy = low_state.imu_state().rpy();
      printf("IMU.pelvis.rpy: %.2f %.2f %.2f\n", rpy[0], rpy[1], rpy[2]);
    }
  }

  void LowCommandWriter() {
    LowCmd_ dds_low_command;
    dds_low_command.mode_pr() = static_cast<uint8_t>(mode_pr_);
    dds_low_command.mode_machine() = mode_machine_;

    const std::shared_ptr<const MotorCommand> mc = motor_command_buffer_.GetData();
    if (mc) {
      for (size_t i = 0; i < G1_NUM_MOTOR; i++) {
        dds_low_command.motor_cmd().at(i).mode() = 1;  // 1:Enable, 0:Disable
        dds_low_command.motor_cmd().at(i).tau() = mc->tau_ff.at(i);
        dds_low_command.motor_cmd().at(i).q() = mc->q_target.at(i);
        dds_low_command.motor_cmd().at(i).dq() = mc->dq_target.at(i);
        dds_low_command.motor_cmd().at(i).kp() = mc->kp.at(i);
        dds_low_command.motor_cmd().at(i).kd() = mc->kd.at(i);
      }

      dds_low_command.crc() = Crc32Core((uint32_t *)&dds_low_command, (sizeof(dds_low_command) >> 2) - 1);
      lowcmd_publisher_->Write(dds_low_command);
    }
  }

  void Control() {
    MotorCommand motor_command_tmp;
    const std::shared_ptr<const MotorState> ms = motor_state_buffer_.GetData();
    
    // Initialize all motors
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
        motor_command_tmp.tau_ff.at(i) = 0.0;
        motor_command_tmp.q_target.at(i) = 0.0;
        motor_command_tmp.dq_target.at(i) = 0.0;
        motor_command_tmp.kp.at(i) = Kp[i];
        motor_command_tmp.kd.at(i) = Kd[i];
    }
    
    if (ms) {
        time_ += control_dt_;
        
        // Define target positions for boxing guard stance
        std::array<double, G1_NUM_MOTOR> target_positions = {};
        
        // Legs position (slightly bent for balance)
        target_positions[LeftHipPitch] = 0.1;
        target_positions[LeftKnee] = -0.2;
        target_positions[LeftAnklePitch] = 0.1;
        target_positions[RightHipPitch] = 0.1;
        target_positions[RightKnee] = -0.2;
        target_positions[RightAnklePitch] = 0.1;
        
        // Waist position (slightly forward)
        target_positions[WaistPitch] = -0.1;
        
        // Left Arm (guard position - high guard)
        target_positions[LeftShoulderPitch] = 0.8;
        target_positions[LeftShoulderRoll] = 0.5;
        target_positions[LeftElbow] = -1.2;
        target_positions[LeftWristRoll] = 0.0;
        
        // Right Arm (ready to punch - chambered position)
        target_positions[RightShoulderPitch] = 0.6;
        target_positions[RightShoulderRoll] = -0.3;
        target_positions[RightElbow] = -1.0;
        target_positions[RightWristRoll] = 0.0;
        
        // Smooth transition to target positions
        double transition_duration = 3.0; // 3 seconds to reach position
        double ratio = std::clamp(time_ / transition_duration, 0.0, 1.0);
        
        // Apply smooth interpolation for all motors
        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
            if (target_positions[i] != 0.0) { // Only interpolate non-zero targets
                motor_command_tmp.q_target.at(i) = 
                    (1.0 - ratio) * ms->q.at(i) + ratio * target_positions[i];
            }
        }
        
        // Optional: Add subtle breathing motion after reaching position
        if (ratio >= 1.0) {
            double breathing_amplitude = 0.02; // Small amplitude
            double breathing_freq = 0.3; // Slow breathing
            double breathing_offset = breathing_amplitude * 
                std::sin(2.0 * M_PI * breathing_freq * (time_ - transition_duration));
            
            motor_command_tmp.q_target.at(WaistPitch) += breathing_offset;
        }
        
        motor_command_buffer_.SetData(motor_command_tmp);
    }
}

int main(int argc, char const *argv[]) {
    if (argc < 2) {
        std::cout << "Usage: g1_boxing_guard_example network_interface" << std::endl;
        exit(0);
    }
    
    std::string networkInterface = argv[1];
    G1BoxingGuard boxing_guard(networkInterface);
    
    while (true) sleep(10);
    return 0;
}

int main(int argc, char const *argv[]) {
  if (argc < 2) {
    std::cout << "Usage: g1_boxing_guard_example network_interface" << std::endl;
    exit(0);
  }
  std::string networkInterface = argv[1];
  G1BoxingGuard boxing_guard(networkInterface);
  while (true) sleep(10);
  return 0;
}