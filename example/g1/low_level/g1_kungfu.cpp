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

// Motor type enum for gain calculation
enum MotorType { GearboxS = 0, GearboxM = 1, GearboxL = 2 };

// Motor type mapping for G1
std::array<MotorType, G1_NUM_MOTOR> G1MotorType{
    // legs
    GearboxM, GearboxM, GearboxM, GearboxL, GearboxS, GearboxS,
    GearboxM, GearboxM, GearboxM, GearboxL, GearboxS, GearboxS,
    // waist
    GearboxM, GearboxS, GearboxS,
    // arms
    GearboxS, GearboxS, GearboxS, GearboxS, GearboxS, GearboxS, GearboxS,
    GearboxS, GearboxS, GearboxS, GearboxS, GearboxS, GearboxS, GearboxS
};

enum class Mode {
  PR = 0,  // Series Control for Pitch/Roll Joints
  AB = 1   // Parallel Control for A/B Joints
};

enum G1JointIndex {
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
  WaistYaw = 12,
  WaistRoll = 13,
  WaistPitch = 14,
  LeftShoulderPitch = 15,
  LeftShoulderRoll = 16,
  LeftShoulderYaw = 17,
  LeftElbow = 18,
  LeftWristRoll = 19,
  LeftWristPitch = 20,
  LeftWristYaw = 21,
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
}

float GetMotorKp(MotorType type) {
  switch (type) {
    case GearboxS: return 40;
    case GearboxM: return 60;
    case GearboxL: return 100;
    default: return 40;
  }
}

float GetMotorKd(MotorType type) {
  switch (type) {
    case GearboxS: return 1;
    case GearboxM: return 1;
    case GearboxL: return 2;
    default: return 1;
  }
}

class G1KungFuExample {
 private:
  double time_;
  double control_dt_;     // [2ms]
  double stage_duration_; // Duration for each stage
  int current_stage_;     // Current kung fu stage
  Mode mode_pr_;
  uint8_t mode_machine_;
  int counter_;

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
  G1KungFuExample(std::string networkInterface)
      : time_(0.0),
        control_dt_(0.002),
        stage_duration_(2.0),
        current_stage_(0),
        mode_pr_(Mode::PR),
        mode_machine_(0),
        counter_(0) {
    
    ChannelFactory::Instance()->Init(0, networkInterface);

    // Initialize motion switcher client
    msc_ = std::make_shared<unitree::robot::b2::MotionSwitcherClient>();
    msc_->SetTimeout(5.0f);
    msc_->Init();
    
    // Release any active motion control services
    std::string form, name;
    while (msc_->CheckMode(form, name), !name.empty()) {
      if (msc_->ReleaseMode()) {
        std::cout << "Failed to switch to Release Mode" << std::endl;
      }
      sleep(5);
    }

    // Create publisher and subscribers
    lowcmd_publisher_.reset(new ChannelPublisher<LowCmd_>(HG_CMD_TOPIC));
    lowcmd_publisher_->InitChannel();
    
    lowstate_subscriber_.reset(new ChannelSubscriber<LowState_>(HG_STATE_TOPIC));
    lowstate_subscriber_->InitChannel(std::bind(&G1KungFuExample::LowStateHandler, this, std::placeholders::_1), 1);
    
    imutorso_subscriber_.reset(new ChannelSubscriber<IMUState_>(HG_IMU_TORSO));
    imutorso_subscriber_->InitChannel(std::bind(&G1KungFuExample::imuTorsoHandler, this, std::placeholders::_1), 1);

    // Create control threads
    command_writer_ptr_ = CreateRecurrentThreadEx("command_writer", UT_CPU_ID_NONE, 2000, &G1KungFuExample::LowCommandWriter, this);
    control_thread_ptr_ = CreateRecurrentThreadEx("control", UT_CPU_ID_NONE, 2000, &G1KungFuExample::Control, this);
    
    std::cout << "G1 Kung Fu Example initialized. Starting kung fu sequence..." << std::endl;
  }

  void imuTorsoHandler(const void *message) {
    IMUState_ imu_torso = *(const IMUState_ *)message;
    auto &rpy = imu_torso.rpy();
    if (counter_ % 500 == 0) {
      printf("IMU.torso.rpy: %.2f %.2f %.2f\n", rpy[0], rpy[1], rpy[2]);
    }
  }

  void LowStateHandler(const void *message) {
    LowState_ low_state = *(const LowState_ *)message;
    if (low_state.crc() != Crc32Core((uint32_t *)&low_state, (sizeof(LowState_) >> 2) - 1)) {
      std::cout << "[ERROR] CRC Error" << std::endl;
      return;
    }

    // Get motor state
    MotorState ms_tmp;
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
      ms_tmp.q.at(i) = low_state.motor_state()[i].q();
      ms_tmp.dq.at(i) = low_state.motor_state()[i].dq();
    }
    motor_state_buffer_.SetData(ms_tmp);

    // Get IMU state
    ImuState imu_tmp;
    imu_tmp.omega = low_state.imu_state().gyroscope();
    imu_tmp.rpy = low_state.imu_state().rpy();
    imu_state_buffer_.SetData(imu_tmp);

    // Update gamepad
    memcpy(rx_.buff, &low_state.wireless_remote()[0], 40);
    gamepad_.update(rx_.RF_RX);

    // Update mode machine
    if (mode_machine_ != low_state.mode_machine()) {
      if (mode_machine_ == 0) {
        std::cout << "G1 type: " << unsigned(low_state.mode_machine()) << std::endl;
      }
      mode_machine_ = low_state.mode_machine();
    }

    // Report status periodically
    if (++counter_ % 500 == 0) {
      auto &rpy = low_state.imu_state().rpy();
      printf("Stage: %d, IMU.pelvis.rpy: %.2f %.2f %.2f\n", 
             current_stage_, rpy[0], rpy[1], rpy[2]);
      counter_ = 0;
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

    // Initialize all motor commands to zero
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
      motor_command_tmp.tau_ff.at(i) = 0.0;
      motor_command_tmp.q_target.at(i) = 0.0;
      motor_command_tmp.dq_target.at(i) = 0.0;
      motor_command_tmp.kp.at(i) = GetMotorKp(G1MotorType[i]);
      motor_command_tmp.kd.at(i) = GetMotorKd(G1MotorType[i]);
    }

    if (ms) {
      time_ += control_dt_;
      double stage_time = fmod(time_, stage_duration_ * 6); // 6 stages total
      current_stage_ = static_cast<int>(stage_time / stage_duration_);
      double t = stage_time - current_stage_ * stage_duration_; // Time within current stage

      switch (current_stage_) {
        case 0:
          // Stage 0: Initialize to zero position
          InitializePosition(motor_command_tmp, ms, t);
          break;
        case 1:
          // Stage 1: Horse stance preparation
          HorseStancePreparation(motor_command_tmp, t);
          break;
        case 2:
          // Stage 2: Left punch
          LeftPunch(motor_command_tmp, t);
          break;
        case 3:
          // Stage 3: Right punch
          RightPunch(motor_command_tmp, t);
          break;
        case 4:
          // Stage 4: Block motion
          BlockMotion(motor_command_tmp, t);
          break;
        case 5:
          // Stage 5: Return to ready position
          ReturnToReady(motor_command_tmp, t);
          break;
      }

      motor_command_buffer_.SetData(motor_command_tmp);
    }
  }

private:
  void InitializePosition(MotorCommand &cmd, const std::shared_ptr<const MotorState> &ms, double t) {
    // Smoothly transition to zero position
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
      double ratio = std::clamp(t / stage_duration_, 0.0, 1.0);
      cmd.q_target.at(i) = (1.0 - ratio) * ms->q.at(i);
    }
  }

  void HorseStancePreparation(MotorCommand &cmd, double t) {
    double ratio = std::clamp(t / stage_duration_, 0.0, 1.0);
    
    // Bend knees for horse stance
    cmd.q_target.at(LeftKnee) = ratio * (-0.5);
    cmd.q_target.at(RightKnee) = ratio * (-0.5);
    
    // Slight hip bend
    cmd.q_target.at(LeftHipPitch) = ratio * 0.2;
    cmd.q_target.at(RightHipPitch) = ratio * 0.2;
    
    // Arms ready position
    cmd.q_target.at(LeftShoulderPitch) = ratio * (-0.5);
    cmd.q_target.at(RightShoulderPitch) = ratio * (-0.5);
    cmd.q_target.at(LeftElbow) = ratio * 1.0;
    cmd.q_target.at(RightElbow) = ratio * 1.0;
  }

  void LeftPunch(MotorCommand &cmd, double t) {
    double ratio = std::sin(M_PI * t / stage_duration_); // Smooth punch motion
    
    // Maintain horse stance
    cmd.q_target.at(LeftKnee) = -0.5;
    cmd.q_target.at(RightKnee) = -0.5;
    cmd.q_target.at(LeftHipPitch) = 0.2;
    cmd.q_target.at(RightHipPitch) = 0.2;
    
    // Left arm punch
    cmd.q_target.at(LeftShoulderPitch) = -0.5 + ratio * 0.8;
    cmd.q_target.at(LeftElbow) = 1.0 - ratio * 0.8;
    cmd.q_target.at(LeftShoulderRoll) = ratio * 0.3;
    
    // Right arm guard
    cmd.q_target.at(RightShoulderPitch) = -0.5;
    cmd.q_target.at(RightElbow) = 1.0;
    cmd.q_target.at(RightShoulderRoll) = -0.2;
    
    // Slight waist rotation
    cmd.q_target.at(WaistYaw) = ratio * 0.3;
  }

  void RightPunch(MotorCommand &cmd, double t) {
    double ratio = std::sin(M_PI * t / stage_duration_);
    
    // Maintain horse stance
    cmd.q_target.at(LeftKnee) = -0.5;
    cmd.q_target.at(RightKnee) = -0.5;
    cmd.q_target.at(LeftHipPitch) = 0.2;
    cmd.q_target.at(RightHipPitch) = 0.2;
    
    // Right arm punch
    cmd.q_target.at(RightShoulderPitch) = -0.5 + ratio * 0.8;
    cmd.q_target.at(RightElbow) = 1.0 - ratio * 0.8;
    cmd.q_target.at(RightShoulderRoll) = -ratio * 0.3;
    
    // Left arm guard
    cmd.q_target.at(LeftShoulderPitch) = -0.5;
    cmd.q_target.at(LeftElbow) = 1.0;
    cmd.q_target.at(LeftShoulderRoll) = 0.2;
    
    // Opposite waist rotation
    cmd.q_target.at(WaistYaw) = -ratio * 0.3;
  }

  void BlockMotion(MotorCommand &cmd, double t) {
    double ratio = std::clamp(t / stage_duration_, 0.0, 1.0);
    
    // Maintain horse stance
    cmd.q_target.at(LeftKnee) = -0.5;
    cmd.q_target.at(RightKnee) = -0.5;
    cmd.q_target.at(LeftHipPitch) = 0.2;
    cmd.q_target.at(RightHipPitch) = 0.2;
    
    // Both arms up for block
    cmd.q_target.at(LeftShoulderPitch) = ratio * (-1.2);
    cmd.q_target.at(RightShoulderPitch) = ratio * (-1.2);
    cmd.q_target.at(LeftShoulderRoll) = ratio * 0.5;
    cmd.q_target.at(RightShoulderRoll) = ratio * (-0.5);
    cmd.q_target.at(LeftElbow) = 1.0 + ratio * 0.5;
    cmd.q_target.at(RightElbow) = 1.0 + ratio * 0.5;
    
    // Reset waist
    cmd.q_target.at(WaistYaw) = 0.0;
  }

  void ReturnToReady(MotorCommand &cmd, double t) {
    double ratio = std::clamp(t / stage_duration_, 0.0, 1.0);
    
    // Gradually return to ready position
    cmd.q_target.at(LeftKnee) = -0.5 * (1.0 - ratio);
    cmd.q_target.at(RightKnee) = -0.5 * (1.0 - ratio);
    cmd.q_target.at(LeftHipPitch) = 0.2 * (1.0 - ratio);
    cmd.q_target.at(RightHipPitch) = 0.2 * (1.0 - ratio);
    
    // Return arms to ready
    cmd.q_target.at(LeftShoulderPitch) = -0.5 * (1.0 - ratio);
    cmd.q_target.at(RightShoulderPitch) = -0.5 * (1.0 - ratio);
    cmd.q_target.at(LeftElbow) = 1.0 * (1.0 - ratio);
    cmd.q_target.at(RightElbow) = 1.0 * (1.0 - ratio);
    
    // Reset all other joints
    cmd.q_target.at(LeftShoulderRoll) = 0.0;
    cmd.q_target.at(RightShoulderRoll) = 0.0;
    cmd.q_target.at(WaistYaw) = 0.0;
  }
};

int main(int argc, char const *argv[]) {
  if (argc < 2) {
    std::cout << "Usage: g1_kungfu_example network_interface" << std::endl;
    exit(0);
  }
  
  std::string networkInterface = argv[1];
  std::cout << "Starting G1 Kung Fu Example..." << std::endl;
  
  G1KungFuExample kungfu_robot(networkInterface);
  
  while (true) {
    sleep(10);
  }
  
  return 0;
}