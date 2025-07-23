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

// Smooth interpolation functions for better motion flow
class MotionInterpolator {
public:
    // Quintic polynomial for smooth acceleration/deceleration
    static double quinticEaseInOut(double t) {
        if (t <= 0.0) return 0.0;
        if (t >= 1.0) return 1.0;
        return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
    }
    
    // Sigmoid function for natural motion curves
    static double sigmoidEase(double t, double steepness = 6.0) {
        if (t <= 0.0) return 0.0;
        if (t >= 1.0) return 1.0;
        double exp_val = exp(-steepness * (t - 0.5));
        return 1.0 / (1.0 + exp_val);
    }
    
    // Custom kick curve for explosive motion
    static double kickCurve(double t) {
        if (t <= 0.0) return 0.0;
        if (t >= 1.0) return 1.0;
        // Fast acceleration, then deceleration
        return 1.0 - (1.0 - t) * (1.0 - t) * (1.0 - t);
    }
    
    // Smooth transition between two values
    static double smoothTransition(double from, double to, double t, double (*easing)(double) = quinticEaseInOut) {
        return from + (to - from) * easing(t);
    }
    
    // Velocity-based interpolation for continuous motion
    static double velocitySmooth(double current, double target, double dt, double max_velocity = 2.0) {
        double diff = target - current;
        double max_change = max_velocity * dt;
        if (std::abs(diff) <= max_change) {
            return target;
        }
        return current + (diff > 0 ? max_change : -max_change);
    }
};

// Joint state tracking for smooth transitions
struct JointState {
    double position = 0.0;
    double velocity = 0.0;
    double target = 0.0;
    double prev_target = 0.0;
    
    void update(double new_target, double dt) {
        prev_target = target;
        target = new_target;
        
        // Calculate smooth velocity
        double target_velocity = (target - position) / dt;
        velocity = 0.7 * velocity + 0.3 * target_velocity; // Low-pass filter
        
        // Update position with velocity limiting
        double max_vel = 3.0; // rad/s
        velocity = std::clamp(velocity, -max_vel, max_vel);
        position += velocity * dt;
    }
};

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

// Adaptive gains based on motion phase
float GetMotorKp(MotorType type, bool is_dynamic_phase = false) {
  float base_kp;
  switch (type) {
    case GearboxS: base_kp = 40; break;
    case GearboxM: base_kp = 60; break;
    case GearboxL: base_kp = 100; break;
    default: base_kp = 40; break;
  }
  // Reduce stiffness during dynamic phases for smoother motion
  return is_dynamic_phase ? base_kp * 0.8 : base_kp;
}
 
float GetMotorKd(MotorType type, bool is_dynamic_phase = false) {
  float base_kd;
  switch (type) {
    case GearboxS: base_kd = 1; break;
    case GearboxM: base_kd = 1; break;
    case GearboxL: base_kd = 2; break;
    default: base_kd = 1; break;
  }
  // Increase damping during dynamic phases for stability
  return is_dynamic_phase ? base_kd * 1.5 : base_kd;
}
 
class G1BackKickExample {
private:
  double time_;
  double control_dt_;
  double stage_duration_;
  int current_stage_;
  Mode mode_pr_;
  uint8_t mode_machine_;
  int counter_;
  
  // Motion state tracking
  std::array<JointState, G1_NUM_MOTOR> joint_states_;
  std::array<double, G1_NUM_MOTOR> reference_positions_;
  bool first_run_;
  double phase_transition_time_;
  int prev_stage_;
 
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
  G1BackKickExample(std::string networkInterface)
      : time_(0.0),
        control_dt_(0.002),
        stage_duration_(2.0),  // Increased for smoother motion
        current_stage_(0),
        mode_pr_(Mode::PR),
        mode_machine_(0),
        counter_(0),
        first_run_(true),
        phase_transition_time_(0.0),
        prev_stage_(-1) {
    
    // Initialize joint states and reference positions
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
        joint_states_[i] = JointState();
        reference_positions_[i] = 0.0;
    }
    
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
    lowstate_subscriber_->InitChannel(std::bind(&G1BackKickExample::LowStateHandler, this, std::placeholders::_1), 1);
    
    imutorso_subscriber_.reset(new ChannelSubscriber<IMUState_>(HG_IMU_TORSO));
    imutorso_subscriber_->InitChannel(std::bind(&G1BackKickExample::imuTorsoHandler, this, std::placeholders::_1), 1);
 
    // Create control threads
    command_writer_ptr_ = CreateRecurrentThreadEx("command_writer", UT_CPU_ID_NONE, 2000, &G1BackKickExample::LowCommandWriter, this);
    control_thread_ptr_ = CreateRecurrentThreadEx("control", UT_CPU_ID_NONE, 2000, &G1BackKickExample::Control, this);
    
    std::cout << "G1 Back Kick Example initialized with optimized fluid motion..." << std::endl;
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
 
    // Get motor state and initialize reference positions on first run
    MotorState ms_tmp;
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
      ms_tmp.q.at(i) = low_state.motor_state()[i].q();
      ms_tmp.dq.at(i) = low_state.motor_state()[i].dq();
      
      if (first_run_) {
        reference_positions_[i] = ms_tmp.q.at(i);
        joint_states_[i].position = ms_tmp.q.at(i);
      }
    }
    motor_state_buffer_.SetData(ms_tmp);
    
    if (first_run_) {
      first_run_ = false;
      std::cout << "Reference positions initialized from current robot state" << std::endl;
    }
 
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
      printf("Stage: %d, IMU.pelvis.rpy: %.2f %.2f %.2f, Phase: %.2fs\n",
             current_stage_, rpy[0], rpy[1], rpy[2], phase_transition_time_);
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
 
    if (!ms) return;
    
    time_ += control_dt_;
    double stage_time = fmod(time_, stage_duration_ * 7); // 7 stages for smoother motion
    current_stage_ = static_cast<int>(stage_time / stage_duration_);
    double t = stage_time - current_stage_ * stage_duration_; // Time within current stage
    double normalized_t = t / stage_duration_;
    
    // Track phase transitions
    if (current_stage_ != prev_stage_) {
      phase_transition_time_ = 0.0;
      prev_stage_ = current_stage_;
      std::cout << "Transitioning to stage " << current_stage_ << std::endl;
    }
    phase_transition_time_ += control_dt_;
    
    // Determine if we're in a dynamic phase
    bool is_dynamic_phase = (current_stage_ == 3 || current_stage_ == 4); // Kick execution and retraction
    
    // Initialize all motor commands
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
      motor_command_tmp.tau_ff.at(i) = 0.0;
      motor_command_tmp.dq_target.at(i) = 0.0;
      motor_command_tmp.kp.at(i) = GetMotorKp(G1MotorType[i], is_dynamic_phase);
      motor_command_tmp.kd.at(i) = GetMotorKd(G1MotorType[i], is_dynamic_phase);
    }
    
    // Define target positions for current stage
    std::array<double, G1_NUM_MOTOR> stage_targets = reference_positions_;
    
    switch (current_stage_) {
      case 0:
        // Stage 0: Smooth transition to ready position
        TransitionToReady(stage_targets, normalized_t);
        break;
      case 1:
        // Stage 1: Weight shift preparation
        PrepareWeightShift(stage_targets, normalized_t);
        break;
      case 2:
        // Stage 2: Pivot and chamber kicking leg behind
        PivotAndChamberBehind(stage_targets, normalized_t);
        break;
      case 3:
        // Stage 3: Execute back kick with hip extension
        ExecuteBackKick(stage_targets, normalized_t);
        break;
      case 4:
        // Stage 4: Controlled retraction from back position
        RetractFromBack(stage_targets, normalized_t);
        break;
      case 5:
        // Stage 5: Lower leg and rebalance
        LowerLegAndRebalance(stage_targets, normalized_t);
        break;
      case 6:
        // Stage 6: Return to neutral stance
        ReturnToNeutral(stage_targets, normalized_t);
        break;
    }
    
    // Update joint states with smooth transitions
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
      joint_states_[i].update(stage_targets[i], control_dt_);
      motor_command_tmp.q_target.at(i) = joint_states_[i].position;
      
      // Add velocity feedforward for smoother motion
      motor_command_tmp.dq_target.at(i) = joint_states_[i].velocity * 0.3;
    }
    
    motor_command_buffer_.SetData(motor_command_tmp);
  }
 
private:
  void TransitionToReady(std::array<double, G1_NUM_MOTOR> &targets, double t) {
    double ease_t = MotionInterpolator::quinticEaseInOut(t);
    
    // Slight ready stance with balanced weight distribution
    targets[LeftHipPitch] = reference_positions_[LeftHipPitch] + ease_t * 0.05;
    targets[RightHipPitch] = reference_positions_[RightHipPitch] + ease_t * 0.05;
    targets[LeftKnee] = reference_positions_[LeftKnee] + ease_t * (-0.1);
    targets[RightKnee] = reference_positions_[RightKnee] + ease_t * (-0.1);
    
    // Arms in ready position
    targets[LeftShoulderPitch] = reference_positions_[LeftShoulderPitch] + ease_t * (-0.2);
    targets[RightShoulderPitch] = reference_positions_[RightShoulderPitch] + ease_t * (-0.2);
  }
  
  void PrepareWeightShift(std::array<double, G1_NUM_MOTOR> &targets, double t) {
    double ease_t = MotionInterpolator::sigmoidEase(t, 4.0);
    
    // Gradual weight shift to right leg
    targets[RightHipPitch] = reference_positions_[RightHipPitch] + 0.05 + ease_t * 0.08;
    targets[RightKnee] = reference_positions_[RightKnee] + (-0.1) + ease_t * (-0.15);
    targets[RightAnklePitch] = reference_positions_[RightAnklePitch] + ease_t * 0.12;
    targets[RightHipRoll] = reference_positions_[RightHipRoll] + ease_t * 0.08;
    
    // Prepare left leg for lifting
    targets[LeftHipPitch] = reference_positions_[LeftHipPitch] + 0.05 + ease_t * 0.05;
    targets[LeftKnee] = reference_positions_[LeftKnee] + (-0.1) + ease_t * (-0.05);
    targets[LeftHipRoll] = reference_positions_[LeftHipRoll] + ease_t * (-0.06);
    
    // Progressive arm movement for balance
    targets[LeftShoulderPitch] = reference_positions_[LeftShoulderPitch] + (-0.2) + ease_t * (-0.15);
    targets[RightShoulderPitch] = reference_positions_[RightShoulderPitch] + (-0.2) + ease_t * (-0.15);
    targets[LeftShoulderRoll] = reference_positions_[LeftShoulderRoll] + ease_t * 0.25;
    targets[RightShoulderRoll] = reference_positions_[RightShoulderRoll] + ease_t * (-0.25);
  }
  
  void PivotAndChamberBehind(std::array<double, G1_NUM_MOTOR> &targets, double t) {
    double ease_t = MotionInterpolator::quinticEaseInOut(t);
    
    // Maintain weight on right leg (standing leg)
    targets[RightHipPitch] = reference_positions_[RightHipPitch] + 0.13 + ease_t * 0.05;
    targets[RightKnee] = reference_positions_[RightKnee] + (-0.25) + ease_t * (-0.1);
    targets[RightAnklePitch] = reference_positions_[RightAnklePitch] + 0.12 + ease_t * 0.03;
    targets[RightHipRoll] = reference_positions_[RightHipRoll] + 0.08;
    
    // Chamber left leg behind - key difference from front kick
    targets[LeftHipPitch] = reference_positions_[LeftHipPitch] + 0.1 + ease_t * (-0.6); // Move hip BACK instead of forward
    targets[LeftKnee] = reference_positions_[LeftKnee] + (-0.15) + ease_t * (-0.8);     // Bend knee to chamber
    targets[LeftAnklePitch] = reference_positions_[LeftAnklePitch] + ease_t * (-0.2);   // Point foot down
    targets[LeftHipRoll] = reference_positions_[LeftHipRoll] + (-0.06);
    
    // Torso lean forward slightly to counterbalance back leg
    targets[WaistPitch] = reference_positions_[WaistPitch] + ease_t * 0.15;
    
    // Arms for balance - opposite to front kick
    targets[LeftShoulderPitch] = reference_positions_[LeftShoulderPitch] + (-0.35) + ease_t * 0.2;  // Swing forward
    targets[RightShoulderPitch] = reference_positions_[RightShoulderPitch] + (-0.35) + ease_t * 0.2;
    targets[LeftShoulderRoll] = reference_positions_[LeftShoulderRoll] + 0.25 + ease_t * (-0.1);
    targets[RightShoulderRoll] = reference_positions_[RightShoulderRoll] + (-0.25) + ease_t * 0.1;
  }
  
  void ExecuteBackKick(std::array<double, G1_NUM_MOTOR> &targets, double t) {
    double kick_curve_t = MotionInterpolator::kickCurve(t); // Explosive motion
    
    // Maintain strong stance on right leg
    targets[RightHipPitch] = reference_positions_[RightHipPitch] + 0.18;
    targets[RightKnee] = reference_positions_[RightKnee] + (-0.35);
    targets[RightAnklePitch] = reference_positions_[RightAnklePitch] + 0.15;
    targets[RightHipRoll] = reference_positions_[RightHipRoll] + 0.08;
    
    // Execute back kick - extend leg backwards with power
    targets[LeftHipPitch] = reference_positions_[LeftHipPitch] + (-0.5) + kick_curve_t * (-0.4); // Strong hip extension
    targets[LeftKnee] = reference_positions_[LeftKnee] + (-0.95) + kick_curve_t * 0.7;            // Extend knee backwards
    targets[LeftAnklePitch] = reference_positions_[LeftAnklePitch] + (-0.2) + kick_curve_t * (-0.1); // Firm foot position
    targets[LeftHipRoll] = reference_positions_[LeftHipRoll] + (-0.06);
    
    // Torso compensation for power and balance
    targets[WaistPitch] = reference_positions_[WaistPitch] + 0.15 + kick_curve_t * 0.1;
    targets[WaistYaw] = reference_positions_[WaistYaw] + kick_curve_t * (-0.05); // Slight rotation for power
    
    // Arms swing forward for counterbalance
    targets[LeftShoulderPitch] = reference_positions_[LeftShoulderPitch] + (-0.15) + kick_curve_t * (-0.3);
    targets[RightShoulderPitch] = reference_positions_[RightShoulderPitch] + (-0.15) + kick_curve_t * (-0.3);
    targets[LeftShoulderRoll] = reference_positions_[LeftShoulderRoll] + 0.15;
    targets[RightShoulderRoll] = reference_positions_[RightShoulderRoll] + (-0.15);
    
    // Add slight elbow bend for natural arm motion
    targets[LeftElbow] = reference_positions_[LeftElbow] + kick_curve_t * (-0.3);
    targets[RightElbow] = reference_positions_[RightElbow] + kick_curve_t * (-0.3);
  }
  
  void RetractFromBack(std::array<double, G1_NUM_MOTOR> &targets, double t) {
    double ease_t = MotionInterpolator::quinticEaseInOut(t);
    
    // Maintain stance on right leg
    targets[RightHipPitch] = reference_positions_[RightHipPitch] + 0.18 - ease_t * 0.05;
    targets[RightKnee] = reference_positions_[RightKnee] + (-0.35) + ease_t * 0.1;
    targets[RightAnklePitch] = reference_positions_[RightAnklePitch] + 0.15 - ease_t * 0.03;
    targets[RightHipRoll] = reference_positions_[RightHipRoll] + 0.08;
    
    // Retract left leg back to chambered position
    targets[LeftHipPitch] = reference_positions_[LeftHipPitch] + (-0.9) + ease_t * 0.4;  // Bring back from extension
    targets[LeftKnee] = reference_positions_[LeftKnee] + (-0.25) + ease_t * (-0.7);      // Re-chamber knee
    targets[LeftAnklePitch] = reference_positions_[LeftAnklePitch] + (-0.3) + ease_t * 0.1;
    targets[LeftHipRoll] = reference_positions_[LeftHipRoll] + (-0.06);
    
    // Gradually reduce torso lean
    targets[WaistPitch] = reference_positions_[WaistPitch] + 0.25 - ease_t * 0.1;
    targets[WaistYaw] = reference_positions_[WaistYaw] + (-0.05) + ease_t * 0.05;
    
    // Return arms to balanced position
    targets[LeftShoulderPitch] = reference_positions_[LeftShoulderPitch] + (-0.45) + ease_t * 0.15;
    targets[RightShoulderPitch] = reference_positions_[RightShoulderPitch] + (-0.45) + ease_t * 0.15;
    targets[LeftShoulderRoll] = reference_positions_[LeftShoulderRoll] + 0.15;
    targets[RightShoulderRoll] = reference_positions_[RightShoulderRoll] + (-0.15);
    
    targets[LeftElbow] = reference_positions_[LeftElbow] + (-0.3) + ease_t * 0.3;
    targets[RightElbow] = reference_positions_[RightElbow] + (-0.3) + ease_t * 0.3;
  }
  
  void LowerLegAndRebalance(std::array<double, G1_NUM_MOTOR> &targets, double t) {
    double ease_t = MotionInterpolator::sigmoidEase(t, 3.0);
    
    // Gradually reduce weight on right leg
    targets[RightHipPitch] = reference_positions_[RightHipPitch] + 0.13 - ease_t * 0.08;
    targets[RightKnee] = reference_positions_[RightKnee] + (-0.25) + ease_t * 0.15;
    targets[RightAnklePitch] = reference_positions_[RightAnklePitch] + 0.12 - ease_t * 0.07;
    targets[RightHipRoll] = reference_positions_[RightHipRoll] + 0.08 - ease_t * 0.05;
    
    // Lower left leg towards ground
    targets[LeftHipPitch] = reference_positions_[LeftHipPitch] + (-0.5) + ease_t * 0.55; // Bring forward to neutral
    targets[LeftKnee] = reference_positions_[LeftKnee] + (-0.95) + ease_t * 0.85;        // Straighten knee
    targets[LeftAnklePitch] = reference_positions_[LeftAnklePitch] + (-0.2) + ease_t * 0.2;
    targets[LeftHipRoll] = reference_positions_[LeftHipRoll] + (-0.06) + ease_t * 0.06;
    
    // Return torso to upright
    targets[WaistPitch] = reference_positions_[WaistPitch] + 0.15 - ease_t * 0.15;
    targets[WaistYaw] = reference_positions_[WaistYaw]; // Already returned to neutral
    
    // Return arms to ready position
    targets[LeftShoulderPitch] = reference_positions_[LeftShoulderPitch] + (-0.3) + ease_t * 0.1;
    targets[RightShoulderPitch] = reference_positions_[RightShoulderPitch] + (-0.3) + ease_t * 0.1;
    targets[LeftShoulderRoll] = reference_positions_[LeftShoulderRoll] + 0.15 - ease_t * 0.15;
    targets[RightShoulderRoll] = reference_positions_[RightShoulderRoll] + (-0.15) + ease_t * 0.15;
  }
  
  void LiftAndChamber(std::array<double, G1_NUM_MOTOR> &targets, double t) {
    // This method is replaced by PivotAndChamberBehind for back kick
    PivotAndChamberBehind(targets, t);
