#include <array>
#include <chrono>
#include <iostream>
#include <thread>

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

static const std::string kTopicArmSDK = "rt/arm_sdk";
static const std::string kTopicState = "rt/lowstate";
constexpr float kPi = 3.141592654;
constexpr float kPi_2 = 1.57079632;

enum JointIndex {
    // Left leg
    kLeftHipPitch,
    kLeftHipRoll,
    kLeftHipYaw,
    kLeftKnee,
    kLeftAnkle,
    kLeftAnkleRoll,

    // Right leg
    kRightHipPitch,
    kRightHipRoll,
    kRightHipYaw,
    kRightKnee,
    kRightAnkle,
    kRightAnkleRoll,

    kWaistYaw,
    kWaistRoll,
    kWaistPitch,

    // Left arm
    kLeftShoulderPitch,
    kLeftShoulderRoll,
    kLeftShoulderYaw,
    kLeftElbow,
    kLeftWristRoll,
    kLeftWristPitch,
    kLeftWristYaw,
    // Right arm
    kRightShoulderPitch,
    kRightShoulderRoll,
    kRightShoulderYaw,
    kRightElbow,
    kRightWristRoll,
    kRightWristPitch,
    kRightWristYaw,

    // These joints are used to make motion unit (High Level) to know that it cannot control the motors on arm because the users are going to control the arm by themselfs.
    kNotUsedJoint,
    kNotUsedJoint1,
    kNotUsedJoint2,
    kNotUsedJoint3,
    kNotUsedJoint4,
    kNotUsedJoint5
};

class ArmController {
public:
    ArmController(const std::string& networkInterface) {
        unitree::robot::ChannelFactory::Instance()->Init(0, networkInterface.c_str());

        // Initialize publisher
        arm_sdk_publisher_.reset(
            new unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(
                kTopicArmSDK));
        arm_sdk_publisher_->InitChannel();

        // Initialize subscriber
        low_state_subscriber_.reset(
            new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(
                kTopicState));
        low_state_subscriber_->InitChannel([this](const void* msg) {
            auto s = (const unitree_hg::msg::dds_::LowState_*)msg;
            memcpy(&state_msg_, s, sizeof(unitree_hg::msg::dds_::LowState_));
        }, 1);

        // Arm joint indices
        arm_joints_ = {
            JointIndex::kLeftShoulderPitch,  JointIndex::kLeftShoulderRoll,
            JointIndex::kLeftShoulderYaw,    JointIndex::kLeftElbow,
            JointIndex::kLeftWristRoll,      JointIndex::kLeftWristPitch,
            JointIndex::kLeftWristYaw,
            JointIndex::kRightShoulderPitch, JointIndex::kRightShoulderRoll,
            JointIndex::kRightShoulderYaw,   JointIndex::kRightElbow,
            JointIndex::kRightWristRoll,     JointIndex::kRightWristPitch,
            JointIndex::kRightWristYaw,
            JointIndex::kWaistYaw,
            JointIndex::kWaistRoll,
            JointIndex::kWaistPitch};

        // Default control parameters
        control_dt_ = 0.02f;
        max_joint_velocity_ = 0.5f;
        sleep_time_ = std::chrono::milliseconds(static_cast<int>(control_dt_ / 0.001f));
        
        // Default PD gains
        kp_ = 60.f;
        kd_ = 1.5f;
        
        // Default target positions (arms down)
        init_pos_ = {0, 0, 0, 0, 0, 0, 0,
                     0, 0, 0, 0, 0, 0, 0,
                     0, 0, 0};
        
        // Default target positions (arms up)
        target_pos_ = {0.f, kPi_2,  0.f, kPi_2, 0.f, 0.f, 0.f,
                       0.f, -kPi_2, 0.f, kPi_2, 0.f, 0.f, 0.f, 
                       0, 0, 0};
    }

    // Initialize arms to default position
    bool InitializeArms(float init_time = 2.0f) {
        std::cout << "Initializing arms ..." << std::endl;
        
        // Get current joint positions
        std::array<float, 17> current_jpos;
        for (int i = 0; i < arm_joints_.size(); ++i) {
            current_jpos.at(i) = state_msg_.motor_state().at(arm_joints_.at(i)).q();
        }

        int init_time_steps = static_cast<int>(init_time / control_dt_);

        for (int i = 0; i < init_time_steps; ++i) {
            // Set weight to 1.0 (full control)
            msg_.motor_cmd().at(JointIndex::kNotUsedJoint).q(1.0f);
            msg_.motor_cmd().at(JointIndex::kLeftHipPitch).q(1.0f);
            msg_.motor_cmd().at(JointIndex::kLeftHipRoll).q(1.0f);
            msg_.motor_cmd().at(JointIndex::kLeftHipYaw).q(1.0f);
            msg_.motor_cmd().at(JointIndex::kLeftKnee).q(1.0f);
            msg_.motor_cmd().at(JointIndex::kLeftAnkle).q(1.0f);
            msg_.motor_cmd().at(JointIndex::kLeftAnkleRoll).q(1.0f);
            msg_.motor_cmd().at(JointIndex::kRightHipPitch).q(1.0f);
            msg_.motor_cmd().at(JointIndex::kRightHipRoll).q(1.0f);
            msg_.motor_cmd().at(JointIndex::kRightHipYaw).q(1.0f);
            msg_.motor_cmd().at(JointIndex::kRightKnee).q(1.0f);
            msg_.motor_cmd().at(JointIndex::kRightAnkle).q(1.0f);
            msg_.motor_cmd().at(JointIndex::kRightAnkleRoll).q(1.0f);



            float phase = 1.0f * i / init_time_steps;

            // Interpolate between current and target positions
            for (int j = 0; j < init_pos_.size(); ++j) {
                float target = init_pos_.at(j) * phase + current_jpos.at(j) * (1 - phase);
                SetJointCommand(arm_joints_.at(j), target, 0.f, kp_, kd_, 0.f);
            }

            // Send command and sleep
            arm_sdk_publisher_->Write(msg_);
            std::this_thread::sleep_for(sleep_time_);
        }

        std::cout << "Initialization done!" << std::endl;
        return true;
    }

    // Move arms to target position
    bool MoveArmsToPosition(const std::array<float, 17>& target_pos, float duration = 5.0f) {
        std::cout << "Moving arms to target position..." << std::endl;
        
        // Get current desired positions (from message)
        std::array<float, 17> current_jpos_des;
        for (int i = 0; i < arm_joints_.size(); ++i) {
            current_jpos_des.at(i) = msg_.motor_cmd().at(arm_joints_.at(i)).q();
        }

        int num_steps = static_cast<int>(duration / control_dt_);
        float max_joint_delta = max_joint_velocity_ * control_dt_;

        for (int i = 0; i < num_steps; ++i) {
            // Update desired positions with velocity limits
            for (int j = 0; j < target_pos.size(); ++j) {
                current_jpos_des.at(j) += 
                    std::clamp(target_pos.at(j) - current_jpos_des.at(j),
                              -max_joint_delta, max_joint_delta);
                
                SetJointCommand(arm_joints_.at(j), 
                              current_jpos_des.at(j), 
                              0.f, kp_, kd_, 0.f);
            }

            // Send command and sleep
            arm_sdk_publisher_->Write(msg_);
            std::this_thread::sleep_for(sleep_time_);
        }

        std::cout << "Movement completed!" << std::endl;
        return true;
    }

    // Stop arm control gradually
    bool StopControl(float stop_time = 2.0f) {
        std::cout << "Stopping arm control..." << std::endl;
        
        float weight = 1.0f;
        float delta_weight = 0.2f * control_dt_;
        int stop_steps = static_cast<int>(stop_time / control_dt_);

        for (int i = 0; i < stop_steps; ++i) {
            // Gradually reduce weight
            weight -= delta_weight;
            weight = std::clamp(weight, 0.f, 1.f);
            msg_.motor_cmd().at(JointIndex::kNotUsedJoint).q(weight);

            // Send command and sleep
            arm_sdk_publisher_->Write(msg_);
            std::this_thread::sleep_for(sleep_time_);
        }

        std::cout << "Control stopped!" << std::endl;
        return true;
    }

    // Set individual joint command
    void SetJointCommand(JointIndex joint, float q, float dq, float kp, float kd, float tau) {
        msg_.motor_cmd().at(joint).q(q);
        msg_.motor_cmd().at(joint).dq(dq);
        msg_.motor_cmd().at(joint).kp(kp);
        msg_.motor_cmd().at(joint).kd(kd);
        msg_.motor_cmd().at(joint).tau(tau);
    }

    // Get current joint positions
    std::array<float, 17> GetCurrentJointPositions() {
        std::array<float, 17> positions;
        for (int i = 0; i < arm_joints_.size(); ++i) {
            positions.at(i) = state_msg_.motor_state().at(arm_joints_.at(i)).q();
        }
        return positions;
    }

    // Set control parameters
    void SetControlParameters(float kp, float kd, float dt, float max_vel) {
        kp_ = kp;
        kd_ = kd;
        control_dt_ = dt;
        max_joint_velocity_ = max_vel;
        sleep_time_ = std::chrono::milliseconds(static_cast<int>(control_dt_ / 0.001f));
    }

    // Set target positions
    void SetTargetPositions(const std::array<float, 17>& target_pos) {
        target_pos_ = target_pos;
    }

private:
    unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_> arm_sdk_publisher_;
    unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> low_state_subscriber_;
    
    unitree_hg::msg::dds_::LowCmd_ msg_;
    unitree_hg::msg::dds_::LowState_ state_msg_;

    std::array<JointIndex, 17> arm_joints_;
    std::array<float, 17> init_pos_;
    std::array<float, 17> target_pos_;

    float kp_;
    float kd_;
    float control_dt_;
    float max_joint_velocity_;
    std::chrono::milliseconds sleep_time_;
};

int main(int argc, char const *argv[]) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        return -1;
    }

    // Create arm controller
    ArmController arm_controller(argv[1]);

    // Wait for user input to initialize
    std::cout << "Press ENTER to initialize arms..." << std::endl;
    std::cin.get();
    arm_controller.InitializeArms();

    // Wait for user input to start movement
    std::cout << "Press ENTER to move arms up..." << std::endl;
    std::cin.get();
    
    // Move arms to target (up) position
    std::array<float, 17> target_up = {0.f, kPi_2,  0.f, kPi_2, 0.f, 0.f, 0.f,
                                      0.f, -kPi_2, 0.f, kPi_2, 0.f, 0.f, 0.f, 
                                      0, 0, 0};
    arm_controller.MoveArmsToPosition(target_up);

    // Wait for user input to move arms down
    std::cout << "Press ENTER to move arms down..." << std::endl;
    std::cin.get();
    
    // Move arms back to initial (down) position
    std::array<float, 17> target_down = {0, 0, 0, 0, 0, 0, 0,
                                        0, 0, 0, 0, 0, 0, 0,
                                        0, 0, 0};
    arm_controller.MoveArmsToPosition(target_down);

    // Stop control
    std::cout << "Press ENTER to stop control..." << std::endl;
    std::cin.get();
    arm_controller.StopControl();

    return 0;
}