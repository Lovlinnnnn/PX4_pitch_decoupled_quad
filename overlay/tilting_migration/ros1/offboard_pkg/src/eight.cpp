/**
 * ROS1 Offboard: ascend 1 m, hover, then fly a keyboard-triggered figure eight.
 * 1: body pitch 0 deg; 2/3: +/-5 deg per press; 8: one figure eight from hover.
 * Pitch keys stay active during the trajectory. No CSV, ESO or pitch slew filter.
 * ROS local setpoints are ENU; custom 32779 pitch is PX4 body FRD/NED radians.
 */

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace keyboard_offboard
{
constexpr double PI = 3.14159265358979323846;
constexpr double ASCENT_DURATION = 10.0;
constexpr double FIGURE8_DURATION = 100.0;
constexpr double FIGURE8_A = 1.5;
constexpr double FIGURE8_B = 1.125;
constexpr int PITCH_STEP_DEG = 5;
constexpr int MAX_PITCH_DEG = 45;
constexpr int MAX_PITCH_STEPS = MAX_PITCH_DEG / PITCH_STEP_DEG;

struct Position {
    double x = 0.0, y = 0.0, z = 0.0;
};

struct Setpoint {
    double x = 0.0, y = 0.0, z = 0.0;
    double vx = 0.0, vy = 0.0, vz = 0.0;
    double ax = 0.0, ay = 0.0, az = 0.0;
    double yaw = 0.0;
};

enum class Phase { TAKEOFF, HOVER, FIGURE8 };

struct Ramp {
    double s, velocity, acceleration;
};

Ramp quintic(double elapsed, double duration)
{
    if (elapsed <= 0.0) { return {0.0, 0.0, 0.0}; }
    if (elapsed >= duration) { return {1.0, 0.0, 0.0}; }
    const double u = elapsed / duration;
    const double u2 = u * u, u3 = u2 * u;
    return {
        u3 * (10.0 + u * (-15.0 + 6.0 * u)),
        30.0 * u2 * (1.0 - u) * (1.0 - u) / duration,
        60.0 * u * (1.0 - u) * (1.0 - 2.0 * u) / (duration * duration)
    };
}

class Mission
{
public:
    Mission(Position initial, double yaw, double started_at)
        : initial_(initial), yaw_(yaw), takeoff_started_(started_at)
    {
        if (!finite(initial) || !std::isfinite(yaw) || !std::isfinite(started_at)) {
            throw std::invalid_argument("initial pose and start time must be finite");
        }
        hover_ = initial_;
        hover_.z += 1.0;
    }

    Phase phase() const { return phase_; }
    int pitch_degrees() const { return pitch_steps_ * PITCH_STEP_DEG; }
    float pitch_radians() const { return static_cast<float>(pitch_degrees() * PI / 180.0); }

    bool key(char key, double now)
    {
        // The requested mission starts accepting commands after reaching 1 m.
        if (phase_ == Phase::TAKEOFF || !std::isfinite(now)) { return false; }
        switch (key) {
        case '1': pitch_steps_ = 0; return true;
        case '2': pitch_steps_ = std::min(MAX_PITCH_STEPS, pitch_steps_ + 1); return true;
        case '3': pitch_steps_ = std::max(-MAX_PITCH_STEPS, pitch_steps_ - 1); return true;
        case '8':
            if (phase_ != Phase::HOVER) { return false; }
            figure8_started_ = now;
            phase_ = Phase::FIGURE8;
            return true;
        default: return false;
        }
    }

    Setpoint update(double now, Position measured)
    {
        Setpoint sp;
        sp.x = hover_.x; sp.y = hover_.y; sp.z = hover_.z; sp.yaw = yaw_;

        if (phase_ == Phase::TAKEOFF) {
            const double elapsed = now - takeoff_started_;
            const Ramp r = quintic(elapsed, ASCENT_DURATION);
            sp.z = initial_.z + r.s;
            sp.vz = r.velocity;
            sp.az = r.acceleration;
            // An elapsed timer alone does not prove the vehicle has arrived.
            if (elapsed >= ASCENT_DURATION && finite(measured)) {
                const double dx = measured.x - hover_.x;
                const double dy = measured.y - hover_.y;
                const double dz = measured.z - hover_.z;
                if (std::sqrt(dx * dx + dy * dy + dz * dz) <= 0.05) {
                    phase_ = Phase::HOVER;
                }
            }
        } else if (phase_ == Phase::FIGURE8) {
            const double elapsed = now - figure8_started_;
            if (elapsed >= FIGURE8_DURATION) {
                phase_ = Phase::HOVER;  // Keep the latest pitch and hover at the center.
            } else if (elapsed > 0.0) {
                // Time reparameterization preserves the figure-eight geometry,
                // with zero velocity and acceleration at both endpoints.
                const Ramp r = quintic(elapsed, FIGURE8_DURATION);
                const double theta = 2.0 * PI * r.s;
                const double omega = 2.0 * PI * r.velocity;
                const double omega_dot = 2.0 * PI * r.acceleration;
                sp.x += FIGURE8_A * std::sin(theta);
                sp.y += FIGURE8_B * std::sin(2.0 * theta);
                sp.vx = FIGURE8_A * std::cos(theta) * omega;
                sp.vy = 2.0 * FIGURE8_B * std::cos(2.0 * theta) * omega;
                sp.ax = FIGURE8_A * (std::cos(theta) * omega_dot
                                    - std::sin(theta) * omega * omega);
                sp.ay = 2.0 * FIGURE8_B * std::cos(2.0 * theta) * omega_dot
                        - 4.0 * FIGURE8_B * std::sin(2.0 * theta) * omega * omega;
            }
        }
        return sp;
    }

private:
    static bool finite(Position p) {
        return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
    }
    Position initial_, hover_;
    double yaw_, takeoff_started_, figure8_started_ = 0.0;
    int pitch_steps_ = 0;
    Phase phase_ = Phase::TAKEOFF;
};

std::array<uint8_t, 128> pitch_payload(float pitch)
{
    static_assert(sizeof(float) == sizeof(uint32_t), "32779 requires float32");
    static_assert(std::numeric_limits<float>::is_iec559, "32779 requires IEEE-754");
    std::array<uint8_t, 128> payload{};
    uint32_t bits = 0;
    std::memcpy(&bits, &pitch, sizeof(bits));
    for (unsigned i = 0; i < 4; ++i) {
        payload[i] = static_cast<uint8_t>(bits >> (8 * i));
    }
    return payload;
}

// Read at most 64 times per cycle. Idle/EOF/error states never disable the next
// cycle, and an input flood cannot indefinitely starve setpoint publication.
template<typename Reader, typename Handler>
void poll_keys(Reader reader, Handler handler)
{
    for (unsigned attempt = 0; attempt < 64; ++attempt) {
        char key = 0;
        const std::ptrdiff_t count = reader(&key);
        if (count == 1) {
            handler(key);
        } else if (count < 0 && errno == EINTR) {
            continue;
        } else {
            break;  // Includes read=0 and EAGAIN/EWOULDBLOCK; try again next cycle.
        }
    }
}
} // namespace keyboard_offboard

#ifndef OFFBOARD_MISSION_TEST
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Tunnel.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>

class NonblockingKeyboard
{
public:
    NonblockingKeyboard()
    {
        if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &old_termios_) != 0) {
            throw std::runtime_error("Run this node in an interactive terminal with rosrun");
        }
        old_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
        if (old_flags_ < 0) { throw std::runtime_error("Cannot read stdin flags"); }
        termios settings = old_termios_;
        settings.c_lflag &= ~(ICANON | ECHO);  // Preserve ISIG, including Ctrl+C.
        settings.c_cc[VMIN] = 0;
        settings.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &settings) != 0) {
            tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
            throw std::runtime_error("Cannot configure keyboard terminal");
        }
        if (fcntl(STDIN_FILENO, F_SETFL, old_flags_ | O_NONBLOCK) != 0) {
            tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
            throw std::runtime_error("Cannot enable nonblocking keyboard");
        }
    }

    ~NonblockingKeyboard()
    {
        fcntl(STDIN_FILENO, F_SETFL, old_flags_);
        tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
    }
    NonblockingKeyboard(const NonblockingKeyboard&) = delete;
    NonblockingKeyboard& operator=(const NonblockingKeyboard&) = delete;

    template<typename Handler> void poll(Handler handler)
    {
        keyboard_offboard::poll_keys(
            [](char* key) {
                const auto count = read(STDIN_FILENO, key, 1);
                if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
                    ROS_ERROR_THROTTLE(1.0, "Keyboard read failed: %s", std::strerror(errno));
                }
                return static_cast<std::ptrdiff_t>(count);
            },
            handler);
    }

private:
    termios old_termios_{};
    int old_flags_ = -1;
};

namespace
{
mavros_msgs::State vehicle_state;
keyboard_offboard::Position measured_position;
double initial_yaw = 0.0;
bool have_pose = false;
ros::WallTime last_pose_received;

void state_cb(const mavros_msgs::State::ConstPtr& msg) { vehicle_state = *msg; }

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    const auto& p = msg->pose.position;
    const auto& q = msg->pose.orientation;
    const double norm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)
        || !std::isfinite(norm2) || norm2 < 1e-12 || msg->header.stamp.isZero()) {
        return;
    }
    measured_position = {p.x, p.y, p.z};
    if (!have_pose) {
        initial_yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y) / norm2,
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z) / norm2);
    }
    have_pose = true;
    last_pose_received = ros::WallTime::now();
}

mavros_msgs::PositionTarget make_position_target(const keyboard_offboard::Setpoint& sp)
{
    mavros_msgs::PositionTarget msg;
    msg.header.stamp = ros::Time::now();
    msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    msg.type_mask = mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    msg.position.x = sp.x; msg.position.y = sp.y; msg.position.z = sp.z;
    msg.velocity.x = sp.vx; msg.velocity.y = sp.vy; msg.velocity.z = sp.vz;
    msg.acceleration_or_force.x = sp.ax;
    msg.acceleration_or_force.y = sp.ay;
    msg.acceleration_or_force.z = sp.az;
    msg.yaw = sp.yaw;
    return msg;
}

void publish_pitch(ros::Publisher& pub, float pitch)
{
    mavros_msgs::Tunnel msg;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.payload_type = 32779;
    msg.payload_length = 4;
    const auto payload = keyboard_offboard::pitch_payload(pitch);
    std::copy(payload.begin(), payload.end(), msg.payload.begin());
    pub.publish(msg);
}

int run_node()
{
    NonblockingKeyboard keyboard;  // Verify keyboard before allowing automatic arming.
    ros::NodeHandle nh;
    const auto state_sub = nh.subscribe("mavros/state", 10, state_cb);
    const auto pose_sub = nh.subscribe("mavros/local_position/pose", 10, pose_cb);
    auto position_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1);
    auto pitch_pub = nh.advertise<mavros_msgs::Tunnel>("mavros/tunnel/in", 1);
    auto mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    auto arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::Rate rate(50.0);

    while (ros::ok() && (!vehicle_state.connected || !have_pose)) {
        keyboard.poll([](char) {});
        ros::spinOnce();
        rate.sleep();
    }
    if (!ros::ok()) { return 0; }

    const keyboard_offboard::Position initial = measured_position;
    keyboard_offboard::Setpoint ground;
    ground.x = initial.x; ground.y = initial.y; ground.z = initial.z; ground.yaw = initial_yaw;
    // Offboard proof of life: pre-send for 2 s at 50 Hz.
    for (int i = 0; ros::ok() && i < 100; ++i) {
        keyboard.poll([](char) {});
        position_pub.publish(make_position_target(ground));
        publish_pitch(pitch_pub, 0.0f);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode mode;
    mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm;
    arm.request.value = true;
    ros::Time last_request = ros::Time::now() - ros::Duration(5.0);
    while (ros::ok() && !(vehicle_state.armed && vehicle_state.mode == "OFFBOARD")) {
        keyboard.poll([](char) {});
        position_pub.publish(make_position_target(ground));
        publish_pitch(pitch_pub, 0.0f);
        if (vehicle_state.connected && ros::Time::now() - last_request >= ros::Duration(5.0)) {
            if (vehicle_state.mode != "OFFBOARD") {
                mode_client.call(mode);
            } else if (!vehicle_state.armed) {
                arming_client.call(arm);
            }
            last_request = ros::Time::now();
        }
        ros::spinOnce();
        rate.sleep();
    }
    if (!ros::ok()) { return 0; }

    keyboard_offboard::Mission mission(initial, initial_yaw, ros::Time::now().toSec());
    ROS_INFO("Ascending 1 m at zero body-pitch reference; keyboard commands become active after arrival.");
    while (ros::ok()) {
        ros::spinOnce();
        if (!vehicle_state.armed || vehicle_state.mode != "OFFBOARD") {
            ROS_WARN("Offboard/arming ended; keyboard mission exits without rearming.");
            break;
        }
        const double now = ros::Time::now().toSec();
        auto measured = measured_position;
        if ((ros::WallTime::now() - last_pose_received).toSec() > 0.5) {
            measured.x = std::numeric_limits<double>::quiet_NaN();
        }
        const auto previous_phase = mission.phase();
        // Update before reading keys: arrival immediately enables this cycle's input.
        auto sp = mission.update(now, measured);
        if (previous_phase != mission.phase() && mission.phase() == keyboard_offboard::Phase::HOVER) {
            ROS_INFO("Hover: 1=0 deg, 2=+5 deg, 3=-5 deg, 8=figure eight (100 s). Pitch range: +/-45 deg.");
        }
        keyboard.poll([&](char key) {
            if (mission.key(key, now)) {
                if (key == '8') {
                    ROS_INFO("Figure eight started; pitch keys 1/2/3 remain active.");
                } else {
                    ROS_INFO("Desired body pitch: %d deg", mission.pitch_degrees());
                }
            }
        });
        sp = mission.update(now, measured);
        // Publication runs every cycle, whether or not any key was pressed.
        position_pub.publish(make_position_target(sp));
        publish_pitch(pitch_pub, mission.pitch_radians());
        rate.sleep();
    }
    return 0;
}
} // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "offb_node_keyboard_pitch_figure8");
    try {
        return run_node();
    } catch (const std::exception& error) {
        ROS_FATAL("Offboard node failed: %s", error.what());
        return 1;
    }
}
#endif // OFFBOARD_MISSION_TEST

