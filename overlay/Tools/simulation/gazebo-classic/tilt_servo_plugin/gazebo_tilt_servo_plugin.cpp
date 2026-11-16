// SPDX-License-Identifier: BSD-3-Clause

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <gazebo/common/Console.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

namespace gazebo
{

class GazeboTiltServoPlugin : public ModelPlugin
{
public:
	~GazeboTiltServoPlugin() override
	{
		{
			// A queued Gazebo callback can outlive the event connection.
			std::lock_guard<std::mutex> guard(_event_gate->mutex);
			_event_gate->active = false;
		}

		_update_connection.reset();
		_command_subscriber.shutdown();

		if (_spinner) {
			_spinner->stop();
		}
	}

	void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
	{
		if (!ros::isInitialized()) {
			gzerr << "Tilt servo requires the ROS1 libgazebo_ros_api_plugin.so system plugin.\n";
			return;
		}

		_controller = model->GetJointController();

		if (!_controller || model->IsStatic() || !readConfiguration(model, sdf)) {
			gzerr << "Invalid tilt servo configuration for model " << model->GetName() << ".\n";
			return;
		}

		const std::string ns = "/" + model->GetName();
		_node = std::make_unique<ros::NodeHandle>(ns);
		_node->setCallbackQueue(&_command_queue);
		_state_publisher = _node->advertise<sensor_msgs::JointState>("joint_states", 1, false);
		_command_subscriber = _node->subscribe("tilt/command", 1, &GazeboTiltServoPlugin::receiveCommand, this);
		_spinner = std::make_unique<ros::AsyncSpinner>(1, &_command_queue);
		_spinner->start();

		const auto gate = _event_gate;
		_update_connection = event::Events::ConnectWorldUpdateBegin(
		[this, gate](const common::UpdateInfo &info) {
			std::lock_guard<std::mutex> guard(gate->mutex);

			if (gate->active) {
				update(info.simTime);
			}
		});
		gzmsg << "Tilt servo: Gazebo position PID active on " << _joints[0].name
		      << " and " << _joints[1].name
		      << ", command " << ns << "/tilt/command, measured state " << ns << "/joint_states\n";
	}

	void Reset() override
	{
		std::lock_guard<std::mutex> guard(_event_gate->mutex);
		resetTracking();
	}

private:
	using SteadyClock = std::chrono::steady_clock;

	struct EventGate {
		std::mutex mutex;
		bool active{true};
	};

	struct ServoJoint {
		physics::JointPtr handle;
		std::string name;
		std::string scoped_name;
		double sign{1.};
		double zero{0.};
		double effort_limit{0.};
	};

	template<typename T>
	static T readValue(const sdf::ElementPtr &sdf, const std::string &key, const T &value)
	{
		return sdf->HasElement(key) ? sdf->Get<T>(key) : value;
	}

	bool readConfiguration(const physics::ModelPtr &model, const sdf::ElementPtr &sdf)
	{
		constexpr double angle_limit_35_degrees = 0.6108652381980153;
		constexpr double half_pi = 1.5707963267948966;
		const double angle_limit = readValue(sdf, "maxAngleRad", angle_limit_35_degrees);
		const double torque_limit = readValue(sdf, "maxTorqueNm", 1.2);
		const double feedback_rate = readValue(sdf, "feedbackRateHz", 50.);
		_target_rate = readValue(sdf, "maxRateRadS", 1.);
		_timeout_seconds = readValue(sdf, "commandTimeoutSec", 0.5);
		_position_p = readValue(sdf, "positionP", 3.);
		_position_i = readValue(sdf, "positionI", 0.);
		_position_d = readValue(sdf, "positionD", 0.2);

		if (!(std::isfinite(angle_limit) && angle_limit > 0. && angle_limit < half_pi
		      && std::isfinite(torque_limit) && torque_limit > 0.
		      && std::isfinite(_target_rate) && _target_rate > 0.
		      && std::isfinite(feedback_rate) && feedback_rate >= 1. && feedback_rate <= 1000.
		      && std::isfinite(_timeout_seconds) && _timeout_seconds > 0.
		      && std::isfinite(_position_p) && _position_p > 0.
		      && std::isfinite(_position_i) && _position_i >= 0.
		      && std::isfinite(_position_d) && _position_d >= 0.)) {
			return false;
		}

		_feedback_period = common::Time(1. / feedback_rate);
		_min_angle = -angle_limit;
		_max_angle = angle_limit;
		_joints[0].name = readValue(sdf, "frontJoint", std::string("front_tilt_joint"));
		_joints[1].name = readValue(sdf, "rearJoint", std::string("behind_tilt_joint"));
		_joints[0].sign = readValue(sdf, "frontSign", 1.);
		_joints[1].sign = readValue(sdf, "rearSign", 1.);
		_joints[0].zero = readValue(sdf, "frontZeroRad", 0.);
		_joints[1].zero = readValue(sdf, "rearZeroRad", 0.);

		for (auto &joint : _joints) {
			joint.handle = model->GetJoint(joint.name);

			if (!joint.handle || joint.handle->DOF() != 1 || !joint.handle->HasType(physics::Base::HINGE_JOINT)
			    || std::fabs(joint.sign) != 1. || !std::isfinite(joint.zero)) {
				return false;
			}

			// Accept model-local or scoped names in SDF, publish unscoped joint names.
			joint.name = joint.handle->GetName();
			joint.scoped_name = joint.handle->GetScopedName();
			const double lower = joint.handle->LowerLimit(0);
			const double upper = joint.handle->UpperLimit(0);
			const double joint_effort_limit = joint.handle->GetEffortLimit(0);
			const double joint_velocity_limit = joint.handle->GetVelocityLimit(0);

			if (!(std::isfinite(lower) && std::isfinite(upper) && lower < joint.zero && joint.zero < upper)
			    || !std::isfinite(joint_effort_limit) || joint_effort_limit == 0.
			    || !std::isfinite(joint_velocity_limit) || joint_velocity_limit == 0.) {
				return false;
			}

			joint.effort_limit = joint_effort_limit > 0. ? std::min(torque_limit, joint_effort_limit) : torque_limit;

			if (joint_velocity_limit > 0.) {
				_target_rate = std::min(_target_rate, joint_velocity_limit);
			}

			const double a = joint.sign * (lower - joint.zero);
			const double b = joint.sign * (upper - joint.zero);
			_min_angle = std::max(_min_angle, std::min(a, b));
			_max_angle = std::min(_max_angle, std::max(a, b));
		}

		return _joints[0].handle != _joints[1].handle && _min_angle < 0. && _max_angle > 0.;
	}

	void receiveCommand(const std_msgs::Float64::ConstPtr &message)
	{
		if (!std::isfinite(message->data)) {
			return;
		}

		std::lock_guard<std::mutex> guard(_command_mutex);
		_requested_angle = std::clamp(message->data, _min_angle, _max_angle);
		_last_command = SteadyClock::now();
	}

	void resetTracking()
	{
		std::lock_guard<std::mutex> guard(_command_mutex);
		_last_command = SteadyClock::time_point{};
		_requested_angle = 0.;
		_slewed_angle = 0.;
		_command_timed_out = false;
		_initialized = false;
	}

	void update(const common::Time &sim_time)
	{
		if (_initialized && sim_time < _last_sim_time) {
			resetTracking();
		}

		if (!_initialized) {
			for (const auto &joint : _joints) {
				const double limit = joint.effort_limit;
				_controller->SetPositionPID(joint.scoped_name,
					common::PID(_position_p, _position_i, _position_d, limit, -limit, limit, -limit));
			}

			_last_sim_time = sim_time;
			_next_feedback_time = sim_time + _feedback_period;
			_initialized = true;
		}

		const double dt = (sim_time - _last_sim_time).Double();
		_last_sim_time = sim_time;

		bool command_timed_out = false;
		{
			std::lock_guard<std::mutex> guard(_command_mutex);
			command_timed_out = _last_command != SteadyClock::time_point{} &&
					    std::chrono::duration<double>(SteadyClock::now() - _last_command).count() >= _timeout_seconds;
			const double largest_step = _target_rate * dt;
			_slewed_angle += std::clamp(_requested_angle - _slewed_angle, -largest_step, largest_step);
		}

		if (command_timed_out && !_command_timed_out) {
			gzwarn << "Tilt servo command stream timed out. Holding the last accepted target.\n";
		}

		_command_timed_out = command_timed_out;

		// Use Gazebo's own per-step PID update. Only targets go into the controller.
		for (const auto &joint : _joints) {
			_controller->SetPositionTarget(joint.scoped_name, joint.zero + joint.sign * _slewed_angle);
		}

		if (sim_time >= _next_feedback_time) {
			publishFeedback(sim_time);
			_next_feedback_time += _feedback_period;

			if (_next_feedback_time <= sim_time) {
				_next_feedback_time = sim_time + _feedback_period;
			}
		}
	}

	void publishFeedback(const common::Time &sim_time)
	{
		sensor_msgs::JointState state;
		state.header.stamp = ros::Time(static_cast<uint32_t>(sim_time.sec), static_cast<uint32_t>(sim_time.nsec));
		state.name.reserve(_joints.size());
		state.position.reserve(_joints.size());
		state.velocity.reserve(_joints.size());
		state.effort.reserve(_joints.size());

		for (const auto &joint : _joints) {
			state.name.push_back(joint.name);
			state.position.push_back(joint.handle->Position(0));
			state.velocity.push_back(joint.handle->GetVelocity(0));
			// At WorldUpdateBegin the actuator effort is from the preceding step.
			state.effort.push_back(joint.handle->GetForce(0));
		}

		_state_publisher.publish(state);
	}

	physics::JointControllerPtr _controller;
	std::array<ServoJoint, 2> _joints{};
	double _min_angle{};
	double _max_angle{};
	double _target_rate{};
	double _timeout_seconds{};
	double _position_p{};
	double _position_i{};
	double _position_d{};
	common::Time _feedback_period{};

	std::mutex _command_mutex;
	SteadyClock::time_point _last_command{};
	double _requested_angle{};

	ros::CallbackQueue _command_queue;
	std::unique_ptr<ros::NodeHandle> _node;
	ros::Subscriber _command_subscriber;
	ros::Publisher _state_publisher;
	std::unique_ptr<ros::AsyncSpinner> _spinner;

	std::shared_ptr<EventGate> _event_gate{std::make_shared<EventGate>()};
	event::ConnectionPtr _update_connection;
	common::Time _last_sim_time{};
	common::Time _next_feedback_time{};
	double _slewed_angle{};
	bool _command_timed_out{false};
	bool _initialized{false};
};

GZ_REGISTER_MODEL_PLUGIN(GazeboTiltServoPlugin)

} // namespace gazebo
