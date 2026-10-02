#include <mutex>
#include <string>

#include <ignition/msgs/twist.pb.h>

#include <ignition/math/Helpers.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>

namespace fleetscope
{
/// \brief First-person character controller. Subscribes to a Twist topic
/// (linear x/y in the player's body frame, linear z > 0.5 = jump request,
/// angular z = yaw rate) and drives the given link by commanding world
/// velocities every physics step. Gravity keeps acting on z, so the player
/// falls, lands and can jump; roll/pitch are pinned so the body stays upright.
class PlayerController : public ignition::gazebo::System,
                         public ignition::gazebo::ISystemConfigure,
                         public ignition::gazebo::ISystemPreUpdate
{
  public: void Configure(const ignition::gazebo::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         ignition::gazebo::EntityComponentManager &_ecm,
                         ignition::gazebo::EventManager &) override
  {
    this->modelEntity = _entity;
    ignition::gazebo::Model model(_entity);

    this->linkName = _sdf->Get<std::string>("link_name", "body").first;
    this->jumpSpeed = _sdf->Get<double>("jump_speed", 4.0).first;
    this->maxSpeed = _sdf->Get<double>("max_speed", 7.0).first;
    this->maxYawRate = _sdf->Get<double>("max_yaw_rate", 8.0).first;
    this->uprightGain = _sdf->Get<double>("upright_gain", 30.0).first;

    std::string topic = "/model/" + model.Name(_ecm) + "/cmd_vel";
    topic = _sdf->Get<std::string>("topic", topic).first;
    this->node.Subscribe(topic, &PlayerController::OnCmdVel, this);
  }

  public: void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                         ignition::gazebo::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;

    if (this->linkEntity == ignition::gazebo::kNullEntity)
    {
      ignition::gazebo::Model model(this->modelEntity);
      this->linkEntity = model.LinkByName(_ecm, this->linkName);
      if (this->linkEntity == ignition::gazebo::kNullEntity)
        return;
      ignition::gazebo::Link link(this->linkEntity);
      link.EnableVelocityChecks(_ecm);
    }

    ignition::msgs::Twist cmd;
    bool stale;
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      if (this->seenSeq == this->appliedSeq)
        ++this->staleSteps;
      else
      {
        this->staleSteps = 0;
        this->appliedSeq = this->seenSeq;
      }
      cmd = this->latestCmd;
      stale = this->staleSteps > 250;  // ~1 s without a fresh command: stop.
    }

    ignition::gazebo::Link link(this->linkEntity);
    const auto rot = ignition::gazebo::worldPose(this->linkEntity, _ecm).Rot();
    const double yaw = rot.Yaw();
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);

    double vx = stale ? 0.0 : cmd.linear().x();
    double vy = stale ? 0.0 : cmd.linear().y();
    double yawRate = stale ? 0.0 : cmd.angular().z();

    const double speed = std::sqrt(vx * vx + vy * vy);
    if (speed > this->maxSpeed)
    {
      vx *= this->maxSpeed / speed;
      vy *= this->maxSpeed / speed;
    }
    yawRate = ignition::math::clamp(yawRate, -this->maxYawRate, this->maxYawRate);

    // Desired velocity in the WORLD frame: horizontal motion rotated by the
    // body's yaw, vertical velocity left to physics (gravity, landing)
    // except for jump impulses. Commanding world-frame velocity matters:
    // a link-frame "forward" would point into the ground the moment the
    // body tilts, making it trip over itself.
    double vz = 0.0;
    const auto curVel = link.WorldLinearVelocity(_ecm);
    if (curVel)
      vz = curVel->Z();
    if (!stale && cmd.linear().z() > 0.5 && std::abs(vz) < 0.1)
      vz = this->jumpSpeed;

    const ignition::math::Vector3d worldLin(cy * vx - sy * vy,
                                            sy * vx + cy * vy, vz);

    // Active uprighting: zeroing the angular velocity alone is not enough,
    // because ground-friction torque re-tilts the body a little every step
    // and the tilt never recovers. Feed back roll/pitch error so the body
    // actively returns to vertical.
    const auto rpy = rot.Euler();
    // Note: no damping term here. We *set* the angular velocity every step,
    // so a term like -kD*w would feed our own previous command back with a
    // gain > 1 and diverge within seconds (observed as an ODE aabb abort).
    const ignition::math::Vector3d worldAng(
        -this->uprightGain * rpy.X(), -this->uprightGain * rpy.Y(), yawRate);

    // SetLinearVelocity/SetAngularVelocity take the link frame: transform.
    link.SetLinearVelocity(_ecm, rot.Inverse() * worldLin);
    link.SetAngularVelocity(_ecm, rot.Inverse() * worldAng);
  }

  private: void OnCmdVel(const ignition::msgs::Twist &_msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->latestCmd = _msg;
    ++this->seenSeq;
  }

  private: ignition::transport::Node node;
  private: ignition::gazebo::Entity modelEntity{ignition::gazebo::kNullEntity};
  private: ignition::gazebo::Entity linkEntity{ignition::gazebo::kNullEntity};
  private: std::string linkName{"body"};
  private: double jumpSpeed{4.0};
  private: double maxSpeed{7.0};
  private: double maxYawRate{8.0};
  private: double uprightGain{30.0};
  private: std::mutex mutex;
  private: ignition::msgs::Twist latestCmd;
  private: uint64_t seenSeq{0};
  private: uint64_t appliedSeq{0};
  private: uint64_t staleSteps{0};
};
}

IGNITION_ADD_PLUGIN(fleetscope::PlayerController,
                    ignition::gazebo::System,
                    fleetscope::PlayerController::ISystemConfigure,
                    fleetscope::PlayerController::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(fleetscope::PlayerController,
                          "fleetscope::PlayerController")
