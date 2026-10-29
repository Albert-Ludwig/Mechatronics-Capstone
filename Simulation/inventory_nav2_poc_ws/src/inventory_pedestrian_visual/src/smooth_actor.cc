// SPDX-License-Identifier: Apache-2.0
// Render-only tracking: this system never controls the pedestrian's physics
// or reads / controls the robot. All body translation is owned by the slider.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <string>

#include <gz/msgs/double.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>

namespace inventory
{
class SmoothActor final : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate
{
  public: void Configure(const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override
  {
    using namespace gz::sim;
    actor = entity;
    if (!ecm.Component<components::Actor>(actor))
      return;
    targetName = sdf->Get<std::string>("target");
    yawRate = sdf->Get<double>("yaw_rate");
    animationRate = sdf->Get<double>("animation_seconds_per_metre");
    node.Subscribe(sdf->Get<std::string>("heading_topic"), &SmoothActor::Heading, this);
    publisher = node.Advertise<gz::msgs::Pose>("/inspection/pedestrian/visual_pose");
    auto pose = ecm.Component<components::Pose>(actor);
    if (!pose)
      return;
    auto base = pose->Data();
    previousPosition = base.Pos();
    // Keep the skin's vertical offset in Pose. TrajectoryPose is world XY/yaw.
    auto trajectory = base;
    trajectory.Pos().Z(0.);
    base.Pos().X(0.);
    base.Pos().Y(0.);
    *pose = components::Pose(base);
    if (auto value = ecm.Component<components::TrajectoryPose>(actor))
      *value = components::TrajectoryPose(trajectory);
    else
      ecm.CreateComponent(actor, components::TrajectoryPose(trajectory));
    if (auto value = ecm.Component<components::AnimationName>(actor))
      *value = components::AnimationName("walk");
    else
      ecm.CreateComponent(actor, components::AnimationName("walk"));
    if (auto value = ecm.Component<components::AnimationTime>(actor))
      *value = components::AnimationTime();
    else
      ecm.CreateComponent(actor, components::AnimationTime());
    ready = true;
  }

  public: void PreUpdate(const gz::sim::UpdateInfo &info,
      gz::sim::EntityComponentManager &ecm) override
  {
    using namespace gz::sim;
    if (!ready || info.paused)
      return;
    if (target == kNullEntity)
      target = ecm.EntityByComponents(components::Name(targetName));
    if (target == kNullEntity)
      return;
    auto body = ecm.Component<components::Pose>(target);
    auto trajectory = ecm.Component<components::TrajectoryPose>(actor);
    auto time = ecm.Component<components::AnimationTime>(actor);
    if (!body || !trajectory || !time)
      return;
    const auto now = std::chrono::duration<double>(info.simTime).count();
    const auto dt = std::clamp(now - previousTime, 0., .05);
    if (now < previousTime)
    {
      yaw = 0.;
      animationTime = 0.;
      angularVelocity = 0.;
      lastPublish = 0.;
    }
    previousTime = now;
    auto position = body->Data().Pos();
    position.Z(0.);
    auto previous = previousPosition;
    previous.Z(0.);
    double distance = (position - previous).Length();
    previousPosition = position;
    // Ignore reset / teleport distances instead of jumping animation phase.
    if (distance > .15)
      distance = 0.;
    constexpr double pi = 3.14159265358979323846;
    const double error = std::remainder(heading.load() - yaw, 2. * pi);
    const double desired = std::copysign(std::min(yawRate, std::sqrt(5. * std::abs(error))), error);
    angularVelocity += std::clamp(desired-angularVelocity, -2.5*dt, 2.5*dt);
    double change = angularVelocity * dt;
    if (std::abs(change) > std::abs(error))
    {
      change = error;
      angularVelocity = 0.;
    }
    yaw += change;
    // Continue small stepping movements during the stationary turn.
    animationTime += animationRate * (distance + .12 * std::abs(change));
    *trajectory = components::TrajectoryPose(gz::math::Pose3d(
      position.X(), position.Y(), 0., 0., 0., yaw));
    *time = components::AnimationTime(
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(animationTime)));
    ecm.SetChanged(actor, components::TrajectoryPose::typeId, ComponentState::OneTimeChange);
    ecm.SetChanged(actor, components::AnimationTime::typeId, ComponentState::OneTimeChange);
    ecm.SetChanged(actor, components::AnimationName::typeId, ComponentState::OneTimeChange);
    if (now-lastPublish >= .02)
    {
      lastPublish = now;
      gz::msgs::Pose message;
      auto stamp = message.mutable_header()->mutable_stamp();
      stamp->set_sec(static_cast<int64_t>(now));
      stamp->set_nsec(static_cast<int32_t>((now-std::floor(now))*1e9));
      message.mutable_position()->set_x(position.X());
      message.mutable_position()->set_y(position.Y());
      message.mutable_orientation()->set_z(std::sin(yaw/2.));
      message.mutable_orientation()->set_w(std::cos(yaw/2.));
      publisher.Publish(message);
    }
  }

  private: void Heading(const gz::msgs::Double &message)
  {
    if (std::isfinite(message.data())) heading.store(message.data());
  }
  private: gz::sim::Entity actor{gz::sim::kNullEntity};
  private: gz::sim::Entity target{gz::sim::kNullEntity};
  private: std::string targetName;
  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher publisher;
  private: std::atomic<double> heading{0.};
  private: gz::math::Vector3d previousPosition;
  private: double previousTime{0.}, animationTime{0.}, yaw{0.};
  private: double angularVelocity{0.}, lastPublish{0.};
  private: double yawRate{1.4}, animationRate{4.15};
  private: bool ready{false};
};
}
GZ_ADD_PLUGIN(inventory::SmoothActor, gz::sim::System,
  inventory::SmoothActor::ISystemConfigure, inventory::SmoothActor::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(inventory::SmoothActor, "inventory::SmoothActor")
