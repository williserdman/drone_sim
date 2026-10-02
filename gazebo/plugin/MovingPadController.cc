#include <chrono>
#include <string>
#include <vector>

#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <sdf/Element.hh>

namespace drone_sim::gazebo
{
class MovingPadController:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
  public: void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &) final
  {
    if (!_sdf->HasElement("joint_name") ||
        !_sdf->HasElement("motion_start_sim_time_s") ||
        !_sdf->HasElement("velocity_mps"))
    {
      gzerr << "MovingPadController requires joint_name, "
            << "motion_start_sim_time_s, and velocity_mps\n";
      return;
    }
    this->motionStartSimTimeS =
        _sdf->Get<double>("motion_start_sim_time_s");
    this->velocityMps = _sdf->Get<double>("velocity_mps");
    if (this->motionStartSimTimeS < 0.0)
    {
      gzerr << "MovingPadController motion start must be nonnegative\n";
      return;
    }
    const auto jointName = _sdf->Get<std::string>("joint_name");
    this->jointEntity = gz::sim::Model(_entity).JointByName(_ecm, jointName);
    if (this->jointEntity == gz::sim::kNullEntity)
    {
      gzerr << "MovingPadController could not resolve joint ["
            << jointName << "]\n";
      return;
    }
    this->configured = true;
  }

  public: void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) final
  {
    if (!this->configured || _info.paused)
      return;
    const double nativeS =
        std::chrono::duration<double>(_info.simTime).count();
    gz::sim::Joint(this->jointEntity).SetVelocity(
        _ecm,
        {nativeS >= this->motionStartSimTimeS ? this->velocityMps : 0.0});
  }

  private: gz::sim::Entity jointEntity{gz::sim::kNullEntity};
  private: double motionStartSimTimeS{0.0};
  private: double velocityMps{0.0};
  private: bool configured{false};
};
}

GZ_ADD_PLUGIN(
    drone_sim::gazebo::MovingPadController,
    gz::sim::System,
    drone_sim::gazebo::MovingPadController::ISystemConfigure,
    drone_sim::gazebo::MovingPadController::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(
    drone_sim::gazebo::MovingPadController,
    "drone_sim::gazebo::MovingPadController")
