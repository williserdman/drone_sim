#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <unistd.h>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>

namespace
{
using namespace std::chrono_literals;

void Require(bool _condition, const std::string &_message)
{
  if (!_condition)
  {
    std::cerr << _message << '\n';
    std::exit(1);
  }
}

class MotionObserver:
    public gz::sim::System,
    public gz::sim::ISystemPostUpdate
{
  public: void PostUpdate(
      const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm) final
  {
    this->simTimeS.store(
        std::chrono::duration<double>(_info.simTime).count());
    this->Observe("moving_pad", _ecm, this->deckX, this->deckZ);
    this->Observe("rider", _ecm, this->riderX, this->riderZ);
  }

  private: void Observe(
      const std::string &_name,
      const gz::sim::EntityComponentManager &_ecm,
      std::atomic<double> &_x,
      std::atomic<double> &_z)
  {
    const auto entity = _ecm.EntityByComponents(
        gz::sim::components::Model(), gz::sim::components::Name(_name));
    if (entity == gz::sim::kNullEntity)
      return;
    const auto pose = gz::sim::worldPose(entity, _ecm);
    _x.store(pose.Pos().X());
    _z.store(pose.Pos().Z());
  }

  public: std::atomic<double> simTimeS{0.0};
  public: std::atomic<double> deckX{0.0};
  public: std::atomic<double> deckZ{0.0};
  public: std::atomic<double> riderX{0.0};
  public: std::atomic<double> riderZ{0.0};
};

void AdvanceTo(
    gz::sim::Server &_server,
    const std::shared_ptr<MotionObserver> &_observer,
    double _targetSimTimeS)
{
  const auto deadline = std::chrono::steady_clock::now() + 8s;
  while (_observer->simTimeS.load() < _targetSimTimeS &&
         std::chrono::steady_clock::now() < deadline)
  {
    Require(_server.RunOnce(false), "real Gazebo server step must succeed");
  }
  Require(_observer->simTimeS.load() >= _targetSimTimeS,
      "real Gazebo server did not reach requested simulation time");
}

void RunFixture(const std::string &_pluginPath, double _velocityMps)
{
  const std::string sdf = std::string{R"(
    <sdf version="1.10">
      <world name="moving_pad_test">
        <gravity>0 0 -9.80665</gravity>
        <physics name="fixed" type="ode">
          <max_step_size>0.001</max_step_size>
          <real_time_factor>0</real_time_factor>
        </physics>
        <plugin filename="gz-sim-physics-system"
                name="gz::sim::systems::Physics"/>
        <model name="moving_pad">
          <static>false</static>
          <link name="deck">
            <pose>0 0 0.1 0 0 0</pose>
            <inertial><mass>50</mass><inertia>
              <ixx>37.5</ixx><iyy>37.5</iyy><izz>75</izz>
            </inertia></inertial>
            <collision name="deck_collision">
              <geometry><box><size>3 3 0.2</size></box></geometry>
              <surface><friction><ode><mu>2</mu><mu2>2</mu2></ode></friction></surface>
            </collision>
          </link>
          <joint name="rail_joint" type="prismatic">
            <parent>world</parent><child>deck</child>
            <axis><xyz>1 0 0</xyz><limit><lower>-1000</lower><upper>1000</upper><effort>1000000</effort></limit></axis>
          </joint>
          <plugin filename=")"} + _pluginPath + R"("
                  name="drone_sim::gazebo::MovingPadController">
            <joint_name>rail_joint</joint_name>
            <motion_start_sim_time_s>0.5</motion_start_sim_time_s>
            <velocity_mps>)" + std::to_string(_velocityMps) + R"(</velocity_mps>
          </plugin>
        </model>
        <model name="rider">
          <pose>0 0 0.3 0 0 0</pose>
          <link name="body">
            <inertial><mass>1</mass><inertia>
              <ixx>0.0066667</ixx><iyy>0.0066667</iyy><izz>0.0066667</izz>
            </inertia></inertial>
            <collision name="rider_collision">
              <geometry><box><size>0.2 0.2 0.2</size></box></geometry>
              <surface><friction><ode><mu>2</mu><mu2>2</mu2></ode></friction></surface>
            </collision>
          </link>
        </model>
      </world>
    </sdf>)";

  gz::sim::ServerConfig config;
  Require(config.SetSdfString(sdf), "moving-pad fixture SDF must parse");
  gz::sim::Server server(config);
  auto observer = std::make_shared<MotionObserver>();
  Require(server.AddSystem(observer).value_or(false),
      "motion observer must join the real server");
  Require(server.RunOnce(true), "server must configure all real systems");

  AdvanceTo(server, observer, 0.45);
  const double warmupDeckX = observer->deckX.load();
  const double warmupRiderX = observer->riderX.load();
  Require(std::abs(warmupDeckX) < 0.005,
      "deck must remain fixed before native warmup release");
  Require(std::abs(warmupRiderX) < 0.01,
      "passive rider must remain fixed before native warmup release");

  AdvanceTo(server, observer, 2.5);
  const double deckDisplacement = observer->deckX.load() - warmupDeckX;
  const double riderDisplacement = observer->riderX.load() - warmupRiderX;
  if (_velocityMps == 0.0)
  {
    Require(std::abs(deckDisplacement) < 0.005,
        "zero-speed deck must remain stationary");
    Require(std::abs(riderDisplacement) < 0.01,
        "zero-speed rider must remain stationary");
  }
  else
  {
    Require(std::abs(deckDisplacement - 1.0) < 0.03,
        "deck must move approximately one meter over two public seconds");
    Require(std::abs(riderDisplacement - deckDisplacement) < 0.08,
        "physical friction must carry the passive rider with the deck");
  }
  Require(observer->riderZ.load() > observer->deckZ.load(),
      "passive rider must remain physically supported above the deck");

  std::cout << "moving_pad velocity=" << _velocityMps
            << " deck_dx=" << deckDisplacement
            << " rider_dx=" << riderDisplacement
            << " rider_z=" << observer->riderZ.load() << '\n';
}
}

int main(int argc, char **argv)
{
  Require(argc == 2, "moving-pad controller plugin path is required");
  const auto partition = "moving_pad_controller_test_" +
      std::to_string(::getpid());
  ::setenv("GZ_PARTITION", partition.c_str(), 1);
  RunFixture(argv[1], 0.5);
  RunFixture(argv[1], 0.0);
  return 0;
}
