#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>

#include <gz/msgs/empty.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>

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

class PhysicalObserver:
    public gz::sim::System,
    public gz::sim::ISystemPostUpdate
{
  public: void PostUpdate(
      const gz::sim::UpdateInfo &,
      const gz::sim::EntityComponentManager &_ecm) final
  {
    std::size_t count = 0;
    _ecm.Each<gz::sim::components::DetachableJoint>(
        [&count](const gz::sim::Entity &,
                 const gz::sim::components::DetachableJoint *)
        {
          ++count;
          return true;
        });
    this->maximumJointCount.store(
        std::max(this->maximumJointCount.load(), count));
    this->currentJointCount.store(count);

    this->ObservePose("payload_3", 2.0, _ecm, this->payload3MaxError);
    this->ObservePose("payload_4", -2.0, _ecm, this->payload4MaxError);
  }

  private: void ObservePose(
      const std::string &_name,
      double _expectedX,
      const gz::sim::EntityComponentManager &_ecm,
      std::atomic<double> &_maximumError)
  {
    const auto entity = _ecm.EntityByComponents(
        gz::sim::components::Model(), gz::sim::components::Name(_name));
    if (entity == gz::sim::kNullEntity)
      return;
    const auto pose = gz::sim::worldPose(entity, _ecm);
    const auto error = std::abs(pose.Pos().X() - _expectedX) +
        std::abs(pose.Pos().Y()) + std::abs(pose.Pos().Z());
    _maximumError.store(std::max(_maximumError.load(), error));
  }

  public: std::atomic<std::size_t> currentJointCount{0};
  public: std::atomic<std::size_t> maximumJointCount{0};
  public: std::atomic<double> payload3MaxError{0.0};
  public: std::atomic<double> payload4MaxError{0.0};
};

template<typename Predicate>
bool AdvanceUntil(gz::sim::Server &_server, Predicate _predicate)
{
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline)
  {
    if (_predicate())
      return true;
    if (!_server.RunOnce(false))
      return false;
    std::this_thread::sleep_for(5ms);
  }
  return _predicate();
}

template<typename Predicate>
bool WaitFor(Predicate _predicate)
{
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < deadline)
  {
    if (_predicate())
      return true;
    std::this_thread::sleep_for(5ms);
  }
  return _predicate();
}
}

int main(int argc, char **argv)
{
  Require(argc == 3,
      "coordinator and detachable-joint plugin paths are required");
  const auto partition =
      "payload_coordinator_test_" + std::to_string(::getpid());
  ::setenv("GZ_PARTITION", partition.c_str(), 1);

  const std::string coordinatorPath = argv[1];
  const std::string jointPath = argv[2];
  const auto jointPlugin = [&jointPath](
      int _payloadId, bool _initiallyAttached)
  {
    const auto id = std::to_string(_payloadId);
    return std::string{R"(
      <plugin filename=")"} + jointPath + R"("
              name="drone_sim::gazebo::DetachableJoint">
        <parent_link>hardpoint</parent_link>
        <child_model>payload_)" + id + R"(</child_model>
        <child_link>body</child_link>
        <attach_topic>/payload/)" + id + R"(/physical/attach</attach_topic>
        <detach_topic>/payload/)" + id + R"(/physical/detach</detach_topic>
        <output_topic>/payload/)" + id + R"(/joint_state</output_topic>
        <initially_attached>)" +
        (_initiallyAttached ? "true" : "false") + R"(</initially_attached>
      </plugin>)";
  };
  const std::string sdf = std::string{R"(
    <sdf version="1.10">
      <world name="coordinator_test">
        <gravity>0 0 0</gravity>
        <plugin filename="gz-sim-physics-system"
                name="gz::sim::systems::Physics"/>
        <model name="host">
          <static>true</static>
          <link name="hardpoint"/>
          <plugin filename=")"} + coordinatorPath + R"("
                  name="drone_sim::gazebo::PayloadCommandCoordinator">
            <command_topic>/payload/3/command</command_topic>
            <physical_state_topic>/payload/3/joint_state</physical_state_topic>
            <result_topic>/payload/3/result</result_topic>
            <stock_attach_topic>/payload/3/physical/attach</stock_attach_topic>
            <stock_detach_topic>/payload/3/physical/detach</stock_detach_topic>
          </plugin>)" +
          jointPlugin(2, true) + jointPlugin(3, false) +
          jointPlugin(4, false) + R"(
        </model>
        <model name="payload_2">
          <link name="body"><inertial><mass>1</mass><inertia>
            <ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz>
          </inertia></inertial></link>
        </model>
        <model name="payload_3">
          <pose>2 0 0 0 0 0</pose>
          <link name="body"><inertial><mass>1</mass><inertia>
            <ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz>
          </inertia></inertial></link>
        </model>
        <model name="payload_4">
          <pose>-2 0 0 0 0 0</pose>
          <link name="body"><inertial><mass>1</mass><inertia>
            <ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz>
          </inertia></inertial></link>
        </model>
      </world>
    </sdf>)";

  gz::sim::ServerConfig config;
  Require(config.SetSdfString(sdf), "test SDF must parse");

  gz::transport::Node node;
  std::mutex mutex;
  std::vector<std::string> results;
  std::vector<std::string> payload2States;
  std::vector<std::string> payload3States;
  std::vector<std::string> payload4States;
  std::atomic<int> attachTriggers{0};
  std::atomic<int> detachTriggers{0};
  auto record = [&mutex](std::vector<std::string> &_target,
                         const gz::msgs::StringMsg &_message)
  {
    std::lock_guard<std::mutex> guard(mutex);
    _target.push_back(_message.data());
  };
  Require(node.Subscribe<gz::msgs::StringMsg>(
      "/payload/3/result",
      std::function<void(const gz::msgs::StringMsg &)>{
          [&record, &results](const auto &_message)
          {
            record(results, _message);
          }}),
      "result subscription must configure");
  for (const auto &[topic, target] : std::vector<std::pair<
      std::string, std::vector<std::string> *>>{
          {"/payload/2/joint_state", &payload2States},
          {"/payload/3/joint_state", &payload3States},
          {"/payload/4/joint_state", &payload4States}})
  {
    Require(node.Subscribe<gz::msgs::StringMsg>(
        topic,
        std::function<void(const gz::msgs::StringMsg &)>{
            [&record, target](const auto &_message)
            {
              record(*target, _message);
            }}),
        "joint-state subscription must configure");
  }
  Require(node.Subscribe<gz::msgs::Empty>(
      "/payload/3/physical/attach",
      std::function<void(const gz::msgs::Empty &)>{
          [&attachTriggers](const auto &) { ++attachTriggers; }}),
      "stock attach subscription must configure");
  Require(node.Subscribe<gz::msgs::Empty>(
      "/payload/3/physical/detach",
      std::function<void(const gz::msgs::Empty &)>{
          [&detachTriggers](const auto &) { ++detachTriggers; }}),
      "stock detach subscription must configure");

  auto commands = node.Advertise<gz::msgs::StringMsg>("/payload/3/command");
  gz::sim::Server server(config);
  auto observer = std::make_shared<PhysicalObserver>();
  Require(server.AddSystem(observer).value_or(false),
      "physical observer must join the real server");
  Require(server.RunOnce(true), "server must configure all real joint systems");
  Require(AdvanceUntil(server, [&]
  {
    std::lock_guard<std::mutex> guard(mutex);
    return payload2States.size() == 1 && payload2States[0] == "attached" &&
        payload3States.size() == 1 && payload3States[0] == "detached" &&
        payload4States.size() == 1 && payload4States[0] == "detached";
  }), "payload 2 must start attached while payloads 3 and 4 start detached");
  Require(observer->currentJointCount == 1 && observer->maximumJointCount == 1,
      "initially detached payloads must never create a joint component");
  Require(observer->payload3MaxError < 1e-9 && observer->payload4MaxError < 1e-9,
      "initially detached payload poses must never snap or move");
  Require(WaitFor([&] { return commands.HasConnections(); }),
      "coordinator command subscription must be discoverable");

  auto publish = [&commands](const std::string &_wire)
  {
    gz::msgs::StringMsg command;
    command.set_data(_wire);
    Require(commands.Publish(command), "test command publication must succeed");
  };
  auto resultCount = [&]
  {
    std::lock_guard<std::mutex> guard(mutex);
    return results.size();
  };

  publish("payload-command-v1|A1|attach");
  publish("payload-command-v1|A1|attach");
  Require(AdvanceUntil(server, [&]
  {
    std::lock_guard<std::mutex> guard(mutex);
    return results.size() == 1 && payload3States.back() == "attached";
  }), "duplicate attach must complete after one stock physical transition");
  Require(attachTriggers == 1, "duplicate attach must publish one stock trigger");
  Require(observer->currentJointCount == 2,
      "attach confirmation must correspond to a real second joint component");

  publish("payload-command-v1|A1|attach");
  Require(WaitFor([&] { return resultCount() == 2; }),
      "completed attach duplicate must replay its exact result");
  Require(attachTriggers == 1,
      "completed attach duplicate must not retrigger physics");

  publish("payload-command-v1|D1|detach");
  publish("payload-command-v1|D1|detach");
  Require(AdvanceUntil(server, [&]
  {
    std::lock_guard<std::mutex> guard(mutex);
    return results.size() == 3 && payload3States.back() == "detached" &&
        observer->currentJointCount == 1;
  }), "duplicate detach must complete after one stock physical transition");
  Require(detachTriggers == 1, "duplicate detach must publish one stock trigger");
  Require(observer->currentJointCount == 1,
      "detach confirmation must correspond to removal of the real joint component");

  publish("payload-command-v1|D1|detach");
  Require(WaitFor([&] { return resultCount() == 4; }),
      "completed detach duplicate must replay its exact result");
  Require(detachTriggers == 1,
      "completed detach duplicate must not retrigger physics");

  publish("payload-command-v1|D1|attach");
  Require(WaitFor([&] { return resultCount() == 5; }),
      "conflicting command identity reuse must return an error");
  Require(attachTriggers == 1,
      "conflicting command identity reuse must not touch stock physics");

  publish("payload-command-v1|bad/id|attach");
  publish("payload-command-v2|M1|attach");
  for (int index = 0; index < 10; ++index)
    Require(server.RunOnce(false), "server must advance malformed-command check");
  std::this_thread::sleep_for(50ms);
  Require(resultCount() == 5, "malformed command IDs and versions must be rejected");
  Require(attachTriggers == 1 && detachTriggers == 1,
      "malformed commands must not touch stock physics");

  {
    std::lock_guard<std::mutex> guard(mutex);
    Require(results[0] == "payload-result-v1|A1|confirmed|attached|OK",
        "attach result must identify confirmed physical truth");
    Require(results[1] == results[0], "attach replay must be byte-identical");
    Require(results[2] == "payload-result-v1|D1|confirmed|detached|OK",
        "detach result must identify confirmed physical truth");
    Require(results[3] == results[2], "detach replay must be byte-identical");
    Require(results[4] ==
        "payload-result-v1|D1|error|unknown|COMMAND_ID_ACTION_CONFLICT",
        "conflicting identity reuse must expose the exact error code");
    Require(payload3States ==
        std::vector<std::string>({"detached", "attached", "detached"}),
        "payload 3 must expose exactly one transition per accepted action");
    Require(payload4States == std::vector<std::string>({"detached"}),
        "payload 4 must remain observably and physically detached");
  }
  return 0;
}
