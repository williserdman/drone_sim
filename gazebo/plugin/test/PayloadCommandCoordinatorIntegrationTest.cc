#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <iterator>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>

#include <gz/msgs/empty.pb.h>
#include <gz/msgs/contacts.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/ContactSensor.hh>
#include <gz/sim/components/ContactSensorData.hh>
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

struct JointTruth
{
  std::int64_t timestampNs;
  std::string state;
};

struct ContactTruth
{
  std::int64_t timestampNs;
  bool grounded;
};

ContactTruth ParseContactTruth(const gz::msgs::Contacts &_message)
{
  const auto &stamp = _message.header().stamp();
  return ContactTruth{
      stamp.sec() * 1'000'000'000LL + stamp.nsec(),
      _message.contact_size() > 0};
}

bool HasExactContactCadence(const std::vector<ContactTruth> &_truth)
{
  if (_truth.size() < 2)
    return false;
  for (std::size_t index = 1; index < _truth.size(); ++index)
  {
    if (_truth[index].timestampNs - _truth[index - 1].timestampNs !=
        50'000'000)
      return false;
  }
  return true;
}

std::optional<JointTruth> ParseJointTruth(const std::string &_wire)
{
  constexpr const char *prefix = "payload-joint-state-v1|";
  if (_wire.rfind(prefix, 0) != 0)
    return std::nullopt;
  const auto separator = _wire.find('|', std::char_traits<char>::length(prefix));
  if (separator == std::string::npos ||
      _wire.find('|', separator + 1) != std::string::npos)
    return std::nullopt;
  const auto timestampText = _wire.substr(
      std::char_traits<char>::length(prefix),
      separator - std::char_traits<char>::length(prefix));
  const auto state = _wire.substr(separator + 1);
  if (timestampText.empty() || (state != "attached" && state != "detached"))
    return std::nullopt;
  try
  {
    std::size_t parsed = 0;
    const auto timestamp = std::stoll(timestampText, &parsed);
    if (parsed != timestampText.size() || timestamp < 0)
      return std::nullopt;
    return JointTruth{timestamp, state};
  }
  catch (...)
  {
    return std::nullopt;
  }
}

std::string LatestState(const std::vector<std::string> &_wires)
{
  if (_wires.empty())
    return {};
  const auto parsed = ParseJointTruth(_wires.back());
  return parsed ? parsed->state : std::string{};
}

std::vector<std::string> StateTransitions(
    const std::vector<std::string> &_wires)
{
  std::vector<std::string> states;
  for (const auto &wire : _wires)
  {
    const auto parsed = ParseJointTruth(wire);
    if (!parsed)
      return {};
    if (states.empty() || states.back() != parsed->state)
      states.push_back(parsed->state);
  }
  return states;
}

bool HasExactTruthCadence(const std::vector<std::string> &_wires)
{
  if (_wires.size() < 2)
    return false;
  auto previous = ParseJointTruth(_wires.front());
  if (!previous)
    return false;
  for (auto iterator = std::next(_wires.begin()); iterator != _wires.end(); ++iterator)
  {
    const auto current = ParseJointTruth(*iterator);
    if (!current || current->timestampNs - previous->timestampNs != 50'000'000)
      return false;
    previous = current;
  }
  return true;
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

class ContactTruthDriver:
    public gz::sim::System,
    public gz::sim::ISystemPreUpdate
{
  public: void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) final
  {
    std::vector<std::pair<gz::sim::Entity, bool>> sensors;
    _ecm.Each<gz::sim::components::ContactSensor,
              gz::sim::components::Name>(
        [&sensors, &_ecm](
            const gz::sim::Entity &_entity,
            const gz::sim::components::ContactSensor *,
            const gz::sim::components::Name *)
        {
          const auto name = gz::sim::scopedName(_entity, _ecm);
          sensors.emplace_back(
              _entity,
              name.find("payload_3") != std::string::npos ||
              name.find("payload_4") != std::string::npos);
          return true;
        });
    for (const auto &[entity, grounded] : sensors)
    {
      gz::msgs::Contacts message;
      const auto timestampNs = std::chrono::duration_cast<
          std::chrono::nanoseconds>(_info.simTime).count();
      message.mutable_header()->mutable_stamp()->set_sec(
          timestampNs / 1'000'000'000LL);
      message.mutable_header()->mutable_stamp()->set_nsec(
          timestampNs % 1'000'000'000LL);
      if (grounded)
        message.add_contact();
      auto data = _ecm.Component<gz::sim::components::ContactSensorData>(
          entity);
      if (data == nullptr)
        _ecm.CreateComponent(
            entity, gz::sim::components::ContactSensorData(message));
      else
        data->SetData(message, [](const auto &_left, const auto &_right)
        {
          return _left.SerializeAsString() == _right.SerializeAsString();
        });
    }
    this->sensorCount = sensors.size();
  }

  public: std::atomic<std::size_t> sensorCount{0};
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
        <contact_sensor>ground_contact</contact_sensor>
        <contact_state_topic>/payload/)" + id + R"(/contact_state</contact_state_topic>
        <initially_attached>)" +
        (_initiallyAttached ? "true" : "false") + R"(</initially_attached>
        <exclusive_parent>true</exclusive_parent>
        <state_publish_period>0.05</state_publish_period>
      </plugin>)";
  };
  const std::string sdf = std::string{R"(
    <sdf version="1.10">
      <world name="coordinator_test">
        <gravity>0 0 0</gravity>
        <physics name="fixed" type="ignored">
          <max_step_size>0.01</max_step_size>
        </physics>
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
          </inertia></inertial>
          <collision name="body_collision"><geometry><box><size>1 1 1</size></box></geometry></collision>
          <sensor name="ground_contact" type="contact"><contact><collision>body_collision</collision></contact></sensor>
          </link>
        </model>
        <model name="payload_3">
          <pose>2 0 0 0 0 0</pose>
          <link name="body"><inertial><mass>1</mass><inertia>
            <ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz>
          </inertia></inertial>
          <collision name="body_collision"><geometry><box><size>1 1 1</size></box></geometry></collision>
          <sensor name="ground_contact" type="contact"><contact><collision>body_collision</collision></contact></sensor>
          </link>
        </model>
        <model name="payload_4">
          <pose>-2 0 0 0 0 0</pose>
          <link name="body"><inertial><mass>1</mass><inertia>
            <ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz>
          </inertia></inertial>
          <collision name="body_collision"><geometry><box><size>1 1 1</size></box></geometry></collision>
          <sensor name="ground_contact" type="contact"><contact><collision>body_collision</collision></contact></sensor>
          </link>
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
  std::vector<ContactTruth> payload2Contacts;
  std::vector<ContactTruth> payload3Contacts;
  std::vector<ContactTruth> payload4Contacts;
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
  for (const auto &[topic, target] : std::vector<std::pair<
      std::string, std::vector<ContactTruth> *>>{
          {"/payload/2/contact_state", &payload2Contacts},
          {"/payload/3/contact_state", &payload3Contacts},
          {"/payload/4/contact_state", &payload4Contacts}})
  {
    Require(node.Subscribe<gz::msgs::Contacts>(
        topic,
        std::function<void(const gz::msgs::Contacts &)>{
            [&mutex, target](const auto &_message)
            {
              std::lock_guard<std::mutex> guard(mutex);
              target->push_back(ParseContactTruth(_message));
            }}),
        "contact-state subscription must configure");
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
  auto payload2Detach =
      node.Advertise<gz::msgs::Empty>("/payload/2/physical/detach");
  gz::sim::Server server(config);
  auto observer = std::make_shared<PhysicalObserver>();
  auto contactDriver = std::make_shared<ContactTruthDriver>();
  Require(server.AddSystem(observer).value_or(false),
      "physical observer must join the real server");
  Require(server.AddSystem(contactDriver).value_or(false),
      "contact truth driver must join the real server");
  Require(server.RunOnce(true), "server must configure all real joint systems");
  Require(WaitFor([&]
  {
    std::vector<std::string> topics;
    node.TopicList(topics);
    return std::count(topics.begin(), topics.end(), "/payload/2/contact_state") == 1 &&
        std::count(topics.begin(), topics.end(), "/payload/3/contact_state") == 1 &&
        std::count(topics.begin(), topics.end(), "/payload/4/contact_state") == 1;
  }), "all three recurrent contact publishers must advertise");
  Require(AdvanceUntil(server, [&]
  {
    std::lock_guard<std::mutex> guard(mutex);
    return LatestState(payload2States) == "attached" &&
        LatestState(payload3States) == "detached" &&
        LatestState(payload4States) == "detached";
  }), "payload 2 must start attached while payloads 3 and 4 start detached");
  std::size_t initialTruthCount = 0;
  {
    std::lock_guard<std::mutex> guard(mutex);
    initialTruthCount = payload3States.size();
  }
  for (int index = 0; index < 6; ++index)
    Require(server.RunOnce(false), "server must advance recurrent truth cadence");
  Require(WaitFor([&]
  {
    std::lock_guard<std::mutex> guard(mutex);
    return payload3States.size() > initialTruthCount;
  }), "unchanged joint state must recur after one 50 ms truth period");
  Require(AdvanceUntil(server, [&]
  {
    std::lock_guard<std::mutex> guard(mutex);
    return HasExactContactCadence(payload2Contacts) &&
        HasExactContactCadence(payload3Contacts) &&
        HasExactContactCadence(payload4Contacts);
  }), "contact truth must recur while simulation time advances");
  {
    std::lock_guard<std::mutex> guard(mutex);
    Require(contactDriver->sensorCount == 3,
        "contact driver must see all three child sensors");
    Require(HasExactTruthCadence(payload2States) &&
        HasExactTruthCadence(payload3States) &&
        HasExactTruthCadence(payload4States),
        "joint truth must carry exact recurrent 50 ms simulation timestamps");
    Require(HasExactContactCadence(payload2Contacts) &&
        HasExactContactCadence(payload3Contacts) &&
        HasExactContactCadence(payload4Contacts),
        "contact truth must recur on the same exact 50 ms simulation grid: " +
        std::to_string(payload2Contacts.size()) + "," +
        std::to_string(payload3Contacts.size()) + "," +
        std::to_string(payload4Contacts.size()));
    Require(!payload2Contacts.back().grounded &&
        payload3Contacts.back().grounded && payload4Contacts.back().grounded,
        "contact truth must preserve both physical false and true states");
  }
  Require(observer->currentJointCount == 1 && observer->maximumJointCount == 1,
      "initially detached payloads must never create a joint component");
  Require(observer->payload3MaxError < 1e-9 && observer->payload4MaxError < 1e-9,
      "initially detached payload poses must never snap or move");
  Require(WaitFor([&] { return commands.HasConnections(); }),
      "coordinator command subscription must be discoverable");
  Require(WaitFor([&] { return payload2Detach.HasConnections(); }),
      "payload 2 stock detach subscription must be discoverable");

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
  Require(WaitFor([&] { return attachTriggers == 1; }),
      "duplicate attach must publish one stock trigger");
  std::size_t occupiedTruthCount = 0;
  {
    std::lock_guard<std::mutex> guard(mutex);
    occupiedTruthCount = payload3States.size();
  }
  for (int index = 0; index < 20; ++index)
    Require(server.RunOnce(false), "server must advance occupied attach check");
  {
    std::lock_guard<std::mutex> guard(mutex);
    Require(results.empty() && LatestState(payload3States) == "detached",
        "an occupied hardpoint must leave payload 3 physically detached");
    bool recurrentDetached =
        payload3States.size() >= occupiedTruthCount + 3;
    for (std::size_t index = occupiedTruthCount;
         recurrentDetached && index < payload3States.size(); ++index)
    {
      const auto truth = ParseJointTruth(payload3States[index]);
      recurrentDetached = truth && truth->state == "detached";
    }
    Require(recurrentDetached && HasExactTruthCadence(payload3States),
        "occupied pending attach must keep publishing detached truth at exact "
        "50 ms cadence");
  }
  Require(observer->currentJointCount == 1 &&
      observer->maximumJointCount == 1,
      "one hardpoint must never create a second joint component");

  gz::msgs::Empty detachPayload2;
  Require(payload2Detach.Publish(detachPayload2),
      "payload 2 stock detach publication must succeed");
  Require(AdvanceUntil(server, [&]
  {
    std::lock_guard<std::mutex> guard(mutex);
    return results.size() == 1 && LatestState(payload2States) == "detached" &&
        LatestState(payload3States) == "attached" &&
        observer->currentJointCount == 1;
  }), "pending payload 3 attach must complete after payload 2 detaches");
  Require(attachTriggers == 1, "duplicate attach must publish one stock trigger");
  Require(observer->currentJointCount == 1 &&
      observer->maximumJointCount == 1,
      "capacity-one handoff must use exactly one real joint component");

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
    return results.size() == 3 && LatestState(payload3States) == "detached" &&
        observer->currentJointCount == 0;
  }), "duplicate detach must complete after one stock physical transition");
  Require(detachTriggers == 1, "duplicate detach must publish one stock trigger");
  Require(observer->currentJointCount == 0,
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
    Require(StateTransitions(payload3States) ==
        std::vector<std::string>({"detached", "attached", "detached"}),
        "payload 3 recurrent truth must expose each accepted transition");
    Require(StateTransitions(payload2States) ==
        std::vector<std::string>({"attached", "detached"}),
        "payload 2 must detach before payload 3 occupies the hardpoint");
    Require(StateTransitions(payload4States) ==
        std::vector<std::string>({"detached"}),
        "payload 4 must remain observably and physically detached");
  }
  return 0;
}
