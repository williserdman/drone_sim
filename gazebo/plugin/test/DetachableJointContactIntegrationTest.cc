#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>

#include <gz/msgs/contacts.pb.h>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>

namespace
{
using namespace std::chrono_literals;

constexpr std::int64_t kTruthPeriodNs = 50'000'000;
constexpr double kPayloadHalfHeightMeters = 0.5;
constexpr double kMaximumSettlingErrorMeters = 0.001;

void Require(bool _condition, const std::string &_message)
{
  if (!_condition)
  {
    std::cerr << _message << '\n';
    std::exit(1);
  }
}

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

bool HasExactCadence(const std::vector<ContactTruth> &_truth)
{
  if (_truth.size() < 3)
    return false;
  for (std::size_t index = 1; index < _truth.size(); ++index)
  {
    if (_truth[index].timestampNs - _truth[index - 1].timestampNs !=
        kTruthPeriodNs)
      return false;
  }
  return true;
}

bool HasStableTruth(
    const std::vector<ContactTruth> &_truth, bool _expectedGrounded)
{
  return HasExactCadence(_truth) &&
      _truth[_truth.size() - 2].grounded == _expectedGrounded &&
      _truth.back().grounded == _expectedGrounded;
}

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
    std::this_thread::sleep_for(2ms);
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

class PhysicalContactObserver:
    public gz::sim::System,
    public gz::sim::ISystemPostUpdate
{
  public: void PostUpdate(
      const gz::sim::UpdateInfo &,
      const gz::sim::EntityComponentManager &_ecm) final
  {
    std::size_t sources = 0;
    std::size_t grounded = 0;
    _ecm.Each<gz::sim::components::Collision,
              gz::sim::components::ContactSensorData>(
        [&sources, &grounded](
            const gz::sim::Entity &,
            const gz::sim::components::Collision *,
            const gz::sim::components::ContactSensorData *_contacts)
        {
          ++sources;
          if (_contacts->Data().contact_size() > 0)
            ++grounded;
          return true;
        });
    this->sourceCount.store(sources);
    this->groundedCount.store(grounded);
    this->ObservePose("payload_3", _ecm,
        this->payload3X, this->payload3Y, this->payload3Z);
    this->ObservePose("payload_4", _ecm,
        this->payload4X, this->payload4Y, this->payload4Z);
  }

  private: void ObservePose(
      const std::string &_name,
      const gz::sim::EntityComponentManager &_ecm,
      std::atomic<double> &_x,
      std::atomic<double> &_y,
      std::atomic<double> &_z)
  {
    const auto entity = _ecm.EntityByComponents(
        gz::sim::components::Model(), gz::sim::components::Name(_name));
    if (entity == gz::sim::kNullEntity)
      return;
    const auto pose = gz::sim::worldPose(entity, _ecm);
    _x.store(pose.Pos().X());
    _y.store(pose.Pos().Y());
    _z.store(pose.Pos().Z());
  }

  public: std::atomic<std::size_t> sourceCount{0};
  public: std::atomic<std::size_t> groundedCount{0};
  public: std::atomic<double> payload3X{0.0};
  public: std::atomic<double> payload3Y{0.0};
  public: std::atomic<double> payload3Z{0.0};
  public: std::atomic<double> payload4X{0.0};
  public: std::atomic<double> payload4Y{0.0};
  public: std::atomic<double> payload4Z{0.0};
};

std::string TruthSummary(const std::vector<ContactTruth> &_truth)
{
  const auto &previous = _truth[_truth.size() - 2];
  const auto &latest = _truth.back();
  return std::to_string(previous.timestampNs) + ":" +
      (previous.grounded ? "true" : "false") + "," +
      std::to_string(latest.timestampNs) + ":" +
      (latest.grounded ? "true" : "false");
}
}

int main(int argc, char **argv)
{
  Require(argc == 2, "detachable-joint plugin path is required");
  const auto partition =
      "detachable_joint_contact_test_" + std::to_string(::getpid());
  ::setenv("GZ_PARTITION", partition.c_str(), 1);

  const std::string jointPath = argv[1];
  const auto jointPlugin = [&jointPath](int _payloadId, bool _attached)
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
        <initially_attached>)" + (_attached ? "true" : "false") + R"(</initially_attached>
        <exclusive_parent>true</exclusive_parent>
        <state_publish_period>0.05</state_publish_period>
      </plugin>)";
  };
  const auto payload = [](int _payloadId, double _x, double _z)
  {
    const auto id = std::to_string(_payloadId);
    return std::string{R"(
        <model name="payload_)"} + id + R"(">
          <pose>)" + std::to_string(_x) + " 0 " + std::to_string(_z) + R"( 0 0 0</pose>
          <link name="body">
            <inertial><mass>1</mass><inertia>
              <ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz>
            </inertia></inertial>
            <collision name="body_collision">
              <geometry><box><size>1 1 1</size></box></geometry>
            </collision>
            <sensor name="ground_contact" type="contact">
              <always_on>true</always_on><update_rate>1000</update_rate>
              <contact><collision>body_collision</collision></contact>
            </sensor>
          </link>
        </model>)";
  };
  const std::string sdf = std::string{R"(
    <sdf version="1.10">
      <world name="contact_test">
        <gravity>0 0 -9.80665</gravity>
        <physics name="fixed" type="ignored">
          <max_step_size>0.001</max_step_size>
          <real_time_factor>1</real_time_factor>
        </physics>
        <plugin filename="gz-sim-physics-system"
                name="gz::sim::systems::Physics"/>
        <plugin filename="gz-sim-sensors-system"
                name="gz::sim::systems::Sensors">
          <render_engine>ogre2</render_engine>
        </plugin>
        <plugin filename="gz-sim-contact-system"
                name="gz::sim::systems::Contact"/>
        <model name="ground">
          <static>true</static>
          <link name="ground_link">
            <collision name="ground_collision">
              <pose>0 0 -0.05 0 0 0</pose>
              <geometry><box><size>20 20 0.1</size></box></geometry>
            </collision>
          </link>
        </model>
        <model name="host">
          <static>true</static>
          <pose>0 0 3 0 0 0</pose>
          <link name="hardpoint"/>)"} +
          jointPlugin(2, true) + jointPlugin(3, false) +
          jointPlugin(4, false) + R"(
        </model>)" + payload(2, 0.0, 3.0) + payload(3, 2.0, 0.5) +
        payload(4, -2.0, 0.5) + R"(
      </world>
    </sdf>)";

  gz::sim::ServerConfig config;
  Require(config.SetSdfString(sdf), "physical contact test SDF must parse");

  gz::transport::Node node;
  std::mutex mutex;
  std::vector<ContactTruth> payload2Truth;
  std::vector<ContactTruth> payload3Truth;
  std::vector<ContactTruth> payload4Truth;
  for (const auto &[topic, target] : std::vector<std::pair<
      std::string, std::vector<ContactTruth> *>>{
          {"/payload/2/contact_state", &payload2Truth},
          {"/payload/3/contact_state", &payload3Truth},
          {"/payload/4/contact_state", &payload4Truth}})
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

  gz::sim::Server server(config);
  auto observer = std::make_shared<PhysicalContactObserver>();
  Require(server.AddSystem(observer).value_or(false),
      "physical contact observer must join the real server");
  Require(server.RunOnce(true), "server must configure all real systems");
  Require(WaitFor([&]
  {
    std::vector<std::string> topics;
    node.TopicList(topics);
    return std::count(topics.begin(), topics.end(),
               "/payload/2/contact_state") == 1 &&
        std::count(topics.begin(), topics.end(),
               "/payload/3/contact_state") == 1 &&
        std::count(topics.begin(), topics.end(),
               "/payload/4/contact_state") == 1;
  }), "all recurrent contact publishers must advertise");
  Require(AdvanceUntil(server, [&]
  {
    return observer->sourceCount == 3 && observer->groundedCount >= 2;
  }), "real Contact system must produce three collision-owned sources with "
       "both grounded payloads true");
  Require(AdvanceUntil(server, [&]
  {
    std::lock_guard<std::mutex> guard(mutex);
    return HasStableTruth(payload2Truth, false) &&
        HasStableTruth(payload3Truth, true) &&
        HasStableTruth(payload4Truth, true);
  }), "PostUpdate must recurrently publish the real collision-owned contact "
       "truth on the exact 50 ms simulation grid");

  Require(std::abs(observer->payload3X.load() - 2.0) <
          kMaximumSettlingErrorMeters &&
      std::abs(observer->payload3Y.load()) < kMaximumSettlingErrorMeters &&
      std::abs(observer->payload3Z.load() - kPayloadHalfHeightMeters) <
          kMaximumSettlingErrorMeters &&
      std::abs(observer->payload4X.load() + 2.0) <
          kMaximumSettlingErrorMeters &&
      std::abs(observer->payload4Y.load()) < kMaximumSettlingErrorMeters &&
      std::abs(observer->payload4Z.load() - kPayloadHalfHeightMeters) <
          kMaximumSettlingErrorMeters,
      "grounded 1 m boxes must physically settle within 1 mm of their "
      "geometry-derived 0.5 m center height without lateral displacement");

  std::lock_guard<std::mutex> guard(mutex);
  std::cout << "physical_contact_truth payload2=" << TruthSummary(payload2Truth)
            << " payload3=" << TruthSummary(payload3Truth)
            << " payload4=" << TruthSummary(payload4Truth)
            << " sources=" << observer->sourceCount
            << " grounded=" << observer->groundedCount << '\n';
  return 0;
}
