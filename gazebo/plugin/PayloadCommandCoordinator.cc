#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>

#include <gz/msgs/empty.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace drone_sim::gazebo
{
namespace
{
constexpr const char *kCommandPrefix = "payload-command-v1";
constexpr const char *kResultPrefix = "payload-result-v1";

struct Command
{
  std::string id;
  std::string action;

  std::string DesiredState() const
  {
    return this->action == "attach" ? "attached" : "detached";
  }
};

struct CompletedCommand
{
  std::string action;
  std::string result;
};

std::optional<Command> ParseCommand(const std::string &_wire)
{
  const auto first = _wire.find('|');
  const auto second = first == std::string::npos
      ? std::string::npos : _wire.find('|', first + 1);
  if (first == std::string::npos || second == std::string::npos ||
      _wire.find('|', second + 1) != std::string::npos ||
      _wire.substr(0, first) != kCommandPrefix)
  {
    return std::nullopt;
  }
  Command command{
      _wire.substr(first + 1, second - first - 1),
      _wire.substr(second + 1)};
  if (command.id.empty() || command.id.size() > 128 ||
      (command.action != "attach" && command.action != "detach"))
  {
    return std::nullopt;
  }
  for (const char character : command.id)
  {
    const bool valid =
        (character >= 'a' && character <= 'z') ||
        (character >= 'A' && character <= 'Z') ||
        (character >= '0' && character <= '9') ||
        character == '_' || character == '.' || character == ':' ||
        character == '-';
    if (!valid)
      return std::nullopt;
  }
  return command;
}

std::string ResultWire(
    const std::string &_id,
    const std::string &_status,
    const std::string &_physicalState,
    const std::string &_code)
{
  return std::string{kResultPrefix} + '|' + _id + '|' + _status + '|' +
      _physicalState + '|' + _code;
}
}

class PayloadCommandCoordinator:
    public gz::sim::System,
    public gz::sim::ISystemConfigure
{
  public: void Configure(
      const gz::sim::Entity &,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &,
      gz::sim::EventManager &) final
  {
    for (const auto &name : {
        "command_topic", "physical_state_topic", "result_topic",
        "stock_attach_topic", "stock_detach_topic"})
    {
      if (!_sdf->HasElement(name))
      {
        gzerr << "PayloadCommandCoordinator requires <" << name << ">\n";
        return;
      }
    }
    this->commandTopic = _sdf->Get<std::string>("command_topic");
    this->physicalStateTopic =
        _sdf->Get<std::string>("physical_state_topic");
    this->resultTopic = _sdf->Get<std::string>("result_topic");
    this->stockAttachTopic = _sdf->Get<std::string>("stock_attach_topic");
    this->stockDetachTopic = _sdf->Get<std::string>("stock_detach_topic");
    if (this->commandTopic == this->physicalStateTopic ||
        this->commandTopic == this->resultTopic ||
        this->physicalStateTopic == this->resultTopic ||
        this->stockAttachTopic == this->stockDetachTopic)
    {
      gzerr << "PayloadCommandCoordinator topics must be distinct\n";
      return;
    }

    this->resultPublisher =
        this->node.Advertise<gz::msgs::StringMsg>(this->resultTopic);
    this->stockAttachPublisher =
        this->node.Advertise<gz::msgs::Empty>(this->stockAttachTopic);
    this->stockDetachPublisher =
        this->node.Advertise<gz::msgs::Empty>(this->stockDetachTopic);
    if (!this->resultPublisher || !this->stockAttachPublisher ||
        !this->stockDetachPublisher)
    {
      gzerr << "PayloadCommandCoordinator could not advertise output topics\n";
      return;
    }
    const bool commandSubscribed = this->node.Subscribe(
        this->commandTopic, &PayloadCommandCoordinator::OnCommand, this);
    const bool stateSubscribed = this->node.Subscribe(
        this->physicalStateTopic,
        &PayloadCommandCoordinator::OnPhysicalState, this);
    if (!commandSubscribed || !stateSubscribed)
    {
      gzerr << "PayloadCommandCoordinator could not subscribe to input topics\n";
    }
  }

  private: void OnCommand(const gz::msgs::StringMsg &_message)
  {
    const auto parsed = ParseCommand(_message.data());
    if (!parsed)
    {
      gzerr << "PayloadCommandCoordinator rejected malformed command\n";
      return;
    }
    std::lock_guard<std::mutex> guard(this->mutex);
    const auto completed = this->completed.find(parsed->id);
    if (completed != this->completed.end())
    {
      if (completed->second.action == parsed->action)
        this->PublishResultLocked(completed->second.result);
      else
        this->PublishConflictLocked(parsed->id);
      return;
    }
    if (this->active && this->active->id == parsed->id)
    {
      if (this->active->action != parsed->action)
        this->PublishConflictLocked(parsed->id);
      return;
    }
    for (const auto &queued : this->queue)
    {
      if (queued.id != parsed->id)
        continue;
      if (queued.action != parsed->action)
        this->PublishConflictLocked(parsed->id);
      return;
    }
    this->queue.push_back(*parsed);
    this->StartNextLocked();
  }

  private: void OnPhysicalState(const gz::msgs::StringMsg &_message)
  {
    if (_message.data() != "attached" && _message.data() != "detached")
    {
      gzerr << "PayloadCommandCoordinator ignored invalid physical state\n";
      return;
    }
    std::lock_guard<std::mutex> guard(this->mutex);
    this->physicalState = _message.data();
    if (this->active && this->active->DesiredState() == *this->physicalState)
    {
      this->CompleteActiveLocked();
      this->StartNextLocked();
    }
  }

  private: void StartNextLocked()
  {
    while (!this->active && !this->queue.empty())
    {
      this->active = std::move(this->queue.front());
      this->queue.pop_front();
      if (this->physicalState &&
          *this->physicalState == this->active->DesiredState())
      {
        this->CompleteActiveLocked();
        continue;
      }
      gz::msgs::Empty trigger;
      auto &publisher = this->active->action == "attach"
          ? this->stockAttachPublisher : this->stockDetachPublisher;
      if (!publisher.Publish(trigger))
      {
        const auto result = ResultWire(
            this->active->id, "error", "unknown",
            "PHYSICAL_COMMAND_PUBLISH_FAILED");
        this->completed.emplace(
            this->active->id,
            CompletedCommand{this->active->action, result});
        this->PublishResultLocked(result);
        this->active.reset();
        continue;
      }
    }
  }

  private: void CompleteActiveLocked()
  {
    const auto result = ResultWire(
        this->active->id, "confirmed", this->active->DesiredState(), "OK");
    this->completed.emplace(
        this->active->id,
        CompletedCommand{this->active->action, result});
    this->PublishResultLocked(result);
    this->active.reset();
  }

  private: void PublishConflictLocked(const std::string &_commandId)
  {
    this->PublishResultLocked(ResultWire(
        _commandId, "error", "unknown", "COMMAND_ID_ACTION_CONFLICT"));
  }

  private: void PublishResultLocked(const std::string &_wire)
  {
    gz::msgs::StringMsg result;
    result.set_data(_wire);
    if (!this->resultPublisher.Publish(result))
    {
      gzerr << "PayloadCommandCoordinator failed to publish result ["
            << _wire << "]\n";
    }
  }

  private: std::mutex mutex;
  private: std::deque<Command> queue;
  private: std::optional<Command> active;
  private: std::optional<std::string> physicalState;
  private: std::unordered_map<std::string, CompletedCommand> completed;
  private: std::string commandTopic;
  private: std::string physicalStateTopic;
  private: std::string resultTopic;
  private: std::string stockAttachTopic;
  private: std::string stockDetachTopic;
  private: gz::transport::Node::Publisher resultPublisher;
  private: gz::transport::Node::Publisher stockAttachPublisher;
  private: gz::transport::Node::Publisher stockDetachPublisher;
  private: gz::transport::Node node;
};
}

GZ_ADD_PLUGIN(
    drone_sim::gazebo::PayloadCommandCoordinator,
    gz::sim::System,
    drone_sim::gazebo::PayloadCommandCoordinator::ISystemConfigure)
GZ_ADD_PLUGIN_ALIAS(
    drone_sim::gazebo::PayloadCommandCoordinator,
    "drone_sim::gazebo::PayloadCommandCoordinator")
