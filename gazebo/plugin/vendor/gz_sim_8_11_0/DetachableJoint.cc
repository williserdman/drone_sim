/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

// Modified for Drone Sim: project-local plugin identity, optional
// initially-detached state, opt-in exclusive-parent arbitration, and recurrent
// timestamped level truth. See
// gazebo/provenance/gz-sim-detachable-joint.json.

#include <chrono>
#include <cmath>
#include <vector>

#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/common/Profiler.hh>

#include <sdf/Element.hh>

#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/ContactSensor.hh"
#include "gz/sim/components/ContactSensorData.hh"
#include "gz/sim/components/DetachableJoint.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

#include "DetachableJoint.hh"

using namespace gz;
using namespace sim;
using drone_sim::gazebo::DetachableJoint;

/////////////////////////////////////////////////
void DetachableJoint::Configure(const Entity &_entity,
               const std::shared_ptr<const sdf::Element> &_sdf,
               EntityComponentManager &_ecm,
               EventManager &/*_eventMgr*/)
{
  this->model = Model(_entity);
  if (!this->model.Valid(_ecm))
  {
    gzerr << "DetachableJoint should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  if (_sdf->HasElement("parent_link"))
  {
    auto parentLinkName = _sdf->Get<std::string>("parent_link");
    this->parentLinkEntity = this->model.LinkByName(_ecm, parentLinkName);
    if (kNullEntity == this->parentLinkEntity)
    {
      gzerr << "Link with name " << parentLinkName
             << " not found in model " << this->model.Name(_ecm)
             << ". Make sure the parameter 'parent_link' has the "
             << "correct value. Failed to initialize.\n";
      return;
    }
  }
  else
  {
    gzerr << "'parent_link' is a required parameter for DetachableJoint. "
              "Failed to initialize.\n";
    return;
  }

  if (_sdf->HasElement("child_model"))
  {
    this->childModelName = _sdf->Get<std::string>("child_model");
  }
  else
  {
    gzerr << "'child_model' is a required parameter for DetachableJoint."
              "Failed to initialize.\n";
    return;
  }

  if (_sdf->HasElement("child_link"))
  {
    this->childLinkName = _sdf->Get<std::string>("child_link");
  }
  else
  {
    gzerr << "'child_link' is a required parameter for DetachableJoint."
              "Failed to initialize.\n";
    return;
  }

  // Setup detach topic
  std::vector<std::string> detachTopics;
  if (_sdf->HasElement("detach_topic"))
  {
    detachTopics.push_back(_sdf->Get<std::string>("detach_topic"));
  }
  detachTopics.push_back("/model/" + this->model.Name(_ecm) +
      "/detachable_joint/detach");

  if (_sdf->HasElement("topic"))
  {
    if (_sdf->HasElement("detach_topic"))
    {
      if (_sdf->Get<std::string>("topic") !=
          _sdf->Get<std::string>("detach_topic"))
      {
        gzerr << "<topic> and <detach_topic> tags have different contents. "
                 "Please verify the correct string and use <detach_topic>."
              << std::endl;
      }
      else
      {
        gzdbg << "Ignoring <topic> tag and using <detach_topic> tag."
              << std::endl;
      }
    }
    else
    {
      detachTopics.insert(detachTopics.begin(),
                          _sdf->Get<std::string>("topic"));
    }
  }

  this->detachTopic = validTopic(detachTopics);
  if (this->detachTopic.empty())
  {
    gzerr << "No valid detach topics for DetachableJoint could be found.\n";
    return;
  }
  gzdbg << "Detach topic is: " << this->detachTopic << std::endl;

  // Setup subscriber for detach topic
  this->node.Subscribe(
      this->detachTopic, &DetachableJoint::OnDetachRequest, this);

  gzdbg << "DetachableJoint subscribing to messages on "
         << "[" << this->detachTopic << "]" << std::endl;

  // Setup attach topic
  std::vector<std::string> attachTopics;
  if (_sdf->HasElement("attach_topic"))
  {
    attachTopics.push_back(_sdf->Get<std::string>("attach_topic"));
  }
  attachTopics.push_back("/model/" + this->model.Name(_ecm) +
      "/detachable_joint/attach");
  this->attachTopic = validTopic(attachTopics);
  if (this->attachTopic.empty())
  {
    gzerr << "No valid attach topics for DetachableJoint could be found.\n";
    return;
  }
  gzdbg << "Attach topic is: " << this->attachTopic << std::endl;

  // Setup subscriber for attach topic
  auto msgCb = std::function<void(const transport::ProtoMsg &)>(
      [this](const auto &)
      {
        if (this->isAttached){
          gzdbg << "Already attached" << std::endl;
          return;
        }
        this->attachRequested = true;
      });

  if (!this->node.Subscribe(this->attachTopic, msgCb))
  {
    gzerr << "Subscriber could not be created for [attach] topic.\n";
    return;
  }

  // Setup output topic
  std::vector<std::string> outputTopics;
  if (_sdf->HasElement("output_topic"))
  {
    outputTopics.push_back(_sdf->Get<std::string>("output_topic"));
  }

  outputTopics.push_back("/model/" + this->childModelName +
      "/detachable_joint/state");

  this->outputTopic = validTopic(outputTopics);
  if (this->outputTopic.empty())
  {
    gzerr << "No valid output topics for DetachableJoint could be found.\n";
    return;
  }
  gzdbg << "Output topic is: " << this->outputTopic << std::endl;

  // Setup publisher for output topic
  this->outputPub = this->node.Advertise<gz::msgs::StringMsg>(
      this->outputTopic);
  if (!this->outputPub)
  {
    gzerr << "Error advertising topic [" << this->outputTopic << "]"
              << std::endl;
    return;
  }

  const bool hasContactSensor = _sdf->HasElement("contact_sensor");
  const bool hasContactStateTopic = _sdf->HasElement("contact_state_topic");
  if (hasContactSensor != hasContactStateTopic)
  {
    gzerr << "contact_sensor and contact_state_topic must be configured "
             "together\n";
    return;
  }
  if (hasContactSensor)
  {
    this->contactSensorName = _sdf->Get<std::string>("contact_sensor");
    this->contactStateTopic = validTopic(
        {_sdf->Get<std::string>("contact_state_topic")});
    if (this->contactSensorName.empty() || this->contactStateTopic.empty())
    {
      gzerr << "No valid recurrent contact source and topic could be found\n";
      return;
    }
    this->contactStatePub = this->node.Advertise<gz::msgs::Contacts>(
        this->contactStateTopic);
    if (!this->contactStatePub)
    {
      gzerr << "Error advertising topic [" << this->contactStateTopic << "]"
            << std::endl;
      return;
    }
  }

  // Supress Child Warning
  this->suppressChildWarning =
      _sdf->Get<bool>("suppress_child_warning", this->suppressChildWarning)
          .first;

  const bool initiallyAttached =
      _sdf->Get<bool>("initially_attached", true).first;
  this->attachRequested = initiallyAttached;
  this->publishInitialDetached = !initiallyAttached;
  this->exclusiveParent =
      _sdf->Get<bool>("exclusive_parent", false).first;
  const auto statePublishPeriod =
      _sdf->Get<double>("state_publish_period", 0.0).first;
  if (!std::isfinite(statePublishPeriod) || statePublishPeriod < 0.0)
  {
    gzerr << "state_publish_period must be finite and nonnegative\n";
    this->validConfig = false;
    return;
  }
  this->statePublishPeriodNs = std::chrono::duration_cast<
      std::chrono::nanoseconds>(
          std::chrono::duration<double>(statePublishPeriod)).count();
  if (statePublishPeriod > 0.0 && this->statePublishPeriodNs <= 0)
  {
    gzerr << "state_publish_period is below nanosecond resolution\n";
    this->validConfig = false;
    return;
  }

  this->validConfig = true;

  this->GetChildModelAndLinkEntities(_ecm);
}

//////////////////////////////////////////////////
void DetachableJoint::GetChildModelAndLinkEntities(
  EntityComponentManager &_ecm)
{
  this->childLinkEntity = kNullEntity;
  // Look for the child model and link
  Entity modelEntity{kNullEntity};

  if ("__model__" == this->childModelName)
  {
    modelEntity = this->model.Entity();
  }
  else
  {
    auto entitiesMatchingName = entitiesFromScopedName(
      this->childModelName, _ecm);

    // TODO(arjoc): There is probably a more efficient way
    // of combining entitiesFromScopedName
    // With filtering.
    // Filter for entities with only models
    std::vector<Entity> candidateEntities;
    std::copy_if(entitiesMatchingName.begin(), entitiesMatchingName.end(),
                std::back_inserter(candidateEntities),
                [&_ecm](Entity e) {
                  return _ecm.EntityHasComponentType(e,
                            components::Model::typeId);
                });

    if (candidateEntities.size() == 1)
    {
      // If there is one entity select that entity itself
      modelEntity = *candidateEntities.begin();
    }
    else
    {
      std::string selectedModelName;
      auto parentEntityScopedPath = scopedName(this->model.Entity(), _ecm);
      // If there is more than one model with the given child model name,
      // the plugin looks for a model which is
      // - a descendant of the plugin's parent model with that name, and
      // - has a child link with the given child link name
      for (auto entity : candidateEntities)
      {
        auto childEntityScope = scopedName(entity, _ecm);
        if (childEntityScope.size() < parentEntityScopedPath.size())
        {
          continue;
        }
        if (childEntityScope.rfind(parentEntityScopedPath, 0) != 0)
        {
          continue;
        }
        if (modelEntity == kNullEntity)
        {

          this->childLinkEntity = _ecm.EntityByComponents(
                    components::Link(), components::ParentEntity(entity),
                    components::Name(this->childLinkName));

          if (kNullEntity != this->childLinkEntity)
          {
                // Only select this child model entity if the entity
                // has a link with the given child link name
                modelEntity = entity;
                selectedModelName = childEntityScope;
                gzdbg << "Selecting " << childEntityScope
                  << " as model to be detached" << std::endl;
          }
          else
          {
            gzwarn << "Found " << childEntityScope
              << " with no link named " << this->childLinkName << std::endl;
          }
        }
        else
        {
          gzwarn << "Found multiple models skipping " << childEntityScope
            << "Using " << selectedModelName << " instead" << std::endl;
        }
      }
    }
  }
  if (kNullEntity != modelEntity)
  {
    this->childLinkEntity = _ecm.EntityByComponents(
        components::Link(), components::ParentEntity(modelEntity),
        components::Name(this->childLinkName));
  }
  else if (!this->suppressChildWarning)
  {
    gzwarn << "Child Model " << this->childModelName
            << " could not be found.\n";
  }
}
//////////////////////////////////////////////////
void DetachableJoint::PreUpdate(
  const UpdateInfo &_info,
  EntityComponentManager &_ecm)
{
  GZ_PROFILE("DetachableJoint::PreUpdate");
  if (this->validConfig && this->publishInitialDetached)
  {
    if (this->statePublishPeriodNs == 0)
      this->PublishJointState(false, _info.simTime.count());
    this->publishInitialDetached = false;
  }
  if (this->validConfig && !this->contactStateTopic.empty() &&
      (this->childLinkEntity == kNullEntity ||
       !_ecm.HasEntity(this->childLinkEntity)))
    this->GetChildModelAndLinkEntities(_ecm);

  // only allow attaching if child entity is detached
  if (this->validConfig && !this->isAttached)
  {
    // return if attach is not requested.
    if (!this->attachRequested){
      this->PublishPeriodicJointState(_info.simTime);
      return;
    }

    if (this->exclusiveParent)
    {
      bool parentOccupied = false;
      _ecm.Each<components::DetachableJoint>(
          [this, &parentOccupied](
              const Entity &, const components::DetachableJoint *_joint)
          {
            if (_joint->Data().parentLink == this->parentLinkEntity)
            {
              parentOccupied = true;
              return false;
            }
            return true;
          });
      if (parentOccupied)
      {
        this->PublishPeriodicJointState(_info.simTime);
        return;
      }
    }

    if (this->childLinkEntity == kNullEntity ||
        !_ecm.HasEntity(this->childLinkEntity))
      this->GetChildModelAndLinkEntities(_ecm);

    if (kNullEntity != this->childLinkEntity)
    {
      // Attach the models
      // We do this by creating a detachable joint entity.
      this->detachableJointEntity = _ecm.CreateEntity();

      _ecm.CreateComponent(
          this->detachableJointEntity,
          components::DetachableJoint({this->parentLinkEntity,
                                        this->childLinkEntity, "fixed"}));
      this->attachRequested = false;
      this->isAttached = true;
      if (this->statePublishPeriodNs == 0)
        this->PublishJointState(this->isAttached, _info.simTime.count());
      gzdbg << "Attaching entity: " << this->detachableJointEntity
              << std::endl;
    }
    else if (!this->suppressChildWarning)
    {
      gzwarn << "Child Link " << this->childLinkName
              << " could not be found.\n";
    }

  }

  // only allow detaching if child entity is attached
  if (this->isAttached)
  {
    if (this->detachRequested && (kNullEntity != this->detachableJointEntity))
    {
      // Detach the models
      gzdbg << "Removing entity: " << this->detachableJointEntity << std::endl;
      _ecm.RequestRemoveEntity(this->detachableJointEntity);
      this->detachableJointEntity = kNullEntity;
      this->detachRequested = false;
      this->isAttached = false;
      if (this->statePublishPeriodNs == 0)
        this->PublishJointState(this->isAttached, _info.simTime.count());
    }
  }
  this->PublishPeriodicJointState(_info.simTime);
}

//////////////////////////////////////////////////
void DetachableJoint::PostUpdate(
  const UpdateInfo &_info,
  const EntityComponentManager &_ecm)
{
  GZ_PROFILE("DetachableJoint::PostUpdate");
  this->PublishPeriodicContactState(_info.simTime, _ecm);
}

//////////////////////////////////////////////////
void DetachableJoint::PublishPeriodicJointState(
    const std::chrono::steady_clock::duration &_simTime)
{
  if (!this->validConfig || this->statePublishPeriodNs == 0)
    return;
  const auto timestampNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
      _simTime).count();
  if (timestampNs < 0 || timestampNs % this->statePublishPeriodNs != 0 ||
      timestampNs == this->lastStatePublishTimestampNs)
    return;
  this->PublishJointState(this->isAttached, timestampNs);
  this->lastStatePublishTimestampNs = timestampNs;
}

//////////////////////////////////////////////////
void DetachableJoint::PublishPeriodicContactState(
    const std::chrono::steady_clock::duration &_simTime,
    const EntityComponentManager &_ecm)
{
  if (!this->validConfig || this->contactStateTopic.empty() ||
      this->statePublishPeriodNs == 0)
    return;
  const auto timestampNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
      _simTime).count();
  if (timestampNs < 0 || timestampNs % this->statePublishPeriodNs != 0 ||
      timestampNs == this->lastContactPublishTimestampNs)
    return;

  if (this->childLinkEntity == kNullEntity ||
      !_ecm.HasEntity(this->childLinkEntity))
    return;
  if (this->contactSensorEntity == kNullEntity ||
      !_ecm.HasEntity(this->contactSensorEntity))
  {
    _ecm.Each<components::ContactSensor,
              components::Name,
              components::ParentEntity>(
        [this](
            const Entity &_entity,
            const components::ContactSensor *,
            const components::Name *_name,
            const components::ParentEntity *_parent)
        {
          if (_name->Data() == this->contactSensorName &&
              _parent->Data() == this->childLinkEntity)
          {
            this->contactSensorEntity = _entity;
            return false;
          }
          return true;
        });
    if (this->contactSensorEntity == kNullEntity)
      return;
  }
  if (this->contactCollisionEntity == kNullEntity ||
      !_ecm.HasEntity(this->contactCollisionEntity))
  {
    const auto sensor = _ecm.Component<components::ContactSensor>(
        this->contactSensorEntity);
    if (sensor == nullptr || sensor->Data() == nullptr ||
        !sensor->Data()->HasElement("contact"))
      return;
    const auto contact = sensor->Data()->GetElement("contact");
    if (contact == nullptr || !contact->HasElement("collision"))
      return;
    const auto collisionName =
        contact->GetElement("collision")->Get<std::string>();
    this->contactCollisionEntity = _ecm.EntityByComponents(
        components::Collision(),
        components::ParentEntity(this->childLinkEntity),
        components::Name(collisionName));
    if (this->contactCollisionEntity == kNullEntity)
      return;
  }

  const auto data = _ecm.Component<components::ContactSensorData>(
      this->contactCollisionEntity);
  if (data == nullptr)
    return;
  auto contactState = data->Data();
  auto stamp = contactState.mutable_header()->mutable_stamp();
  stamp->set_sec(timestampNs / 1'000'000'000LL);
  stamp->set_nsec(timestampNs % 1'000'000'000LL);
  this->contactStatePub.Publish(contactState);
  this->lastContactPublishTimestampNs = timestampNs;
}

//////////////////////////////////////////////////
void DetachableJoint::PublishJointState(bool attached, std::int64_t timestampNs)
{
  msgs::StringMsg detachedStateMsg;
  const std::string state = attached ? "attached" : "detached";
  if (this->statePublishPeriodNs == 0)
    detachedStateMsg.set_data(state);
  else
    detachedStateMsg.set_data(
        "payload-joint-state-v1|" + std::to_string(timestampNs) + '|' + state);
  this->outputPub.Publish(detachedStateMsg);
}

//////////////////////////////////////////////////
void DetachableJoint::OnDetachRequest(const msgs::Empty &)
{
  if (!this->isAttached){
    gzdbg << "Already detached" << std::endl;
    return;
  }
  this->detachRequested = true;
}

GZ_ADD_PLUGIN(DetachableJoint,
                    System,
                    DetachableJoint::ISystemConfigure,
                    DetachableJoint::ISystemPreUpdate,
                    DetachableJoint::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(DetachableJoint,
  "drone_sim::gazebo::DetachableJoint")
