#include <mutex>
#include <sstream>
#include <string>

#include <QEvent>
#include <QString>

#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/Static.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/gui/GuiEvents.hh>
#include <gz/sim/gui/GuiSystem.hh>

class ReadOnlyInspector final : public gz::sim::GuiSystem
{
  Q_OBJECT
  Q_PROPERTY(QString properties READ Properties NOTIFY PropertiesChanged)

  public: void LoadConfig(const tinyxml2::XMLElement *) override
  {
    if (this->title.empty())
      this->title = "Read-only inspector";
    gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(this);
  }

  public: void Update(
      const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &_ecm) override
  {
    gz::sim::Entity entity;
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      entity = this->selected;
    }

    std::ostringstream output;
    if (entity == gz::sim::kNullEntity)
    {
      output << "Select an entity in the scene to inspect its properties.";
    }
    else
    {
      output << "entity: " << entity << '\n';
      if (const auto *component =
              _ecm.Component<gz::sim::components::Name>(entity))
        output << "name: " << component->Data() << '\n';

      output << "type: " << this->EntityType(entity, _ecm) << '\n';
      if (const auto *component =
              _ecm.Component<gz::sim::components::ParentEntity>(entity))
        output << "parent: " << component->Data() << '\n';
      if (const auto *component =
              _ecm.Component<gz::sim::components::Static>(entity))
        output << "static: " << std::boolalpha << component->Data() << '\n';
      if (const auto *component =
              _ecm.Component<gz::sim::components::Pose>(entity))
        output << "pose: " << component->Data() << '\n';
      if (const auto *component =
              _ecm.Component<gz::sim::components::LinearVelocity>(entity))
        output << "linear velocity: " << component->Data() << '\n';
      if (const auto *component =
              _ecm.Component<gz::sim::components::AngularVelocity>(entity))
        output << "angular velocity: " << component->Data() << '\n';
    }

    const QString next = QString::fromStdString(output.str());
    if (next != this->properties)
    {
      this->properties = next;
      emit this->PropertiesChanged();
    }
  }

  public: QString Properties() const
  {
    return this->properties;
  }

  signals: void PropertiesChanged();

  protected: bool eventFilter(QObject *_object, QEvent *_event) override
  {
    if (_event->type() == gz::sim::gui::events::EntitiesSelected::kType)
    {
      const auto *selectedEvent =
          static_cast<gz::sim::gui::events::EntitiesSelected *>(_event);
      if (!selectedEvent->Data().empty())
      {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->selected = selectedEvent->Data().front();
      }
    }
    else if (_event->type() ==
             gz::sim::gui::events::DeselectAllEntities::kType)
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      this->selected = gz::sim::kNullEntity;
    }
    return QObject::eventFilter(_object, _event);
  }

  private: static const char *EntityType(
      gz::sim::Entity _entity, const gz::sim::EntityComponentManager &_ecm)
  {
    if (_ecm.Component<gz::sim::components::World>(_entity))
      return "world";
    if (_ecm.Component<gz::sim::components::Model>(_entity))
      return "model";
    if (_ecm.Component<gz::sim::components::Link>(_entity))
      return "link";
    if (_ecm.Component<gz::sim::components::Joint>(_entity))
      return "joint";
    if (_ecm.Component<gz::sim::components::Collision>(_entity))
      return "collision";
    if (_ecm.Component<gz::sim::components::Visual>(_entity))
      return "visual";
    if (_ecm.Component<gz::sim::components::Sensor>(_entity))
      return "sensor";
    return "entity";
  }

  private: std::mutex mutex;
  private: gz::sim::Entity selected{gz::sim::kNullEntity};
  private: QString properties{
      "Select an entity in the scene to inspect its properties."};
};

GZ_ADD_PLUGIN(ReadOnlyInspector, gz::gui::Plugin)

#include "read_only_inspector.moc"
