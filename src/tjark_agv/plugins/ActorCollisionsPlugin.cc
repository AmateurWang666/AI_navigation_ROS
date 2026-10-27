/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
 */

#include "ActorCollisionsPlugin.hh"

#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <map>
#include <string>

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(ActorCollisionsPlugin)

ActorCollisionsPlugin::ActorCollisionsPlugin() = default;

void ActorCollisionsPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  auto actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  if (!actor)
  {
    gzerr << "ActorCollisionsPlugin requires an actor model." << std::endl;
    return;
  }

  std::map<std::string, ignition::math::Vector3d> scaling;
  std::map<std::string, ignition::math::Pose3d> offsets;

  if (_sdf->HasElement("scaling"))
  {
    auto elem = _sdf->GetElement("scaling");
    while (elem)
    {
      if (!elem->HasAttribute("collision"))
      {
        gzwarn << "Skipping scaling element without collision attribute."
               << std::endl;
        elem = elem->GetNextElement("scaling");
        continue;
      }

      const auto name = elem->Get<std::string>("collision");
      if (elem->HasAttribute("scale"))
        scaling[name] = elem->Get<ignition::math::Vector3d>("scale");
      if (elem->HasAttribute("pose"))
        offsets[name] = elem->Get<ignition::math::Pose3d>("pose");
      elem = elem->GetNextElement("scaling");
    }
  }

  for (const auto &link : actor->GetLinks())
  {
    link->Init();

    if (scaling.empty())
      continue;

    for (const auto &collision : link->GetCollisions())
    {
      const auto name = collision->GetName();

      if (scaling.find(name) != scaling.end())
      {
        auto boxShape = boost::dynamic_pointer_cast<physics::BoxShape>(
            collision->GetShape());
        if (boxShape)
          boxShape->SetSize(boxShape->Size() * scaling[name]);
      }

      if (offsets.find(name) != offsets.end())
      {
        collision->SetInitialRelativePose(
            offsets[name] + collision->InitialRelativePose());
      }
    }
  }
}
}  // namespace gazebo
