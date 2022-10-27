// Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/MsgPack.h"
#include "carla/rpc/Location.h"
#include "carla/rpc/Vector3D.h"
#include "carla/rpc/ObjectLabel.h"

namespace carla {
namespace rpc {

  struct LabelledPoint {

    LabelledPoint () {}
    LabelledPoint (Location location, CityObjectLabel label)
     : _location(location), _label(label)
     {}
    LabelledPoint(Location location, CityObjectLabel label, Vector3D normal, float friction)
     : _location(location), _label(label), _normal(normal), _friction(friction)
     {}
    Location _location;

    CityObjectLabel _label;

    Vector3D _normal;
    float _friction = -1.0;

    MSGPACK_DEFINE_ARRAY(_location, _label, _normal, _friction);

  };

}
}
