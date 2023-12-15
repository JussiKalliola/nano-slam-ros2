
#ifndef ATLAS_CONVERTER_HPP_
#define ATLAS_CONVERTER_HPP

#include "orbslam3_interfaces/msg/atlas.hpp"
#include "orbslam3_interfaces/msg/atlas_actions.hpp"

#include "MapPoint.h"
#include "Map.h"
#include "KeyFrame.h"
#include "Atlas.h"

#include "Converter.hpp"


namespace Converter {

  class AtlasConverter {
    using map_point = orbslam3_interfaces::msg::MapPoint;
    using map = orbslam3_interfaces::msg::Map;
    using atlas = orbslam3_interfaces::msg::Atlas;
    using atlas_action = orbslam3_interfaces::msg::AtlasActions;

    using orb_map_point = ORB_SLAM3::MapPoint;
    using orb_map = ORB_SLAM3::Map;
    using orb_keyframe = ORB_SLAM3::KeyFrame;
    using orb_atlas = ORB_SLAM3::Atlas;



  };
};

#endif
