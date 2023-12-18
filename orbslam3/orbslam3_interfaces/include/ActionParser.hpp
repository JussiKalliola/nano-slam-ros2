#ifndef ACTION_PARSER_HPP_
#define ACTION_PARSER_HPP

#include "orbslam3_interfaces/msg/atlas.hpp"
#include "orbslam3_interfaces/msg/atlas_actions.hpp"

#include "MapPoint.h"
#include "Map.h"
#include "KeyFrame.h"
#include "Atlas.h"

#include "Converter.hpp"

namespace Parser {
  
  using map_point = orbslam3_interfaces::msg::MapPoint;
  using map = orbslam3_interfaces::msg::Map;
  using atlas = orbslam3_interfaces::msg::Atlas;
  using atlas_action = orbslam3_interfaces::msg::AtlasActions;
  using kf_action = orbslam3_interfaces::msg::KeyFrameActions;

  using orb_map_point = ORB_SLAM3::MapPoint;
  using orb_map = ORB_SLAM3::Map;
  using orb_keyframe = ORB_SLAM3::KeyFrame;
  using orb_atlas = ORB_SLAM3::Atlas;

  class Action {
    
    public:
      /* KEYFRAME ACTION FUNCTIONS */
      static kf_action FormKFActionRosMsg(int actionId, unsigned long int id) {
        kf_action kfAMsg;

        return kfAMsg;
      }

      static kf_action FormKFActionRosMsg(int actionId, bool boolAction) {
        kf_action kfAMsg;

        return kfAMsg;
      }

      static kf_action FormKFActionRosMsg(int actionId, unsigned long int id, long int vectorIdx) {
        kf_action kfAMsg;

        return kfAMsg;
      }

      static kf_action FormKFActionRosMsg(int actionId, Eigen::Vector3f t) {
        kf_action kfAMsg;

        return kfAMsg;
      }

      static kf_action FormKFActionRosMsg(int actionId, Sophus::SE3<float> p) {
        kf_action kfAMsg;

        return kfAMsg;
      }
      /* ATLAS ACTION FUNCTIONS */
      static atlas_action FormAtlasActionRosMsg(int actionId, bool boolAction) {
        atlas_action aMsg;
        aMsg.change_map_to_id = -1;
        aMsg.add_kf_id = -1;
        aMsg.add_map_point_id = -1;
        aMsg.set_map_bad_id = -1;


        if (actionId==4) { 
          aMsg.create_new_map = boolAction;
        }

        if (actionId==5) {
          aMsg.inform_new_big_change = boolAction;
        }

        if (actionId==6) {
          aMsg.clear_current_map = boolAction;
        }

        if (actionId==7) {
          aMsg.clear_atlas = boolAction;
        }

        if (actionId==8) {
          aMsg.remove_bad_maps = boolAction;
        }

        if (actionId==9) {
          aMsg.set_intertial_sensor = boolAction;
        }
        
        if (actionId==10) {
          aMsg.set_imu_initialized = boolAction;
        }
        
        return aMsg;
      }
      static atlas_action FormAtlasActionRosMsg(int actionId, unsigned long int id) {
        atlas_action aMsg;

        aMsg.change_map_to_id = -1;
        aMsg.add_kf_id = -1;
        aMsg.add_map_point_id = -1;
        aMsg.set_map_bad_id = -1;
        
        if (actionId == 0) {
          aMsg.add_map_point_id = id;
        }     
        
        if (actionId==1) {
          aMsg.change_map_to_id = id;
        }

        if (actionId==2) {
          aMsg.add_kf_id = id;
        }

        if (actionId==3) {
          aMsg.set_map_bad_id = id;
        }
        
        
        return aMsg;
      }

      static int parseAtlasAction(atlas_action::SharedPtr rAA, std::map<long unsigned int, ORB_SLAM3::Map*> mpOrbMaps, std::map<long unsigned int, ORB_SLAM3::KeyFrame*> mpOrbKeyFrames, std::map<long unsigned int, ORB_SLAM3::MapPoint*> mpOrbMapPoints) {

        if (rAA->add_map_point_id > 0) {
          if (mpOrbMapPoints.find(rAA->add_map_point_id) != mpOrbMapPoints.end()) {
            return 0;
          }     
        }


        if (rAA->change_map_to_id > 0) {
          if (mpOrbMaps.find(rAA->change_map_to_id) != mpOrbMaps.end()) {
            return 1;
          }
        }

        if (rAA->add_kf_id > 0) {
          if (mpOrbKeyFrames.find(rAA->add_kf_id) != mpOrbKeyFrames.end()) {
            return 2;
          }
        }

        if (rAA->set_map_bad_id > 0) {
          if (mpOrbMaps.find(rAA->set_map_bad_id) != mpOrbMaps.end()) {
            return 3;
          }
        }
        
        if (rAA->create_new_map) { 
          return 4;
        }

        if (rAA->inform_new_big_change) {
          return 5;
        }

        if (rAA->clear_current_map) {
          return 6;
        }


        if (rAA->clear_atlas) {
          return 7;
        }

        if (rAA->remove_bad_maps) {
          return 8;
        }

        if (rAA->set_intertial_sensor) {
          return 9;
        }
        
        if (rAA->set_imu_initialized) {
          return 10;
        }
          
        return -1;
      }


  };
};

#endif
