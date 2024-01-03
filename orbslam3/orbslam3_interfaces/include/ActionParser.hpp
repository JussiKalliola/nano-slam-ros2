#ifndef ACTION_PARSER_HPP_
#define ACTION_PARSER_HPP

#include "orbslam3_interfaces/msg/map_point.hpp"
#include "orbslam3_interfaces/msg/map_point_actions.hpp"

#include "orbslam3_interfaces/msg/atlas.hpp"
#include "orbslam3_interfaces/msg/atlas_actions.hpp"

#include "orbslam3_interfaces/msg/key_frame.hpp"
#include "orbslam3_interfaces/msg/key_frame_actions.hpp"

#include "orbslam3_interfaces/msg/key_frame_database_actions.hpp"

#include "MapPoint.h"
#include "Map.h"
#include "KeyFrame.h"
#include "Atlas.h"

#include "Converter.hpp"

namespace Parser {
  
  using map_point = orbslam3_interfaces::msg::MapPoint;
  using map_point_actions = orbslam3_interfaces::msg::MapPointActions;
  using map = orbslam3_interfaces::msg::Map;
  using atlas = orbslam3_interfaces::msg::Atlas;
  using atlas_action = orbslam3_interfaces::msg::AtlasActions;
  using kf_action = orbslam3_interfaces::msg::KeyFrameActions;
  using kf_db_action = orbslam3_interfaces::msg::KeyFrameDatabaseActions;

  using orb_map_point = ORB_SLAM3::MapPoint;
  using orb_map = ORB_SLAM3::Map;
  using orb_keyframe = ORB_SLAM3::KeyFrame;
  using orb_atlas = ORB_SLAM3::Atlas;

  class Action {
    
    public:
      /* KEYFRAME ACTION FUNCTIONS */
      static kf_action::SharedPtr FormDefaultKFAction() {
        kf_action::SharedPtr kfAMsg = std::make_shared<kf_action>();
        // kfAMsg.set_pose = geometry_msgs::msg::Pose(); //0

        //geometry_msgs/Vector3 set_velocity //1

        kfAMsg->add_connection_id = -1;
        kfAMsg->add_connection_weight = -1; //2
        //

        kfAMsg->add_map_point_mp_id = -1; //3         //# map point id
        kfAMsg->add_map_point_vector_idx = -1; //3     // idx in the vector

        kfAMsg->erase_map_point_vector_idx = -1; //4   // Remove map point at vector idx

        kfAMsg->erase_map_point_id = -1; //5          // Find map point ptr with id, remove that

        kfAMsg->replace_map_point_id = -1; //6
        kfAMsg->replace_map_point_vector_idx = -1; //6

        kfAMsg->add_child_id = -1;//7

        kfAMsg->erase_child_id = -1;//8

        kfAMsg->change_parent_id = -1;//9

        kfAMsg->add_loop_edge_id = -1;//10

        kfAMsg->add_merge_edge_id = -1; //11


        kfAMsg->erase_connection_id = -1; //12

        //kfAMsg.set_new_bias = orbslam3_interfaces::msg::IMUBias(); //13

        kfAMsg->update_map_id = -1; // 14

        //bool compute_bow 15

        //bool update_best_covisibles 16

        //bool update_connections_up_parent 17

        //bool set_first_connection 18

        //bool set_not_erase 19

        //bool set_erase 20

        //bool set_bad_flag 21
        return kfAMsg;
      }

      static kf_action::SharedPtr FormKFActionRosMsg(int actionId, unsigned long int id) {
        kf_action::SharedPtr kfAMsg = FormDefaultKFAction();

        if(actionId==4) kfAMsg->erase_map_point_vector_idx = id; //4   // Remove map point at vector idx
        if(actionId==5) kfAMsg->erase_map_point_id = id; //5          // Find map point ptr with id, remove that
        if(actionId==7) kfAMsg->add_child_id = id;//7
        if(actionId==8) kfAMsg->erase_child_id = id;//8
        if(actionId==9) kfAMsg->change_parent_id = id;//9
        if(actionId==10) kfAMsg->add_loop_edge_id = id;//10
        if(actionId==11) kfAMsg->add_merge_edge_id = id; //11
        if(actionId==12) kfAMsg->erase_connection_id = id; //12
        if(actionId==14) kfAMsg->update_map_id = id; // 14
        
        return kfAMsg;
      }

      static kf_action::SharedPtr FormKFActionRosMsg(int actionId, bool boolAction) {
        kf_action::SharedPtr kfAMsg = FormDefaultKFAction();
        
        if(actionId==15) kfAMsg->compute_bow = true;
        if(actionId==16) kfAMsg->update_best_covisibles = true;
        if(actionId==17) kfAMsg->update_connections_up_parent = true;
        if(actionId==18) kfAMsg->set_first_connection = true;
        if(actionId==19) kfAMsg->set_not_erase = true;
        if(actionId==20) kfAMsg->set_erase = true;
        if(actionId==21) kfAMsg->set_bad_flag = true;
        
        return kfAMsg;
      }

      static kf_action::SharedPtr FormKFActionRosMsg(int actionId, unsigned long int id, long int vectorIdx) {
        kf_action::SharedPtr kfAMsg = FormDefaultKFAction();

        if(actionId==2) {
          kfAMsg->add_connection_id = id;
          kfAMsg->add_connection_weight = vectorIdx; //2
        }
        
        if(actionId==3) {
          kfAMsg->add_map_point_mp_id = id; //3         //# map point id
          kfAMsg->add_map_point_vector_idx = vectorIdx; //3     // idx in the vector
        }
        
        if(actionId==6) {
          kfAMsg->replace_map_point_id = id; //6
          kfAMsg->replace_map_point_vector_idx = vectorIdx; //6
        }
        
        return kfAMsg;
      }

      static kf_action::SharedPtr FormKFActionRosMsg(int actionId, Eigen::Vector3f t) {
        kf_action::SharedPtr kfAMsg = FormDefaultKFAction();

        if(actionId==1) {
          kfAMsg->bool_set_velocity = true;
          kfAMsg->set_velocity = Converter::CppToRos::EigenVector3fToVector3(t); //0
        }
        return kfAMsg;
      }

      static kf_action::SharedPtr FormKFActionRosMsg(int actionId, Sophus::SE3<float> p) {
        kf_action::SharedPtr kfAMsg = FormDefaultKFAction();
        
        if(actionId==0) {
          kfAMsg->bool_set_pose = true;
          kfAMsg->set_pose = Converter::CppToRos::SophusSE3fToPose(p); //0
        }

        return kfAMsg;
      }
      static kf_action::SharedPtr FormKFActionRosMsg(int actionId, ORB_SLAM3::IMU::Bias oImuB) {
        kf_action::SharedPtr kfAMsg = FormDefaultKFAction();
        
        if(actionId==13) {
          kfAMsg->bool_set_new_bias = true;
          kfAMsg->set_new_bias = Converter::OrbToRos::ImuBiasToRosBias(oImuB); //13
        }

        return kfAMsg;
      }
      
      static int parseKFAction(kf_action::SharedPtr rKf, std::map<long unsigned int, ORB_SLAM3::Map*> mpOrbMaps, std::map<long unsigned int, ORB_SLAM3::KeyFrame*> mpOrbKeyFrames, std::map<long unsigned int, ORB_SLAM3::MapPoint*> mpOrbMapPoints) {
        if(mpOrbKeyFrames.find(rKf->kf_id) != mpOrbKeyFrames.end()) {
          if(rKf->bool_set_pose) return 0;
          if(rKf->bool_set_velocity) return 1;
          if(rKf->add_connection_id >= 0 && rKf->add_connection_weight >= 0 && mpOrbKeyFrames.find(rKf->add_connection_id) != mpOrbKeyFrames.end()) return 2; 
          if(rKf->add_map_point_mp_id >= 0 && rKf->add_map_point_vector_idx >= 0 && mpOrbMapPoints.find(rKf->add_map_point_mp_id) != mpOrbMapPoints.end()) return 3; 
          if(rKf->erase_map_point_vector_idx >= 0) return 4; // Remove map point at vector idx
          if(rKf->erase_map_point_id >= 0 && mpOrbMapPoints.find(rKf->erase_map_point_id) != mpOrbMapPoints.end()) return 5; // Find map point ptr with id, remove that
          if(rKf->replace_map_point_id >= 0 && rKf->replace_map_point_vector_idx >= 0 && mpOrbMapPoints.find(rKf->replace_map_point_id) != mpOrbMapPoints.end()) return 6; 
          if(rKf->add_child_id >= 0 && mpOrbKeyFrames.find(rKf->add_child_id) != mpOrbKeyFrames.end()) return 7; 
          if(rKf->erase_child_id >= 0 && mpOrbKeyFrames.find(rKf->erase_child_id) != mpOrbKeyFrames.end()) return 8; 
          if(rKf->change_parent_id >= 0 && mpOrbKeyFrames.find(rKf->change_parent_id) != mpOrbKeyFrames.end()) return 9; 
          if(rKf->add_loop_edge_id >= 0 && mpOrbKeyFrames.find(rKf->add_loop_edge_id) != mpOrbKeyFrames.end()) return 10; 
          if(rKf->add_merge_edge_id >= 0 && mpOrbKeyFrames.find(rKf->add_merge_edge_id) != mpOrbKeyFrames.end()) return 11; 
          if(rKf->erase_connection_id >= 0 && mpOrbKeyFrames.find(rKf->erase_connection_id) != mpOrbKeyFrames.end()) return 12; 
          if(rKf->bool_set_new_bias) return 13; 
          if(rKf->update_map_id && mpOrbMaps.find(rKf->update_map_id) != mpOrbMaps.end()) return 14; 
          if(rKf->compute_bow) return 15;
          if(rKf->update_best_covisibles) return 16;
          if(rKf->update_connections_up_parent) return 17;
          if(rKf->set_first_connection) return 18;
          if(rKf->set_not_erase) return 19;
          if(rKf->set_erase) return 20;
          if(rKf->set_bad_flag) return 21;

        }
        
        return -1;
      }



      /* ATLAS ACTION FUNCTIONS */
      static atlas_action::SharedPtr FormAtlasActionRosMsg(int actionId, bool boolAction) {
        atlas_action::SharedPtr aMsg = std::make_shared<atlas_action>();
        
        aMsg->change_map_to_id = -1;
        aMsg->add_kf_id = -1;
        aMsg->add_map_point_id = -1;
        aMsg->set_map_bad_id = -1;
        if (actionId==4) aMsg->create_new_map = boolAction;
        if (actionId==5) aMsg->inform_new_big_change = boolAction;
        if (actionId==6) aMsg->clear_current_map = boolAction;
        if (actionId==7) aMsg->clear_atlas = boolAction;
        if (actionId==8) aMsg->remove_bad_maps = boolAction;
        if (actionId==9) aMsg->set_intertial_sensor = boolAction; 
        if (actionId==10) aMsg->set_imu_initialized = boolAction;
        
        return aMsg;
      }
      static atlas_action::SharedPtr FormAtlasActionRosMsg(int actionId, unsigned long int id) {
        atlas_action::SharedPtr aMsg = std::make_shared<atlas_action>();

        aMsg->change_map_to_id = -1;
        aMsg->add_kf_id = -1;
        aMsg->add_map_point_id = -1;
        aMsg->set_map_bad_id = -1;
        
        if (actionId==0) aMsg->add_map_point_id = id;
        if (actionId==1) aMsg->change_map_to_id = id;
        if (actionId==2) aMsg->add_kf_id = id;
        if (actionId==3) aMsg->set_map_bad_id = id;        
        
        return aMsg;
      }

      static int parseAtlasAction(atlas_action::SharedPtr rAA, std::map<long unsigned int, ORB_SLAM3::Map*> mpOrbMaps, std::map<long unsigned int, ORB_SLAM3::KeyFrame*> mpOrbKeyFrames, std::map<long unsigned int, ORB_SLAM3::MapPoint*> mpOrbMapPoints) {

        if (rAA->add_map_point_id > 0) {
          if (mpOrbMapPoints.find(rAA->add_map_point_id) != mpOrbMapPoints.end()) return 0;
        }


        if (rAA->change_map_to_id > 0) {
          if (mpOrbMaps.find(rAA->change_map_to_id) != mpOrbMaps.end()) return 1;
        }

        if (rAA->add_kf_id > 0) {
          if (mpOrbKeyFrames.find(rAA->add_kf_id) != mpOrbKeyFrames.end()) return 2;
        }

        if (rAA->set_map_bad_id > 0) {
          if (mpOrbMaps.find(rAA->set_map_bad_id) != mpOrbMaps.end()) return 3;
        }
  
        if (rAA->create_new_map) return 4;
        if (rAA->inform_new_big_change) return 5;
        if (rAA->clear_current_map) return 6;
        if (rAA->clear_atlas) return 7;
        if (rAA->remove_bad_maps) return 8;
        if (rAA->set_intertial_sensor) return 9;  
        if (rAA->set_imu_initialized) return 10;
          
        return -1;
      }
      





      /* KEYFRAME DATABASE FUNCTIONS */
      static kf_db_action::SharedPtr FormKFDBActionRosMsg(int actionId, bool boolAction)
      {
        kf_db_action::SharedPtr kfdbActionMsg = std::make_shared<kf_db_action>();

        kfdbActionMsg->add_kf_id = -1;
        kfdbActionMsg->erase_kf_id = -1;
        kfdbActionMsg->clear = true; // this action can be only KeyFrameDatabase::clear()
        kfdbActionMsg->clear_map_id = -1;
      
        return kfdbActionMsg;
      }

      static kf_db_action::SharedPtr FormKFDBActionRosMsg(int actionId, unsigned long int id)
      {
        kf_db_action::SharedPtr kfdbActionMsg = std::make_shared<kf_db_action>();
        
        kfdbActionMsg->add_kf_id = -1;
        kfdbActionMsg->erase_kf_id = -1;
        kfdbActionMsg->clear_map_id = -1;
        
        if(actionId==0) kfdbActionMsg->add_kf_id = id;
        if(actionId==1) kfdbActionMsg->erase_kf_id = id;
        if(actionId==2) kfdbActionMsg->clear_map_id = id;
      
        return kfdbActionMsg;
      }
      
      static kf_db_action::SharedPtr FormKFDBActionRosMsg(int actionId, unsigned long int id, float minScore, std::vector<unsigned long int> vpLoopCandId, std::vector<unsigned long int> vpMergeCandId)
      {
        kf_db_action::SharedPtr kfdbActionMsg = std::make_shared<kf_db_action>();
        
        kfdbActionMsg->add_kf_id = -1;
        kfdbActionMsg->erase_kf_id = -1;
        kfdbActionMsg->clear_map_id = -1;
        
        kfdbActionMsg->detect_candidates = true;
        kfdbActionMsg->detect_candidates_kf_id = id;
        kfdbActionMsg->detect_candidates_min_score = minScore;
        kfdbActionMsg->detect_candidates_loop_cand_ids = vpLoopCandId;
        kfdbActionMsg->detect_candidates_merge_cand_ids = vpMergeCandId;

      
        return kfdbActionMsg;
      }
      
      static kf_db_action::SharedPtr FormKFDBActionRosMsg(int actionId, unsigned long int id, std::vector<unsigned long int> vpLoopCandId, std::vector<unsigned long int> vpMergeCandId, int n)
      {
        kf_db_action::SharedPtr kfdbActionMsg = std::make_shared<kf_db_action>();
        
        kfdbActionMsg->add_kf_id = -1;
        kfdbActionMsg->erase_kf_id = -1;
        kfdbActionMsg->clear_map_id = -1;
        if(actionId==5) {
          kfdbActionMsg->detect_best_candidates = true;
          kfdbActionMsg->detect_best_candidates_kf_id = id;
          kfdbActionMsg->detect_best_candidates_loop_cand_ids = vpLoopCandId;
          kfdbActionMsg->detect_best_candidates_merge_cand_ids = vpMergeCandId;
          kfdbActionMsg->detect_best_candidates_n_min_words = n;
        } 

        if(actionId==6) {
          kfdbActionMsg->detect_n_best_candidates = true;
          kfdbActionMsg->detect_n_best_candidates_kf_id = id;
          kfdbActionMsg->detect_n_best_candidates_loop_cand_ids = vpLoopCandId;
          kfdbActionMsg->detect_n_best_candidates_merge_cand_ids = vpMergeCandId;
          kfdbActionMsg->detect_n_best_candidates_n_num_cands = n;
        } 
      
        return kfdbActionMsg;
      }
      

      static int parseKFDBAction(kf_db_action::SharedPtr rKfdb, std::map<long unsigned int, ORB_SLAM3::Map*> mpOrbMaps, std::map<long unsigned int, ORB_SLAM3::KeyFrame*> mpOrbKeyFrames, std::map<long unsigned int, ORB_SLAM3::MapPoint*> mpOrbMapPoints) {

        if (rKfdb->add_kf_id > 0) {
          if (mpOrbKeyFrames.find(rKfdb->add_kf_id) != mpOrbKeyFrames.end()) return 0;
        }

        if (rKfdb->erase_kf_id > 0) {
          if (mpOrbKeyFrames.find(rKfdb->erase_kf_id) != mpOrbKeyFrames.end()) return 1;
        }

        if (rKfdb->clear) return 2;
        
        if (rKfdb->clear_map_id > 0) {
          if (mpOrbMaps.find(rKfdb->clear_map_id) != mpOrbMaps.end()) return 3;
        }

        //if (rKfdb->detect_candidates) return 4;
        //if (rKfdb->detect_best_candidates) return 5;
        //if (rKfdb->detect_n_best_candidates) return 6;
          
        return -1;
      }










      static map_point_actions::SharedPtr FormDefaultMapPointAction()
      {
        map_point_actions::SharedPtr mpActionMsg = std::make_shared<map_point_actions>();;
          
        mpActionMsg->add_observation_kf_id = -1;
        mpActionMsg->add_observation_vector_idx = -1;
        mpActionMsg->erase_observation_kf_id = -1;
        mpActionMsg->replace_id = -1;
        mpActionMsg->update_map_id = -1;
      
        return mpActionMsg;
      }
  
      
      static map_point_actions::SharedPtr FormMapPointActionRosMsg(int actionId, bool boolAction)
      {
        map_point_actions::SharedPtr mpActionMsg = FormDefaultMapPointAction();
        
        if(actionId == 3) mpActionMsg->set_bad_flag = true;
        if(actionId == 7) mpActionMsg->compute_distinctive_descriptors = true;
        if(actionId == 8) mpActionMsg->update_normal_and_depth = true;


        return mpActionMsg;
      }


      static map_point_actions::SharedPtr FormMapPointActionRosMsg(int actionId, unsigned long int id, int idx)
      {
        map_point_actions::SharedPtr mpActionMsg = FormDefaultMapPointAction();
        
        mpActionMsg->add_observation_kf_id = id;
        mpActionMsg->add_observation_vector_idx = idx;

        return mpActionMsg;
      }
      
      static map_point_actions::SharedPtr FormMapPointActionRosMsg(int actionId, unsigned long int id)
      {
        map_point_actions::SharedPtr mpActionMsg = FormDefaultMapPointAction();
        

        if(actionId == 2) mpActionMsg->erase_observation_kf_id = id;
        if(actionId == 4) mpActionMsg->replace_id = id;
        if(actionId == 10) mpActionMsg->update_map_id = id;
        return mpActionMsg;
      }

      static map_point_actions::SharedPtr FormMapPointActionRosMsg(int actionId, int n)
      {
        map_point_actions::SharedPtr mpActionMsg = FormDefaultMapPointAction();
        

        if(actionId == 5) 
        {
          mpActionMsg->increase_visibility_bool = true;
          mpActionMsg->increase_visibility_n = n;
        } 
        
        if(actionId == 6) 
        {
          mpActionMsg->increase_found_bool = true;
          mpActionMsg->increase_found_n = n;
        } 
        return mpActionMsg;
      }


      static map_point_actions::SharedPtr FormMapPointActionRosMsg(int actionId, Eigen::Vector3f vec)
      {
        map_point_actions::SharedPtr mpActionMsg = FormDefaultMapPointAction();
        

        if(actionId == 0) 
        {
          mpActionMsg->set_pose_bool = true;
          mpActionMsg->set_pose = Converter::CppToRos::EigenVector3fToVector3(vec);
        } 
        
        if(actionId == 9) 
        {
          mpActionMsg->set_normal_vector_bool = true;
          mpActionMsg->set_normal_vector = Converter::CppToRos::EigenVector3fToVector3(vec);
        } 
        return mpActionMsg;
      }


      static int parseMapPointAction(map_point_actions::SharedPtr rMpA, std::map<long unsigned int, ORB_SLAM3::Map*> mpOrbMaps, std::map<long unsigned int, ORB_SLAM3::KeyFrame*> mpOrbKeyFrames, std::map<long unsigned int, ORB_SLAM3::MapPoint*> mpOrbMapPoints) {

        if(mpOrbMapPoints.find(rMpA->mp_id) != mpOrbMapPoints.end()) {
          if(rMpA->set_pose_bool) return 0;
          
          if(rMpA->add_observation_kf_id > -1) 
          {
            if (mpOrbKeyFrames.find(rMpA->add_observation_kf_id) != mpOrbKeyFrames.end()) return 1;
          }

          if(rMpA->erase_observation_kf_id > -1)
          {
            if (mpOrbKeyFrames.find(rMpA->erase_observation_kf_id) != mpOrbKeyFrames.end()) return 2;
          }
          
          if(rMpA->set_bad_flag) return 3;
          if(rMpA->replace_id > -1)
          {
            if (mpOrbMapPoints.find(rMpA->replace_id) != mpOrbMapPoints.end()) return 4;
          }
          
          if(rMpA->increase_visibility_bool) return 5;
          if(rMpA->increase_found_bool) return 6;
          if(rMpA->compute_distinctive_descriptors) return 7;
          if(rMpA->update_normal_and_depth) {
            if(mpOrbMapPoints[rMpA->mp_id]->GetReferenceKeyFrame() != nullptr) {
              return 8;
            }
          }
          if(rMpA->set_normal_vector_bool) return 9;

          if(rMpA->update_map_id > -1)
          {
            if (mpOrbMaps.find(rMpA->update_map_id) != mpOrbMaps.end()) return 10;
          }
        }

        return -1;
      }

  };
};

#endif
