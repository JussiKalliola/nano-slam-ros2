
#ifndef MAP_CONVERTER_HPP_
#define MAP_CONVERTER_HPP

#include "orbslam3_interfaces/msg/map_point.hpp"
#include "orbslam3_interfaces/msg/map.hpp"
#include "orbslam3_interfaces/msg/map_actions.hpp"

#include "MapPoint.h"
#include "Map.h"
#include "KeyFrame.h"

#include "Converter.hpp"


namespace Converter {

  class MapConverter {
    using map_point = orbslam3_interfaces::msg::MapPoint;
    using map = orbslam3_interfaces::msg::Map;

    using orb_map_point = ORB_SLAM3::MapPoint;
    using orb_map = ORB_SLAM3::Map;
    using orb_keyframe = ORB_SLAM3::KeyFrame;

    public:
      static map OrbMapToRosMap(orb_map* opM) {
        std::mutex mMutexNewMP;
        std::lock_guard<std::mutex> lock(mMutexNewMP);
        
        map rM;
        //KeyFrame[] mvp_keyframe_origins           // vector<KeyFrame*> mvpKeyFrameOrigins;
        rM.mv_backup_keyframe_origins_id = opM->mvBackupKeyFrameOriginsId;    // vector<unsigned long int> mvBackupKeyFrameOriginsId;
        if(opM->mpFirstRegionKF != nullptr) rM.mp_first_region_kf_id = opM->mpFirstRegionKF->mnId;               // KeyFrame* mpFirstRegionKF;
                                                  // std::mutex mMutexMapUpdate;
        rM.mb_fail = opM->mbFail;                              // bool mbFail;

        // Size of the thumbnail (always in power of 2)
        rM.thumb_width = opM->THUMB_WIDTH;                         // static const int THUMB_WIDTH = 512;
        rM.thumb_height = opM->THUMB_HEIGHT;                        // static const int THUMB_HEIGHT = 512;

        rM.n_next_id = opM->nNextId;                          // static long unsigned int nNextId;

        // DEBUG: show KFs which are used in LBA
        rM.ms_opt_kfs = opM->GetOptKFs();                       // std::set<long unsigned int> msOptKFs;
        rM.ms_fixed_kfs = opM->GetFixedKFs();                     // std::set<long unsigned int> msFixedKFs;

        // protected:

        rM.mn_id = opM->GetId();                              // long unsigned int mnId;
        

        std::cout << " #################### num of map points "<< opM->GetAllMapPoints().size() << std::endl;
        for(ORB_SLAM3::MapPoint* mp : opM->GetAllMapPoints())
        {
          rM.msp_map_points.push_back(Converter::MapPointConverter::ORBSLAM3MapPointToROS(mp));
        }

        if(opM->GetAllKeyFrames().size() > 0) {
          for(ORB_SLAM3::KeyFrame* kf : opM->GetAllKeyFrames())
          {
            rM.msp_keyframes.push_back(Converter::KeyFrameConverter::ORBSLAM3KeyFrameToROS(kf));
          }
        }
        //MapPoint[] msp_map_points                 // std::set<MapPoint*> mspMapPoints;
        //KeyFrame[] msp_keyframes                  // std::set<KeyFrame*> mspKeyFrames;

        // Save/load, the set structure is broken in libboost 1.58 for ubuntu 16.04, a vector is serializated
        rM.mvp_backup_map_points_ids = opM->GetBackupMapPointsId();          // std::vector<MapPoint*> mvpBackupMapPoints;
        rM.mvp_backup_keyframes_ids = opM->GetBackupKeyFrames();           // std::vector<KeyFrame*> mvpBackupKeyFrames;

        //KeyFrame mp_kf_initial                    // KeyFrame* mpKFinitial;
        //KeyFrame mp_kf_lower_id                   // KeyFrame* mpKFlowerID;
        rM.mn_backup_kf_initial_id = opM->GetInitKFid();           // unsigned long int mnBackupKFinitialID;
        rM.mn_backup_kf_lower_id = opM->GetBackupKFLowerID();               // unsigned long int mnBackupKFlowerID;

        rM.mvp_reference_map_points_id = opM->GetBackupReferenceMapPointsId();      // std::vector<MapPoint*> mvpReferenceMapPoints;

        rM.mb_imu_initialized = opM->GetImuInitialized();                   // bool mbImuInitialized;

        rM.mn_map_change  = opM->GetMapChange();                      // int mnMapChange;
        rM.mn_map_change_notified = opM->GetMapChangeNotified();               // int mnMapChangeNotified;
         
        rM.mn_init_kf_id = opM->GetInitKFid();                    // long unsigned int mnInitKFid;
        rM.mn_max_kf_id = opM->GetMaxKFid();                      // long unsigned int mnMaxKFid;
        // long unsigned int mnLastLoopKFid;

        // Index related to a big change in the map (loop closure, global BA)
        rM.mn_big_change_idx = opM->GetBigChangeIdx();                   // int mnBigChangeIdx;



        rM.m_is_in_use = opM->IsInUse();                         // bool mIsInUse;
        rM.m_has_thumbnail = opM->HasThumbnail();                      // bool mHasTumbnail;
        rM.m_bad = opM->GetIsBad();                                // bool mbBad = false;

        rM.mb_is_inertial = opM->IsInertial();                       // bool mbIsInertial;
        rM.mb_imu_ba1 = opM->GetIniertialBA1();                           // bool mbIMU_BA1;
        rM.mb_imu_ba2 = opM->GetIniertialBA2();                           // bool mbIMU_BA2;


        std::cout << "Map." << opM->GetId() << " Before ROS Broadcast - #MP=" << opM->GetAllMapPoints().size() <<", #KF=" << opM->GetAllKeyFrames().size() << ", #RefMPs" << opM->GetReferenceMapPoints().size() << ", initKF=" << opM->GetInitKFid() << ", MaxKF=" << opM->GetMaxKFid() << ", OriginKFid" << opM->GetOriginKF()->mnId << std::endl;


        return rM;
      }
      
      static orb_map* RosMapToOrbMap(map::SharedPtr rM, std::map<long unsigned int, orb_keyframe*> mpOrbKeyFrames, std::map<long unsigned int, orb_map_point*> mpOrbMapPoints, std::map<long unsigned int, orb_map*> mpOrbMaps, bool* bUnprocessed) {        
      orb_map* opM = new orb_map();
      std::cout << std::endl; 
      if(mpOrbMaps.find(rM->mn_id) != mpOrbMaps.end())
      {
        orb_map* pM = mpOrbMaps[rM->mn_id];

        std::set<orb_map_point*> mspMapPoints;
        for(map_point mp : rM->msp_map_points)
        {
          if(mpOrbMapPoints.find(mp.mn_id) != mpOrbMapPoints.end())
          {
            mspMapPoints.insert(mpOrbMapPoints[mp.mn_id]);
          } else {
            std::cout << "MapPoint not found " << mp.mn_id << std::endl;
          }
          //bool bUnprocessed = false;
          //mspMapPoints.insert(Converter::MapPointConverter::RosMapPointToOrb(mp, mpOrbKeyFrames, mpOrbMapPoints, mpOrbMaps, &bUnprocessed));
        }
        std::cout << "Number of map points in update " << mspMapPoints.size() << ", #msg " << rM->msp_map_points.size() << ", total=" << mpOrbMapPoints.size() << std::endl; 
        pM->UpdateMapPoints(mspMapPoints);

        std::set<orb_keyframe*> mspKeyFrames;

        for(keyframe kf : rM->msp_keyframes)
        {
          if(mpOrbKeyFrames.find(kf.mn_id) != mpOrbKeyFrames.end())
          {
            mspKeyFrames.insert(mpOrbKeyFrames[kf.mn_id]);
          } else {
            std::cout << "KeyFrame not found " << kf.mn_id << std::endl;
          }
          //bool bUnprocessed = false;
          //mspKeyFrames.insert(Converter::KeyFrameConverter::ROSKeyFrameToORBSLAM3(kf, mpOrbKeyFrames, mpOrbMapPoints, mpOrbMaps, &bUnprocessed));
        }

        std::cout << "Number of keyframes in update " << mspKeyFrames.size() << std::endl; 
        pM->UpdateKeyFrames(mspKeyFrames);


        orb_keyframe* mpKFinitial = nullptr;
        if(rM->mn_backup_kf_initial_id != -1 && mpOrbKeyFrames.find(rM->mn_backup_kf_initial_id) != mpOrbKeyFrames.end())
        {
          mpKFinitial = mpOrbKeyFrames[rM->mn_backup_kf_initial_id];
        } else {
          std::cout << "KeyFrame initial not found " << rM->mn_backup_kf_initial_id << std::endl;
        }
      
        pM->UpdateInitialKF(mpKFinitial);
        


        orb_keyframe* mpKFlowerID = nullptr;
        if(rM->mn_backup_kf_lower_id != -1 && mpOrbKeyFrames.find(rM->mn_backup_kf_lower_id) != mpOrbKeyFrames.end())
        {
          mpKFlowerID = mpOrbKeyFrames[rM->mn_backup_kf_lower_id];
        } else {
          std::cout << "KeyFrame lower not found " << rM->mn_backup_kf_lower_id << std::endl;
        }


        pM->UpdateLowerKF(mpKFlowerID);


        std::vector<orb_map_point*> mvpReferenceMapPoints;
        for(unsigned long int id : rM->mvp_reference_map_points_id)
        {
          if(mpOrbMapPoints.find(id) != mpOrbMapPoints.end())
          {
            mvpReferenceMapPoints.push_back(mpOrbMapPoints[id]);
          } else {
            std::cout << "MapPoint ref not found " << id << std::endl;
          }
          //bool bUnprocessed = false;
          //mspMapPoints.insert(Converter::MapPointConverter::RosMapPointToOrb(mp, mpOrbKeyFrames, mpOrbMapPoints, mpOrbMaps, &bUnprocessed));
        }


        std::cout << "Number of ref map points in update " << mvpReferenceMapPoints.size() << ", #msg " << rM->mvp_reference_map_points_id.size() << ", total=" << mpOrbMapPoints.size() << std::endl; 
        pM->UpdateReferenceMapPoints(mvpReferenceMapPoints);
        

        std::cout << "Map." << pM->GetId() << " After update - #MP=" << pM->GetAllMapPoints().size() <<", #KF=" << pM->GetAllKeyFrames().size() << ", #RefMPs" << pM->GetReferenceMapPoints().size() << ", initKF=" << pM->GetInitKFid() << ", MaxKF=" << pM->GetMaxKFid() << ", OriginKFid" << pM->GetOriginKF()->mnId << std::endl;
      }



      std::cout << std::endl; 
      return opM;

    } 

  };

};


#endif
