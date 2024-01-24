
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
        //std::mutex mMutexNewMP;
        //std::lock_guard<std::mutex> lock(mMutexNewMP);
        
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
        

        //std::cout << " #################### num of map points "<< opM->GetAllMapPoints().size() << std::endl;
        for(ORB_SLAM3::MapPoint* mp : opM->GetAllMapPoints())
        {
          rM.msp_map_points.push_back(Converter::MapPointConverter::ORBSLAM3MapPointToROS(mp));
        }
        
        //std::cout << "before all keyframes" << std::endl;
        if(opM->GetAllKeyFrames().size() > 0) {
          for(ORB_SLAM3::KeyFrame* kf : opM->GetAllKeyFrames())
          {
            //if(kf == nullptr) std::cout << "kf is null" << std::endl;
            //std::cout << "processing kf=" << kf->mnId << std::endl;
            rM.msp_keyframes.push_back(Converter::KeyFrameConverter::ORBSLAM3KeyFrameToROS(kf));
          }
        }
        //MapPoint[] msp_map_points                 // std::set<MapPoint*> mspMapPoints;
        //KeyFrame[] msp_keyframes                  // std::set<KeyFrame*> mspKeyFrames;

        //std::cout << "before backups" << std::endl;
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



        //std::cout << "before bools" << std::endl;
        rM.m_is_in_use = opM->IsInUse();                         // bool mIsInUse;
        rM.m_has_thumbnail = opM->HasThumbnail();                      // bool mHasTumbnail;
        rM.m_bad = opM->GetIsBad();                                // bool mbBad = false;

        rM.mb_is_inertial = opM->IsInertial();                       // bool mbIsInertial;
        rM.mb_imu_ba1 = opM->GetIniertialBA1();                           // bool mbIMU_BA1;
        rM.mb_imu_ba2 = opM->GetIniertialBA2();                           // bool mbIMU_BA2;


        std::cout << "Map." << opM->GetId() << " Before ROS Broadcast - #MP=" << opM->GetAllMapPoints().size() <<", #KF=" << opM->GetAllKeyFrames().size() << ", #RefMPs" << opM->GetReferenceMapPoints().size() << ", initKF=" << opM->GetInitKFid() << ", MaxKF=" << opM->GetMaxKFid() << ", OriginKFid" << opM->GetOriginKF()->mnId << std::endl;


        return rM;
      }
      
      static orb_map* RosMapToOrbMap(map::SharedPtr rM) {        
        

        bool mbFail = rM->mb_fail;
        std::vector<unsigned long int> mvOptKFs = rM->ms_opt_kfs;
        std::set<unsigned long int> msOptKFs(mvOptKFs.begin(), mvOptKFs.end());
        std::vector<unsigned long int> mvFixedKFs = rM->ms_fixed_kfs;
        std::set<unsigned long int> msFixedKFs(mvFixedKFs.begin(), mvFixedKFs.end());
        long unsigned int mnId = rM->mn_id;
        std::vector<std::string> mvpBackupMapPointsId = rM->mvp_backup_map_points_ids;
        std::vector<unsigned long int> mvpBackupKeyFramesId = rM->mvp_backup_keyframes_ids;
        unsigned long int mnBackupKFinitialID = rM->mn_backup_kf_initial_id;
        unsigned long int mnBackupKFlowerID = rM->mn_backup_kf_lower_id;
        std::vector<std::string> mvpBackupReferenceMapPointsId = rM->mvp_reference_map_points_id;
        bool mbImuInitialized = rM->mb_imu_initialized;
        int mnMapChange = rM->mn_map_change;
        int mnMapChangeNotified = rM->mn_map_change_notified;
        long unsigned int mnInitKFid = rM->mn_init_kf_id;
        long unsigned int mnMaxKFid = rM->mn_max_kf_id;
        int mnBigChangeIdx = rM->mn_big_change_idx;
        bool mIsInUse = rM->m_is_in_use;
        bool mHasTumbnail = rM->m_has_thumbnail;
        bool mbBad = rM->m_bad;
        bool mbIsInertial = rM->mb_is_inertial;
        bool mbIMU_BA1 = rM->mb_imu_ba1;
        bool mbIMU_BA2 = rM->mb_imu_ba2;

        orb_map* pM = new orb_map(mbFail, msOptKFs, msFixedKFs, mnId, mvpBackupMapPointsId, mvpBackupKeyFramesId, mnBackupKFinitialID, mnBackupKFlowerID, mvpBackupReferenceMapPointsId, mbImuInitialized, mnMapChange, mnMapChangeNotified, mnInitKFid, mnMaxKFid, mnBigChangeIdx, mIsInUse, mHasTumbnail, mbBad, mbIsInertial, mbIMU_BA1, mbIMU_BA2);

        //std::cout << "Map." << pM->GetId() << " After update - #MP=" << pM->GetAllMapPoints().size() <<", #KF=" << pM->GetAllKeyFrames().size() << ", #RefMPs" << pM->GetReferenceMapPoints().size() << ", initKF=" << pM->GetInitKFid() << ", MaxKF=" << pM->GetMaxKFid() << ", OriginKFid" << pM->GetOriginKF()->mnId << std::endl;

        return pM;

    } 

  };

};


#endif
