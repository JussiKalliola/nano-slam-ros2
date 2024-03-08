#ifndef MAPPOINT_CONVERTER_HPP_
#define MAPPOINT_CONVERTER_HPP

#include "orbslam3_interfaces/msg/map_point.hpp"

#include "MapPoint.h"
#include "Map.h"
#include "KeyFrame.h"

#include "Converter.hpp"


namespace Converter {

  class MapPointConverter {
    using map_point = orbslam3_interfaces::msg::MapPoint;

    using orb_map_point = ORB_SLAM3::MapPoint;
    using orb_map = ORB_SLAM3::Map;
    using orb_keyframe = ORB_SLAM3::KeyFrame;

    public:
      static map_point ORBSLAM3MapPointToROS(orb_map_point* pMp, long unsigned int hostKfId=-1) {
        //std::mutex mMutex;
        //std::lock_guard<std::mutex> lock(mMutex);
        
        map_point msgMp = FormDefaultMapPointMessage();
        
        // public
        //std::cout << "MapPoint public" << std::endl;
        msgMp.mn_id = pMp->mnId;
        msgMp.m_str_hex_id = pMp->mstrHexId;

        msgMp.n_next_id = pMp->nNextId;
        msgMp.mn_first_kf_id = pMp->mnFirstKFid;
        msgMp.mn_first_frame = pMp->mnFirstFrame;
        msgMp.n_obs = pMp-> nObs;
        
        //std::cout << "MapPoint vars used by the trackin" << std::endl;
        // Variables used by the tracking
        msgMp.m_track_proj_x = pMp->mTrackProjX;
        msgMp.m_track_proj_y = pMp->mTrackProjY;
        msgMp.m_track_depth = pMp->mTrackDepth;
        msgMp.m_track_depth_r = pMp->mTrackDepthR;
        msgMp.m_track_proj_xr = pMp->mTrackProjXR;
        msgMp.m_track_proj_yr = pMp->mTrackProjYR;
        msgMp.mb_track_in_view = pMp->mbTrackInView;
        msgMp.mb_track_in_view_r = pMp->mbTrackInViewR;
        msgMp.mn_track_scale_level = pMp->mnTrackScaleLevel;
        msgMp.mn_track_scale_level_r = pMp->mnTrackScaleLevelR;
        msgMp.m_track_view_cos = pMp->mTrackViewCos;
        msgMp.m_track_view_cos_r = pMp->mTrackViewCosR;
        msgMp.mn_track_reference_for_frame = pMp->mnTrackReferenceForFrame;
        msgMp.mn_last_frame_seen = pMp->mnLastFrameSeen;
        
        //std::cout << "MapPoint vars used by local mapping" << std::endl;
        // Variables used by local mapping
        msgMp.mn_ba_local_for_kf = pMp->mnBALocalForKF;
        msgMp.mn_fuse_candidate_for_kf = pMp->mnFuseCandidateForKF;
        
        //std::cout << "MapPoint vars used by loop closing" << std::endl;
        // Variables used by loop closing
        msgMp.mn_loop_point_for_kf = pMp->mnLoopPointForKF;
        msgMp.mn_corrected_by_kf = pMp->mnCorrectedByKF;
        msgMp.mn_corrected_referece = pMp->mnCorrectedReference;
        msgMp.m_pos_gba = CppToRos::EigenVector3fToVector3(pMp->mPosGBA);
        msgMp.mn_ba_global_for_kf = pMp->mnBAGlobalForKF;
        msgMp.mn_ba_local_for_merge = pMp->mnBALocalForMerge;
        
        //std::cout << "MapPoint vars used by merging" << std::endl;
        // Variables used by merging
        msgMp.m_pos_merge = CppToRos::EigenVector3fToVector3(pMp->mPosMerge);
        msgMp.m_normal_vector_merge = CppToRos::EigenVector3fToVector3(pMp->mNormalVectorMerge);
        // std::cout << "MapPoint vars used by inverse depht optimizatino" << std::endl;
        // For inverse depth optimization
        msgMp.m_inv_depth = pMp->mInvDepth;
        msgMp.m_init_u = pMp->mInitU;
        msgMp.m_init_v = pMp->mInitV;
        //KeyFrame mp_host_kf           // This one we have as mp_host_kf_id
        orb_keyframe* mpHostKF = pMp->mpHostKF; 
        msgMp.mp_host_kf_id = (mpHostKF) ? mpHostKF->mnId : -1; // static_cast<long long int>(pMp->mBackupHostKFId); //(pMp->mpHostKF != nullptr) ? pMp->mpHostKF->mnId : -1;
        //std::cout << "MapPoint origin id" << std::endl;
        msgMp.mn_origin_map_id = pMp->mnOriginMapId;
        
        // std::cout << "MapPoint protected" << std::endl;
        // Protected
        // Position in absolute coordinates
        msgMp.m_world_pos = CppToRos::EigenVector3fToVector3(pMp->GetWorldPos());
        
        // std::cout << "MapPoint observations" << std::endl;
        // Keyframes observing the point and associated index in keyframe
        msgMp.m_observations = CppToRos::MapToRosKeyValuePairVector(pMp->GetObservations()); // std::map<KeyFrame*, std::tuple<int,int>> m_observations # check how to convert to ros2
        
        // For save relation without pointer, this is necessary for save/load function
        //std::cout << pMp->GetObservationsBackup1().size() << ", " << pMp->GetObservationsBackup2().size() << std::endl;
        msgMp.m_backup_observations_id1 = CppToRos::MapToRosIntTupleVector(pMp->GetObservationsBackup1()); // std::map<long unsignet int, int> m_backup_observations_id1 # maybe do another msg type with uint64, int and make this as array
        msgMp.m_backup_observations_id2 = CppToRos::MapToRosIntTupleVector(pMp->GetObservationsBackup2()); // std::map<long unsigned int, int> m_backup_observations_id2
                                      // Both of these we have as m_observations
        // Mean viewing direction
        msgMp.m_normal_vector = CppToRos::EigenVector3fToVector3(pMp->GetNormal());
        
        // Best Descriptor to fast matching
        msgMp.m_descriptor = CppToRos::CVMatToImage(pMp->GetDescriptor());
        
        // Refernce KeyFrame
        //KeyFrame mp_ref_kf          // This one we have as m_backup_ref_kf_id
        orb_keyframe* mpRefKF = pMp->GetReferenceKeyFrame();
        msgMp.m_backup_ref_kf_id = (mpRefKF) ? mpRefKF->mnId : pMp->GetRefBackup(); //static_cast<long long int>(pMp->GetRefBackup()); //pMp->GetReferenceKeyFrame()->mnId;
        
        //std::cout << "MapPoint Tracking counters" << std::endl;
        // Tracking counters
        msgMp.mn_visible = pMp->GetVisible();
        msgMp.mn_found = pMp->GetFound();
       

        // std::cout << "MapPoint isbad?" << std::endl;
        // Bad flag (we do not currently erase MapPoint from memory)
        msgMp.mb_bad = pMp->isBad();
        //MapPoint mp_replaced        // This one we have as m_bakup_replaced_id
        //std::cout << "MapPoint Get Replaced?" << std::endl;
        // For save relation without pointer, this is necessary for save/load function
        //std::cout << "check if replaced" << std::endl;
        orb_map_point* mpReplaced = pMp->GetReplaced();
        msgMp.m_backup_replaced_id = (mpReplaced) ? mpReplaced->mstrHexId : ""; //static_cast<long long int>(pMp->GetReplacedBackup()); //(pKfr != nullptr) ? pKfr->mnId : -1; 
        

        //std::cout << "MapPoint distance invariance" << std::endl;
        // Scale invariance distances
        msgMp.mf_min_distance = pMp->GetMinDistance();
        msgMp.mf_max_distance = pMp->GetMaxDistance();
        
        //std::cout << "MapPoint end" << std::endl;
        //Map mp_map                  // This one we have as mp_map_id 
        orb_map* pM = pMp->GetMap();
        msgMp.mp_map_id = (pM != nullptr) ? pM->GetId() : -1; 

        //std::cout << "==========ORB>ROS=========== HOST KF ID=" << msgMp.mp_host_kf_id;
        //if(mpHostKF) std::cout << "," << mpHostKF->mnId;
        //std::cout << ", REF ID=" << msgMp.m_backup_ref_kf_id; 
        //if(mpRefKF) std::cout << "," << mpRefKF->mnId;
        //std::cout << ", REPLACED ID=" << msgMp.m_backup_replaced_id;
        //if(mpReplaced) std::cout << "," << mpReplaced->mnId;
        //std::cout << " =============================" << std::endl;
        return msgMp;
      } 

      static orb_map_point* RosMapPointToOrb(map_point::SharedPtr rMp, orb_map_point* mpExistingMP = static_cast<orb_map_point*>(NULL)) {
        
        std::mutex mMutexNewMP;
        std::lock_guard<std::mutex> lock(mMutexNewMP);
        
        //long long int mnId = (mbMPExists) ? rMp->mn_id : -1;
        std::string mstrHexId = rMp->m_str_hex_id;
        long int mnFirstKFid = rMp->mn_first_kf_id;
        long int mnFirstFrame = rMp->mn_first_frame;
        int nObs = rMp->n_obs;
        float mTrackProjX = rMp->m_track_proj_x;
        float mTrackProjY = rMp->m_track_proj_y;
        float mTrackDepth = rMp->m_track_depth;
        float mTrackDepthR = rMp->m_track_depth_r;
        float mTrackProjXR = rMp->m_track_proj_xr;
        float mTrackProjYR = rMp->m_track_proj_yr;
        bool mbTrackInView = rMp->mb_track_in_view;
        bool mbTrackInViewR = rMp->mb_track_in_view_r;
        int mnTrackScaleLevel = rMp->mn_track_scale_level;
        int mnTrackScaleLevelR = rMp->mn_track_scale_level_r;
        float mTrackViewCos = rMp->m_track_view_cos;
        float mTrackViewCosR = rMp->m_track_view_cos_r;
        long unsigned int mnTrackReferenceForFrame = rMp->mn_track_reference_for_frame;
        long unsigned int mnLastFrameSeen = rMp->mn_last_frame_seen;
        long unsigned int mnBALocalForKF = rMp->mn_ba_local_for_kf;
        long unsigned int mnFuseCandidateForKF = rMp->mn_fuse_candidate_for_kf;
        long unsigned int mnLoopPointForKF = rMp->mn_loop_point_for_kf;
        long unsigned int mnCorrectedByKF = rMp->mn_corrected_by_kf;
        long unsigned int mnCorrectedReference = rMp->mn_corrected_referece;
        Eigen::Vector3f mPosGBA = RosToCpp::Vector3ToEigenVector3f(rMp->m_pos_gba);
        long unsigned int mnBAGlobalForKF = rMp->mn_ba_global_for_kf;
        long unsigned int mnBALocalForMerge = rMp->mn_ba_local_for_merge;
        Eigen::Vector3f mPosMerge = RosToCpp::Vector3ToEigenVector3f(rMp->m_pos_merge);
        Eigen::Vector3f mNormalVectorMerge = RosToCpp::Vector3ToEigenVector3f(rMp->m_normal_vector_merge);
        double mInvDepth = rMp->m_inv_depth;
        double mInitU = rMp->m_init_u;
        double mInitV = rMp->m_init_v;
        
        //orb_keyframe* mpHostKF = nullptr;
        //if(mpOrbKeyFrames.find(rMp->mp_host_kf_id) != mpOrbKeyFrames.end()) {
        //  mpHostKF = mpOrbKeyFrames[rMp->mp_host_kf_id];
        //} else if(rMp->mp_host_kf_id > -1) {
        //  std::cout << "hostkf is unprocessed" << std::endl;
        //  *bUnprocessed = true;
        //}
        long long int mBackupHostKFId = rMp->mp_host_kf_id;
        
        unsigned int mnOriginMapId = rMp->mn_origin_map_id;
        Eigen::Vector3f mWorldPos = RosToCpp::Vector3ToEigenVector3f(rMp->m_world_pos);
        //std::map<orb_keyframe*,std::tuple<int,int> > mObservations = RosToCpp::KeyValuePairVectorToMap(rMp->m_observations, mpOrbKeyFrames);
        //std::cout << "BEFORE :: obs1=" << rMp->m_backup_observations_id1.size() << ", obs2=" << rMp->m_backup_observations_id2.size() << std::endl;
        std::map<long unsigned int, int> mBackupObservationsId1 = RosToCpp::IntTupleVectorToMap(rMp->m_backup_observations_id1);
        std::map<long unsigned int, int> mBackupObservationsId2 = RosToCpp::IntTupleVectorToMap(rMp->m_backup_observations_id2);

        //std::cout << std::endl;
        //std::cout << "AFTER :: obs1=" <<mBackupObservationsId1.size() << ", obs2=" << mBackupObservationsId2.size() << std::endl;
        Eigen::Vector3f mNormalVector = RosToCpp::Vector3ToEigenVector3f(rMp->m_normal_vector);
        cv::Mat mDescriptor = RosToCpp::ImageToCVMat(rMp->m_descriptor);
        
        //orb_keyframe* mpRefKF = nullptr;
        //if(mpOrbKeyFrames.find(rMp->m_backup_ref_kf_id) != mpOrbKeyFrames.end()) {
        //  mpRefKF = mpOrbKeyFrames[rMp->m_backup_ref_kf_id];
        //} else if(rMp->m_backup_ref_kf_id > -1) {
        //  std::cout << "refkf is unprocessed " << rMp->m_backup_ref_kf_id << std::endl;
        //  *bUnprocessed = true;
        //}

        long long int mBackupRefKFId = rMp->m_backup_ref_kf_id;
        int mnVisible = rMp->mn_visible;
        int mnFound = rMp->mn_found;
        bool mbBad = rMp->mb_bad;
        
        //orb_map_point* mpReplaced = nullptr;
        //if(mpOrbMapPoints.find(rMp->m_backup_replaced_id) != mpOrbMapPoints.end()) {
        //  mpReplaced = mpOrbMapPoints[rMp->m_backup_replaced_id];
        //} else if(rMp->m_backup_replaced_id > -1) {
        //  std::cout << "replacedmp is unprocessed" << std::endl;
        //  *bUnprocessed = true;
        //}
        
        std::string mBackupReplacedId = rMp->m_backup_replaced_id;
        float mfMinDistance = rMp->mf_min_distance;
        float mfMaxDistance = rMp->mf_max_distance;
        
        //ORB_SLAM3::Map* mpMap = nullptr;
        //if(mpOrbMaps.find(rMp->mp_map_id) != mpOrbMaps.end()) {
        //  mpMap = mpOrbMaps[rMp->mp_map_id];
        //} else if(rMp->mp_map_id > -1) {
        //  std::cout << "map is unprocessed" << std::endl;
        //  *bUnprocessed = true;
        //}
        
        //std::cout << "===================== HOST KF ID=" << mBackupHostKFId << ", REF ID=" << mBackupRefKFId << ", REPLACED ID=" << mBackupReplacedId << " =============================" << std::endl;
        if(mpExistingMP)
        {
          mpExistingMP->UpdateMapPoint(mnFirstKFid, mnFirstFrame, nObs, mTrackProjX, mTrackProjY, mTrackDepth, mTrackDepthR, mTrackProjXR, mTrackProjYR, mbTrackInView, mbTrackInViewR, mnTrackScaleLevel, mnTrackScaleLevelR, mTrackViewCos, mTrackViewCosR, mnTrackReferenceForFrame, mnLastFrameSeen, mnBALocalForKF, mnFuseCandidateForKF, mnLoopPointForKF, mnCorrectedByKF, mnCorrectedReference, mPosGBA, mnBAGlobalForKF, mnBALocalForMerge, mPosMerge, mNormalVectorMerge, mInvDepth, mInitU, mInitV, /*mpHostKF,*/ mBackupHostKFId, mnOriginMapId, mWorldPos, /*mObservations,*/ mBackupObservationsId1, mBackupObservationsId2, mNormalVector, mDescriptor, /*mpRefKF,*/ mBackupRefKFId, mnVisible, mnFound, mbBad, /*mpReplaced,*/ mBackupReplacedId, mfMinDistance, mfMaxDistance /*mpMap*/);
        return mpExistingMP;
        } else
          return new orb_map_point(-1, mstrHexId, mnFirstKFid, mnFirstFrame, nObs, mTrackProjX, mTrackProjY, mTrackDepth, mTrackDepthR, mTrackProjXR, mTrackProjYR, mbTrackInView, mbTrackInViewR, mnTrackScaleLevel, mnTrackScaleLevelR, mTrackViewCos, mTrackViewCosR, mnTrackReferenceForFrame, mnLastFrameSeen, mnBALocalForKF, mnFuseCandidateForKF, mnLoopPointForKF, mnCorrectedByKF, mnCorrectedReference, mPosGBA, mnBAGlobalForKF, mnBALocalForMerge, mPosMerge, mNormalVectorMerge, mInvDepth, mInitU, mInitV, /*mpHostKF,*/ mBackupHostKFId, mnOriginMapId, mWorldPos, /*mObservations,*/ mBackupObservationsId1, mBackupObservationsId2, mNormalVector, mDescriptor, /*mpRefKF,*/ mBackupRefKFId, mnVisible, mnFound, mbBad, /*mpReplaced,*/ mBackupReplacedId, mfMinDistance, mfMaxDistance /*mpMap*/);
      }

      static map_point FormDefaultMapPointMessage()
      {
        map_point msgMp;

        msgMp.mp_host_kf_id = -1;
        msgMp.m_backup_ref_kf_id = -1; 
        msgMp.m_backup_replaced_id= "";       
        msgMp.mp_map_id = -1;

        return msgMp;

      }
      
      static void FillMapPointData(orb_map_point* mopMp, map_point::SharedPtr mrpMp, std::map<long unsigned int, orb_keyframe*> mpOrbKeyFrames, std::map<long unsigned int, orb_map*> mpOrbMaps, std::map<std::string, orb_map_point*> mpOrbMapPoints, bool* bUnprocessed) 
      {

        if(mpOrbKeyFrames.find(mrpMp->mp_host_kf_id) != mpOrbKeyFrames.end()) {
          mopMp->mpHostKF = mpOrbKeyFrames[mrpMp->mp_host_kf_id];
        } else if(mrpMp->mp_host_kf_id > -1) {
          *bUnprocessed = true;
        }
         
        if(mpOrbKeyFrames.find(mrpMp->m_backup_ref_kf_id) != mpOrbKeyFrames.end()) {
          mopMp->SetMpRefKF(mpOrbKeyFrames[mrpMp->m_backup_ref_kf_id]);
        } else if(mrpMp->m_backup_ref_kf_id > -1) {
          *bUnprocessed = true;
        }

        
        if(mpOrbMapPoints.find(mrpMp->m_backup_replaced_id) != mpOrbMapPoints.end()) {
          mopMp->Replace(mpOrbMapPoints[mrpMp->m_backup_replaced_id]);
        } else if(mrpMp->m_backup_replaced_id.length() == 6) {
          *bUnprocessed = true;
        }
        
        if(mpOrbMaps.find(mrpMp->mp_map_id) != mpOrbMaps.end()) {
          mopMp->UpdateMap(mpOrbMaps[mrpMp->mp_map_id]);
        } else if(mrpMp->mp_map_id > -1) {
          *bUnprocessed = true;
        }
      }

    private:

  };

};


#endif
