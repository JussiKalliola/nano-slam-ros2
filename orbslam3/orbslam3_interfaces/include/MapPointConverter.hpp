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
        map_point msgMp;
        
        // public
        // std::cout << "MapPoint public" << std::endl;
        msgMp.mn_id = pMp->mnId;
        msgMp.n_next_id = pMp->nNextId;
        msgMp.mn_first_kf_id = pMp->mnFirstKFid;
        msgMp.mn_first_frame = pMp->mnFirstFrame;
        msgMp.n_obs = pMp-> nObs;
        
        // std::cout << "MapPoint vars used by the trackin" << std::endl;
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
        
        // std::cout << "MapPoint vars used by local mapping" << std::endl;
        // Variables used by local mapping
        msgMp.mn_ba_local_for_kf = pMp->mnBALocalForKF;
        msgMp.mn_fuse_candidate_for_kf = pMp->mnFuseCandidateForKF;
        
        // std::cout << "MapPoint vars used by loop closing" << std::endl;
        // Variables used by loop closing
        msgMp.mn_loop_point_for_kf = pMp->mnLoopPointForKF;
        msgMp.mn_corrected_by_kf = pMp->mnCorrectedByKF;
        msgMp.mn_corrected_referece = pMp->mnCorrectedReference;
        msgMp.m_pos_gba = CppToRos::EigenVector3fToVector3(pMp->mPosGBA);
        msgMp.mn_ba_global_for_kf = pMp->mnBAGlobalForKF;
        msgMp.mn_ba_local_for_merge = pMp->mnBALocalForMerge;
        
        // std::cout << "MapPoint vars used by merging" << std::endl;
        // Variables used by merging
        msgMp.m_pos_merge = CppToRos::EigenVector3fToVector3(pMp->mPosMerge);
        msgMp.m_normal_vector_merge = CppToRos::EigenVector3fToVector3(pMp->mNormalVectorMerge);
        
        // std::cout << "MapPoint vars used by inverse depht optimizatino" << std::endl;
        // For inverse depth optimization
        msgMp.m_inv_depth = pMp->mInvDepth;
        msgMp.m_init_u = pMp->mInitU;
        msgMp.m_init_v = pMp->mInitV;
        //KeyFrame mp_host_kf           // This one we have as mp_host_kf_id
        
        orb_keyframe* mpHostKf = pMp->mpHostKF;
        if (hostKfId == -1) {
          msgMp.mp_host_kf_id = (mpHostKf != nullptr) ? mpHostKf->mnId : -1;
        } else {
          msgMp.mp_host_kf_id = hostKfId;
        }

        // std::cout << "MapPoint origin id" << std::endl;
        msgMp.mn_origin_map_id = pMp->mnOriginMapId;
        
        // std::cout << "MapPoint protected" << std::endl;
        // Protected
        // Position in absolute coordinates
        msgMp.m_world_pos = CppToRos::EigenVector3fToVector3(pMp->GetWorldPos());
        
        // std::cout << "MapPoint observations" << std::endl;
        // Keyframes observing the point and associated index in keyframe
        msgMp.m_observations = CppToRos::MapToRosKeyValuePairVector(pMp->GetObservations()); // std::map<KeyFrame*, std::tuple<int,int>> m_observations # check how to convert to ros2
        
        // For save relation without pointer, this is necessary for save/load function
        msgMp.m_backup_observations_id1 = CppToRos::MapToRosIntTupleVector(pMp->GetObservationsBackup1()); // std::map<long unsignet int, int> m_backup_observations_id1 # maybe do another msg type with uint64, int and make this as array
        msgMp.m_backup_observations_id2 = CppToRos::MapToRosIntTupleVector(pMp->GetObservationsBackup2()); // std::map<long unsigned int, int> m_backup_observations_id2
                                      // Both of these we have as m_observations

        // Mean viewing direction
        msgMp.m_normal_vector = CppToRos::EigenVector3fToVector3(pMp->GetNormal());
        
        // Best Descriptor to fast matching
        msgMp.m_descriptor = CppToRos::CVMatToImage(pMp->GetDescriptor());
        
        // Refernce KeyFrame
        //KeyFrame mp_ref_kf          // This one we have as m_backup_ref_kf_id
        msgMp.m_backup_ref_kf_id = pMp->GetReferenceKeyFrame()->mnId;
        
        // std::cout << "MapPoint Tracking counters" << std::endl;
        // Tracking counters
        msgMp.mn_visible = pMp->GetVisible();
        msgMp.mn_found = pMp->GetFound();
       

        // std::cout << "MapPoint isbad?" << std::endl;
        // Bad flag (we do not currently erase MapPoint from memory)
        msgMp.mb_bad = pMp->isBad();
        //MapPoint mp_replaced        // This one we have as m_bakup_replaced_id
        // std::cout << "MapPoint Get Replaced?" << std::endl;
        // For save relation without pointer, this is necessary for save/load function
        orb_map_point* pKfr = pMp->GetReplaced();
        msgMp.m_backup_replaced_id = (pKfr != nullptr) ? pKfr->mnId : -1; 
        

        // Scale invariance distances
        msgMp.mf_min_distance = pMp->GetMinDistanceInvariance();
        msgMp.mf_max_distance = pMp->GetMaxDistanceInvariance();
        
        // std::cout << "MapPoint end" << std::endl;
        //Map mp_map                  // This one we have as mp_map_id 
        orb_map* pM = pMp->GetMap();
        msgMp.mp_map_id = (pM != nullptr) ? pM->GetId() : -1; 

        return msgMp;
      } 
  };

};


#endif
