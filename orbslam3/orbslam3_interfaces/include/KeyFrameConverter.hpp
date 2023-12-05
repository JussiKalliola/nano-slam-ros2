#ifndef KEYFRAME_CONVERTER_HPP_
#define KEYFRAME_CONVERTER_HPP_

#include "orbslam3_interfaces/msg/key_frame.hpp"
#include "orbslam3_interfaces/msg/frame.hpp"
#include "orbslam3_interfaces/msg/atlas.hpp"
#include "orbslam3_interfaces/msg/key_frame_database.hpp"
#include "orbslam3_interfaces/msg/map.hpp"
#include "orbslam3_interfaces/msg/map_point.hpp"
#include "orbslam3_interfaces/msg/imu_bias.hpp"
#include "orbslam3_interfaces/msg/imu_preintegrated.hpp"
#include "orbslam3_interfaces/msg/matrix.hpp"
#include "orbslam3_interfaces/msg/int_tuple.hpp"
#include "orbslam3_interfaces/msg/key_value_pair.hpp"
#include "orbslam3_interfaces/msg/bow_feature_vector.hpp"
#include "orbslam3_interfaces/msg/bow_vector.hpp"
#include "orbslam3_interfaces/msg/grid3_d.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"
 
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include "System.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "Tracking.h"
#include "Converter.h"
#include "ImuTypes.h"
#include "MapPoint.h"

#include "MapPointConverter.hpp"
#include "Converter.hpp"

namespace Converter {
  class KeyFrameConverter {
    using keyframe = orbslam3_interfaces::msg::KeyFrame; 
    using map_point = orbslam3_interfaces::msg::MapPoint;

    using orb_keyframe = ORB_SLAM3::KeyFrame; 
    using orb_map_point = ORB_SLAM3::MapPoint;
    using orb_map = ORB_SLAM3::Map;

    public: 
      static orb_keyframe* ROSKeyFrameToORBSLAM3(keyframe::SharedPtr rKf) {
        orb_keyframe* pOKf = new orb_keyframe();
       

        //For loop -> pOkf->AddConnection(ConvertConnRosToOrb(rKf->m_backup_connected_keyframe_id_weights ))
        //For loop -> pOKf->AddChild(ConvertChildRosToOrb(rKf->mv_backup_childrens_id))
        //pOkf->ChangeParent(ConvertIdToKeyFrame(rKf->m_backup_parent_id))
        //For loop -> pOKf->AddLoopEdge(ConvertRosLoopEdgeToOrb(rKf->mv_backup_loop_edges_id))
        //For Loop -> pOKf->AddMergeEdge(ConvertRosMergeEdgeToOrb(rKf->mv_backup_merge_edges_id))
        //For Loop -> pOKf->AddMapPoint(ConvertRosMPToOrb(rKf->mvp_map_points))
        //pOKf->SetORBVocabulary() I DONT HAVE THIS YET
        //pOKf->SetKeyFrameDatabase() I DONT HAVE THIS YET
        pOKf->bImu = rKf->b_imu;
        pOKf->nNextId = rKf->n_next_id;
        pOKf->mnId = rKf->mn_id;
        //pOKf->mnFrameId = rKf->mn_frame_id;                               // const
        //pOKf->mTimeStamp = rKf->m_time_stamp;                             // const
        //pOKf->mnGridCols = rKf->mn_grid_cols;                             // const
        //pOKf->mnGridRows = rKf->mn_grid_rows;                             // const
        //pOKf->mfGridElementWidthInv = rKf->mn_grid_element_width_inv;     // const
        //pOKf->mfGridElementHeightInv = rKf->mn_grid_element_height_inv;   // const
        pOKf->mnTrackReferenceForFrame = rKf->mn_track_reference_for_frame;
        pOKf->mnFuseTargetForKF = rKf->mn_fuse_target_for_kf;
        pOKf->mnBALocalForKF = rKf->mn_ba_local_for_kf;
        pOKf->mnBAFixedForKF = rKf->mn_ba_fixed_for_kf;
        pOKf->mnNumberOfOpt = rKf->mn_number_of_opt;
        pOKf->mnLoopQuery = rKf->mn_loop_query;
        pOKf->mnLoopWords = rKf->mn_loop_words;
        pOKf->mLoopScore = rKf->m_loop_score;
        pOKf->mnRelocQuery = rKf->mn_reloc_query;
        pOKf->mnRelocWords = rKf->mn_reloc_words;
        pOKf->mRelocScore = rKf->mn_reloc_score;
        pOKf->mnMergeQuery = rKf->mn_merge_query;
        pOKf->mnMergeWords = rKf->mn_merge_words;
        pOKf->mMergeScore = rKf->m_merge_score;
        pOKf->mnPlaceRecognitionQuery = rKf->mn_place_recognition_query;
        pOKf->mnPlaceRecognitionWords = rKf->mn_place_recognition_words;
        pOKf->mPlaceRecognitionScore = rKf->m_place_recognition_score;
        pOKf->mbCurrentPlaceRecognition = rKf->mb_current_place_recognition;
        // pOKf->mTcwGBA = PoseToSophusSE3f(rKf->m_tcw_gba);
        // pOKf->mTcwBefGBA = PoseToSophusSE3f(rKf->m_tcw_bef_gba);
        // pOKf->mVwbGBA = Vecto3ToEigenVector3f(rKf->m_vwb_gba);
        // pOKf->mVwbBefGBA = Vecto3ToEigenVector3f(rKf->m_vwb_def_gba);
        // pOKf->mBiasGBA = RosImuBiasToOrb(rKf->m_bias_gba);
        pOKf->mnMergeCorrectedForKF = rKf->mn_merge_corrected_for_kf;
        pOKf->mnMergeForKF = rKf->mn_merge_for_kf;
        pOKf->mfScaleMerge = rKf->mf_scale_merge;
        pOKf->mnBALocalForMerge = rKf->mn_ba_local_for_merge;
        pOKf->mfScale = rKf->mf_scale;
        //pOKf->fx = rKf->fx;                                   // const                                   
        //pOKf->fy = rKf->fy;                                   // const
        //pOKf->cx = rKf->cx;                                   // const
        //pOKf->cy = rKf->cy;                                   // const
        //pOKf->invfx = rKf->invfx;                             // const
        //pOKf->invfy = rKf->invfy;                             // const
        //pOKf->mbf = rKf->mbf; fy;                             // const
        //pOKf->mb = rKf->mb; ; fy;                             // const
        //pOKf->mThDepth = rKf->m_th_depth;                     // const
        //pOKf->mDistCoef = ImageToCVMat(rKf->m_dist_coef);
        //pOKf->N = rKf->n;                                     // const
        //pOKf->mvKeys = GeomPoseToCVKeyPoint(rKf->mv_keys);
        //pOKf->mvKeysUn = GeomPoseToCVKeyPoint(rKf->mv_keyS_un);
        //pOKf->mvuRight = rKf->mvu_right;                      // incorrect datatype
        //pOKf->mvDepth = rKf->mv_depth;                        // incorrect datatype
        //pOKf->mDescriptors = ImageToCVMat(rKf->m_descriptors);
        // pOKf->mBowVec = ROSToDBoW2Vector(rKf->m_bow_vec);
        // pOKf->mFeatVec = ROSToDBoW2FeatVector(rKf->m_feat_vec);
        // pOKf->mTcp = PoseToSophusSE3f(rKf->m_tcp);
        //pOKf->mnScaleLevels = rKf->mn_scale_levels;           // const
        //pOKf->mfScaleFactor = rKf->mf_scale_factor;           // const
        //pOKf->mfLogScaleFactor = rKf->mf_log_scale_factor;    // const
        //pOKf->mvScaleFactors = rKf->mv_scale_factors;         // incorrect datatype
        //pOKf->mvLevelSigma2 = rKf->mv_level_sigma2;           // incorrect datatype
        //pOKf->mvInvLevelSigma2 = rKf->mv_inv_level_sigma2;      // incorrect datatype
        //pOKf->mnMinX = rKf->mn_min_x;                         // const
        //pOKf->mnMinY = rKf->mn_min_y;                         // const
        //pOKf->mnMaxX = rKf->mn_max_x;                         // const
        //pOKf->mnMaxY = rKf->mn_max_y;                         // const
        //pOKf->mPrevKF = GetKeyFrameById(rKf->???)
        //pOKf->mNextKF = GetKeyFrameById(rKf->???)
        //pOKf->mpImuPreintegrated = ROSImuPreintToOrb(rKf->mp_imu_preintegrated)
        //pOKf->mImuCalib = ROSImuCalibToOrb(rKf->???)
        pOKf->mnOriginMapId = rKf->mn_origin_map_id;
        pOKf->mNameFile = rKf->m_name_file;
        pOKf->mnDataset = rKf->mn_dataset;
        //pOKf->mvpLoopCandKFs = GetKeyFrameById(rKf->mv_backup_loop_edges_id)
        //pOKf->mvpMergeCandKFs = GetKeyFrameById(rKf->mv_backup_merge_edges_id)
        
        // Protected:
        //pOKf->SetPose(rKf->m_tcw);                  // mTcw
        //pOKf->mRcw = ??
        //pOKf->SetVelocity(rKf->m_vw);               // mVw
        //pOKf->mRwc = ??;
        


        // sophus poses
        //Sophus::SE3<float> mTcw;
        //pOKf->SetmRcw(Vector3ToEigenVector3f(pKf->m_rcw));  //Eigen::Matrix3f mRcw;
        //pOKf->SetmTwc(PoseToSophusSE3f(rKf->m_twc));        //Sophus::SE3<float> mTwc;
        //pOKf->SetmRwc(Vector3ToEigenVector3f(pKf->m_rwc));  //Eigen::Matrix3f mRwc;

        // IMU position
        //pOKf->SetmOwb(Vector3ToEigenVector3f(rKf->m_owb));  //Eigen::Vector3f mOwb;
        // Velocity (Only used for inertial SLAM)
        //pOKf->SetmVw(Vector3ToEigenVector3f(rKf->m_vw));    //Eigen::Vector3f mVw;
        //pOKf->SetMbHasVelocitY(rKf->mb_has_velocity);         //bool mbHasVelocity;

        //Transformation matrix between cameras in stereo fisheye
        //pOKf->SetmTlr(PoseToSophusSE3f(m_tlr));              //Sophus::SE3<float> mTlr;
        //pOKf->SetmTrl(PoseToSophusSE3f(m_trl));              //Sophus::SE3<float> mTrl;

        // Imu bias
        //pOKf->SetNewBias(ConvertRosToImu(rKf->m_imu_bias));   //mImuBias

        // MapPoints associated to keypoints
        //pOKf->SetMvpMapPoints(RosMapPointToORB(rKf->mvp_map_points));       //std::vector<MapPoint*> mvpMapPoints;
        // For save relation without pointer, this is necessary for save/load function
        //pOKf->SetMvBackupMapPointsId(rKf->mv_backup_map_points_id);           //std::vector<long long int> mvBackupMapPointsId;

        // BoW
        //pOKf->SetMpKeyFrameDB(rKf->mp-keyframe_db);           //KeyFrameDatabase* mpKeyFrameDB;
        //pOKf->SetORBvocabuary(rKf->mp_orb_vocabulary);        //ORBVocabulary* mpORBvocabulary;

        // Grid over the image to speed up feature matching
        //pOKf->SetmGrid(RosGrid3DToOrb(rKf->m_grid));          //std::vector< std::vector <std::vector<size_t> > > mGrid;

        //pOKf->SetmConnectedKeyFrameWeights(RosIdsToKeyFrames(rKf->m_backup_connected_keyframe_id_weights))//std::map<KeyFrame*,int> mConnectedKeyFrameWeights;
        //std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
        //std::vector<int> mvOrderedWeights;
        // For save relation without pointer, this is necessary for save/load function
        //std::map<long unsigned int, int> mBackupConnectedKeyFrameIdWeights;

        // Spanning Tree and Loop Edges
        //pOKf->SetMbFirstConnection(pKf->mb_first_connection);   //bool mbFirstConnection;
        //pOKf->SetMpParent();                                    //KeyFrame* mpParent;
        //pOKf->SetMspChildrens();                                //std::set<KeyFrame*> mspChildrens;
        //pOKf->SetMspLoopEdges();                                //std::set<KeyFrame*> mspLoopEdges;
        //pOKf->SetMspMergeEdges();                               //std::set<KeyFrame*> mspMergeEdges;
        // For save relation without pointer, this is necessary for save/load function
        pOKf->SetmBackupParentId(rKf->m_backup_parent_id);        //long long int mBackupParentId;
        //pOKf->SetMvBackupChildrensId();                         //std::vector<long unsigned int> mvBackupChildrensId;
        //pOKf->SetMvBackupLoopEdgesId();                         //std::vector<long unsigned int> mvBackupLoopEdgesId;
        //pOKf->SetMvBackupMergeEdgesId();                        //std::vector<long unsigned int> mvBackupMergeEdgesId;

        // Bad flags
        pOKf->SetMbNotErase(rKf->mb_not_erase);                   //bool mbNotErase;
        pOKf->SetMbToBeErased(rKf->mb_to_be_erased);              //bool mbToBeErased;
        pOKf->SetMbBad(rKf->mb_bad);                              //bool mbBad;    

        //pOKf->SetmHalfBaseLine(rKf->m_half_baseline);           //float mHalfBaseline; // Only for visualization

        //pOKf->SetMpMap();                                       //Map* mpMap;

        // Backup variables for inertial
        //pOKf->SetmBackupPrevKFId(rKf->m_backup_prev_kf_id);       //long long int mBackupPrevKFId;
        //pOKf->SetmBackupNextKFId(rKf->m_backup_next_kf_id);       //long long int mBackupNextKFId;
        //pOKf->SetmBackupImuPreintegrated();                     //IMU::Preintegrated mBackupImuPreintegrated;

        // Backup for Cameras
        //unsigned int mnBackupIdCamera, mnBackupIdCamera2;

        // Calibration
        //pOKf->SetMk();                                          //Eigen::Matrix3f mK_;

        pOKf->mvLeftToRightMatch = rKf->mv_left_to_right_match;
        pOKf->mvRightToLeftMatch = rKf->mv_right_to_left_match;
        //pOKf->NLeft = rKf->n_left;
        //pOKf->NRight = rKf->n_right;
        //pOKf->mGridRight = ConvertGrid3DToOrb(rKf->m_grid_right);




        return pOKf;

      }   


      static keyframe ORBSLAM3KeyFrameToROS(orb_keyframe* pKf) {
        keyframe msgKf;
        std::cout << "ORBSLAM3KeyFrameToROS" << std::endl;
        // Public 1
        msgKf.b_imu = pKf->bImu;

        // public 2
        msgKf.n_next_id = pKf->nNextId;
        msgKf.mn_id = pKf->mnId;
        msgKf.mn_frame_id = pKf->mnFrameId; //const

        msgKf.m_time_stamp = pKf->mTimeStamp;//const

        // Grid (to speed up feature matching)
        msgKf.mn_grid_cols = pKf->mnGridCols; //const
        msgKf.mn_grid_rows = pKf->mnGridRows;//const
        msgKf.mf_grid_element_width_inv = pKf->mfGridElementWidthInv;// const
        msgKf.mf_grid_element_height_inv = pKf->mfGridElementHeightInv;// const

        // Variables used by the tracking
        msgKf.mn_track_reference_for_frame = pKf->mnTrackReferenceForFrame;
        msgKf.mn_fuse_target_for_kf = pKf->mnFuseTargetForKF;

        // Variables used by the local mapping
        msgKf.mn_ba_local_for_kf = pKf->mnBALocalForKF;
        msgKf.mn_ba_fixed_for_kf = pKf->mnBAFixedForKF;

        // Number of optimizations by BA (amount of iterations in BA)
        msgKf.mn_number_of_opt = pKf->mnNumberOfOpt;

        // Variables used by the keyframe database
        msgKf.mn_loop_query = pKf->mnLoopQuery;
        msgKf.mn_loop_words = pKf->mnLoopWords; 
        msgKf.m_loop_score = pKf->mLoopScore;
        msgKf.mn_reloc_query = pKf->mnRelocQuery;
        msgKf.mn_reloc_words = pKf->mnRelocWords;
        msgKf.mn_reloc_score = pKf->mRelocScore;
        msgKf.mn_merge_query = pKf->mnMergeQuery;
        msgKf.mn_merge_words = pKf->mnMergeWords;
        msgKf.m_merge_score = pKf->mMergeScore;
        msgKf.mn_place_recognition_query = pKf->mnPlaceRecognitionQuery;
        msgKf.mn_place_recognition_words = pKf->mnPlaceRecognitionWords;
        msgKf.m_place_recognition_score = pKf->mPlaceRecognitionScore;

        msgKf.mb_current_place_recognition = pKf->mbCurrentPlaceRecognition;

        // Variables used by loop closing
        msgKf.m_tcw_gba = CppToRos::SophusSE3fToPose(pKf->mTcwGBA);
        msgKf.m_tcw_gba = CppToRos::SophusSE3fToPose(pKf->mTcwGBA); //Sophus::SE3f mTcwGBA;
        msgKf.m_tcw_bef_gba = CppToRos::SophusSE3fToPose(pKf->mTcwBefGBA); //Sophus::SE3f mTcwBefGBA;
        msgKf.m_vwb_gba = CppToRos::EigenVector3fToVector3(pKf->mVwbGBA);
        msgKf.m_vwb_bef_gba = CppToRos::EigenVector3fToVector3(pKf->mVwbBefGBA);
        
        msgKf.m_bias_gba = OrbToRos::ImuBiasToRosBias(pKf->mBiasGBA);
        
        msgKf.mn_ba_global_for_kf = pKf->mnBAGlobalForKF;

        // Variables used by merging
        msgKf.m_tcw_merge = CppToRos::SophusSE3fToPose(pKf->mTcwMerge); //Sophus::SE3f mTcwMerge;
        msgKf.m_tcw_bef_merge = CppToRos::SophusSE3fToPose(pKf->mTcwBefMerge);// Sophus::SE3f mTcwBefMerge;
        msgKf.m_twc_bef_merge = CppToRos::SophusSE3fToPose(pKf->mTwcBefMerge);// Sophus::SE3f mTwcBefMerge;
        msgKf.m_vwb_merge = CppToRos::EigenVector3fToVector3(pKf->mVwbMerge);
        msgKf.m_vwb_bef_merge = CppToRos::EigenVector3fToVector3(pKf->mVwbBefMerge);
        
        msgKf.m_bias_merge = OrbToRos::ImuBiasToRosBias(pKf->mBiasMerge); //IMU::Bias mBiasMerge;

        msgKf.mn_merge_corrected_for_kf = pKf->mnMergeCorrectedForKF;
        msgKf.mn_merge_for_kf = pKf->mnMergeForKF;
        msgKf.mf_scale_merge = pKf->mfScaleMerge;
        msgKf.mn_ba_local_for_merge = pKf->mnBALocalForMerge;

        msgKf.mf_scale = pKf->mfScale;

        // Calibration parameters
        msgKf.fx = pKf->fx;
        msgKf.fy = pKf->fy;
        msgKf.cx = pKf->cx;
        msgKf.cy = pKf->cy;
        msgKf.invfx = pKf->invfx;
        msgKf.invfy = pKf->invfy;
        msgKf.mbf = pKf->mbf; 
        msgKf.mb = pKf->mb;
        msgKf.m_th_depth = pKf->mThDepth;// const
        msgKf.m_dist_coef = CppToRos::CVMatToImage(pKf->mDistCoef);



        // Number of KeyPoints
        msgKf.n = pKf->N; //const

        // KeyPoints, stereo coordinate and descriptors (all associated by an index)
        msgKf.mv_keys = CppToRos::CVKeypointVectorToPose2DVector(pKf->mvKeys); //const std::vector<cv::KeyPoint> mvKeys;
        msgKf.mv_keys_un = CppToRos::CVKeypointVectorToPose2DVector(pKf->mvKeysUn); //const std::vector<cv::KeyPoint> mvKeysUn;
        msgKf.mvu_right = pKf->mvuRight; //const std::vector<float> mvuRight; // negative value for monocular points
        msgKf.mv_depth = pKf->mvuRight;//const std::vector<float> mvDepth; // negative value for monocular points 
        msgKf.m_descriptors = CppToRos::CVMatToImage(pKf->mDescriptors); 

        // BoW
        msgKf.m_bow_vec = OrbToRos::DBoW2VectorToRosBowVector(pKf->mBowVec); //DBoW2::BowVector mBowVec;
        msgKf.m_feat_vec = OrbToRos::DBoW2FeatVectorToRosBowFeatureVector(pKf->mFeatVec); //DBoW2::FeatureVector mFeatVec;

        // Pose relative to parent (this is computed when bad flag is activated)
        msgKf.m_tcp = CppToRos::SophusSE3fToPose(pKf->mTcp); //Sophus::SE3f mTcp;

        // Scale
        msgKf.mn_scale_levels = pKf->mnScaleLevels; //const int mnScaleLevels;
        msgKf.mf_scale_factor = pKf->mfScaleFactor;//const float mfScaleFactor;
        msgKf.mf_log_scale_factor = pKf->mfLogScaleFactor;//const float mfLogScaleFactor;
        msgKf.mv_scale_factors = pKf->mvScaleFactors;//const std::vector<float> mvScaleFactors;
        msgKf.mv_level_sigma2 = pKf->mvLevelSigma2;//const std::vector<float> mvLevelSigma2;
        msgKf.mv_inv_level_sigma2 = pKf->mvInvLevelSigma2;//const std::vector<float> mvInvLevelSigma2;

        // Image bounds and calibration
        msgKf.mn_min_x = pKf->mnMinX;//const int mnMinX;
        msgKf.mn_min_y = pKf->mnMinY;//const int mnMinY;
        msgKf.mn_max_x = pKf->mnMaxX;//const int mnMaxX;
        msgKf.mn_max_y = pKf->mnMaxY;//const int mnMaxY;

        // Preintegrated IMU measurements from previous keyframe
        //KeyFrame m_prev_kf
        //KeyFrame m_next_kf
        if(pKf->mpImuPreintegrated != nullptr) {

          msgKf.mp_imu_preintegrated = OrbToRos::ImuPreintegratedToRosPreintegrated(pKf->mpImuPreintegrated); //IMU::Preintegrated* mpImuPreintegrated;
          msgKf.m_backup_imu_preintegrated = OrbToRos::ImuPreintegratedToRosPreintegrated(pKf->mpImuPreintegrated);//IMU::Preintegrated mBackupImuPreintegrated;
        }
        //IMU::Calib mImuCalib;

        msgKf.mn_origin_map_id = pKf->mnOriginMapId;

        msgKf.m_name_file = pKf->mNameFile;
          
        msgKf.mn_dataset = pKf->mnDataset;
        
        
        //KeyFrame[] mvp_loop_cand_kfs #std::vector <KeyFrame*> mvpLoopCandKFs;
        //KeyFrame[] mvp_merge_cand_kfs #std::vector <KeyFrame*> mvpMergeCandKFs;
        std::vector<orb_keyframe*> mvpLoopCandKFs=pKf->mvpLoopCandKFs;
        std::vector<long unsigned int> mvpLoopCandKFsId;
        
        std::vector<orb_keyframe*> mvpMergeCandKFs=pKf->mvpMergeCandKFs;
        std::vector<long unsigned int> mvpMergeCandKFsId;

        for (const auto& kf : mvpLoopCandKFs) {
          mvpLoopCandKFsId.push_back(kf->mnId);
        }

        for (const auto& kf : mvpMergeCandKFs) {
          mvpMergeCandKFsId.push_back(kf->mnId);
        }

        msgKf.mvp_loop_cand_kfs_id = mvpLoopCandKFsId;
        msgKf.mvp_merge_cand_kfs_id = mvpMergeCandKFsId;

        std::cout << "end of first public" << std::endl;
        // ------------------------------------------------------------------------
        // protected

        geometry_msgs::msg::Pose mTcw = CppToRos::SophusSE3fToPose(pKf->GetPose());
        msgKf.m_tcw = mTcw;  //Sophus::SE3<float> mTcw;
        msgKf.m_rcw = mTcw.orientation; //Eigen::Matrix3f mRcw;
        
        geometry_msgs::msg::Pose mTwc = CppToRos::SophusSE3fToPose(pKf->GetPoseInverse());
        msgKf.m_twc = mTwc; //Sophus::SE3<float> mTwc;
        msgKf.m_rwc = mTwc.orientation; //Eigen::Matrix3f mRwc;
        std::cout << "IMU position" << std::endl;
        // IMU position
        msgKf.m_owb = CppToRos::EigenVector3fToVector3(pKf->GetImuPosition()); //Eigen::Vector3f mOwb;
        // Velocity (Only used for inertial SLAM)
        msgKf.m_vw = CppToRos::EigenVector3fToVector3(pKf->GetVelocity()); //Eigen::Vector3f mVw;
        msgKf.mb_has_velocity = pKf->isVelocitySet();
        
        std::cout << "Transformation matrix between cameras in stereo fisheye" << std::endl;
        // Transformation matrix between cameras in stereo fisheye
        msgKf.m_tlr = CppToRos::SophusSE3fToPose(pKf->GetRelativePoseTrl()); // Sophus::SE3f GetRelativePoseTrl();
        msgKf.m_trl = CppToRos::SophusSE3fToPose(pKf->GetRelativePoseTlr()); // Sophus::SE3f GetRelativePoseTlr();

        // Imu bias 
        msgKf.m_imu_bias = OrbToRos::ImuBiasToRosBias(pKf->GetImuBias()); //IMU::Bias mImuBias;

        std::cout << "MapPoints associated to keypoints" << std::endl;
        // MapPoints associated to keypoints
        std::set<orb_map_point*> mps;
        std::vector<map_point> msgMps;
        std::vector<long int> msgMpsBackup;
        mps = pKf->GetMapPoints();
        for (const auto& mp : mps) {
          msgMps.push_back(MapPointConverter::ORBSLAM3MapPointToROS(mp, pKf->mnId));
          msgMpsBackup.push_back(mp->mnId);
        }
        
        std::cout << "after map point stuff." << std::endl;

        msgKf.mvp_map_points = msgMps; //std::vector<MapPoint*> mvpMapPoints;
        // For save relation without pointer, this is necessary for save/load function
        msgKf.mv_backup_map_points_id = msgMpsBackup; //std::vector<long long int> mvBackupMapPointsId;

        // BoW
        //KeyFrameDatabase mp_key_frame_db //KeyFrameDatabase* mpKeyFrameDB;
        //ORBVocabulary* mpORBvocabulary;


        // Grid over the image to speed up feature matching
        msgKf.m_grid = CppToRos::VectorToGrid3D(pKf->GetMGrid()); //std::vector< std::vector <std::vector<size_t> > > mGrid;

        //std::map<KeyFrame*,int> mConnectedKeyFrameWeights;                    // Done in m_backup_connected_keyframe_id_weights
        
        std::vector<orb_keyframe*> mvpOrderedConnectedKeyFrames = pKf->GetVectorCovisibleKeyFrames();
        std::vector<long unsigned int> mvpOrderedConnectedKeyFramesId;
        for (orb_keyframe* kf : mvpOrderedConnectedKeyFrames) {
          mvpOrderedConnectedKeyFramesId.push_back(kf->mnId);
        }
        msgKf.mvp_ordered_connected_keyframes_id = mvpOrderedConnectedKeyFramesId; //std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;      
        
        std::vector<int> mvOrderedWeights;

        for(orb_keyframe* kf : mvpOrderedConnectedKeyFrames) {
          mvOrderedWeights.push_back(pKf->GetWeight(kf));
        }
        msgKf.mv_ordered_weights = mvOrderedWeights; //std::vector<int> mvOrderedWeights;
        
        std::cout << "after weights" << std::endl;
        // For save relation without pointer, this is necessary for save/load function
        msgKf.m_backup_connected_keyframe_id_weights = CppToRos::MapToRosIntTupleVector(pKf->GetBackupConnectedKeyFrameIdWeights()); //std::map<long unsigned int, int> mBackupConnectedKeyFrameIdWeights;
        
        // Spanning Tree and Loop Edges
        //bool mb_first_connection //bool mbFirstConnection;
        //KeyFrame mp_parent //KeyFrame* mpParent;
        //KeyFrame[] msp_childrens //std::set<KeyFrame*> mspChildrens;
        //KeyFrame[] msp_loop_edges //std::set<KeyFrame*> mspLoopEdges;
        //KeyFrame[] msp_merge_edges //std::set<KeyFrame*> mspMergeEdges;
        

        // For save relation without pointer, this is necessary for save/load function
        orb_keyframe* pKfP = pKf->GetParent();
        msgKf.m_backup_parent_id = (pKfP != nullptr) ? pKfP->mnId : -1;

        std::set<orb_keyframe*> mspChildrens;
        std::vector<long unsigned int> mspChildrensId;
        mspChildrens=pKf->GetChilds();
        
        for(const orb_keyframe* c : mspChildrens) {
          mspChildrensId.push_back(c->mnId);
        }
        msgKf.mv_backup_childrens_id = mspChildrensId; //std::vector<long unsigned int> mvBackupChildrensId;
        
        std::cout << "pKf->mspChildrensId" << std::endl;
        std::set<orb_keyframe*> mspLoopEdges;
        std::vector<long unsigned int> mspLoopEdgesId;
        mspLoopEdges=pKf->GetLoopEdges();

        for(const orb_keyframe* e : mspLoopEdges) {
          mspLoopEdgesId.push_back(e->mnId);
        }
        msgKf.mv_backup_loop_edges_id = mspLoopEdgesId; //std::vector<long unsigned int> mvBackupLoopEdgesId;
          
        std::cout << "pKf->mspLoopEdgesId" << std::endl;
        std::set<orb_keyframe*> mspMergeEdges;
        std::vector<long unsigned int> mspMergeEdgesId;
        mspMergeEdges=pKf->GetMergeEdges();

        for(const orb_keyframe* m : mspMergeEdges) {
          mspMergeEdgesId.push_back(m->mnId);
        }
        msgKf.mv_backup_merge_edges_id = mspMergeEdgesId; //std::vector<long unsigned int> mvBackupMergeEdgesId;

        std::cout << "After the loops" << std::endl;

        // Bad flags
        msgKf.mb_not_erase = pKf->GetNotErase(); //bool mbNotErase;
        msgKf.mb_to_be_erased = pKf->GetToBeErased(); //bool mbToBeErased;
        msgKf.mb_bad = pKf->isBad(); //bool mbBad;    

        std::cout << "Bad flags" << std::endl;
        //float32 m_half_baseline //float mHalfBaseline; // Only for visualization

        // Map mp_map //Map* mpMap;
        orb_map* pM = pKf->GetMap();
        msgKf.mp_map_id = (pM != nullptr) ? pM->GetId() : -1; 
        
        // Backup variables for inertial
        orb_keyframe* pKfPrev = pKf->mPrevKF;
        orb_keyframe* pKfNext = pKf->mNextKF;

        msgKf.m_backup_prev_kf_id = (pKfPrev != nullptr) ? pKfPrev->mnId : -1;
        msgKf.m_backup_next_kf_id = (pKfNext != nullptr) ? pKfNext->mnId : -1;


        // Backup for Cameras
        ORB_SLAM3::GeometricCamera* mpCamera = pKf->mpCamera;
        ORB_SLAM3::GeometricCamera* mpCamera2 = pKf->mpCamera2;

        msgKf.mn_backup_id_camera = (mpCamera != nullptr) ? mpCamera->GetId() : -1; 
        msgKf.mn_backup_id_camera2 = (mpCamera2 != nullptr) ? mpCamera2->GetId() : -1;

        std::cout << "Backup for cams" << std::endl;
        // Calibration
        msgKf.m_k_calib = CppToRos::EigenMatrix3ToMatrix(pKf->GetCalibrationMatrix()); //Eigen::Matrix3f mK_;


        // ---------------------------------------------------------------------------------
        // public
        // GeometricCamera* mpCamera, *mpCamera2;

        //Indexes of stereo observations correspondences
        msgKf.mv_left_to_right_match = pKf->mvLeftToRightMatch;
        msgKf.mv_right_to_left_match = pKf->mvRightToLeftMatch;//std::vector<int> mvLeftToRightMatch, mvRightToLeftMatch;

        // KeyPoints in the right image (for stereo fisheye, coordinates are needed)
        msgKf.mv_keys_right = CppToRos::CVKeypointVectorToPose2DVector(pKf->mvKeysRight); //const std::vector<cv::KeyPoint> mvKeysRight;
        std::cout << "after cv keypoints" << std::endl;
        msgKf.n_left = pKf->NLeft;
        msgKf.n_right = pKf->NRight;//const int NLeft, NRight;


        // Figure how to take nulls into consideration with grids. Not needed for monocular.
        //msgKf.m_grid_right = toGrid3D(pKf->mGridRight); //std::vector< std::vector <std::vector<size_t> > > mGridRight;

        std::cout << "in the end of the message." << std::endl;
        return msgKf;

      }


  };
};


#endif
