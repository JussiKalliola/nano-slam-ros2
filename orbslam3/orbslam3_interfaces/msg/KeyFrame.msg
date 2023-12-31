# Public 1
bool b_imu

# public 2
uint64 n_next_id
uint64 mn_id
uint64 mn_frame_id #const

float64 m_time_stamp #const

# Grid (to speed up feature matching)
int32 mn_grid_cols #const
int32 mn_grid_rows #const
float32 mf_grid_element_width_inv # const
float32 mf_grid_element_height_inv # const

# Variables used by the tracking
uint64 mn_track_reference_for_frame
uint64 mn_fuse_target_for_kf

# Variables used by the local mapping
uint64 mn_ba_local_for_kf
uint64 mn_ba_fixed_for_kf

# Number of optimizations by BA (amount of iterations in BA)
uint64 mn_number_of_opt

# Variables used by the keyframe database
uint64 mn_loop_query
int32 mn_loop_qords
float32 m_loop_score
uint64 mn_reloc_query
int32 mn_reloc_words
float32 mn_reloca_score
uint64 mn_merge_query
int32 mn_merge_words
float32 m_merge_score
uint64 mn_place_recognition_query
int32 mn_place_recognition_words
float32 m_place_recognition_score

bool mb_current_place_recognition

# Variables used by loop closing
#Sophus::SE3f mTcwGBA;
#Sophus::SE3f mTcwBefGBA;
geometry_msgs/Vector3 m_vwb_gba
geometry_msgs/Vector3 m_vwb_bef_gba
#IMU::Bias mBiasGBA;
uint64 mn_ba_global_for_kf

# Variables used by merging
#Sophus::SE3f mTcwMerge;
#Sophus::SE3f mTcwBefMerge;
#Sophus::SE3f mTwcBefMerge;
geometry_msgs/Vector3 m_vwb_merge
geometry_msgs/Vector3 m_vwb_bef_merge
#IMU::Bias mBiasMerge;
uint64 mn_merge_corrected_for_kf
uint64 mn_merge_for_kf
float32 mf_scale_merge
uint64 mn_ba_local_for_merge

float32 mf_scale

# Calibration parameters
float32 fx
float32 fy
float32 cx
float32 cy
float32 invfx
float32 invfy
float32 mbf 
float32 mb 
float32 m_th_depth # const
sensor_msgs/Image m_dist_coef

# Number of KeyPoints
int32 n #const

# KeyPoints, stereo coordinate and descriptors (all associated by an index)
#const std::vector<cv::KeyPoint> mvKeys;
#const std::vector<cv::KeyPoint> mvKeysUn;
float32[] mvu_right #const std::vector<float> mvuRight; // negative value for monocular points
float32[] mv_depth #const std::vector<float> mvDepth; // negative value for monocular points
sensor_msgs/Image m_descriptors #const cv::Mat mDescriptors;

# BoW
#DBoW2::BowVector mBowVec;
#DBoW2::FeatureVector mFeatVec;

# Pose relative to parent (this is computed when bad flag is activated)
#Sophus::SE3f mTcp;

# Scale
int32 mn_scale_levels #const int mnScaleLevels;
float32 mf_scale_factor #const float mfScaleFactor;
float32 mf_log_scale_factor #const float mfLogScaleFactor;
float32[] mv_scale_factors #const std::vector<float> mvScaleFactors;
float32[] mv_level_sigma2  #const std::vector<float> mvLevelSigma2;
float32[] mv_inv_level_sigma2 #const std::vector<float> mvInvLevelSigma2;

# Image bounds and calibration
int32 mn_min_x #const int mnMinX;
int32 mn_min_y #const int mnMinY;
int32 mn_max_x #const int mnMaxX;
int32 mn_max_y #const int mnMaxY;

# Preintegrated IMU measurements from previous keyframe
#KeyFrame m_prev_kf
#KeyFrame m_next_kf

#IMU::Preintegrated* mpImuPreintegrated;
#IMU::Calib mImuCalib;

uint32 mn_origin_map_id

string m_name_file

int32 mn_dataset

#KeyFrame[] mvp_loop_cand_kfs #std::vector <KeyFrame*> mvpLoopCandKFs;
#KeyFrame[] mvp_merge_cand_kfs #std::vector <KeyFrame*> mvpMergeCandKFs;



# protected

#Sophus::SE3<float> mTcw;
#Eigen::Matrix3f mRcw;
#Sophus::SE3<float> mTwc;
#Eigen::Matrix3f mRwc;

# IMU position
#Eigen::Vector3f mOwb;
# Velocity (Only used for inertial SLAM)
#Eigen::Vector3f mVw;
bool mb_has_velocity

# Transformation matrix between cameras in stereo fisheye
#Sophus::SE3<float> mTlr;
#Sophus::SE3<float> mTrl;

# Imu bias
#IMU::Bias mImuBias;

# MapPoints associated to keypoints
#1MapPoint[] mvp_map_points #std::vector<MapPoint*> mvpMapPoints;
# For save relation without pointer, this is necessary for save/load function
int64[] mv_backup_map_points_id #std::vector<long long int> mvBackupMapPointsId;

# BoW
KeyFrameDatabase mp_key_frame_db #KeyFrameDatabase* mpKeyFrameDB;
#ORBVocabulary* mpORBvocabulary;

# Grid over the image to speed up feature matching
#std::vector< std::vector <std::vector<size_t> > > mGrid;

#std::map<KeyFrame*,int> mConnectedKeyFrameWeights;
#KeyFrame[] mvp_orfered_connected_keyframes #std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
int32[] mv_ordered_weights #std::vector<int> mvOrderedWeights;
# For save relation without pointer, this is necessary for save/load function
#std::map<long unsigned int, int> mBackupConnectedKeyFrameIdWeights;

# Spanning Tree and Loop Edges
bool mb_first_connection #bool mbFirstConnection;
#KeyFrame mp_parent #KeyFrame* mpParent;
#KeyFrame[] msp_childrens #std::set<KeyFrame*> mspChildrens;
#KeyFrame[] msp_loop_edges #std::set<KeyFrame*> mspLoopEdges;
#KeyFrame[] msp_merge_edges #std::set<KeyFrame*> mspMergeEdges;
# For save relation without pointer, this is necessary for save/load function
int64 m_backup_parent_id #long long int mBackupParentId;
uint64[] mv_backup_childrens_id #std::vector<long unsigned int> mvBackupChildrensId;
uint64[] mv_backup_loop_edges_id #std::vector<long unsigned int> mvBackupLoopEdgesId;
uint64[] mv_backup_merge_edges_id #std::vector<long unsigned int> mvBackupMergeEdgesId;

# Bad flags
bool mb_not_erase #bool mbNotErase;
bool mb_to_be_erased #bool mbToBeErased;
bool mb_bad #bool mbBad;    

float32 m_half_baseline #float mHalfBaseline; // Only for visualization

#Map mp_map #Map* mpMap;

# Backup variables for inertial
int64 m_backup_prev_kf_id #long long int mBackupPrevKFId;
int64 m_backup_next_kf_id #long long int mBackupNextKFId;
#IMU::Preintegrated mBackupImuPreintegrated;

# Backup for Cameras
uint32 mn_backup_id_camera 
uint32 mn_backup_id_camera2 #unsigned int mnBackupIdCamera, mnBackupIdCamera2;

# Calibration
# Eigen::Matrix3f mK_;

# Mutex
#std::mutex mMutexPose; // for pose, velocity and biases
#std::mutex mMutexConnections;
#std::mutex mMutexFeatures;
#std::mutex mMutexMap;

# public
#GeometricCamera* mpCamera, *mpCamera2;

# Indexes of stereo observations correspondences
int32[] mv_left_to_right_match
int32[] mv_right_to_left_match #std::vector<int> mvLeftToRightMatch, mvRightToLeftMatch;

#Sophus::SE3f GetRelativePoseTrl();
#Sophus::SE3f GetRelativePoseTlr();

# KeyPoints in the right image (for stereo fisheye, coordinates are needed)
#const std::vector<cv::KeyPoint> mvKeysRight;

int32 n_left
int32 n_right #const int NLeft, NRight;

#std::vector< std::vector <std::vector<size_t> > > mGridRight;
