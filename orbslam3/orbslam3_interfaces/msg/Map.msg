# From public
KeyFrame[] mvp_keyframe_origins
uint64[] mv_backup_keyframe_origins_id
KeyFrame mp_first_region_kf

# This avoid that two points are created simultaneously in seperate threads (id conflict)
# mutex m_mutex_point_creation # Check what can be done with this one

bool mb_fail

uint64 n_next_id

# From protected
uint64 mn_id

#MapPoint mvp_backup_map_points
KeyFrame mvp_backup_keyframes

KeyFrame mp_kf_initial
KeyFrame mp_kf_flower_id

uint64 mn_backup_kf_initial_id
uint64 mn_backup_kf_lower_id

#MapPoint[] mvp_reference_map_points

bool mb_imu_initialized

int32 mn_map_change
int32 mn_map_change_notified

uint64 mn_init_kf_id
uint64 mn_max_kf_id

# index related to a big change in the map (loop closure, global ba)
int32 mn_big_change_idx

bool m_is_in_use
bool m_has_thumbnail
bool mb_bad

bool mb_is_inertial
bool mb_imu_ba1
bool mb_imu_ba2



MapPoint mvp_backup_map_points
MapPoint[] mvp_reference_map_points
