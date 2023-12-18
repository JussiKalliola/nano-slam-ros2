uint64 kf_id
string system_id

bool compute_bow

geometry_msgs/Pose set_pose

geometry_msgs/Vector3 set_velocity

int64 add_connection_id
int64 add_connection_weight

bool update_best_covisibles

int64 add_map_point_mp_id         # map point id
int64 add_map_point_vector_idx     # idx in the vector

int64 erase_map_point_vector_idx   # Remove map point at vector idx

int64 erase_map_point_id          # Find map point ptr with id, remove that

int64 replace_map_point_id
int64 replace_map_point_vector_idx

bool update_connections_up_parent

int64 add_child_id

int64 erase_child_id

int64 change_parent_id

bool set_first_connection

int64 add_loop_edge_id

int64 add_merge_edge_id

bool set_not_erase

bool set_erase

bool set_bad_flag

int64 erase_connection_id

IMUBias set_new_bias

int64 update_map_id

# uint64 set_orb_vocab_id
# uint64 set_kf_db_id

