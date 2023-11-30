# Preintegration of Imu measurements
float32 d_t
Matrix c    # Eigen::Matrix<float,15,15> C;
Matrix info # Eigen::Matrix<float,15,15> Info;
Matrix nga  # Eigen::DiagonalMatrix<float,6> Nga, NgaWalk;
Matrix nga_walk
