// 旋转向量转旋转矩阵
cv::Mat R;
cv::Rodrigues(rvec, R);
Eigen::Matrix3d r;
cv::cv2eigen(R, r);

Eigen::Matrix3d r_AngleAxisd;
for ( int i=0; i<3; i++ )
    for ( int j=0; j<3; j++ )
        r_AngleAxisd(i,j) = R.at<double>(i,j);

// 旋转矩阵和平移向量组装变换矩阵
Eigen::Isometry3d transform_matrix_rotate = Eigen::Isometry3d::Identity();
transform_matrix_rotate.pretranslate(Eigen::Vector3d(tvec.at<double>(0, 0), tvec.at<double>(0, 1), tvec.at<double>(0, 2)));
transform_matrix_rotate.rotate(r);
std::cout << "transform_matrix_rotate matrix: \n" << transform_matrix_rotate.matrix() << std::endl;

Eigen::Isometry3d transform_matrix_AngleAxisd = Eigen::Isometry3d::Identity();
Eigen::AngleAxisd angle(r);
Eigen::Translation<double,3> trans(tvec.at<double>(0,0), tvec.at<double>(0,1), tvec.at<double>(0,2));
transform_matrix_AngleAxisd = angle;
transform_matrix_AngleAxisd(0,3) = tvec.at<double>(0,0);
transform_matrix_AngleAxisd(1,3) = tvec.at<double>(0,1);
transform_matrix_AngleAxisd(2,3) = tvec.at<double>(0,2);
std::cout << "transform_matrix_AngleAxisd matrix: \n" << transform_matrix_AngleAxisd.matrix() << std::endl;

// 将平移向量和旋转矩阵转换成变换矩阵
Eigen::Isometry3d T_AngleAxisd = Eigen::Isometry3d::Identity();
Eigen::AngleAxisd angle_AngleAxisd(r_AngleAxisd);
T_AngleAxisd = angle_AngleAxisd;
T_AngleAxisd(0,3) = tvec.at<double>(0,0);
T_AngleAxisd(1,3) = tvec.at<double>(1,0);
T_AngleAxisd(2,3) = tvec.at<double>(2,0);
cout<<"T_AngleAxisd\n"<<T_AngleAxisd.matrix()<<endl;

   