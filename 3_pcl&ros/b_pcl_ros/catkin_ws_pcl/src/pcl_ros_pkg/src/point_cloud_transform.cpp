#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

class pcl_sub
{
private:
    ros::NodeHandle n;
    ros::Subscriber subCloud;
    ros::Publisher pubCloud;
    // 接收到的点云消息
    sensor_msgs::PointCloud2 msg;   
    // 等待发布的点云消息             
    sensor_msgs::PointCloud2 adjust_msg; 
    // 建立了一个pcl的点云，用于完成点云的中间处理过程         
    pcl::PointCloud<pcl::PointXYZRGB> adjust_pcl; 

public:
    // https://blog.csdn.net/u014587147/article/details/75647002 构造函数
    pcl_sub():n("~")
    {   
        // 接收点云数据，进入回调函数 getcloud() /point_cloud_topic 为订阅的点云话题名
        // 1 为待处理信息队列大小，一次只处理一个消息
        // &pcl_sub::getcloud 调用的函数指针，即回调函数。
        // this 回调函数所在的类
        subCloud = n.subscribe<sensor_msgs::PointCloud2>("/point_cloud_pub_topic", 1, &pcl_sub::getcloud, this); 
        // 发布位姿变换后的点云/adjusted_cloud
        pubCloud = n.advertise<sensor_msgs::PointCloud2>("/adjustd_cloud", 1);                                     
    }

    // 回调函数
    void getcloud(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
    {   
        // 放在这里是因为，每次都需要重新初始化
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr adjust_pcl_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); 
        // 把msg消息转化为点云
        pcl::fromROSMsg(*laserCloudMsg, *adjust_pcl_ptr);                                             

        // 定义一个旋转矩阵 虽然称为3d，实质上是4x4的矩阵(旋转R+平移t)
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        Eigen::AngleAxisd rotationVector(M_PI/4,Eigen::Vector3d(0,0,1));
        Eigen::Matrix3d rotationMatrix=Eigen::Matrix3d::Identity();
        rotationMatrix=rotationVector.toRotationMatrix();
        //旋转部分赋值
        T.linear() = rotationMatrix;
        //平移部分赋值
        T.translation() = Eigen::Vector3d(0, 0, 0);

        // 执行变换，并将结果保存在新创建的 transformed_cloud 中
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
        pcl::transformPointCloud (*adjust_pcl_ptr, *transformed_cloud, T.matrix());

        adjust_pcl = *transformed_cloud;
        for (int i = 0; i < adjust_pcl.points.size(); i++)
        //把接收到的点云位置不变，颜色全部变为红色
        {
            adjust_pcl.points[i].r = 255;
            adjust_pcl.points[i].g = 0;
            adjust_pcl.points[i].b = 0;
        }

        // 将点云转化为消息才能发布
        pcl::toROSMsg(adjust_pcl, adjust_msg); 
        // 发布调整之后的点云数据 主题为/adjustd_cloud
        pubCloud.publish(adjust_msg);          
    }

    // 析构函数
    ~pcl_sub() {}
};

int main(int argc, char **argv)
{
    // 初始化了一个节点，名字为colored
    ros::init(argc, argv, "point_cloud_transform"); 
    // 创建一个对象，回调函数放在了构造函数中，所以创建对象的时候会顺序调用构造函数，回调函数，然后点云的接收、转换、发布就完成了。
    pcl_sub ps;
    ros::spin();
    return 0;
}
