#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
 
 
int main(int argc, char **argv) {
    cout << "Test PCL !!!" << endl;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    uint8_t r(255), g(15), b(15);
    for (float z(-1.0); z <= 1.0; z += 0.05)
    {
      for (float angle(0.0); angle <= 360.0; angle += 5.0)
      {
	pcl::PointXYZRGB point;
	point.x = 0.5 * cosf (pcl::deg2rad(angle));
	point.y = sinf (pcl::deg2rad(angle));
	point.z = z;
	uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
		static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
	point.rgb = *reinterpret_cast<float*>(&rgb);
	point_cloud_ptr->points.push_back (point);
      }
      if (z < 0.0)
      {
	r -= 12;
	g += 12;
      }
      else
      {
	g -= 12;
	b += 12;
      }
    }
    point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
    point_cloud_ptr->height = 1;
    
    pcl::visualization::CloudViewer viewer ("test");
    viewer.showCloud(point_cloud_ptr);
    while (!viewer.wasStopped()){ };
    return 0;
}

