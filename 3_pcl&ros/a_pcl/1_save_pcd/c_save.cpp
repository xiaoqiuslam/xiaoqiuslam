#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char **argv)
{
    pcl::PCDWriter writer;
    writer.write ("../xxx.pcd", *cloud_hull, false);
    return (0);
}