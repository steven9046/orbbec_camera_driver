
#include <pangolin/pangolin.h> // use this to show
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace viewer{
class Viewer {
public:    
    Viewer();
    ~Viewer();
    Viewer(int height, int width, std::string name);
    void setup();
    std::string window_name_;
    int window_height_, window_width_;
    void setWindowName(std::string name);
    void run();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_;
    void setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);
};
}