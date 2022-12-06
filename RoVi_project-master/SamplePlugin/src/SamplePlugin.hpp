#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

//#include <QObject>
#undef foreach
#include <boost/foreach.hpp>

// RobWork includes
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rwlibs/simulation/GLFrameGrabber25D.hpp>


#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>


// RobWorkStudio includes
#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition
#include <rws/RobWorkStudioPlugin.hpp>

// OpenCV 3
#include <opencv2/opencv.hpp>

// Qt
#include <QTimer>

#include "ui_SamplePlugin.h"

#include <rws/RobWorkStudio.hpp>

#include <QPushButton>

#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellLoader.hpp>

#include <functional>

#include <QApplication>
#include <QChartView>
#include <QLineSeries>
#include <QMainWindow>

//PCL includes

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>

#include <pcl/common/transforms.h>

#include <pcl/segmentation/extract_clusters.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/common/time.h>

typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

using pclPoint = pcl::PointXYZ;
using pclCloud = pcl::PointCloud<pclPoint>;

//Other includes
#include <math.h>

using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::sensor;
using namespace rwlibs::opengl;
using namespace rwlibs::simulation;

using namespace std;
using namespace rw::math;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;


using namespace rws;

using namespace cv;

using namespace std::placeholders;


class SamplePlugin: public rws::RobWorkStudioPlugin, private Ui::SamplePlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")
public:
    SamplePlugin();
    virtual ~SamplePlugin();

    virtual void open(rw::models::WorkCell* workcell);

    virtual void close();

    virtual void initialize();

private slots:
    void btnPressed();
    void timer();
    void getImage();
    void get25DImage();
    void computeDisparity();
    void p2pInter(Q start, Q end, int steps);
    void p2pInterBlend(Q start, Q end, int steps, int blend);
    void graphPath ();
    void locateObject ();
    std::vector<rw::math::Q> getConfigurations(const std::string nameGoal, const std::string nameTcp, rw::models::SerialDevice::Ptr robot, rw::models::WorkCell::Ptr wc, rw::kinematics::State state)
    void testReachability();
    void locateObject_imagebased();
    
    void stateChangedListener(const rw::kinematics::State& state);

    bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q);
    void createPathRRTConnect(Q from, Q to,  double extend, double maxTime);
	void printProjectionMatrix(std::string frameName);

private:
    static cv::Mat toOpenCVImage(const rw::sensor::Image& img);

    QTimer* _timer;
    QTimer* _timer25D;
    
    rw::models::WorkCell::Ptr _wc;
    rw::kinematics::State _state;
    rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
    rwlibs::simulation::GLFrameGrabber* _framegrabber;
    rwlibs::simulation::GLFrameGrabber25D* _framegrabber25D;    
    std::vector<std::string> _cameras;
    std::vector<std::string> _cameras25D;
    Device::Ptr _device;
    Device::Ptr _gripper;
    MovableFrame::Ptr _bottle_frame;
    Frame::Ptr _gripper_frame;
    Frame::Ptr _table_frame;
    Frame::Ptr _cam_left_frame;
    Frame::Ptr _ur_base_frame;
    QPath _path;
    QPath _path_rrt;
    int _step;
    int _grasp_step;
    int _release_step;

    Q _home  = Q(7, -0.348717, -1.58493, 2.58239, -1.02573, 1.49383, 0, 0.05);
    Q temp  = Q(6, 0, 0, 0, 0, 0, 0);
    Q gripper_temp = Q(1,0);
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
