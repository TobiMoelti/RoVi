#include "SamplePlugin.hpp"

#include <filesystem>
//asad
SamplePlugin::SamplePlugin () : RobWorkStudioPlugin ("SamplePluginUI", QIcon ((std::filesystem::path(__FILE__).parent_path()/"pa_icon.png").c_str())) ,_path(), _path_rrt()
{
    
    setupUi (this);

    _timer = new QTimer (this);
    connect (_timer, SIGNAL (timeout ()), this, SLOT (timer ()));

    // now connect stuff from the ui component
    connect (_btn_im, SIGNAL (pressed ()), this, SLOT (btnPressed ()));
    connect (_btn_disp, SIGNAL (pressed ()), this, SLOT (btnPressed ()));
    connect (_btn_p2p, SIGNAL (pressed ()), this, SLOT (btnPressed ()));
    connect (_btn_p2p_blend, SIGNAL (pressed ()), this, SLOT (btnPressed ()));
    connect (_btn0, SIGNAL (pressed ()), this, SLOT (btnPressed ()));
    connect (_btn1, SIGNAL (pressed ()), this, SLOT (btnPressed ()));
    connect (_btn_graph, SIGNAL (pressed ()), this, SLOT (btnPressed ()));
    connect (_btn_loca, SIGNAL (pressed ()), this, SLOT (btnPressed ()));
    connect (_btn_loca, SIGNAL (pressed ()), this, SLOT (btnPressed ()));
    connect (_btn_img, SIGNAL (pressed ()), this, SLOT (btnPressed ()));
    connect (_btn_reachability, SIGNAL (pressed ()), this, SLOT (btnPressed ()));

    
    _framegrabber = NULL;

    _cameras    = {"Camera_Right", "Camera_Left"};
    _cameras25D = {"Scanner25D"};
}

SamplePlugin::~SamplePlugin ()
{
    delete _textureRender;
    delete _bgRender;
}

void SamplePlugin::initialize ()
{
    log ().info () << "INITALIZE"
                   << "\n";

    getRobWorkStudio ()->stateChangedEvent ().add (
        std::bind (&SamplePlugin::stateChangedListener, this, std::placeholders::_1), this);

    // Auto load workcell

    std::filesystem::path wc_path (__FILE__);
    wc_path          = wc_path.parent_path() / "../../WorkCell/Scene.wc.xml";
	std::cout << "wc path: " << wc_path << std::endl;
    WorkCell::Ptr wc = WorkCellLoader::Factory::load (wc_path.string ());
    getRobWorkStudio ()->setWorkCell (wc);
}

void SamplePlugin::open (WorkCell* workcell)
{
    log ().info () << "OPEN"
                   << "\n";
    _wc    = workcell;
    _state = _wc->getDefaultState ();

    log ().info () << workcell->getFilename () << "\n";

    if (_wc != NULL) {
        // Add the texture render to this workcell if there is a frame for texture
        Frame* textureFrame = _wc->findFrame ("MarkerTexture");
        if (textureFrame != NULL) {
            getRobWorkStudio ()->getWorkCellScene ()->addRender (
                "TextureImage", _textureRender, textureFrame);
        }
        // Add the background render to this workcell if there is a frame for texture
        Frame* bgFrame = _wc->findFrame ("Background");
        if (bgFrame != NULL) {
            getRobWorkStudio ()->getWorkCellScene ()->addRender (
                "BackgroundImage", _bgRender, bgFrame);
        }

        // Create a GLFrameGrabber if there is a camera frame with a Camera property set
        Frame* cameraFrame = _wc->findFrame (_cameras[0]);
        if (cameraFrame != NULL) {
            if (cameraFrame->getPropertyMap ().has ("Camera")) {
                // Read the dimensions and field of view
                double fovy;
                int width, height;
                std::string camParam = cameraFrame->getPropertyMap ().get< std::string > ("Camera");
                std::istringstream iss (camParam, std::istringstream::in);
                iss >> fovy >> width >> height;
                // Create a frame grabber
                _framegrabber             = new GLFrameGrabber (width, height, fovy);
                SceneViewer::Ptr gldrawer = getRobWorkStudio ()->getView ()->getSceneViewer ();
                _framegrabber->init (gldrawer);
            }
        }

        Frame* cameraFrame25D = _wc->findFrame (_cameras25D[0]);
        if (cameraFrame25D != NULL) {
            if (cameraFrame25D->getPropertyMap ().has ("Scanner25D")) {
                // Read the dimensions and field of view
                double fovy;
                int width, height;
                std::string camParam =
                    cameraFrame25D->getPropertyMap ().get< std::string > ("Scanner25D");
                std::istringstream iss (camParam, std::istringstream::in);
                iss >> fovy >> width >> height;
                // Create a frame grabber
                _framegrabber25D          = new GLFrameGrabber25D (width, height, fovy);
                SceneViewer::Ptr gldrawer = getRobWorkStudio ()->getView ()->getSceneViewer ();
                _framegrabber25D->init (gldrawer);
            }
        }

        _device = _wc->findDevice ("UR-6-85-5-A");
        _gripper = _wc->findDevice("WSG50");
        _bottle_frame = _wc->findFrame<MovableFrame>("Bottle");
        _gripper_frame = _wc->findFrame("GraspTCP");
        _table_frame = _wc->findFrame("Table");
        _cam_left_frame = _wc->findFrame("Camera_Left");
        _ur_base_frame = _wc->findFrame("URReference");


        for (int i = 0; i<6;i++)
        {
            temp(i) = _home(i);
        }
        gripper_temp(0) = _home(6);
        _gripper->setQ(gripper_temp ,_state);
        _device->setQ (temp, _state);
        getRobWorkStudio ()->setState (_state);

        /*
        _device->setQ(_home,_state);
        getRobWorkStudio ()->setState (_state);
        */
        /*
         Setting gripper posistion
        Q gripper_newpos (0.055);
        _gripper->setQ(gripper_newpos,_state);
        getRobWorkStudio ()->setState (_state);
        */
        _step   = -1;
        _grasp_step = -1;
        _release_step = -1;
    }
}

void SamplePlugin::close ()
{
    log ().info () << "CLOSE"
                   << "\n";

    // Stop the timer
    _timer->stop ();
    // Remove the texture render
    Frame* textureFrame = _wc->findFrame ("MarkerTexture");
    if (textureFrame != NULL) {
        getRobWorkStudio ()->getWorkCellScene ()->removeDrawable ("TextureImage", textureFrame);
    }
    // Remove the background render
    Frame* bgFrame = _wc->findFrame ("Background");
    if (bgFrame != NULL) {
        getRobWorkStudio ()->getWorkCellScene ()->removeDrawable ("BackgroundImage", bgFrame);
    }
    // Delete the old framegrabber
    if (_framegrabber != NULL) {
        delete _framegrabber;
    }
    _framegrabber = NULL;
    _wc           = NULL;
}

Mat SamplePlugin::toOpenCVImage (const Image& img)
{
    Mat res (img.getHeight (), img.getWidth (), CV_8SC3);
    res.data = (uchar*) img.getImageData ();
    return res;
}

void SamplePlugin::btnPressed ()
{
    QObject* obj = sender ();
    //Calculate RRT path
    if (obj == _btn0) {
        //		log().info() << "Button 0\n";
        //		// Toggle the timer on and off
        //		if (!_timer25D->isActive())
        //		    _timer25D->start(100); // run 10 Hz
        //		else
        //			_timer25D->stop();
        _timer->stop ();
        rw::math::Math::seed ();
        double extend  = 0.05;
        double maxTime = 60;
        Q from (6, -1.144, -0.082, -0.159, -3.666, -1.571, 0); 
        Q to (6, 0.526, 0.027, -0.159, -3.666, -1.571, 0);    // From pose estimation
        createPathRRTConnect (from, to, extend, maxTime);
    }
    //Run RRT path
    else if (obj == _btn1) {
        log ().info () << "Button 1\n";
        // Toggle the timer on and off
        if (!_timer->isActive ()) {
            _timer->start (100);    // run 10 Hz
            _step = 0;
        }
        else
            _step = 0;
    }
    else if (obj == _btn_im) {
        getImage ();
    }
    else if (obj == _btn_disp) {
        computeDisparity ();
    }
    else if (obj == _btn_p2p) {
        _path.clear();
        // home -> approach_pick -> pick -> grasp -> leave_pick -> approach_place -> place -> release -> leave place -> home
        Q approach_pick (7, -2.26994, -1.80947, 2.75821, -0.945113, 0.885598, -0.00230383, 0.05); 
        Q pick (7, -1.98538, -1.49461, 2.49235, -0.994681, 1.17016, -0.00118682, 0.05);
        Q grasp (7, -1.98538, -1.49461, 2.49235, -0.994681, 1.17016, -0.00118682, 0.045);
        Q leave_pick (7,-1.98536, -1.75336, 2.36054, -0.604146, 1.17017, -0.00118682, 0.045);
        Q approach_place (7, 2.07509, -1.33762, 1.93182, -0.594197, 2.07509, -0.0015708, 0.045);
        Q place (7, 2.07509, -1.16715, 2.02579, -0.858615, 2.07509, -0, 0.045);
        Q release (7, 2.07509, -1.16715, 2.02579, -0.858615, 2.07509, -0, 0.05);
        Q leave_place (7, 2.07509, -1.33762, 1.93182, -0.594197, 2.07509, -0.0015708, 0.05);

        p2pInter (_home, approach_pick, 20);
        p2pInter (approach_pick, pick, 10);
        p2pInter (pick, grasp, 5);
        p2pInter (grasp, leave_pick, 10);
        p2pInter (leave_pick, approach_place, 20);
        p2pInter (approach_place, place , 10);
        p2pInter (place, release, 5);
        p2pInter (release, leave_place, 10);
        p2pInter (leave_place, _home ,20);


    }
    else if (obj == _btn_p2p_blend) {
        _path.clear();
        // home -> approach_pick -> pick -> grasp -> leave_pick -> approach_place -> place -> release -> leave place -> home
        Q approach_pick (7, -2.26994, -1.80947, 2.75821, -0.945113, 0.885598, -0.00230383, 0.05); 
        Q pick (7, -1.98538, -1.49461, 2.49235, -0.994681, 1.17016, -0.00118682, 0.05);
        Q grasp (7, -1.98538, -1.49461, 2.49235, -0.994681, 1.17016, -0.00118682, 0.045);
        Q leave_pick (7,-1.98536, -1.75336, 2.36054, -0.604146, 1.17017, -0.00118682, 0.045);
        Q approach_place (7, 2.07509, -1.33762, 1.93182, -0.594197, 2.07509, -0.0015708, 0.045);
        Q place (7, 2.07509, -1.16715, 2.02579, -0.858615, 2.07509, -0, 0.045);
        Q release (7, 2.07509, -1.16715, 2.02579, -0.858615, 2.07509, -0, 0.05);
        Q leave_place (7, 2.07509, -1.33762, 1.93182, -0.594197, 2.07509, -0.0015708, 0.05);

        p2pInterBlend (_home, approach_pick, 20,6);
        p2pInterBlend (approach_pick, pick, 10,2);
        p2pInterBlend (pick, grasp, 5,2);
        _grasp_step = 35;
        p2pInterBlend (grasp, leave_pick, 10,2);
        p2pInterBlend (leave_pick, approach_place, 20,6);
        p2pInterBlend (approach_place, place , 10,2);
        p2pInterBlend (place, release, 5,2);
        _release_step = 85;
        p2pInterBlend (release, leave_place, 10,2);
        p2pInterBlend (leave_place, _home ,20,6);

    }
    else if (obj == _btn_graph) {
        graphPath();
    } else if (obj == _btn_loca){
        locateObject();
    }
     else if (obj == _btn_reachability) {
        testReachability ();
    }
     else if (obj == _btn_img) {
        locateObject_imagebased ();
    }
}

/*
void SamplePlugin::graphPath ()
{
    //Save bigger image?
    QtCharts::QLineSeries* jointPosSeries[7];
    QtCharts::QLineSeries* jointVelSeries[7];
    QtCharts::QLineSeries* jointAcelSeries[7];
    QtCharts::QLineSeries* carPosSeries[3];
    QtCharts::QLineSeries* carVelSeries[3];
    QtCharts::QLineSeries* carAcelSeries[3];
    double prevJointPos[7];
    double prevJointVel[7];
    double prevCarPos[3];
    double prevCarVel[3];

    for (int i = 0; i<7;i++)
    {
        jointPosSeries[i] = new QtCharts::QLineSeries ();
        jointVelSeries[i] = new QtCharts::QLineSeries ();
        jointAcelSeries[i] = new QtCharts::QLineSeries ();
        prevJointPos[i] = _path.at(0)(i);
        prevJointVel[i] = 0.0;
    }
    //getting initial cartesian position
    for (int k = 0; k<6;k++)
    {
        temp(k) = _path.at(0)(k);
    }
    _device->setQ (temp, _state);
    Transform3D endEff = _ur_base_frame->fTf(_gripper_frame,_state);

    for (int i = 0; i<3;i++)
    {
        carPosSeries[i] = new QtCharts::QLineSeries ();
        carVelSeries[i] = new QtCharts::QLineSeries ();
        carAcelSeries[i] = new QtCharts::QLineSeries ();
        prevCarPos[i] = endEff.P()[i];
        prevCarVel[i] = 0.0;
    }

    double jointVel = 0;
    double carVel = 0;
    for (int i = 0; (size_t) i < _path.size ();i++)
    {
        for (int k = 0; k<6;k++)
        {
            temp(k) = _path.at(i)(k);
        }
        _device->setQ (temp, _state);
        endEff = _ur_base_frame->fTf(_gripper_frame,_state);

        for (int j = 0; j<3;j++)
        {
            carPosSeries[j]->append (i, endEff.P()[j]);
            carVel = endEff.P()[j] - prevCarPos[j];
            carVelSeries[j]->append (i, carVel);
            carAcelSeries[j]->append (i, carVel-prevCarVel[j]);
            prevCarPos[j] = endEff.P()[j];
            prevCarVel[j] = carVel;
        }
        
        for (int j = 0; j<7;j++)
        {
            jointPosSeries[j]->append (i, _path.at(i)(j));
            jointVel = _path.at(i)(j) - prevJointPos[j];
            jointVelSeries[j]->append (i, jointVel);
            jointAcelSeries[j]->append (i, jointVel-prevJointVel[j]);
            prevJointPos[j] = _path.at(i)(j);
            prevJointVel[j] = jointVel;
        }
    }
    //Posistion chart
    //Joint
    QtCharts::QChart* chartPos = new QtCharts::QChart ();
    chartPos->legend ()->hide ();
    for (int i = 0; i<7;i++)
    {
        chartPos->addSeries (jointPosSeries[i]);
    }
    chartPos->createDefaultAxes ();
    chartPos->setTitle ("Joint Position");
    QtCharts::QChartView* chartViewPos = new QtCharts::QChartView (chartPos);
    chartViewPos->setRenderHint (QPainter::Antialiasing);
    //Cartesian
    QtCharts::QChart* chartPosCar = new QtCharts::QChart ();
    chartPosCar->legend ()->hide ();
    for (int i = 0; i<3;i++)
    {
        chartPosCar->addSeries (carPosSeries[i]);
    }
    chartPosCar->createDefaultAxes ();
    chartPosCar->setTitle ("Cartesian Position");
    QtCharts::QChartView* chartViewPosCar = new QtCharts::QChartView (chartPosCar);
    chartViewPosCar->setRenderHint (QPainter::Antialiasing);

    //Velocity chart
    //Position
    QtCharts::QChart* chartVel = new QtCharts::QChart ();
    chartVel->legend ()->hide ();
    for (int i = 0; i<7;i++)
    {
        chartVel->addSeries (jointVelSeries[i]);
    }
    chartVel->createDefaultAxes ();
    chartVel->setTitle ("Joint Velocity");
    QtCharts::QChartView* chartViewVel = new QtCharts::QChartView (chartVel);
    chartViewVel->setRenderHint (QPainter::Antialiasing);
    //Cartesian
    QtCharts::QChart* chartVelCar = new QtCharts::QChart ();
    chartVelCar->legend ()->hide ();
    for (int i = 0; i<3;i++)
    {
        chartVelCar->addSeries (carVelSeries[i]);
    }
    chartVelCar->createDefaultAxes ();
    chartVelCar->setTitle ("Cartesian Velocity");
    QtCharts::QChartView* chartViewVelCar = new QtCharts::QChartView (chartVelCar);
    chartViewVelCar->setRenderHint (QPainter::Antialiasing);

    //Acceleration chart
    //Position
    QtCharts::QChart* chartAcel = new QtCharts::QChart ();
    chartAcel->legend ()->hide ();
    for (int i = 0; i<7;i++)
    {
        chartAcel->addSeries (jointAcelSeries[i]);
    }
    chartAcel->createDefaultAxes ();
    chartAcel->setTitle ("Joint Acceleration");
    QtCharts::QChartView* chartViewAcel = new QtCharts::QChartView (chartAcel);
    chartViewAcel->setRenderHint (QPainter::Antialiasing);
    //Cartesian
    QtCharts::QChart* chartAcelCar = new QtCharts::QChart ();
    chartAcelCar->legend ()->hide ();
    for (int i = 0; i<3;i++)
    {
        chartAcelCar->addSeries (carAcelSeries[i]);
    }
    chartAcelCar->createDefaultAxes ();
    chartAcelCar->setTitle ("Cartesian Acceleration");
    QtCharts::QChartView* chartViewAcelCar = new QtCharts::QChartView (chartAcelCar);
    chartViewAcelCar->setRenderHint (QPainter::Antialiasing);

    //Saving charts
    QPixmap pPos = chartViewPos->grab();
    pPos.save("Path_pos_joint.png", "PNG");
    QPixmap pVel = chartViewVel->grab();
    pVel.save("Path_vel_joint.png", "PNG");
    QPixmap pAcel = chartViewAcel->grab();
    pAcel.save("Path_acel_joint.png", "PNG");
    QPixmap pPosCar = chartViewPosCar->grab();
    pPosCar.save("Path_pos_car.png", "PNG");
    QPixmap pVelCar = chartViewVelCar->grab();
    pVelCar.save("Path_vel_car.png", "PNG");
    QPixmap pAcelCar = chartViewAcelCar->grab();
    pAcelCar.save("Path_acel_car.png", "PNG");

    int index = _combo_graph->currentIndex();
    unsigned int maxW = 480;
    unsigned int maxH = 640;
    if (index == 0)
    {
        _label->setPixmap (pPos.scaled (maxW, maxH, Qt::KeepAspectRatio));
    } else if (index == 1)
    {
        _label->setPixmap (pVel.scaled (maxW, maxH, Qt::KeepAspectRatio));
    } else if (index == 2)
    {
        _label->setPixmap (pAcel.scaled (maxW, maxH, Qt::KeepAspectRatio));
    }else if (index == 3)
    {
        _label->setPixmap (pPosCar.scaled (maxW, maxH, Qt::KeepAspectRatio));
    }else if (index == 4)
    {
        _label->setPixmap (pVelCar.scaled (maxW, maxH, Qt::KeepAspectRatio));
    } else
    {
        _label->setPixmap (pAcelCar.scaled (maxW, maxH, Qt::KeepAspectRatio));
    }
    
}
*/

//Code from ReachabilityAnalysis Module 3 of the course RoVi2022
std::vector<rw::math::Q> SamplePlugin::getConfigurations(const std::string nameGoal, const std::string nameTcp, rw::models::SerialDevice::Ptr robot, rw::models::WorkCell::Ptr wc, rw::kinematics::State state)
{
    // Get, make and print name of frames
    const std::string robotName = robot->getName();
    const std::string nameRobotBase = robotName + "." + "Base";
    const std::string nameRobotTcp = robotName + "." + "TCP";

    // Find frames and check for existence
    rw::kinematics::Frame* frameGoal = wc->findFrame(nameGoal);
    rw::kinematics::Frame* frameTcp = wc->findFrame(nameTcp);
    rw::kinematics::Frame* frameRobotBase = wc->findFrame(nameRobotBase);
    rw::kinematics::Frame* frameRobotTcp = wc->findFrame(nameRobotTcp);
    if(frameGoal==NULL || frameTcp==NULL || frameRobotBase==NULL || frameRobotTcp==NULL)
    {
        std::cout << " ALL FRAMES NOT FOUND:" << std::endl;
        std::cout << " Found \"" << nameGoal << "\": " << (frameGoal==NULL ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameTcp << "\": " << (frameTcp==NULL ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotBase << "\": " << (frameRobotBase==NULL ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotTcp << "\": " << (frameRobotTcp==NULL ? "NO!" : "YES!") << std::endl;
    }

    // Make "helper" transformations
    rw::math::Transform3D<> frameBaseTGoal = rw::kinematics::Kinematics::frameTframe(frameRobotBase, frameGoal, state);
    rw::math::Transform3D<> frameTcpTRobotTcp = rw::kinematics::Kinematics::frameTframe(frameTcp, frameRobotTcp, state);

    // get grasp frame in robot tool frame
    rw::math::Transform3D<> targetAt = frameBaseTGoal * frameTcpTRobotTcp;

    rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSovler = rw::common::ownedPtr( new rw::invkin::ClosedFormIKSolverUR(robot, state) );
    return closedFormSovler->solve(targetAt, state);


}

void SamplePlugin::testReachability()
{
	// find relevant frames
	rw::kinematics::MovableFrame::Ptr bottleFrame = _wc->findFrame<rw::kinematics::MovableFrame>("Bottle");
	if(NULL==bottleFrame){
		RW_THROW("COULD not find movable frame Cylinder ... check model");
		return -1;
	}

	rw::models::SerialDevice::Ptr robotUR5 = _wc->findDevice<rw::models::SerialDevice>("UR-6-85-5-A");
	if(NULL==robotUR5){
		RW_THROW("COULD not find device UR5 ... check model");
		return -1;
	}

	rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));


	// get the default state
	State state = _wc->getDefaultState();
	std::vector<rw::math::Q> collisionFreeSolutions;
	std::vector<rw::math::Q> allSolutions;

	for(double rollAngle=0; rollAngle<360.0; rollAngle+=1.0){ // for every degree around the roll axis

		bottleFrame->moveTo(
				rw::math::Transform3D<>(
						rw::math::Vector3D<>(bottleFrame->getTransform(state).P()),
						rw::math::RPY<>(rollAngle*rw::math::Deg2Rad,0,0)
						)
				, state);


		std::vector<rw::math::Q> solutions = getConfigurations("GraspTarget", "GraspTCP", robotUR5, _wc, state);

		for(unsigned int i=0; i<solutions.size(); i++){
			// set the robot in that configuration and check if it is in collision
			robotUR5->setQ(solutions[i], state);
			allSolutions.push_back(solutions[i]);
			if( !detector->inCollision(state,NULL,true) ){
				collisionFreeSolutions.push_back(solutions[i]); // save it
				break; // we only need one
			}
		}
	}

	std:: cout << "Current position of the robot vs object to be grasped has: "
			   << collisionFreeSolutions.size()
			   << " collision-free inverse kinematics solutions!" << std::endl;
			   std:: cout << "all solutions number "
			   << allSolutions.size()
			   << " !" << std::endl;


	// visualize them
	TimedStatePath tStatePath;
	double time=0;
	for(unsigned int i=0; i<collisionFreeSolutions.size(); i++){
		robotUR5->setQ(collisionFreeSolutions[i], state);
		tStatePath.push_back(TimedState(time,state));
		time+=0.01;
	}

	TimedStatePath tStatePath2;
	time=0;
	for(unsigned int i=0; i<allSolutions.size(); i++){
		robotUR5->setQ(allSolutions[i], state);
		tStatePath2.push_back(TimedState(time,state));
		time+=0.01;
	}

	rw::loaders::PathLoader::storeTimedStatePath(*_wc, tStatePath, "../../WorkCell/ReachabilityAnalysis.rwplay");
	//rw::loaders::PathLoader::storeTimedStatePath(*_wc, tStatePath2, "../../scene/all.rwplay");

}

void SamplePlugin::get25DImage ()
{
    if (_framegrabber25D != NULL) {
        for (size_t i = 0; i < _cameras25D.size (); i++) {
            // Get the image as a RW image
            Frame* cameraFrame25D = _wc->findFrame (_cameras25D[i]);    // "Camera");
            _framegrabber25D->grab (cameraFrame25D, _state);

            // const Image& image = _framegrabber->getImage();

            const rw::geometry::PointCloud* img = &(_framegrabber25D->getImage ());

            std::ofstream output (_cameras25D[i] + ".pcd");
            output << "# .PCD v.5 - Point Cloud Data file format\n";
            output << "FIELDS x y z\n";
            output << "SIZE 4 4 4\n";
            output << "TYPE F F F\n";
            output << "WIDTH " << img->getWidth () << "\n";
            output << "HEIGHT " << img->getHeight () << "\n";
            output << "POINTS " << img->getData ().size () << "\n";
            output << "DATA ascii\n";
            for (const auto& p_tmp : img->getData ()) {
                rw::math::Vector3D< float > p = p_tmp;
                output << p (0) << " " << p (1) << " " << p (2) << "\n";
            }
            output.close ();
        }
    }
}

void SamplePlugin::p2pInter(Q start, Q end, int steps)
{
    for(int i = 0; i < steps + 1; i++)
    {
        _path.push_back(start + ((i*1.0)/steps)*(end-start));
    }
}

void SamplePlugin::p2pInterBlend(Q start, Q end, int steps, int blend)
{
    double accel = 1.0/(blend*(steps-blend));
    for(int i = 0; i < steps +1; i++)
    {
        if(i<blend)
        {
            //accel phase
            _path.push_back(start+0.5*accel*i*i*(end-start));
        } else if (i>steps-blend)
        {
            //decel phase
            _path.push_back(end-0.5*accel*(steps-i)*(steps-i)*(end-start));
        } else
        {
            //coasting phase
            _path.push_back(start+accel*blend*(i-blend/2)*(end-start));
        }
    }
}

void SamplePlugin::getImage ()
{
    if (_framegrabber != NULL) {
        for (size_t i = 0; i < _cameras.size (); i++) {
            // Get the image as a RW image
            Frame* cameraFrame = _wc->findFrame (_cameras[i]);    // "Camera");
            _framegrabber->grab (cameraFrame, _state);

            const rw::sensor::Image* rw_image = &(_framegrabber->getImage ());

            // Convert to OpenCV matrix.
            cv::Mat image = cv::Mat (rw_image->getHeight (),
                                     rw_image->getWidth (),
                                     CV_8UC3,
                                     (rw::sensor::Image*) rw_image->getImageData ());

            // Convert to OpenCV image
            Mat imflip, imflip_mat, image_mat;
            cv::flip (image, imflip, 1);
            cv::cvtColor (imflip, imflip_mat, COLOR_RGB2BGR);
            cv::cvtColor (image, image_mat, COLOR_RGB2BGR);

            cv::imwrite (_cameras[i] + ".png", imflip_mat);

            // Show in QLabel
            QImage img (imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
            QPixmap p         = QPixmap::fromImage (img);
            unsigned int maxW = 480;
            unsigned int maxH = 640;
            _label->setPixmap (p.scaled (maxW, maxH, Qt::KeepAspectRatio));
        }
    }
    /*
    std::cout << "Camera_Right" << std::endl;
    printProjectionMatrix ("Camera_Right");
    std::cout << "Camera_Left" << std::endl;
    printProjectionMatrix ("Camera_Left");
    */
    
}

void SamplePlugin::computeDisparity()
{
    //Partially from disparity ex
    cv::Mat imgL = cv::imread("Camera_Left.png");
    cv::Mat imgR = cv::imread("Camera_Right.png");

    cv::cvtColor(imgL, imgL, cv::COLOR_BGR2GRAY);
    cv::cvtColor(imgR, imgR, cv::COLOR_BGR2GRAY);
    cv::imwrite ("Camera_Left_Gray.png", imgL);
    cv::imwrite ("Camera_Right_Gray.png", imgR);

    cv::Mat disp;
    //                                               nDisparities,Blocksize
    cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create(_nDisparity->value(),_windowSize->value());
    sbm->compute(imgL,imgR,disp);

    //save disp
    cv::imwrite ("disparity.png", disp);

    //define Q matrix for reproject
	cv::Mat Q = cv::Mat::zeros(4, 4, CV_64F);
	Q.at<double>(0, 0) = 1;
	Q.at<double>(1, 1) = 1;
	Q.at<double>(0, 3) = -640/2; //center x
	Q.at<double>(1, 3) = -480/2; // center y
	Q.at<double>(2, 3) = 514.6816609222941; //focal length tan(FOVY/2) in rad FOVY 50 deg THIS IS PROP WRONG!!!!!!!!!!!!!!!!!
	Q.at<double>(3, 2) = -10.0; // 1/Tx
    cv::Mat points;
    disp.convertTo(disp, CV_32FC1);
    disp = disp/16;
    cv::reprojectImageTo3D(disp, points, Q, true);
    
    pclPoint p_default;
    pclCloud::Ptr dst(new pclCloud());
    for (int i = 0; i < points.rows; i++) {
        for (int j = 0; j < points.cols; j++) {
            cv::Vec3f xyz = points.at<cv::Vec3f>(i, j);
            // Check if points are too far away, if not take them into account
            if (fabs(xyz[2]) < 1) {
                pclPoint pn;
                pn.x = xyz[0];
                pn.y = -1.0*xyz[1];
                pn.z = xyz[2];
                dst->push_back(pn);
            }
        }
    }

    pclCloud::Ptr cloud(new pclCloud());
    Transform3D  transformCloud = _table_frame->fTf(_cam_left_frame,_state);
    pcl::transformPointCloud(*dst, *cloud, transformCloud.e()); 

    pcl::io::savePCDFileASCII("cloud.pcd", *cloud);
    // from point cloud lecture ex
    // Voxel grid
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud (cloud);
    vox.setLeafSize (0.005f, 0.005f, 0.005f);
    vox.filter (*cloud);
    pcl::io::savePCDFileASCII("cloud_voxel.pcd", *cloud);
    // Outlier removal
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> out;
    out.setInputCloud (cloud);
    out.setMeanK (50); //amount of points used
    out.setStddevMulThresh (0.001);
    out.filter (*cloud);
    pcl::io::savePCDFileASCII("cloud_outlier.pcd", *cloud);
    // spatial filtering
    pclCloud::Ptr cloud2(new pclCloud());
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.1 , 0.5);
    pass.filter (*cloud2);
    pass.setInputCloud (cloud2);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-0.38 , 0.38);
    pass.filter (*cloud2);
    pass.setInputCloud (cloud2);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (0.32 , 0.58);
    pass.filter (*cloud2);
    pcl::io::savePCDFileASCII("cloud_spatial.pcd", *cloud2);

    //normalize disp
    cv::Mat disp_display;
    cv::normalize(disp,disp_display,0.,255.,cv::NORM_MINMAX,CV_8U);
    // Show in QLabel
    QImage img (disp_display.data, disp_display.cols, disp_display.rows, disp_display.step, QImage::Format_Grayscale8);
    QPixmap p = QPixmap::fromImage (img);
    unsigned int maxW = 480;
    unsigned int maxH = 640;
    _label->setPixmap (p.scaled (maxW, maxH, Qt::KeepAspectRatio));
}

void SamplePlugin::locateObject ()
{
    

    pclCloud::Ptr cloud(new pclCloud());
    pcl::io::loadPCDFile("cloud_spatial.pcd",*cloud);
    
    //source https://pcl.readthedocs.io/en/latest/cluster_extraction.html
    // Creating the KdTree object for the search method of the extraction
    
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
    std::cout << "cluster indices: " << cluster_indices.size() << std::endl;

    //source https://pcl.readthedocs.io/projects/tutorials/en/pcl-1.11.0/alignment_prerejective.html
    // Point clouds
    PointCloudT::Ptr object (new PointCloudT);
    PointCloudT::Ptr object_aligned (new PointCloudT);
    PointCloudT::Ptr scene (new PointCloudT);
    FeatureCloudT::Ptr object_features (new FeatureCloudT);
    FeatureCloudT::Ptr scene_features (new FeatureCloudT);

    pcl::io::loadPCDFile("../src/bottle.pcd",*object);

    int j = 0;
    for (const auto& cluster : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& idx : cluster.indices) {
            cloud_cluster->push_back((*cloud)[idx]);
        } 
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        pcl::io::savePCDFileASCII(ss.str (), *cloud_cluster);


        pcl::copyPointCloud(*cloud_cluster, *scene);
        // Estimate normals for scene
        pcl::console::print_highlight ("Estimating scene normals...\n");
        pcl::NormalEstimationOMP<PointNT,PointNT> nest;
        nest.setRadiusSearch (0.01);
        nest.setInputCloud (scene);
        nest.compute (*scene);
        
        // Estimate features
        pcl::console::print_highlight ("Estimating features...\n");
        FeatureEstimationT fest;
        fest.setRadiusSearch (0.025);
        fest.setInputCloud (object);
        fest.setInputNormals (object);
        fest.compute (*object_features);
        fest.setInputCloud (scene);
        fest.setInputNormals (scene);
        fest.compute (*scene_features);
        
        // Perform alignment
        pcl::console::print_highlight ("Starting alignment...\n");
        pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
        align.setInputSource (object);
        align.setSourceFeatures (object_features);
        align.setInputTarget (scene);
        align.setTargetFeatures (scene_features);
        align.setMaximumIterations (50000); // Number of RANSAC iterations
        align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
        align.setCorrespondenceRandomness (5); // Number of nearest features to use
        align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
        align.setMaxCorrespondenceDistance (2.5f * 0.005f); // Inlier threshold
        align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
        {
            pcl::ScopeTime t("Alignment");
            align.align (*object_aligned);
        }
        
        if (align.hasConverged ())
        {
            // Print results
            printf ("\n");
            Eigen::Matrix4f transformation = align.getFinalTransformation ();
            pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
            pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
            pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
            pcl::console::print_info ("\n");
            pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
            pcl::console::print_info ("\n");
            pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());
            /*
            // Show alignment
            pcl::visualization::PCLVisualizer visu("Alignment");
            visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
            visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
            visu.spin ();
            */
        }
        else
        {
            pcl::console::print_error ("Alignment failed!\n");
        }
        j++;
    }
}

void SamplePlugin::locateObject_imagebased ()
{
    //get the captured image
    cv::Mat imgL = cv::imread("Camera_Left.png");
    cv::Mat imgR = cv::imread("Camera_Right.png");

    //convert into grayscale and save
    cv::cvtColor(imgL, imgL, cv::COLOR_BGR2GRAY);
    cv::cvtColor(imgR, imgR, cv::COLOR_BGR2GRAY);
    cv::imwrite ("Camera_Left_Gray.png", imgL);
    cv::imwrite ("Camera_Right_Gray.png", imgR);

    //undistort images 

    //imagebased estimation of pose
}

void SamplePlugin::timer ()
{
    if (0 <= _step && (size_t) _step < _path.size ()) {
        for (int i = 0; i<6;i++)
        {
            temp(i) = _path.at(_step)(i);
        }
        gripper_temp(0) = _path.at(_step)(6);
        _gripper->setQ(gripper_temp ,_state);
        _device->setQ (temp, _state);
        if (_step == _grasp_step)
        {
            Transform3D gripper_to_bottle = _gripper_frame->fTf(_bottle_frame,_state);
            _bottle_frame->setTransform(gripper_to_bottle ,_state);
            _bottle_frame->attachTo(_gripper_frame,_state);
        } else if (_step == _release_step)
        {
            Transform3D table_to_bottle = _table_frame->fTf(_bottle_frame,_state);
            _bottle_frame->setTransform(table_to_bottle ,_state);
            _bottle_frame->attachTo(_table_frame,_state);
        }
        getRobWorkStudio ()->setState (_state);
        _step++;
    }
}

void SamplePlugin::stateChangedListener (const State& state)
{
    _state = state;
}

bool SamplePlugin::checkCollisions (Device::Ptr device, const State& state,
                                    const CollisionDetector& detector, const Q& q)
{
    State testState;
    CollisionDetector::QueryResult data;
    bool colFrom;

    testState = state;
    device->setQ (q, testState);
    colFrom = detector.inCollision (testState, &data);
    if (colFrom) {
        cerr << "Configuration in collision: " << q << endl;
        cerr << "Colliding frames: " << endl;
        FramePairSet fps = data.collidingFrames;
        for (FramePairSet::iterator it = fps.begin (); it != fps.end (); it++) {
            cerr << (*it).first->getName () << " " << (*it).second->getName () << endl;
        }
        return false;
    }
    return true;
}

//code from SamplePlugin (provided for the project)
void SamplePlugin::createPathRRTConnect (Q from, Q to, double extend, double maxTime)
{
    _device->setQ (from, _state);
    getRobWorkStudio ()->setState (_state);
    CollisionDetector detector (_wc, ProximityStrategyFactory::makeDefaultCollisionStrategy ());
    PlannerConstraint constraint = PlannerConstraint::make (&detector, _device, _state);
    QSampler::Ptr sampler        = QSampler::makeConstrained (QSampler::makeUniform (_device),
                                                       constraint.getQConstraintPtr ());
    QMetric::Ptr metric          = MetricFactory::makeEuclidean< Q > ();
    QToQPlanner::Ptr planner =
        RRTPlanner::makeQToQPlanner (constraint, sampler, metric, extend, RRTPlanner::RRTConnect);

    _path_rrt.clear ();
    if (!checkCollisions (_device, _state, detector, from))
        cout << from << " is in colission!" << endl;
    if (!checkCollisions (_device, _state, detector, to))
        cout << to << " is in colission!" << endl;
    ;
    Timer t;
    t.resetAndResume ();
    planner->query (from, to, _path_rrt, maxTime);
    t.pause ();

    if (t.getTime () >= maxTime) {
        cout << "Notice: max time of " << maxTime << " seconds reached." << endl;
    }

    const int duration = 10;

    if (_path_rrt.size () == 2) {    // The interpolated path between Q start and Q goal is collision
                                 // free. Set the duration with respect to the desired velocity
        LinearInterpolator< Q > linInt (from, to, duration);
        QPath tempQ;
        for (int i = 0; i < duration + 1; i++) {
            tempQ.push_back (linInt.x (i));
        }

        _path_rrt = tempQ;
    }
}

void SamplePlugin::printProjectionMatrix (std::string frameName)
{
    Frame* cameraFrame = _wc->findFrame (frameName);
    if (cameraFrame != NULL) {
        if (cameraFrame->getPropertyMap ().has ("Camera")) {
            // Read the dimensions and field of view
            double fovy;
            int width, height;
            std::string camParam = cameraFrame->getPropertyMap ().get< std::string > ("Camera");
            std::istringstream iss (camParam, std::istringstream::in);
            iss >> fovy >> width >> height;

            double fovy_pixel = height / 2 / tan (fovy * (2 * M_PI) / 360.0 / 2.0);

            Eigen::Matrix< double, 3, 4 > KA;
            KA << fovy_pixel, 0, width / 2.0, 0, 0, fovy_pixel, height / 2.0, 0, 0, 0, 1, 0;

            std::cout << "Intrinsic parameters:" << std::endl;
            std::cout << KA << std::endl;

            Transform3D<> camPosOGL   = cameraFrame->wTf (_state);
            Transform3D<> openGLToVis = Transform3D<> (RPY<> (-Pi, 0, Pi).toRotation3D ());
            Transform3D<> H           = inverse (camPosOGL * inverse (openGLToVis));

            std::cout << "Extrinsic parameters:" << std::endl;
            std::cout << H.e () << std::endl;

            
            //define cameraMatrix 
            cv::Mat cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);
            cameraMatrix.at<double>(0, 0) = fovy_pixel;
            cameraMatrix.at<double>(1, 1) = fovy_pixel;
            cameraMatrix.at<double>(0, 2) = width/2.0;
            cameraMatrix.at<double>(1, 2) = height/2.0;
            cameraMatrix.at<double>(2, 2) = 1;

            std::cout << "Camera Matrix: " << cameraMatrix << std::endl;
            cv::Mat distCoeffs = cv::Mat::zeros(4,1,CV_64F);
            std::cout << "Distortion coefficents: " << distCoeffs << std::endl;
            cv::Size imageSize = cv::Size(height,width);
            std::cout << "Image size: " << imageSize << std::endl;

            cv::Mat R = cv::Mat::zeros(3, 3, CV_64F);
            R.at<double>(0, 0) = 1;
            R.at<double>(1, 1) = 1;
            R.at<double>(2, 2) = 1;
            std::cout << "Rotation matrix: " << R << std::endl;

            cv::Mat T = cv::Mat::zeros(3, 1, CV_64F);
            T.at<double>(0, 0) = 0.1;
            std::cout << "Translation matrix: " << T << std::endl;

            cv::Mat R1,R2,P1,P2,Q;
            int flag(0);
            double alpha(0);
            cv::Size newImgSize(0,0);
            cv::Rect ROI1(0,0,0,0), ROI2(0,0,0,0);
        //  cv::stereoRectify(IA,           IA,         IA,           IA,         Size,      IA,IA,OA, OA, OA, OA, OA,int, double, cv::Size, cv::Rect*, cv::Rect*)
            cv::stereoRectify(cameraMatrix, distCoeffs, cameraMatrix, distCoeffs, imageSize, R, T, R1, R2 ,P1 ,P2 ,Q, flag, alpha, newImgSize,&ROI1,&ROI2);

            std::cout << "Q: " << Q << std::endl;
            std::cout << "newImage size: " << newImgSize << std::endl;
            
        }
    }
}
