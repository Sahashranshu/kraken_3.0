#include <cams/cams.h>
#include <string>

int camMsg = 0;  //0 for frontcam 1 for bottomcam
int cameraNo ;
bool camOpen = false;
int flag = 0;

VideoCapture cam;

void msgCallback(const std_msgs::String::ConstPtr& msg)
{
    camMsg = atoi(msg->data.c_str());

    if(camMsg == cameraNo && !camOpen)
    {
        if(cam.open(cameraNo))
        {
            cout << "FrontCam opened successfully." << endl;
            camOpen = true;
        }
    }
    else
        if(camMsg != cameraNo && camOpen)
        {
            cam.release();
            cout << "FrontCam closed successfully." << endl;
            camOpen = false;
        }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "frontcam");

    ros::NodeHandle _nh;
    image_transport::ImageTransport _it(_nh);
    image_transport::Publisher _image_pub = _it.advertise(topics::CAMERA_FRONT_RAW_IMAGE, 1);
    ros::Subscriber _sub = _nh.subscribe(topics::CAMERA_CAM_SWITCH, 1, msgCallback);

    sensor_msgs::ImagePtr _publishImage;
    cv_bridge::CvImage _image;
    _image.encoding = "bgr8";
    
   	camInterface fc;
    if (argc >= 2)
    {
    	//making it feasible for user to enter both GUID or cameraNo at command.
  		if(argc==3){
	  		string type = argv[2];
	    	if(type == "GUID"){
	    		cameraNo = fc.GUID2Index((uint64_t)strtoull(argv[1], NULL, 16));
	    		if(cameraNo<0){
					cout<<"Failed to determine index with the given GUID.\n";
					ros::shutdown();
			    }
			    ROS_INFO_STREAM("cameraNo: "<<cameraNo);
			    fc.printCamDetails("bottomCamSet.st");
			    fc.printCamFeatureList("bottomCamFeatureList.ls");
                fc.setCamParameters("bottomCamSettings.st");
                cam.open(cameraNo);
                camOpen = true;
	    	}
	    }
    	else{
	        cameraNo = atoi(argv[1]);
	        camInterface fc;

	        //comment this line if you are using webcam.
	        //fc.printCamDetails(cameraNo, "bottomCamSet.st");
			//fc.printCamFeatureList("bottomCamFeatureList.ls");
            //fc.setCamParameters("bottomCamSettings.st");
	        
	        cam.open(cameraNo);
	        camOpen = true;
	    }
    }

    // frontcam GUID:		frontcam GUID (AVT Guppy PRO F046C): 0x000A4701120AC45A
    else
    {
    	cameraNo = fc.GUID2Index((uint64_t)0x000A4701120AC45A);
        fc.printCamDetails("frontCamSet.st");
		if(cameraNo<0){
			cout<<"Failed to determine index. Make sure the Guppy Pro is connected and then try again.\n";
			ros::shutdown();
	    }
        ROS_INFO_STREAM("cameraNo: "<<cameraNo);
        fc.printCamDetails("bottomCamSet.st");
		fc.printCamFeatureList("bottomCamFeatureList.ls");
        fc.setCamParameters("bottomCamSettings.st");
        cam.open(cameraNo);
        camOpen = true;
    }

    ros::Rate _looprate(10);

    while(ros::ok())
    {
        if(camOpen)
        {
            cam >> _image.image;

            _publishImage = _image.toImageMsg();

            _image_pub.publish(_publishImage);
        }

        ros::spinOnce();
        _looprate.sleep();
    }

    return 0;
}
