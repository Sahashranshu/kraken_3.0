#include <cams/cams.h>
#include <string>


int cameraNo;
int camMsg = 0;  //0 for frontcam 1 for bottom cam
bool camOpen = false;
int flag = 0;

VideoCapture cam;

void msgCallback(const std_msgs::String::ConstPtr& msg)
{
    camMsg = atoi(msg->data.c_str());

    if(camMsg == cameraNo && !camOpen)
    {
	cout<<"inside msg callback\n";
        if(cam.open(cameraNo))
        {
            cout << "BottomCam opened successfully." << endl;
            camOpen = true;
        }
    }
    else
        if(camMsg != cameraNo && camOpen)
        {
            cam.release();
            cout << "BottomCam closed successfully." << endl;
            camOpen = false;
        }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bottomcam");
    ros::NodeHandle _nh;
    image_transport::ImageTransport _it(_nh);
    image_transport::Publisher _image_pub = _it.advertise(topics::CAMERA_BOTTOM_RAW_IMAGE, 1);
    ros::Subscriber _sub = _nh.subscribe(topics::CAMERA_CAM_SWITCH, 1, msgCallback);

    sensor_msgs::ImagePtr _publishImage;
    cv_bridge::CvImage _image;
    _image.encoding = "bgr8";
    camInterface bc;

    if (argc >= 2)
    {	
    	//making it feasible for user to enter both GUID or cameraNo at command.
  		if(argc==3){
	  		string type = argv[2];
	    	if(type == "GUID"){
	    		cameraNo = bc.GUID2Index((uint64_t)strtoull(argv[1], NULL, 16));
	    		if(cameraNo<0){
					cout<<"Failed to determine index with the given GUID.\n";
					ros::shutdown();
			    }
				ROS_INFO_STREAM("cameraNo: "<<cameraNo);
				bc.printCamDetails("bottomCamDetails.dt");
                bc.printCamFeatureList("bottomCamFeatureList.ls");
                bc.setCamParameters("bottomCamSettings.st");
				cam.open(cameraNo);
		        camOpen = true;
	    	}
    	}
    	else{
	        cameraNo = atoi(argv[1]);

	        //comment this line if you are using webcam.
	        //bc.printCamDetails(cameraNo, "bottomCamSet.st");
            //bc.printCamDetails(cameraNo, "bottomCamDetails.dt");
            //bc.printCamFeatureList(cameraNo, "bottomCamFeatureList.ls");
            //bc.setCamParameters(cameraNo, "bottomCamSettings.st")
	        
	        cam.open(cameraNo);
	        camOpen = true;
	    }
    }
    
    // GUID for bottomcam (Basler A601fc):	0x00305300013A66F1	GUID for frontcam:
    else
    {
    	cameraNo = bc.GUID2Index((uint64_t)0x00305300013A66F1);
    	if(cameraNo<0){
			cout<<"Failed to determine index. Make sure the bottomcam Basler is connected and then try again.\n";
			ros::shutdown();
    	}
    	bc.printCamDetails("bottomCamSet.st");
        bc.printCamFeatureList("bottomCamFeatureList.ls");
        bc.setCamParameters("bottomCamSettings.st");
    	ROS_INFO_STREAM("cameraNo: "<<cameraNo);
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
