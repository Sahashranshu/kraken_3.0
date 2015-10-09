#include <stdint.h>
#include <dc1394/camera.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/String.h>
#include <resources/topicHeader.h>
#include <string>


using namespace std;
using namespace cv;

///////////////CLASS FOR INTERFACING CAMERA WITH SYSTEM/////////////////////////////


//structure internally used.//////////////////////////////
typedef struct __CvDC1394{
    __CvDC1394();
    ~__CvDC1394();
    dc1394_t* dc;
} CvDC1394;
CvDC1394::__CvDC1394(){
    dc = dc1394_new();
}
CvDC1394::~__CvDC1394(){
    if (dc)
    dc1394_free(dc);
    dc = 0;
}
static CvDC1394 dc1394;


/* camInterface class currently contains 2 functions. 
1. camInterface() - constructor for feeding in the GUID  
2. GUID2Index(uint64_t) - Obtaining camera index from GUID.*/
typedef class __camInterface{

	private:
		dc1394camera_list_t* camList;
		dc1394error_t err;
		uint64_t guid;
		uint32_t featureValue;
		dc1394camera_t* cam; //stores the information about the camera.
		dc1394featureset_t* camFeatures; //stores all the features of a given camera.
	public:
		__camInterface(uint64_t);
		__camInterface(int);
		__camInterface();
		int GUID2Index(uint64_t);

		//Index2GUID is used internally.
		uint64_t Index2GUID(int);

		int GUID2Index();
		bool getCamSettings(uint64_t);
		bool getCamSettings(int);
		bool getCamSettings();
		bool printCamSettings(uint64_t, const char*);
		bool printCamSettings(int, const char*);
		bool printCamSettings(const char*);
		/*need parameters from testing. Not functional yet.
		bool setCamSettings(uint64_t, const char*);
		bool setCamSettings(const char*);*/
} camInterface;

//constructor function with GUID index.
camInterface::__camInterface(uint64_t GUID){
	camList = 0;
	guid = GUID;
	if(!dc1394.dc){
	  cout<<"Failed initializing the driver structure.\n";
	  ros::shutdown();
	}
}

camInterface::__camInterface(int index){
	uint64_t GUID = Index2GUID(index);
	if(GUID){
		cout<<"Error retrieving GUID\n";
		ros::shutdown();
	}
	camList = 0;
	guid = GUID;
	if(!dc1394.dc){
	  cout<<"Failed initializing the driver structure.\n";
	  ros::shutdown();
	}
}

//constructor function without GUID index for manually feeding it in functions.
camInterface::__camInterface(){
	camList = 0;
	if(!dc1394.dc){
	  cout<<"Failed initializing the driver structure.\n";
	  ros::shutdown();
	}
}

//GUID2Index function returns the index value for corresponding GUID. (with GUID)
int camInterface::GUID2Index(uint64_t GUID){
	guid = GUID;
	return GUID2Index();
}

//GUID2Index function without GUID
int camInterface::GUID2Index(){
	bool result = false;
	if(!guid){
	  cout<<"GUID not known already\n";
	  return false;
	}	
	err = dc1394_camera_enumerate(dc1394.dc, &camList);
	if (err < 0 || !camList){
	  cout<<"Failed retrieving camera list\n";
	  return -1;
	}
	for(int index=0; index < (int)camList->num; ++index){
	  if(guid == camList->ids[index].guid)
	    result = true;
	    return index;
	}
	if(result){
	  cout<<"No cameras found of given GUID.\n";
	  return -1;
	}
	return -1;
}

//Index to GUID for returning GUID of a specific Index value.
uint64_t camInterface::Index2GUID(int index){
	err = dc1394_camera_enumerate(dc1394.dc, &camList);
	if (err < 0 || !camList){
		cout<<"Failed retrieving camera list\n";
		return -1;
	}
	if(index > (int)camList->num){
		cout<<"Given index exceeds the total number of cameras operational on system. Invalid Index.\n";
		return -1;
	}
	return camList->ids[index].guid;
}

///getCamSettings() - wrapper function for setting camera, with GUID//////
bool camInterface::getCamSettings(uint64_t GUID){
	guid = GUID;
	return getCamSettings();
}

bool camInterface::getCamSettings(int index){
	uint64_t GUID = Index2GUID(index);
	if(GUID){
		cout<<"Error retrieving GUID\n";
		ros::shutdown();
	}
	guid = GUID;
	return getCamSettings(GUID);
}

//getCamSettings without guid////////////////////////////////////////////
bool camInterface::getCamSettings(){
	if(!guid){
	  cout<<"GUID not known already\n";
	  return false;
	}
	//creating new camera structure.
	if(!cam){
		cam = dc1394_camera_new(dc1394.dc, guid);
		if(!cam){
		  cout<<"No cameras found of given GUID.\n";
		  return false;
	}
	//extracting list of features in an array.
	err = dc1394_feature_get_all(cam, camFeatures);
	if(err<0){
	  cout<<"Error extracting list of features from camera\n";
	  return false;
	}
	//extracting bounds 
	err = dc1394_feature_get(cam, camFeatures->feature);
	if(err<0){
	  cout<<"Feature list's end limit reached\n";
	  return false;
	}
	return true;
}

//wrapper function for printing all the camera information.///////////
bool camInterface::printCamSettings(uint64_t GUID, const char* camNamePrint){
	guid = GUID;
	return printCamSettings(camNamePrint);
}

//THIS FUNCTION WILL NOT WORK FOR USB CAMS SUCH AS WEBCAM. GUIDs are only for IEEE1394 complaint cameras.
bool camInterface::printCamSettings(int index, const char* camNamePrint){
	uint64_t GUID = Index2GUID(index);
	if(GUID){
		cout<<"Error retrieving GUID\n";
		ros::shutdown();
	}
	guid = GUID;
	return printCamSettings(GUID, camNamePrint);
}

//without GUID
bool camInterface::printCamSettings(const char* camNamePrint){
	if(!guid){
	  cout<<"GUID not known already\n";
	  return false;
	}
	FILE *file;	
	file = fopen(camNamePrint, "w");
	if(!file){
	  cout<<"Error opening file\n";
	  return false;
	}		
	if(!camFeatures){
		getCamSettings(guid);
	}
	err = dc1394_feature_print_all(camFeatures, file);
	if(err<0){
	    cout<<"Error printing feature list\n";
	    return false;
	}
	fclose(file);
	return true;
}

///////////wrapper function for setting the camera//////////////////////////////
bool camInterface::setCamSettings(uint64_t GUID, const char* camNameSet){
	guid = GUID;
	return setCamSettings(camNameSet);
}

//without GUID
bool camInterface::setCamSettings(const char* camNameSet){
	dc1394error_t errRead = 0;
	bool result = true;
	if(!guid){
		cout<<"GUID not known already\n";
		return false;
	}
	ifstream settingsFile(camNameSet);
	if(!settingsFile){
		cout<<"Could not open the settings file.\n";
		return false;
	}
	string camParameter;
	if(!cam){
		cam = dc1394_camera_new(dc1394.dc, guid);
		if(!cam){
		  cout<<"No cameras found of given GUID.\n";
		  return false;
	}

	while(getline(settingsFile, camParameter){
		case(camParameter.c_str()){
			switch "BRIGHTNESS":
				getline(settingsFile, camParameter);
				err = dc1394_feature_set_absolute_value(cams, DC1394_FEATURE_BRIGHTNESS, stof(camParameter, NULL));
				errRead = dc1394_feature_get_value(cams, DC1394_FEATURE_BRIGHTNESS, &featureValue);
				if(errRead == 0)
					ROS::INFO::STREAM("BRIGHTNESS: "<<featureValue);
				else{  
					result = false;
					errRead = 0;
				}
				break;
			switch "EXPOSURE":
				getline(settingsFile, camParameter);
				err = dc1394_feature_set_absolute_value(cams, (1+DC1394_FEATURE_BRIGHTNESS), stof(camParameter, NULL)); 
				errRead = dc1394_feature_get_value(cams, (1+DC1394_FEATURE_BRIGHTNESS), &featureValue);
				if(errRead == 0)
					ROS::INFO::STREAM("EXPOSURE: "<<featureValue);
				else{  
					result = false;
					errRead = 0;
				}
				break;
			switch "SHARPNESS":
				getline(settingsFile, camParameter);
				err = dc1394_feature_set_absolute_value(cams, (2+DC1394_FEATURE_BRIGHTNESS), stof(camParameter, NULL)); 
				errRead = dc1394_feature_get_value(cams, (2+DC1394_FEATURE_BRIGHTNESS), &featureValue);
				if(errRead == 0)
					ROS::INFO::STREAM("SHARPNESS: "<<featureValue);
				else{  
					result = false;
					errRead = 0;
				}
				break;
			switch "WHITE_BALANCE":
				getline(settingsFile, camParameter);
				err = dc1394_feature_set_absolute_value(cams, (3+DC1394_FEATURE_BRIGHTNESS), stof(camParameter, NULL)); 
				errRead = dc1394_feature_get_value(cams, (3+DC1394_FEATURE_BRIGHTNESS), &featureValue);
				if(errRead == 0)
					ROS::INFO::STREAM("WHITE_BALANCE: "<<featureValue);
				else{  
					result = false;
					errRead = 0;
				}
				break;
			switch "HUE":
				getline(settingsFile, camParameter);
				err = dc1394_feature_set_absolute_value(cams, (4+DC1394_FEATURE_BRIGHTNESS), stof(camParameter, NULL)); 
				errRead = dc1394_feature_get_value(cams, (4+DC1394_FEATURE_BRIGHTNESS), &featureValue);
				if(errRead == 0)
					ROS::INFO::STREAM("HUE: "<<featureValue);
				else{  
					result = false;
					errRead = 0;
				}
				break;
			switch "SATURATION":
				getline(settingsFile, camParameter);
				err = dc1394_feature_set_absolute_value(cams, (5+DC1394_FEATURE_BRIGHTNESS), stof(camParameter, NULL)); 
				errRead = dc1394_feature_get_value(cams, (5+DC1394_FEATURE_BRIGHTNESS), &featureValue);
				if(errRead == 0)
					ROS::INFO::STREAM("SATURATION: "<<featureValue);
				else{  
					result = false;
					errRead = 0;
				}
				break;
			switch "GAMMA":
				getline(settingsFile, camParameter);
				err = dc1394_feature_set_absolute_value(cams, (6+DC1394_FEATURE_BRIGHTNESS), stof(camParameter, NULL)); 
				errRead = dc1394_feature_get_value(cams, (6+DC1394_FEATURE_BRIGHTNESS), &featureValue);
				if(errRead == 0)
					ROS::INFO::STREAM("GAMMA: "<<featureValue);
				else{  
					result = false;
					errRead = 0;
				}
				break;
			switch "SHUTTER":
				getline(settingsFile, camParameter);
				err = dc1394_feature_set_absolute_value(cams, (7+DC1394_FEATURE_BRIGHTNESS), stof(camParameter, NULL)); 
				errRead = dc1394_feature_get_value(cams, (7+DC1394_FEATURE_BRIGHTNESS), &featureValue);
				if(errRead == 0)
					ROS::INFO::STREAM("SHUTTER: "<<featureValue);
				else{  
					result = false;
					errRead = 0;
				}
				break;
			switch "GAIN":
				getline(settingsFile, camParameter);
				err = dc1394_feature_set_absolute_value(cams, (8+DC1394_FEATURE_BRIGHTNESS), stof(camParameter, NULL)); 
				errRead = dc1394_feature_get_value(cams, (8+DC1394_FEATURE_BRIGHTNESS), &featureValue);
				if(errRead == 0)
					ROS::INFO::STREAM("GAIN: "<<featureValue);
				else{  
					result = false;
					errRead = 0;
				}
				break;
			switch "IRIS":
				getline(settingsFile, camParameter);
				err = dc1394_feature_set_absolute_value(cams, (9+DC1394_FEATURE_BRIGHTNESS), stof(camParameter, NULL)); 
				errRead = dc1394_feature_get_value(cams, (9+DC1394_FEATURE_BRIGHTNESS), &featureValue);
				if(errRead == 0)
					ROS::INFO::STREAM("IRIS: "<<featureValue);
				else{  
					result = false;
					errRead = 0;
				}
				break;
			switch "FOCUS":
				getline(settingsFile, camParameter);
				err = dc1394_feature_set_absolute_value(cams, (10+DC1394_FEATURE_BRIGHTNESS), stof(camParameter, NULL)); 
				errRead = dc1394_feature_get_value(cams, (10+DC1394_FEATURE_BRIGHTNESS), &featureValue);
				if(errRead == 0)
					ROS::INFO::STREAM("FOCUS: "<<featureValue);
				else{  
					result = false;
					errRead = 0;
				}
				break;
			switch "TEMPERATURE":
				getline(settingsFile, camParameter);
				err = dc1394_feature_set_absolute_value(cams, (11+DC1394_FEATURE_BRIGHTNESS), stof(camParameter, NULL)); 
				errRead = dc1394_feature_get_value(cams, (11+DC1394_FEATURE_BRIGHTNESS), &featureValue);
				if(errRead == 0)
					ROS::INFO::STREAM("TEMPERATURE: "<<featureValue);
				else{  
					result = false;
					errRead = 0;
				}
				break;
			switch "TRIGGER":
				getline(settingsFile, camParameter);
				err = dc1394_feature_set_absolute_value(cams, (12+DC1394_FEATURE_BRIGHTNESS), stof(camParameter, NULL)); 
				errRead = dc1394_feature_get_value(cams, (12+DC1394_FEATURE_BRIGHTNESS), &featureValue);
				if(errRead == 0)
					ROS::INFO::STREAM("TRIGGER: "<<featureValue);
				else{  
					result = false;
					errRead = 0;
				}
				break;
			switch "TRIGGER_DELAY":
				getline(settingsFile, camParameter);
				err = dc1394_feature_set_absolute_value(cams, (13+DC1394_FEATURE_BRIGHTNESS), stof(camParameter, NULL)); 
				errRead = dc1394_feature_get_value(cams, (13+DC1394_FEATURE_BRIGHTNESS), &featureValue);
				if(errRead == 0)
					ROS::INFO::STREAM("TRIGGER_DELAY: "<<featureValue);
				else{  
					result = false;
					errRead = 0;
				}
				break;
			switch "WHITE_SHADING":
				getline(settingsFile, camParameter);
				err = dc1394_feature_set_absolute_value(cams, (14+DC1394_FEATURE_BRIGHTNESS), stof(camParameter, NULL)); 
				errRead = dc1394_feature_get_value(cams, (14+DC1394_FEATURE_BRIGHTNESS), &featureValue);
				if(errRead == 0)
					ROS::INFO::STREAM("WHITE_SHADING: "<<featureValue);
				else{  
					result = false;
					errRead = 0;
				}
				break;
			switch "FRAME_RATE":
				getline(settingsFile, camParameter);
				err = dc1394_feature_set_absolute_value(cams, (15+DC1394_FEATURE_BRIGHTNESS), stof(camParameter, NULL)); 
				errRead = dc1394_feature_get_value(cams, (15+DC1394_FEATURE_BRIGHTNESS), &featureValue);
				if(errRead == 0)
					ROS::INFO::STREAM("FRAME_RATE: "<<featureValue);
				else{  
					result = false;
					errRead = 0;
				}
				break;
			switch "ZOOM":
				getline(settingsFile, camParameter);
				err = dc1394_feature_set_absolute_value(cams, (16+DC1394_FEATURE_BRIGHTNESS), stof(camParameter, NULL)); 
				errRead = dc1394_feature_get_value(cams, (16+DC1394_FEATURE_BRIGHTNESS), &featureValue);
				if(errRead == 0)
					ROS::INFO::STREAM("ZOOM: "<<featureValue);
				else{  
					result = false;
					errRead = 0;
				}
				break;
			switch "PAN":
				getline(settingsFile, camParameter);
				err = dc1394_feature_set_absolute_value(cams, (17+DC1394_FEATURE_BRIGHTNESS), stof(camParameter, NULL)); 
				errRead = dc1394_feature_get_value(cams, (17+DC1394_FEATURE_BRIGHTNESS), &featureValue);
				if(errRead == 0)
					ROS::INFO::STREAM("PAN: "<<featureValue);
				else{  
					result = false;
					errRead = 0;
				}
				break;
			switch "TILT":
				getline(settingsFile, camParameter);
				err = dc1394_feature_set_absolute_value(cams, (18+DC1394_FEATURE_BRIGHTNESS), stof(camParameter, NULL)); 
				errRead = dc1394_feature_get_value(cams, (18+DC1394_FEATURE_BRIGHTNESS), &featureValue);
				if(errRead == 0)
					ROS::INFO::STREAM("TILT: "<<featureValue);
				else{  
					result = false;
					errRead = 0;
				}
				break;
			switch "OPTICAL_FILTER":
				getline(settingsFile, camParameter);
				err = dc1394_feature_set_absolute_value(cams, (19+DC1394_FEATURE_BRIGHTNESS), stof(camParameter, NULL)); 
				errRead = dc1394_feature_get_value(cams, (19+DC1394_FEATURE_BRIGHTNESS), &featureValue);
				if(errRead == 0)
					ROS::INFO::STREAM("OPTICAL_FILTER: "<<featureValue);
				else{  
					result = false;
					errRead = 0;
				}
				break;
			switch "CAPTURE_SIZE":
				getline(settingsFile, camParameter);
				err = dc1394_feature_set_absolute_value(cams, (20+DC1394_FEATURE_BRIGHTNESS), stof(camParameter, NULL)); 
				errRead = dc1394_feature_get_value(cams, (20+DC1394_FEATURE_BRIGHTNESS), &featureValue);
				if(errRead == 0)
					ROS::INFO::STREAM("CAPTURE_SIZE: "<<featureValue);
				else{  
					result = false;
					errRead = 0;
				}
				break;
			switch "CAPTURE_QUALITY":
				getline(settingsFile, camParameter);
				err = dc1394_feature_set_absolute_value(cams, (21+DC1394_FEATURE_BRIGHTNESS), stof(camParameter, NULL)); 
				errRead = dc1394_feature_get_value(cams, (21+DC1394_FEATURE_BRIGHTNESS), &featureValue);
				if(errRead == 0)
					ROS::INFO::STREAM("CAPTURE_QUALITY: "<<featureValue);
				else{  
					result = false;
					errRead = 0;
				}
				break;
		}
	}
	if(err<0){
			cout<<"Not all parameters were successfully updated.\n";
			return false;
		}
		if(result == false){
			cout<<"Not all parameters were successfully read.\n";
			return false;
	}
	return true;
}
///////////////////////////////////////////////////////////////////////////////

