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
#include <fstream>
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
		bool getCamDetails(uint64_t);
		bool getCamDetails(int);
		bool getCamDetails();
		bool printCamDetails(uint64_t, const char*);
		bool printCamDetails(int, const char*);
		bool printCamDetails(const char*);
		///*need parameters from testing. Not functional yet.
		bool printCamFeatureList(uint64_t, const char*);
		bool printCamFeatureList(int, const char*);
		bool printCamFeatureList(const char*);
		bool setCamParameters(uint64_t, const char*);
		bool setCamParameters(const char*);
		bool setCamParameters(int, const char*);
} camInterface;

//constructor function with GUID index.
camInterface::__camInterface(uint64_t GUID){
	camList = 0;
	guid = GUID;
	if(!dc1394.dc){
	  ROS_INFO_STREAM("Failed initializing the driver structure.\n");
	  ros::shutdown();
	}
}

camInterface::__camInterface(int index){
	uint64_t GUID = Index2GUID(index);
	if(GUID){
		ROS_INFO_STREAM("Error retrieving GUID\n");
		ros::shutdown();
	}
	camList = 0;
	guid = GUID;
	if(!dc1394.dc){
	  ROS_INFO_STREAM("Failed initializing the driver structure.\n");
	  ros::shutdown();
	}
}

//constructor function without GUID index for manually feeding it in functions.
camInterface::__camInterface(){
	camList = 0;
	if(!dc1394.dc){
	  ROS_INFO_STREAM("Failed initializing the driver structure.\n");
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
	bool result = true;
	if(!guid){
	  ROS_INFO_STREAM("GUID not known already\n");
	  return -1;
	}	
	err = dc1394_camera_enumerate(dc1394.dc, &camList);
	if (err < DC1394_SUCCESS || !camList){
	  ROS_INFO_STREAM("Failed retrieving camera list\n");
	  return -1;
	}

	for(int index=0; index < (int)camList->num; ++index){
		if(guid == camList->ids[index].guid){
			result = false;
			return index;
		}
	}
	if(!result){
	  ROS_INFO_STREAM("No cameras found of given GUID.\n");
	  return -1;
	}
	return -1;
}

//Index to GUID for returning GUID of a specific Index value.
uint64_t camInterface::Index2GUID(int index){
	err = dc1394_camera_enumerate(dc1394.dc, &camList);
	if (err < DC1394_SUCCESS || !camList){
		ROS_INFO_STREAM("Failed retrieving camera list\n");
		return -1;
	}
	if(index > (int)camList->num){
		ROS_INFO_STREAM("Given index exceeds the total number of cameras operational on system. Invalid Index.\n");
		return -1;
	}
	return camList->ids[index].guid;
}

///getCamDetails() - wrapper function for setting camera, with GUID//////
bool camInterface::getCamDetails(uint64_t GUID){
	guid = GUID;
	return getCamDetails();
}

bool camInterface::getCamDetails(int index){
	uint64_t GUID = Index2GUID(index);
	if(GUID){
		ROS_INFO_STREAM("Error retrieving GUID\n");
		ros::shutdown();
	}
	guid = GUID;
	return getCamDetails();
}

//getCamDetails without guid////////////////////////////////////////////
bool camInterface::getCamDetails(){
	if(!guid){
	  ROS_INFO_STREAM("GUID not known already\n");
	  return false;
	}
	//creating new camera structure.
	cam = dc1394_camera_new(dc1394.dc, guid);
	if(!cam){
	  ROS_INFO_STREAM("No cameras found of given GUID.\n");
	  return false;
	}
	//extracting list of features in an array.
	err = dc1394_feature_get_all(cam, camFeatures);
	if(err < DC1394_SUCCESS){
	  ROS_INFO_STREAM("Error extracting list of features from camera\n");
	  return false;
	}
	//extracting bounds 
	err = dc1394_feature_get(cam, camFeatures->feature);
	if(err < DC1394_SUCCESS){
	  ROS_INFO_STREAM("Feature list's end limit reached\n");
	  return false;
	}
	return true;
}

//wrapper function for printing all the camera information.///////////
bool camInterface::printCamDetails(uint64_t GUID, const char* detailsFilename){
	guid = GUID;
	return printCamDetails(detailsFilename);
}

//THIS FUNCTION WILL NOT WORK FOR USB CAMS SUCH AS WEBCAM. GUIDs are only for IEEE1394 complaint cameras.
bool camInterface::printCamDetails(int index, const char* detailsFilename){
	uint64_t GUID = Index2GUID(index);
	if(GUID){
		ROS_INFO_STREAM("Error retrieving GUID\n");
		ros::shutdown();
	}
	guid = GUID;
	return printCamDetails(detailsFilename);
}

//without GUID
bool camInterface::printCamDetails(const char* detailsFilename){
	if(!guid){
	  ROS_INFO_STREAM("GUID not known already\n");
	  return false;
	}
	FILE *file;	
	file = fopen(detailsFilename, "w");
	if(!file){
	  ROS_INFO_STREAM("Error opening file\n");
	  return false;
	}		
	if(!camFeatures){
		if(getCamDetails(guid)){
			ROS_INFO_STREAM("Error retrieving camera details\n");
		}
	}
	err = dc1394_feature_print_all(camFeatures, file);
	if(err < DC1394_SUCCESS){
	    ROS_INFO_STREAM("Error printing feature list\n");
	    return false;
	}
	fclose(file);
	return true;
}

//////////wrapper function for checking available feature of the camera.////////
bool camInterface::printCamFeatureList(uint64_t GUID, const char* featureListFilename){
	guid = GUID;
	return printCamFeatureList(featureListFilename);
}

bool camInterface::printCamFeatureList(int index, const char* featureListFilename){
	uint64_t GUID = Index2GUID(index);
		if(GUID){
			ROS_INFO_STREAM("Error retrieving GUID\n");
			ros::shutdown();
		}
		guid = GUID;
	return printCamFeatureList(featureListFilename);
}

bool camInterface::printCamFeatureList(const char* featureListFilename){
	bool result = true;
	string parameter[] = {"BRIGHTNESS", "EXPOSURE", "SHARPNESS", "WHITE_BALANCE", "HUE", "SATURATION", "GAMMA", "SHUTTER",
		  		          "IRIS", "FOCUS", "TEMPERATURE", "TRIGGER", "TRIGGER_DELAY", "WHITE_SHADING", "FRAME_RATE", "ZOOM",
				          "PAN", "TILT", "OPTICAL_FILTER", "CAPTURE_SIZE", "CAPTURE_QUALITY"};
	if(!guid){
	  ROS_INFO_STREAM("GUID not known already\n");
	  return false;
	}
	ofstream featureListFile(featureListFilename);
	if(!featureListFile){
		ROS_INFO_STREAM("Error opening feature list file\n");
		return false;
	}
	if(!cam){
		cam = dc1394_camera_new(dc1394.dc, guid);
		if(!cam){
		  ROS_INFO_STREAM("No cameras found of given GUID.\n");
		  return false;
		}
	}
	dc1394bool_t check = DC1394_FALSE;
	for(int i=0; i<(int)(DC1394_FEATURE_NUM); ++i){
		err = dc1394_feature_is_present(cam, (dc1394feature_t)(i + DC1394_FEATURE_MIN), &check);
		if(err < DC1394_SUCCESS){
			ROS_INFO_STREAM("Error reading some features\n");
			result = false;
		}
		else{
			if(check == DC1394_TRUE){
				featureListFile<< parameter[i];
				check = DC1394_FALSE;
			}
		}
	}
	featureListFile.close();
	return result;
}

///////////wrapper function for setting the camera//////////////////////////////
bool camInterface::setCamParameters(uint64_t GUID, const char* featureListFilename){
	guid = GUID;
	return setCamParameters(featureListFilename);
}

bool camInterface::setCamParameters(int index, const char* featureListFilename){
	uint64_t GUID = Index2GUID(index);
	if(GUID){
		ROS_INFO_STREAM("Error retrieving GUID\n");
		ros::shutdown();
	}
	guid = GUID;
	return setCamParameters(featureListFilename);
}

//without GUID
bool camInterface::setCamParameters(const char* featureListFilename){
	dc1394error_t errRead = DC1394_SUCCESS;
	bool result = true;
	string parameter[] = {"BRIGHTNESS", "EXPOSURE", "SHARPNESS", "WHITE_BALANCE", "HUE", "SATURATION", "GAMMA", "SHUTTER",
		  		          "IRIS", "FOCUS", "TEMPERATURE", "TRIGGER", "TRIGGER_DELAY", "WHITE_SHADING", "FRAME_RATE", "ZOOM",
				          "PAN", "TILT", "OPTICAL_FILTER", "CAPTURE_SIZE", "CAPTURE_QUALITY"};
	uint32_t featureValue;

	if(!guid){
		ROS_INFO_STREAM("GUID not known already\n");
		return false;
	}
	ifstream settingsFile(featureListFilename);
	if(!settingsFile){
		ROS_INFO_STREAM("Could not open the settings file.\n");
		return false;
	}
	string camParameter;
	if(!cam){
		cam = dc1394_camera_new(dc1394.dc, guid);
		if(!cam){
		  ROS_INFO_STREAM("No cameras found of given GUID.\n");
		  return false;
		}
	}

	while(getline(settingsFile, camParameter)){
		for(int i = 0; i < DC1394_FEATURE_NUM; ++i){
			if(camParameter == parameter[i]){
				//jumping on the next line which is the parameter value of the parameter.
				getline(settingsFile, camParameter);
				err = dc1394_feature_set_absolute_value(cam, (dc1394feature_t)(i + DC1394_FEATURE_MIN), strtof(camParameter.c_str(), NULL));
				if(err < DC1394_SUCCESS){
					ROS_INFO_STREAM("Error setting parameter: "<<parameter[i]<<"\n");
					result = false;
				}
				errRead = dc1394_feature_get_value(cam, (dc1394feature_t)(i + DC1394_FEATURE_MIN), &featureValue);
				if(errRead < 0){
					ROS_INFO_STREAM("Error reading parameter: "<<parameter[i]<<"\n");
					result = false;
				}
				else{
					ROS_INFO_STREAM(parameter[i]<<": "<<featureValue);
				}
			}
		}
	}
	settingsFile.close();
	return result;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

