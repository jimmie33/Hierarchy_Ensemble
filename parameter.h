#ifndef _PARAMETER_
#define _PARAMETER_

#define RESULT_OUTPUT_XML_FILE "temp.xml"
#define GT_XML_FILE "TownCentre_GT.xml"

extern double AUX_phi1;
extern double AUX_phi2;
extern double AUX_gamma1;
extern double AUX_gamma2;
extern double AUX_N_max;
extern double AUX_K;
extern double AUX_kf_process;
extern double AUX_kf_measure;


// multi-object level tracking parameter
const static int TRACKER_NUM=60;
const static int TRA_GROUP_SIZE=10*AUX_N_max;
const static double GT_TO_DETECTION_RATIO=1.0;//0.64
const static double TRACKING_TO_GT_RATIO=0.5;//0.5
const static double TRACKING_TO_DETECTION=GT_TO_DETECTION_RATIO*TRACKING_TO_GT_RATIO;

#define HIST_MATCH_THRESH_CONT 0.4//

//single object level parameter
#define HIST_MATCH_UPDATE 0.01
#define SCALE_UPDATE_RATE 0.4
#define _FRAME_RATE 25
#define TIME_WINDOW_SIZE 11


//#define RECORD_DETECTION//for detection output
//#define  DETECION_RESPONSE_THRESH 0.0
//hog detection 
const static double HOG_DETECT_WIN_RATIO=1.667;//1.667
//#define XML_RATIO 1.0//for xml detector 0.6 for pets
//for xml detector
//#define DETECTION_WITH_CONFIDENCE
//#define THRESH_CONFIDENCE 0.0//0.4 for 2, 0.6 for 1

#endif