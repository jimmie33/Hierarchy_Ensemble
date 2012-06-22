#ifndef APP_TEMPLATE_H
#define APP_TEMPLATE_H

#include <list>

#include "opencv2/opencv.hpp"

#include "util.h"
#include "parameter.h"


#define BIN_NUMBER 32 
static int hSize[]={BIN_NUMBER,BIN_NUMBER}; //2D histogram

/*
Appearance Template:
Each appearance template contains a 2D histogram, which is coded by
"int channel[2]", representing the selected channel, and "Mat hist", the
corresponding 2D histogram.
*/


class AppTemplate 
{
	
public:
	AppTemplate(const AppTemplate& tracker);
	AppTemplate(const Mat* frame_set,      const Rect iniWin,              int ID);
	//						[frame in RGB,HSV,Lab]  [initial detection window]  	       

	// calculate back-projection map
	void calcBP(const Mat* frame_set, Mat& occ_map,    Rect ROI); 
	//                                                            [occupancy map]  
	void calcScore(Rect b_inner,Rect b_outer);//all argument is relative to confidence_map roi
	
	inline Mat& getConfidenceMap(){return confidence_map;}
	inline Point2f getShiftVector(){return shift_vector;}
	inline double getScore(){return score;}
	inline int getID(){return ID;}

private:
	const int ID;
	int channels[2];
	Mat hist;

	float _hRang[2][2];
	const float* hRange[2];
	
	Mat confidence_map;
	Point2f shift_vector;
	double score;
};

#endif