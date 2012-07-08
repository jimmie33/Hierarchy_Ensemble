/*************************************************************
*	Implemetation of the multi-person tracking system described in paper
*	"Online Multi-person Tracking by Tracker Hierarchy", Jianming Zhang, 
*	Liliana Lo Presti, Stan Sclaroff, AVSS 2012
*	http://www.cs.bu.edu/groups/ivc/html/paper_view.php?id=268
*
*	Copyright (C) 2012 Jianming Zhang
*
*	This program is free software: you can redistribute it and/or modify
*	it under the terms of the GNU General Public License as published by
*	the Free Software Foundation, either version 3 of the License, or
*	(at your option) any later version.
*
*	This program is distributed in the hope that it will be useful,
*	but WITHOUT ANY WARRANTY; without even the implied warranty of
*	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*	GNU General Public License for more details.
*
*	You should have received a copy of the GNU General Public License
*	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*	If you have problems about this software, please contact: jmzhang@bu.edu
***************************************************************/


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