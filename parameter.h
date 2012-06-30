/*************************************************************
*	Implemetation of the multi-person tracking system described in paper
*	"Online Multi-person Tracking by Tracker Hierarchy", Jianming Zhang, 
*	Liliana Lo Presti, Stan Sclaroff, AVSS 2012
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


#ifndef _PARAMETER_
#define _PARAMETER_

#define RESULT_OUTPUT_XML_FILE "output.xml"
#define GT_XML_FILE "TownCentre_GT.xml"

// multi-object level tracking parameter
const static int TRACKER_NUM=60;
const static int TRA_GROUP_SIZE=10;
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