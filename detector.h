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


#ifndef DETECTOR_H
#define DETECTOR_H

#include "opencv2/opencv.hpp"
#include "opencv2/gpu/gpu.hpp"

#include "util.h"
#include "dataReader.h"
#include "parameter.h"

#define HOG 1
#define XML 2

class Detector
{
public:
	Detector(int t):type(t){}
	virtual void detect(const Mat& frame)=0;
	inline vector<Rect> getDetection(){return detection;}
	inline vector<double> getResponse(){return response;}
	void draw(Mat& frame);

protected:
	vector<Rect> detection;
	vector<double> response;
	int type;
};

class XMLDetector:public Detector
{
	xmlDocPtr file;
	xmlNodePtr frame;
	xmlChar* temp;
	bool open_success;
public:
	XMLDetector(const char* filename);
	~XMLDetector()
	{
		xmlFreeDoc(file);
		xmlCleanupParser();
		xmlMemoryDump();
	}
	virtual void detect(const Mat& f);
};


class HogDetector:public Detector
{
public:
	HogDetector();
	virtual void detect(const Mat& frame);

private:
	HOGDescriptor cpu_hog;
	vector<float> detector;
	vector<float> repsonse;//classifier response
};


#endif