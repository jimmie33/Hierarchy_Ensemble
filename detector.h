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