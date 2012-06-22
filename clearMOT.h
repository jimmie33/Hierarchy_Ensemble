#ifndef CLEAR_MOT
#define CLEAR_MOT

#include <cstdlib>
#include <ctime>
#include <cstdio>

#include <iostream>
#include <fstream>

#include "opencv2/opencv.hpp"
#include "munkres.h"

#include "libxml/parser.h"
#include "libxml/tree.h"
#include "libxml/encoding.h"
#include "libxml/xmlwriter.h"

#include "util.h"

typedef struct ResultWindow 
{
	int ID;
	Rect window;
	double response;
	ResultWindow(int id,Rect wind,double r=1):ID(id),window(wind),response(r){}

}ResultWindow;

typedef struct MOTResult
{
	double MOTA;
	double MOTP;//average overlap over union, bigger the better
	int missing;
	int false_positive;
	int ID_switch;
}MOTResult;

class C_Mot
{
public:
	void dealWith(vector<ResultWindow> gt,vector<ResultWindow> hp);
	// if is_det = true, then it will output precision and recall
	MOTResult getMOT(bool is_det=false) ;
	void paintFrame(Mat& frame);

	C_Mot(double thresh=0.3)
		:IOU_threshold(thresh),frame_count(0),error_sum(0),M_sum(0),FP_sum(0),IS_sum(0),match_count(0),gt_count(0){}
	~C_Mot(){}

private:
	inline double getDistance(ResultWindow w1,ResultWindow w2)
	{
		Rect op=w1.window & w2.window;
		return 1-(double)op.area()/(double)(w1.window.area()+w2.window.area()-op.area());
	}

	double error_sum;
	double M_sum;
	double FP_sum;
	double IS_sum;
	double match_count;
	double gt_count;//object present
	int frame_count;

	double IOU_threshold;

	map<int,int> gt_book_keeping;//keep the last match ID
	map<int,int> hp_book_keeping;//keep the last match ID

	//for debugging
	vector<ResultWindow> matched_couple;
	vector<ResultWindow>switched_couple;
	vector<ResultWindow> miss_detection;
	vector<ResultWindow> false_alarm;
};

class ResultParser
{
public:
	vector<ResultWindow> readNextFrame();
	inline bool isEnd()
	{
		return frame==NULL ? true:false;
	}

	ResultParser(const char* filename,double r=1.0,double h=1.0,double w=1.0);
	~ResultParser()
	{
		xmlFreeDoc(file);
		xmlCleanupParser();
		xmlMemoryDump();
	}	

private:
	xmlDocPtr file;
	xmlNodePtr frame;
	xmlChar* temp;
	bool open_success;
	double ratio;
	double h_ratio;
	double w_ratio;
};


#endif