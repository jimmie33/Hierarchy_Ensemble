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


#ifndef DATA_READER_H
#define DATA_READER_H

#include <cstdio>
#include <iostream>

#include "libxml/parser.h"
#include "libxml/tree.h"
#include "libxml/encoding.h"
#include "libxml/xmlwriter.h"
#include "opencv2/opencv.hpp"

#include "util.h"

using namespace cv;
using namespace std;

#define VIDEO 0
#define IMAGE 1

typedef struct Result2D
{
	int id;
	float xc, yc;  //center point 
	float w, h; //width, height
	double response;
	Result2D(int i,float x_,float y_,float w_,float h_,double res=1)
		:id(i),xc(x_),yc(y_),w(w_),h(h_),response(res){}
	Result2D(){}
}Result2D;

class SeqReader //sequence reader interface
{
public:
	SeqReader(){};
	virtual void readImg(Mat& frame)=0;
};

class BBoxReader // interface for reading bounding boxes from files
{
public:
	virtual bool getNextFrameResult(vector<Result2D>& result)=0;
};

class BBoxWriter // interface for reading bounding boxes from files
{
public:
	virtual bool putNextFrameResult(vector<Result2D>& result)=0;
};

/* ****** ****** */

class VideoReader:public SeqReader
{
public:
	VideoReader(const string filename):capture(filename){}
	virtual void readImg(Mat& frame){	capture>>frame;}

private:
	VideoCapture capture;
};

class ImageDataReader:public SeqReader
{
public:
	ImageDataReader(const string dir);
	virtual void readImg(Mat& frame);

private:
	int _file_counter;
	string _directory;
	vector<string> _m_fileNames;
};

class XMLBBoxReader:public BBoxReader
{
public:
	XMLBBoxReader(const char* filename);	
	~XMLBBoxReader()
	{
		xmlFreeDoc(file);
	}
	inline bool getOpenSuc(){return open_success;}
	virtual bool getNextFrameResult(vector<Result2D>& result);

private:
	xmlDocPtr file;
	xmlNodePtr frame;
	xmlChar* temp;
	bool open_success;
};
#define  ENCODING "UTF-8"
class XMLBBoxWriter: public BBoxWriter
{
public:
	XMLBBoxWriter(const char* filename);	
	~XMLBBoxWriter()
	{
		rc = xmlTextWriterEndDocument(writer);
		xmlFreeTextWriter(writer);
	}
	virtual bool putNextFrameResult(vector<Result2D>& result);
	inline bool getOpenSuc(){return open_success;}

private:
	int rc;
	xmlTextWriterPtr writer;
	xmlChar *tmp;
	bool open_success;
	int frameCount;
};


#endif