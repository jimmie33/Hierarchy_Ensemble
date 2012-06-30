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

#include <ctime>
#include <iostream>
#include <sstream>
#include <fstream>

#include "libxml/parser.h"
#include "libxml/tree.h"
#include "libxml/xmlmemory.h"

#include "tracker.h"
#include "detector.h"
#include "dataReader.h"
#include "multiTrackAssociation.h"
#include "parameter.h"

using namespace cv;
using namespace std;

static string _sequence_path_;
static string _detection_xml_file_;

//Configuration by default
int MAX_TRACKER_NUM;
int MAX_TEMPLATE_SIZE;
int EXPERT_THRESH;
double BODYSIZE_TO_DETECTION_RATIO;
double TRACKING_TO_BODYSIZE_RATIO;
int FRAME_RATE;
double TIME_WINDOW_SIZE;
double HOG_DETECT_FRAME_RATIO;

void read_config()
{
	ifstream conf_file("config.txt");
	
	if (!conf_file.is_open())
	{
		cerr<<"fail to load config.txt."<<endl;
		exit(1);
	}

	string line;
	while (conf_file.good())
	{
		getline(conf_file,line);
		istringstream line_s(line);
		string field;
		line_s>>field;
		if (field.compare("MAX_TRACKER_NUM:")==0)
			line_s>>MAX_TRACKER_NUM;
		else if (field.compare("FRAME_RATE:")==0)
			line_s>>FRAME_RATE;
		else if (field.compare("TIME_WINDOW_SIZE:")==0)
			line_s>>TIME_WINDOW_SIZE;
		else if (field.compare("HOG_DETECT_FRAME_RATIO:")==0)
			line_s>>HOG_DETECT_FRAME_RATIO;
		else if (field.compare("MAX_TEMPLATE_SIZE:")==0)
			line_s>>MAX_TEMPLATE_SIZE;
		else if (field.compare("EXPERT_THRESH")==0)
			line_s>>EXPERT_THRESH;
		else if (field.compare("BODYSIZE_TO_DETECTION_RATIO:")==0)
			line_s>>BODYSIZE_TO_DETECTION_RATIO;
		else if (field.compare("TRACKING_TO_BODYSIZE_RATIO:")==0)
			line_s>>TRACKING_TO_BODYSIZE_RATIO;
	}
	conf_file.close();
}

void multiTrack(int readerType,int detectorType)
{
	namedWindow("multiTrack",CV_WINDOW_AUTOSIZE);
	SeqReader* reader;
	Mat frame;
	switch (readerType)
	{
	case IMAGE:
		reader=new ImageDataReader(_sequence_path_);
		break;
	case VIDEO:
		reader=new VideoReader(_sequence_path_);
		break;
	default:
		cerr<<"no such reader type!"<<endl;
		return ;
	}
	reader->readImg(frame);
	if (frame.data==NULL)
	{
		cerr<<"fail to open pictures!"<<endl;
		return ;
	}

	Detector* detector;
	switch (detectorType)
	{
	case HOG:
		detector=new HogDetector();
		break;
	case XML:
		detector=new XMLDetector(_detection_xml_file_.c_str());
		break;
	default:
		detector=new HogDetector();
		break;
	}

	TrakerManager mTrack(detector,frame,EXPERT_THRESH);
	
	for (int frameCount=0;frame.data!=NULL;frameCount++)
	{
		mTrack.doWork(frame);
		imshow("multiTrack", frame);
		
		reader->readImg(frame);

		char c = waitKey(1);
		if(c == 'q') break;
		else if (c=='p')
		{
			cvWaitKey(0);
		}
		else if(c != -1)
		{
			mTrack.setKey(c);
		}
	}

	delete reader;
	delete detector;
}

void help()
{
	cout<<"usage: \n\n"
		"1.\n" 
		"Hierarchy_Ensemble <sequence_path> <is_image>\n"
		"(by default, it uses hog detector in opencv to detect pedestrians)\n\n"

		"2.\n"
		"Hierarchy_Ensemble <sequence_path> <is_image> <detection_xml_file_path>\n"
		"(it uses detection stored in the specified xml file. You may rescale the detection bounding box "
		"by tuning parameters in the \"he_config.txt\")\n\n"

		"<is_image>: \'1\' for image format data. \'0\' for video format data.\n";
	getchar();
}

int main(int argc,char** argv)
{
	if (argc !=3 && argc !=4)
	{
		help();
		exit(1);
	}

	read_config();

	_sequence_path_=string(argv[1]);
	int seq_format;
	
	if (atof(argv[2])==1.0)
		seq_format=IMAGE;
	else 
		seq_format=VIDEO;
	
	if (argc>3)
	{
		_detection_xml_file_=string(argv[3]);
		multiTrack(seq_format,XML);
	}
	else
		multiTrack(seq_format,HOG);
	
	return 0;
}