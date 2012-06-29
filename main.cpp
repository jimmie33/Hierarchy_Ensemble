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
#include "clearMOT.h"
#include "parameter.h"

using namespace cv;
using namespace std;

//#define RECORD_VIDEO
//#define  RECORD_IMAGE

static string _sequence_path_;
static string _detection_xml_file_;
static string _result_xml_file_;
static string _gt_xml_file_;// ground truth

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
#ifdef RECORD_VIDEO
	VideoWriter videoWriter("result.avi",CV_FOURCC('D','I','V','X'),10,frame.size());
#endif

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

	TrakerManager mTrack(detector,frame,5);
	
	for (int frameCount=0;frame.data!=NULL && frameCount<=4500;frameCount++)
	{
		//resize(frame,frame,Size(1280,960));
		mTrack.doWork(frame);
		imshow("multiTrack", frame);
		
#ifdef RECORD_VIDEO
		videoWriter<<frame;
#endif
		//resize(frame,frame,Size(480,360));
#ifdef RECORD_IMAGE
		char buff[10];
		sprintf(buff,"%d",frameCount);
		string s=buff;
		s.insert(s.begin(),4-s.length(),'0');
		string filename="TrackletsResult\\img_"+s;
		filename=filename+".jpeg";
		cout<<filename<<endl;
		imwrite(filename,frame);
#endif
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

int calMOT(int readerType,double w_rescale_ratio=1.0,double h_rescale_ratio=1.0)
{
	ResultParser gt(GT_XML_FILE);
	ResultParser hp(RESULT_OUTPUT_XML_FILE,1.0,h_rescale_ratio,w_rescale_ratio);
	C_Mot mot;

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
		return 0;
	}
	reader->readImg(frame);
	mot.dealWith(gt.readNextFrame(),hp.readNextFrame());
	while(!gt.isEnd() && !hp.isEnd())
	{
		mot.paintFrame(frame);
		imshow("multiTrack",frame);
		waitKey(1);
		mot.dealWith(gt.readNextFrame(),hp.readNextFrame());
		reader->readImg(frame);
	}
	mot.getMOT();
	//getchar();
	return 1;
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
	
	// TODO: delete th calMOT module in the release version
	calMOT(VIDEO,1.0,1.0);//for pets open_cv w:0.9(0.8), h:1.2(1.05); 1.0 1.3 for town;
	return 0;
}