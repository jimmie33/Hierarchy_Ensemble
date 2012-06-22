#include <time.h>
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

//TEMP:
double AUX_phi1=1.0;
double AUX_phi2=1.0;
double AUX_gamma1=1.0;
double AUX_gamma2=1.0;
double AUX_N_max=1.0;
double AUX_K=1.0;
double AUX_kf_process=1.0;
double AUX_kf_measure=1.0;


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

	TrakerManager mTrack(detector,frame,5*AUX_K);
	
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
	/*
	if (argc !=3 && argc !=4)
	{
		help();
		exit(1);
	}
	*/
	AUX_phi1=atof(argv[4]);
	AUX_phi2=atof(argv[5]);
	AUX_gamma1=atof(argv[6]);
	AUX_gamma2=atof(argv[7]);
	AUX_N_max=atof(argv[8]);
	AUX_K=atof(argv[9]);
	AUX_kf_process=atof(argv[10]);

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