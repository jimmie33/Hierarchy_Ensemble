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


#include "detector.h"
#include <sstream>

void Detector::draw(Mat& frame)
{
	for (size_t i=0;i<detection.size();++i)
	{
		rectangle(frame,detection[i],Scalar((3*i)%256,(57*i)%256,(301*i)%256));
	}
}

/* ****** ****** */

XMLDetector::XMLDetector(const char* filename):Detector(XML)
{
	open_success=true;
	file=xmlReadFile(filename,"UTF-8",XML_PARSE_RECOVER);
	if (file == NULL)
	{
		cout<<"fail to open"<<endl;
		open_success=false;
	}
	if (open_success)
	{
		frame=xmlDocGetRootElement(file);
		if (frame==NULL)
		{
			cout<<"empty file"<<endl;
			open_success=false;
		}
		if (xmlStrcmp(frame->name,BAD_CAST"dataset"))
		{
			cout<<"bad file"<<endl;
			open_success=false;
		}
		frame=frame->children;
		while (xmlStrcmp(frame->name,BAD_CAST"frame"))
		{
			frame=frame->next;
		}			
	}	
}
void XMLDetector::detect(const Mat& f)
{
	detection.clear();
	response.clear();
	if (frame!=NULL)
	{
		xmlNodePtr objectList=frame->children;
		while (xmlStrcmp(objectList->name,BAD_CAST"objectlist"))
		{
			objectList=objectList->next;
		}
		xmlNodePtr object=objectList->children;
		while (object!=NULL && xmlStrcmp(object->name,BAD_CAST"object"))
		{
			object=object->next;
		}
		while (object!=NULL)//object level
		{
			float confidence=1;
			Rect res;
			xmlNodePtr box=object->children;
			while (xmlStrcmp(box->name,BAD_CAST"box") )
			{
				box=box->next;
			}
			temp=xmlGetProp(box,BAD_CAST"h");
			res.height=(int)(string2float((char*)temp)+0.5);
			xmlFree(temp);
			temp=xmlGetProp(box,BAD_CAST"w");
			res.width=(int)(string2float((char*)temp)+0.5);
			xmlFree(temp);
			temp=xmlGetProp(box,BAD_CAST"xc");
			res.x=(int)(string2float((char*)temp)-0.5*res.width+0.5);
			xmlFree(temp);
			temp=xmlGetProp(box,BAD_CAST"yc");
			res.y=(int)(string2float((char*)temp)-0.5*res.height+0.5);
			xmlFree(temp);

			detection.push_back(res);
			response.push_back(confidence);
			object=object->next;
			while (object!=NULL && xmlStrcmp(object->name,BAD_CAST"object"))
			{
				object=object->next;

			}
		}
	}		
	if (frame!=NULL)
	{
		frame=frame->next;
	}
	while (frame!=NULL && xmlStrcmp(frame->name,BAD_CAST"frame"))
	{
		frame=frame->next;
	}
}

/* ****** ****** */

HogDetector::HogDetector():Detector(HOG),cpu_hog(Size(64,128), Size(16, 16), Size(8, 8), Size(8, 8), 9, 1, -1, 
	HOGDescriptor::L2Hys, 0.2, false, cv::HOGDescriptor::DEFAULT_NLEVELS)
{
	detector = HOGDescriptor::getDefaultPeopleDetector();
	cpu_hog.setSVMDetector(detector);
}
void HogDetector::detect(const Mat& frame)
{
	cpu_hog.detectMultiScale(frame, detection, response, 0.0, Size(8,8),Size(0, 0), 1.05, 2);//-0.2
	for (vector<Rect>::iterator it=detection.begin(); it<detection.end(); it++)
	{
		it->x=(int)(it->x/HOG_DETECT_FRAME_RATIO);
		it->y=(int)(it->y/HOG_DETECT_FRAME_RATIO);
		it->width=(int)(it->width/HOG_DETECT_FRAME_RATIO);
		it->height=(int)(it->height/HOG_DETECT_FRAME_RATIO);
	}
}

/* ****** ****** */

TxtDetector::TxtDetector(const char* filename)
	:_file_p(filename),_count(1),_reader_pos(-1),_is_eof(false),Detector(TXT)
{
	if (!_file_p.is_open())
	{
		cerr<<"cannot open file: "<<filename<<endl;
		return;
	}
	if (!_file_p.eof())
	{
		char buff[255];
		_file_p.getline(buff,255);
		_buff=string(buff);
		istringstream ss(_buff);
		ss>>_reader_pos;
	}
	else
		_is_eof=true;
}

void TxtDetector::detect(const Mat& frame)
{
	detection.clear();

	while (!_is_eof && _reader_pos<=_count)
	{
		// deal with buffer
		if (_reader_pos==_count && _buff.length()!=0)
		{
			istringstream ss(_buff);
			int frame_id;
			float x1,y1,x2,y2;
			ss>>frame_id>>x1>>y1>>x2>>y2;
			detection.push_back(scaleWin(Rect(x1,y1,x2-x1+1,y2-y1+1),1.2,1.0));
		}

		if (!_file_p.eof())
		{
			char buff[255];
			_file_p.getline(buff,255);
			_buff=string(buff);
			istringstream ss(_buff);
			ss>>_reader_pos;
		}
		else
			_is_eof=true;
	}

	_count++;
}