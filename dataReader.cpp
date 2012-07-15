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


#include "OS_specific.h"
#include "dataReader.h"


ImageDataReader::ImageDataReader(const string dir):_directory(dir),_file_counter(0)
{
	char tmpDirSpec[MAX_PATH+1];
	sprintf (tmpDirSpec, "%s*", _directory.c_str());

#if OS_type==2 // for windows
	WIN32_FIND_DATAA f;
	HANDLE h = FindFirstFileA(tmpDirSpec , &f); // read .
	if(h != INVALID_HANDLE_VALUE)
	{
		FindNextFileA(h, &f);	//read ..
		while(FindNextFileA(h, &f))
			_m_fileNames.push_back(f.cFileName);
	}
	FindClose(h);
#endif

#if OS_type==1 // for linux
	
	DIR *dp;
	struct dirent *dirp;
	if((dp = opendir(dir.c_str())) == NULL) {
		cout << "Error opening " << dir << endl;
	}

	readdir(dp);//.
	readdir(dp);//..
	while ((dirp = readdir(dp)) != NULL) {
		string filename(dirp->d_name);
		if (
			filename.find(".jpg")!=string::npos ||
			filename.find(".jpeg")!=string::npos ||
			filename.find(".png")!=string::npos ||
			filename.find(".bmp")!=string::npos
			)
			_m_fileNames.push_back(filename);
		
	}
	std::sort(_m_fileNames.begin(),_m_fileNames.end());
	closedir(dp);
#endif
}
void ImageDataReader::readImg(Mat& frame)
{
	frame.data=NULL;
	if (_file_counter>=_m_fileNames.size())
		return;
	frame=imread(_directory+_m_fileNames[_file_counter]);
	_file_counter++;
}

/* ****** ****** */

XMLBBoxReader::XMLBBoxReader(const char* filename)
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
	}				
}
bool XMLBBoxReader::getNextFrameResult(vector<Result2D>& result)
{
	bool rt=false;
	result.clear();
	while (frame!=NULL)
	{
		if (!xmlStrcmp(frame->name,BAD_CAST"frame"))
		{
			rt=true;//get the successive frame
			xmlNodePtr objectList;
			objectList=frame->children;
			while (objectList!=NULL)//objectlist level
			{
				if (!xmlStrcmp(objectList->name,BAD_CAST"objectlist"))
				{
					xmlNodePtr object=objectList->children;
					while (object!=NULL)//object level
					{
						if (!xmlStrcmp(object->name,BAD_CAST"object"))
						{
							Result2D res;
							temp=xmlGetProp(object,BAD_CAST"id");
							res.id=string2int((char*)temp);
							xmlFree(temp);
							xmlNodePtr box=object->children;
							while (box!=NULL)
							{
								if (!xmlStrcmp(box->name,BAD_CAST"box"))
								{
									temp=xmlGetProp(box,BAD_CAST"h");
									res.h=(float)string2float((char*)temp);
									xmlFree(temp);
									temp=xmlGetProp(box,BAD_CAST"w");
									res.w=(float)string2float((char*)temp);
									xmlFree(temp);
									temp=xmlGetProp(box,BAD_CAST"xc");
									res.xc=(float)string2float((char*)temp);
									xmlFree(temp);
									temp=xmlGetProp(box,BAD_CAST"yc");
									res.yc=(float)string2float((char*)temp);
									xmlFree(temp);
									break;
								}
								box=box->next;
							}
							result.push_back(res);
						}
						object=object->next;
					}
					break;
				}	
				objectList=objectList->next;
			}
			break;
		}
		frame=frame->next;
	}
	if (frame!=NULL)
	{
		frame=frame->next;
	}		
	return rt;
}	

/* ****** ****** */

XMLBBoxWriter::XMLBBoxWriter(const char* filename):frameCount(0)
{
	open_success=true;
	writer = xmlNewTextWriterFilename(filename, 0);
	if (writer == NULL) {
		printf("testXmlwriterFilename: Error creating the xml writer\n");
		open_success=false;
	}
	rc=xmlTextWriterStartDocument(writer,NULL,ENCODING,NULL);
	rc=xmlTextWriterStartElement(writer, BAD_CAST "dataset");
}
bool XMLBBoxWriter::putNextFrameResult(vector<Result2D>& result)
{
	rc = xmlTextWriterStartElement(writer, BAD_CAST "frame");
	rc = xmlTextWriterWriteFormatAttribute(writer, BAD_CAST "number",
		"%d",frameCount);
	rc=xmlTextWriterStartElement(writer,BAD_CAST"objectlist");
	vector<Result2D>::iterator it;
	for (it=result.begin();it<result.end();it++)
	{
		rc = xmlTextWriterStartElement(writer, BAD_CAST "object");
		rc = xmlTextWriterWriteFormatAttribute(writer, BAD_CAST "id",
			"%d",(*it).id);
		rc=xmlTextWriterWriteFormatAttribute(writer, BAD_CAST "confidence",
			"%lf",(*it).response);
		rc = xmlTextWriterStartElement(writer, BAD_CAST "box");
		rc = xmlTextWriterWriteFormatAttribute(writer, BAD_CAST "h",
			"%f",(*it).h);
		rc = xmlTextWriterWriteFormatAttribute(writer, BAD_CAST "w",
			"%f",(*it).w);
		rc = xmlTextWriterWriteFormatAttribute(writer, BAD_CAST "xc",
			"%f",(*it).xc);
		rc = xmlTextWriterWriteFormatAttribute(writer, BAD_CAST "yc",
			"%f",(*it).yc);
		//end box
		rc = xmlTextWriterEndElement(writer);
		//end object
		rc = xmlTextWriterEndElement(writer);
	}
	//end objectlist
	rc = xmlTextWriterEndElement(writer);
	//end frame
	rc = xmlTextWriterEndElement(writer);

	frameCount++;
	return true;
}