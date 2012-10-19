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


#ifndef UTIL_
#define  UTIL_

#include <opencv2/opencv.hpp>
#include <stdio.h>
#include "OS_specific.h"

using namespace cv;
using namespace std;

// fast image data access wrapper
template<class T> class Img_
{
public:
	Img_(IplImage* img=0){imgp=img;}
	~Img_(){imgp=0;}
	void operator=(IplImage* img){imgp=img;}
	inline T* operator[](const int rowIndx)
	{
		return (T*)(imgp->imageData+rowIndx*imgp->widthStep);
	}
private:
	IplImage* imgp;
};
typedef struct  
{
	unsigned char b,g,r;
}RgbPixel;
typedef struct
{
	float b,g,r;
}RgbPixelFloat;
typedef Img_<RgbPixel> RgbImage_;
typedef Img_<RgbPixelFloat> RgbImageFloat_;
typedef Img_<unsigned char> BwImage_;
typedef Img_<float> BwImageFloat_;

template<class T> class Img
{
public:
	Img(Mat& img=0){imgp=&img;}
	~Img(){imgp=0;}
	void operator=(const Mat& img){imgp=&img;}
	inline T* operator[](const int rowIndx)
	{
		return (T*)(imgp->data+rowIndx*imgp->step);
	}
private:
	Mat* imgp;
};
typedef Img<RgbPixel> RgbImage;
typedef Img<RgbPixelFloat> RgbImageFloat;
typedef Img<unsigned char> BwImage;
typedef Img<float> BwImageFloat;

/* ****** ****** */

#define COLOR(ID) Scalar(255-(5*ID)%256,255-(57*ID)%256,255-(1035*ID)%256)

//distance of rectangle
#define OVERLAP 0
inline double getRectDist(Rect r1,Rect r2,int type=OVERLAP)
{
	Rect op=r1&r2;
	return 1-(double)op.area()/(double)(r1.area()+r2.area()-op.area());
}
inline int string2int(const char* s)
{
	int i;
	if(sscanf(s, "%d", &i) == EOF )
	{
		cout<<"error reading integer"<<endl;
	}
	return i;
}

inline float string2float(const char* s)
{
	return (float)atof(s);
}

inline double _string2double(const string s)
{
	return atof(s.c_str());
}

inline string _double2string(double d)
{
	ostringstream s;
	s << d;
	return s.str();
}

inline int _char_p2int(const char* s)
{
	return atoi(s);
}

inline Rect scaleWin(Rect win, double scale)
{
	return Rect(
		(int)(win.x+0.5*(1-scale)*win.width),	// CAUTION: could be rounded better (+0.5)
		(int)(win.y+0.5*(1-scale)*win.height),	 
		(int)(win.width*scale),
		(int)(win.height*scale));
}
inline Rect scaleWin(Rect win, double scale_w,double scale_h)
{
	return Rect(
		(int)(win.x+0.5*(1-scale_w)*win.width),	// CAUTION: could be rounded better (+0.5)
		(int)(win.y+0.5*(1-scale_h)*win.height),	 
		(int)(win.width*scale_w),
		(int)(win.height*scale_h));
}

#if OS_type==1//linux
inline char* itoa(int value, char* str, int radix) {
	static char dig[] =
		"0123456789"
		"abcdefghijklmnopqrstuvwxyz";
	int n = 0, neg = 0;
	unsigned int v;
	char* p, *q;
	char c;
	if (radix == 10 && value < 0) {
		value = -value;
		neg = 1;
	}
	v = value;
	do {
		str[n++] = dig[v%radix];
		v /= radix;
	} while (v);
	if (neg)
		str[n++] = '-';
	str[n] = '\0';
	for (p = str, q = p + (n-1); p < q; ++p, --q)
		c = *p, *p = *q, *q = c;
	return str;
}
#endif

#endif

