#ifndef UTIL_
#define  UTIL_

#include "opencv2/opencv.hpp"

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
inline double getRectDist(Rect r1,Rect r2,int type)
{
	Rect op=r1&r2;
	return 1-(double)op.area()/(double)(r1.area()+r2.area()-op.area());
}
inline int string2int(const char* s)
{
	int i;
	if(sscanf_s(s, "%d", &i) == EOF )
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

#endif