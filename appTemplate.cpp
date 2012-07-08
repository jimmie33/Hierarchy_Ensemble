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



#include "appTemplate.h"

//for the channel comparison
typedef struct ChannelScore
{
	int idx;
	double score;
	ChannelScore(int i,double s):idx(i), score(s){}
}ChannelScore;
inline bool compareChannel(ChannelScore c1,ChannelScore c2)
{
	return c1.score<c2.score ? true:false;
}
// Variance Ratio
double getVR(Mat hist1,Mat hist2)
{
	Mat idx=Mat::zeros(hist1.rows,hist1.cols,CV_32FC1);
	for (int i=0;i<idx.rows;++i)
	{
		float* r_ptr=idx.ptr<float>(i);
		r_ptr[0]=(float)i;
	}
	double mean_idx=hist1.dot(idx);
	Mat temp=idx-mean_idx;
	temp=temp.mul(temp);
	double variance1=hist1.dot(temp);

	mean_idx=hist2.dot(idx);
	temp=idx-mean_idx;
	temp=temp.mul(temp);
	double variance2=hist2.dot(temp);

	Mat hist_mean=(hist1+hist2)*0.5;
	mean_idx=hist_mean.dot(idx);
	temp=idx-mean_idx;
	temp=temp.mul(temp);
	double variance_mean=hist_mean.dot(temp);

	return variance_mean/(variance1+variance2);
}

AppTemplate::AppTemplate(const Mat* frame_set, const Rect iniWin,int ID)
	:ID(ID)//bgr,hsv,lab
{	
	//get roi out of frame set
	Rect body_win=scaleWin(iniWin,1/TRACKING_TO_BODYSIZE_RATIO);
	Rect roi_win(body_win.x-body_win.width,body_win.y-body_win.width,3*body_win.width,2*body_win.width+body_win.height);
	body_win= body_win&Rect(0,0,frame_set[0].cols,frame_set[0].rows);
	roi_win=roi_win&Rect(0,0,frame_set[0].cols,frame_set[0].rows);
	Mat roi_set[]={Mat(frame_set[0],roi_win),Mat(frame_set[1],roi_win),Mat(frame_set[2],roi_win)};

	
	Rect iniWin_roi=iniWin-Point(roi_win.x,roi_win.y);

	//scores for each channel
	list<ChannelScore> channel_score;
	
	Mat mask_roi(roi_set[0].rows,roi_set[0].cols,CV_8UC1,Scalar(0));
	rectangle(mask_roi,iniWin_roi,Scalar(255),-1);
	Mat inv_mask_roi(roi_set[0].rows,roi_set[0].cols,CV_8UC1,Scalar(255));
	rectangle(inv_mask_roi,body_win-Point(roi_win.x,roi_win.y),Scalar(0),-1);

	//calculate score for each channel
	Mat temp_hist;
	Mat temp_bp;
	int hist_size[]={BIN_NUMBER};
	for (int i=0;i<9;i++)
	{
		float range1[]={0,255};
		if (i==3)
		{
			range1[1]=179;
		}
		const float* hist_range[]={range1};
		
		calcHist(roi_set,3,&i,inv_mask_roi,temp_hist,1,hist_size,hist_range);
		normalize(temp_hist,temp_hist,255,0.0,NORM_L1);//scale to 255 for display

		calcBackProject(roi_set,3,&i,temp_hist,temp_bp,hist_range);
		int c[]={0};
		int hs[]={BIN_NUMBER};
		float hr[]={0,255};
		const float* hrr[]={hr};
		Mat hist_fore;
		Mat hist_back;
		calcHist(&temp_bp,1,c,mask_roi,hist_fore,1,hs,hrr);
		calcHist(&temp_bp,1,c,inv_mask_roi,hist_back,1,hs,hrr);
		normalize(hist_fore,hist_fore,1.0,0.0,NORM_L1);
		normalize(hist_back,hist_back,1.0,0.0,NORM_L1);
		//deal with gray image to get rid of #IND
		double score=getVR(hist_back,hist_fore);
		score=score==score ? score:0;
		channel_score.push_back(ChannelScore(i,score));
	}

	//choose the 2 highest scored channels
	channel_score.sort(compareChannel);
	channels[0]=channel_score.back().idx;
	channel_score.pop_back();
	channels[1]=channel_score.back().idx;
	
	//using 2 best channel to calculate histogram
	for (int i=0;i<2;++i)
	{
		_hRang[i][0]=0;
		if (channels[i]==3)
			_hRang[i][1]=179;	
		else
			_hRang[i][1]=255;	
		hRange[i]=_hRang[i];
	}
	calcHist(roi_set,3,channels,inv_mask_roi,temp_hist,2,hSize,hRange);
	normalize(temp_hist,temp_hist,255,0,NORM_L1);
	Mat final_mask;//mask for sampling
	calcBackProject(roi_set,3,channels,temp_hist,final_mask,hRange);
	threshold(final_mask,final_mask,5,255,CV_THRESH_BINARY_INV);
	          
	final_mask=min(final_mask,mask_roi);

	//choose the best two feature space for foreground****************
	Mat hist_fore,hist_back;
	channel_score.clear();
	double sum_score=0;
	for (int i=0;i<9;i++)
	{
		float range1[]={0,255};
		if (i==3)
		{
			range1[1]=179;
		}
		const float* hist_range[]={range1};
		Mat temp_hist_neg;
		calcHist(roi_set,3,&i,final_mask,temp_hist,1,hist_size,hist_range);
		normalize(temp_hist,temp_hist,255,0,NORM_L1);
		calcHist(roi_set,3,&i,inv_mask_roi,temp_hist_neg,1,hist_size,hist_range);
		normalize(temp_hist_neg,temp_hist_neg,255,0,NORM_L1);
		log(temp_hist,temp_hist);		
		log(temp_hist_neg,temp_hist_neg);
		temp_hist=temp_hist-temp_hist_neg;
		threshold(temp_hist,temp_hist,0,255,CV_THRESH_TOZERO);
		normalize(temp_hist,temp_hist,255,0.0,NORM_L1);//scale to 255 for display

		calcBackProject(roi_set,3,&i,temp_hist,temp_bp,hist_range);
		int c[]={0};
		int hs[]={BIN_NUMBER};
		float hr[]={0,255};
		const float* hrr[]={hr};
		calcHist(&temp_bp,1,c,final_mask,hist_fore,1,hs,hrr);
		calcHist(&temp_bp,1,c,inv_mask_roi,hist_back,1,hs,hrr);
		normalize(hist_fore,hist_fore,1.0,0.0,NORM_L1);
		normalize(hist_back,hist_back,1.0,0.0,NORM_L1);
		double score=getVR(hist_back,hist_fore);
		score=score==score ? score:0;
		channel_score.push_back(ChannelScore(i,score));
		sum_score+=exp(score);
	}


	channel_score.sort(compareChannel);
	channels[0]=channel_score.back().idx;
	channel_score.pop_back();
	channels[1]=channel_score.back().idx;

	for (int i=0;i<2;++i)
	{
		_hRang[i][0]=0;
		if (channels[i]==3)
			_hRang[i][1]=179;	
		else
			_hRang[i][1]=255;	
		hRange[i]=_hRang[i];
	}
	calcHist(roi_set,3,channels,final_mask,hist,2,hSize,hRange);///////////////////
	normalize(hist,hist,255,0,NORM_L1);

	//recover the shift_vector
	Mat backPro;
	calcBackProject(roi_set,3,channels,hist,backPro,hRange);
	iniWin_roi=iniWin-Point(roi_win.x,roi_win.y);
	Point2f origin_point_roi((float)(iniWin_roi.x+0.5*iniWin_roi.width),(float)(iniWin_roi.y+0.5*iniWin_roi.height));
	meanShift(backPro,iniWin_roi,TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));

	Point2f shift_point_roi((float)(iniWin_roi.x+0.5*iniWin_roi.width),(float)(iniWin_roi.y+0.5*iniWin_roi.height));
	shift_vector=(shift_point_roi-origin_point_roi)*(1/(float)iniWin.width);
}
AppTemplate::AppTemplate(const AppTemplate& tracker): ID(tracker.ID)
{
	tracker.hist.copyTo(hist);
	tracker.confidence_map.copyTo(confidence_map);
	shift_vector=tracker.shift_vector;
	score=tracker.score;
	for (int i=0;i<2;i++)
	{
		channels[i]=tracker.channels[i];
		for (int j=0;j<2;j++)
		{
			_hRang[i][j]=tracker._hRang[i][j];
		}
	}
	hRange[0]=_hRang[0];
	hRange[1]=_hRang[1];
}

void AppTemplate::calcBP(const Mat* frame_set, Mat& occ_map,Rect ROI)//*******************
{
	confidence_map=Mat::zeros(ROI.height,ROI.width,CV_8UC1);
	Rect frame_win(0,0,frame_set[0].cols,frame_set[0].rows);
	Rect roi=frame_win & ROI;//the rest of the win will be filled with zero

	//CAUTION: cannot generalize to other structure of feature channels
	Mat roi_set[]={Mat(frame_set[0],roi), Mat(frame_set[1],roi),Mat(frame_set[2],roi)};
	Mat roi_backproj(confidence_map,roi-Point(ROI.x, ROI.y));
	Mat roi_mask(occ_map,roi);//occ_map: 1 for no occupancy, 0 for occupancy
	calcBackProject(roi_set,3,channels,hist,roi_backproj,hRange);

	roi_backproj.setTo(Scalar(0.0),roi_mask);
	confidence_map.convertTo(confidence_map,CV_32FC1);//[0,255]
}
void AppTemplate::calcScore(Rect b_inner,Rect b_outer)//*******************
{
	Mat cm;
	confidence_map.copyTo(cm);
	Rect rw=b_inner&Rect(0,0,cm.cols,cm.rows);
	Scalar fg=mean(cm(rw));//be careful with the range
	Mat mask=Mat::zeros(confidence_map.size(),CV_8UC1);
	rectangle(mask,b_outer,Scalar(1),-1);//mask out the whole GT
	cm.setTo(Scalar(0),mask);
	Mat matching_map;
	if (confidence_map.rows==0)
	{
		score=0;
		return;
	}
	matchTemplate(cm,Mat(b_inner.height,b_inner.width,CV_32FC1,Scalar(255)),matching_map,CV_TM_SQDIFF);
	Point minloc;
	minMaxLoc(matching_map,0,0,&minloc);
	Scalar bg=mean(cm(Rect(minloc.x,minloc.y,b_inner.width,b_inner.height)));
	score=(fg[0]-bg[0]);///fg[0];
}