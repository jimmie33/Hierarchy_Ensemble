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

#include "tracker.h"

#define SCALE_UPDATE_RATE 0.4
#define HIST_MATCH_UPDATE 0.01

list<EnsembleTracker*> EnsembleTracker::_TRASH_LIST;
void EnsembleTracker::dump()
{
	for (list<EnsembleTracker*>::iterator it=_neighbors.begin();it!=_neighbors.end();it++)
	{
		(*it)->refcDec1();
	}
	_TRASH_LIST.push_back(this);
	_is_dumped=true;
}
void EnsembleTracker::emptyTrash()
{
	for (list<EnsembleTracker*>::iterator it=_TRASH_LIST.begin();it!=_TRASH_LIST.end();)
	{
		if ((*it)->_refc==0)
		{
			delete (*it);
			_TRASH_LIST.erase(it++);
			continue;
		}
		it++;
	}
}
inline bool compareTemplate(AppTemplate* t1,AppTemplate* t2)
{
	return t1->getScore()>t2->getScore() ? true:false;
}
EnsembleTracker::EnsembleTracker(int id,Size body_size,double phi1,double phi2,double phi_max)
	:_refc(0),_is_dumped(false),
	_phi1_(phi1),
	_phi2_(phi2),
	_phi_max_(phi_max),
	_novice_status_count(0),
	_template_count(0),
	_ID(id),
	_kf(4,2,0),
	_is_novice(false),
	_match_radius(0),
	hist_match_score(0),
	_added_new(true),
	_record_idx(0)
	//tracking_count(1)
{
	_retained_template=0;
	//initialize kalman filter
	_kf.transitionMatrix=*(Mat_<float>(4,4)<<1,0,1,0,0,1,0,1,0,0,1,0,0,0,0,1);
	setIdentity(_kf.measurementMatrix);
	updateKfCov(body_size.width);
	setIdentity(_kf.errorCovPost,Scalar::all(3.0*(double)body_size.width*body_size.width));

	//for long-time appearance template: RGB
	histSize[0]=8;histSize[1]=8;histSize[2]=8;
	channels[0]=0;channels[1]=1;channels[2]=2;
	for (int i=0;i<3;++i)
	{
		_hRang[i][0]=0;
		_hRang[i][1]=255;	
		hRange[i]=_hRang[i];
	}

	//for calculating hitting rate in a time window
	_recentHitRecord=Mat::zeros(2,4*FRAME_RATE,CV_64FC1);
}
EnsembleTracker::~EnsembleTracker()
{
	list<AppTemplate*>::iterator it;
	for (it=_template_list.begin();it!=_template_list.end();it++)
	{
		delete *it;
	}
	delete _retained_template;
}
void EnsembleTracker::updateNeighbors(
	list<EnsembleTracker*> tr_list,
	double dis_thresh_r,
	double scale_r1, double scale_r2,
	double hist_thresh)
{
	// erase neighbors
	for (list<EnsembleTracker*>::iterator it=_neighbors.begin();it!=_neighbors.end();)
	{
		// delete dumped neighbors
		if ((*it)->getIsDumped())
		{
			(*it)->refcDec1();
			_neighbors.erase(it++);
			continue;
		}
		Rect r=(*it)->getBodysizeResult();
		Point2f c1(r.x+0.5f*r.width,r.y+0.5f*r.height);
		Point2f c2(_result_bodysize_temp.x+0.5f*_result_bodysize_temp.width,_result_bodysize_temp.y+0.5f*_result_bodysize_temp.height);
		double dis=sqrt(pow(c1.x-c2.x,2.0f)+pow(c1.y-c2.y,2.0f));
		double scale_ratio=(double)r.width/(double)_result_bodysize_temp.width;
		// delete neighbors if they are novices or they are out of the radius
		if (
			dis>dis_thresh_r*_match_radius || 
			scale_ratio>scale_r1 ||scale_ratio<scale_r2 || 
			(*it)->getIsNovice())
		{
			(*it)->refcDec1();
			_neighbors.erase(it++);
			continue;
		}
		it++;
	}
	// add new neighbors
	for (list<EnsembleTracker*>::iterator it=tr_list.begin();it!=tr_list.end();it++)
	{
		if ((*it)->getIsNovice() || (*it)->getID()==_ID)
		{
			continue;
		}
		bool alreadyFound=false;
		for (list<EnsembleTracker*>::iterator k=_neighbors.begin();k!=_neighbors.end();k++)
		{
			if ((*k)->getID()==(*it)->getID())
			{
				alreadyFound=true;
				continue;
			}
		}
		if (alreadyFound)
		{
			continue;
		}
		Rect r=(*it)->getBodysizeResult();
		Point c((int)(r.x+0.5*r.width),(int)(r.y+0.5*r.height));
		double dis=sqrt((_result_bodysize_temp.x+0.5*_result_bodysize_temp.width-c.x)*(_result_bodysize_temp.x+0.5*_result_bodysize_temp.width-c.x)+(_result_bodysize_temp.y+0.5*_result_bodysize_temp.height-c.y)*(_result_bodysize_temp.y+0.5*_result_bodysize_temp.height-c.y));
		double scale_ratio=(double)r.width/(double)_result_bodysize_temp.width;
		double h_match=compareHisto((*it)->hist);
		if (
			dis<dis_thresh_r*_match_radius && 
			scale_ratio<scale_r1 && scale_ratio>scale_r2 && 
			h_match>hist_thresh)//histogram distance threshold
		{
			(*it)->refcAdd1();// one reference
			_neighbors.push_back((*it));
		}
	}	
}
void EnsembleTracker::addAppTemplate(const Mat* frame_set,Rect iniWin)
{
	setAddNew(true);// set the flag
	_recentHitRecord.at<double>(0,_record_idx)=1;
	_recentHitRecord.at<double>(1,_record_idx)=1.0;
	
	//generate new appearance template and add to list
	AppTemplate* tra_template=new AppTemplate(frame_set,iniWin,_template_count);
	_template_list.push_back(tra_template);
	
	//update window size
	Size2f detection_size((float)iniWin.width,(float)iniWin.height);
	_window_size.width= _template_list.size()==1 ? (float)iniWin.width : (float)(_window_size.width*(1-SCALE_UPDATE_RATE)+detection_size.width*SCALE_UPDATE_RATE);
	_window_size.height= _template_list.size()==1 ? (float)iniWin.height : (float)(_window_size.height*(1-SCALE_UPDATE_RATE)+detection_size.height*SCALE_UPDATE_RATE);
	
	// update the tracking result
	if (
		 _result_history.size()==0|| // if new tracker
		getIsNovice()) // if novice
	{
		_result_temp=iniWin;
		_result_last_no_sus=iniWin;
		_result_bodysize_temp=scaleWin(iniWin,1/TRACKING_TO_BODYSIZE_RATIO);
		_retained_template=new AppTemplate(*tra_template);
	}
	_template_count++;
}
void EnsembleTracker::calcConfidenceMap(const Mat* frame_set,Mat& occ_map)//**********************
{
	// use the kalman filter prediction to locate the roi of confidence map (backprojection map)
	_kf.predict();
	Point center((int)_kf.statePre.at<float>(0,0),(int)_kf.statePre.at<float>(1,0));
	double w=_window_size.width/TRACKING_TO_BODYSIZE_RATIO;
	double h=_window_size.height/TRACKING_TO_BODYSIZE_RATIO; 
	h+=2*w;
	w+=2*w;

	Rect roi_win((int)(center.x-0.5*w), (int)(center.y-0.5*h),(int)w,(int)h);
	_cm_win=roi_win;
	_confidence_map=Mat::zeros((int)h,(int)w,CV_32FC1);

	//PREVENTING FROM OVERLAPPING WITH FRIEND
	Mat final_occ_map;
	occ_map.copyTo(final_occ_map);
	for (list<EnsembleTracker*>::iterator it=_neighbors.begin();it!=_neighbors.end();it++)
	{
		// mask neighbors' area if they are not novices and they have more templates than this one
		if ((*it)==NULL || (*it)->getIsNovice() || (*it)->getTemplateNum()<(int)_template_list.size())
			continue;
		Rect r=scaleWin((*it)->getBodysizeResult(),1.0);
		ellipse(final_occ_map,Point((int)(r.x+0.5*r.width),(int)(r.y+0.5*r.height)),Size((int)(0.5*r.width),(int)(0.5*r.height)),0,0,360,Scalar(1),-1);
	}
	
	if (!getIsNovice() || _template_list.size()>0)
	{
		list<AppTemplate*>::iterator it;
		float c=0;
		for (it=_template_list.begin();it!=_template_list.end();it++)
		{
			AppTemplate* tr=*it;
			Point shift_vector=tr->getShiftVector()*_window_size.width;//the tracking window
			tr->calcBP(frame_set,final_occ_map,roi_win+shift_vector);
			_confidence_map+=tr->getConfidenceMap();//
			c+=1;//
		} 
		_confidence_map/=MAX(c,0.0001);
	}
	else//when suspension, use the last deleted tracker to draw the confidence map
	{
		Point shift_vector=_retained_template->getShiftVector()*_window_size.width;
		_retained_template->calcBP(frame_set,final_occ_map,roi_win+shift_vector);
		_confidence_map+=_retained_template->getConfidenceMap();
	}	
}
void EnsembleTracker::track(const Mat* frame_set,Mat& occ_map)
{
	// update covariance of kalman filter
	updateKfCov(getBodysizeResult().width);

	//for calculation of hitting rate
	_record_idx=(_record_idx+1)-_recentHitRecord.cols*((_record_idx+1)/_recentHitRecord.cols);
	_recentHitRecord.at<double>(0,_record_idx)=0.0;
	_recentHitRecord.at<double>(1,_record_idx)=0.0;

	// reset the flag, it will be set as true if a new detection is matched
	setAddNew(false);

	_kf.predict();

	
	if (getIsNovice())
		_novice_status_count++;

	double alpha;
	//matching radius updating
	alpha = MIN(_phi1_*sqrt(_kf.errorCovPre.at<float>(0,0))/(double)_result_bodysize_temp.width+_phi2_,_phi_max_);
	_match_radius=alpha*_result_bodysize_temp.width;

	// center the initial window in the confidence map. NOTE: the confidence map's location is using kalman filter (see calBP)
	Rect iniWin(
		(int)(0.5*(_confidence_map.cols-_window_size.width)),
		(int)(0.5*(_confidence_map.rows-_window_size.height)),
		(int)_window_size.width,
		(int)_window_size.height);
	meanShift(_confidence_map,iniWin,TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));

	// locate the result window in the picture and update the body-size window too 
	_result_temp=iniWin+Point(_cm_win.x,_cm_win.y);
	_result_bodysize_temp=scaleWin(_result_temp,1/TRACKING_TO_BODYSIZE_RATIO);

	if (getIsNovice())
	{
		// copy the errorCovePre  and observation to post without correction
		_kf.errorCovPre.copyTo(_kf.errorCovPost);
		_kf.statePost.at<float>(0,0)=(float)(_result_temp.x+0.5*_result_temp.width);
		_kf.statePost.at<float>(1,0)=(float)(_result_temp.y+0.5*_result_temp.height);
	}
	else
	{		
		// register the last non-novice position
		_result_last_no_sus=_result_temp;
		// do correction
		correct_kf(_kf,_result_temp);
	}	
}
void EnsembleTracker::calcScore()
{
	Rect roi_result=_result_temp-Point(_cm_win.x,_cm_win.y);
	Rect roi_bodysize=scaleWin(roi_result,1/TRACKING_TO_BODYSIZE_RATIO);

	if (getIsNovice())
		return;

	list<AppTemplate*>::iterator it;
	for (it=_template_list.begin();it!=_template_list.end();it++)
	{
		if (!getIsNovice())
		{
			(*it)->calcScore(roi_result,roi_bodysize);
		}		
	}
	_template_list.sort(compareTemplate);//high to low
}
void EnsembleTracker::deletePoorTemplate(double threshold)
{
	if (getIsNovice())
		return;
	
	list<AppTemplate*>::iterator it;
	for (it=_template_list.begin();it!=_template_list.end();)
	{
		AppTemplate* tr=*it;
		if (tr->getScore()<=threshold)
		{
			if (_template_list.size()==1)
			{
				delete _retained_template;
				_retained_template =tr;
				_template_list.erase(it++);
				continue;
			}
			delete tr;
			_template_list.erase(it++);
		}
		else
			it++;
	}
}
void EnsembleTracker::deletePoorestTemplate()
{
	delete _template_list.back();
	_template_list.pop_back();
}
void EnsembleTracker::demote()
{
	_is_novice=true;
	_novice_status_count=0;
}
void EnsembleTracker::promote()
{
	_is_novice=false;
	_novice_status_count=0;

	// reset the kalman filter's post state
	Rect win=getResult();
	_kf.statePost.at<float>(0,0)=(float)(win.x+0.5*win.width);
	_kf.statePost.at<float>(1,0)=(float)(win.y+0.5*win.height);

}
void EnsembleTracker::updateMatchHist(Mat& frame)
{
	Rect roi_result=getResult();
	Rect roi_result_bodysize=scaleWin(roi_result,1/TRACKING_TO_BODYSIZE_RATIO);
	Rect win=roi_result_bodysize&Rect(0,0,frame.cols,frame.rows);
	Mat roi(frame,win);
	Mat temp;
	Mat mask_win=Mat::zeros(roi.size(),CV_8UC1);
	ellipse(mask_win,Point((int)(0.5*mask_win.cols),(int)(0.5*mask_win.rows)),Size((int)(0.35*mask_win.cols),(int)(0.35*mask_win.rows)),0,0,360,Scalar(1),-1);
	calcHist(&roi,1,channels,mask_win,temp,3,histSize,hRange);
	normalize(temp,temp,1,0,NORM_L1);
	if (_result_history.size()==1)
	{
		hist_match_score=1;
		hist=temp;
		return;
	}
	hist_match_score=compareHist(hist,temp,CV_COMP_INTERSECT);
	hist+=HIST_MATCH_UPDATE*temp;
	normalize(hist,hist,1,0,NORM_L1);
}
double EnsembleTracker::compareHisto(Mat& frame, Rect win)
{
	Rect roi_win=win & Rect(0,0,frame.cols,frame.rows);
	Mat roi(frame,roi_win);
	Mat temp;
	Mat mask_win=Mat::zeros(roi.size(),CV_8UC1);
	ellipse(mask_win,Point((int)(0.5*mask_win.cols),(int)(0.5*mask_win.rows)),Size((int)(0.35*mask_win.cols),(int)(0.35*mask_win.rows)),0,0,360,Scalar(1),-1);
	calcHist(&roi,1,channels,Mat(),temp,3,histSize,hRange);
	normalize(temp,temp,1,0,NORM_L1);
	return compareHist(hist,temp,CV_COMP_INTERSECT);
}
void EnsembleTracker::registerTrackResult()
{
	if (!getIsNovice())
	{
		_result_history.push_back(_result_temp);
		_result_last_no_sus=_result_temp;
		if (_result_history.size()==1)// new tracker
		{
			init_kf(_result_temp);
			//_filter_result_history.push_back(_result_temp);
		}
		else
		{
			//Point center((int)_kf.statePost.at<float>(0,0),(int)_kf.statePost.at<float>(1,0));
			//_filter_result_history.push_back(Rect((int)(center.x-0.5*_result_temp.width),(int)(center.y-0.5*_result_temp.height),_result_temp.width,_result_temp.height));
		}									
	}
	else //when _is_novice, it only depend on motion model
	{
		//Point center((int)_kf.statePost.at<float>(0,0),(int)_kf.statePost.at<float>(1,0));
		//Rect lastWin=_filter_result_history.back();
		//Rect win=Rect((int)(center.x-0.5*_result_temp.width),(int)(center.y-0.5*_result_temp.height),_result_temp.width,_result_temp.height);
		//_filter_result_history.push_back(win);
		_result_history.push_back(_result_temp);//****************
	}		
}	