#include "clearMOT.h"
using namespace std;

void C_Mot::dealWith(vector<ResultWindow> gt,vector<ResultWindow> hp)
{
	frame_count++;
	matched_couple.clear();
	switched_couple.clear();
	miss_detection.clear();
	false_alarm.clear();
	/*
	vector<ResultWindow> hp_temp;
	for (int i=0;i<hp.size();i++)
	{
		Rect intersect=hp[i].window&Rect(0,0,1920,1080);
		if (hp[i].ID!=8 && hp[i].ID!=15 && hp[i].ID!=29)
			hp_temp.push_back(hp[i]);
	}
	hp=hp_temp;
	*/
	//update gt_count
	gt_count+=gt.size();
	//book keeping of hypothesis
	map<int,int>::iterator hp_book_it;
	for (vector<ResultWindow>::iterator hp_it=hp.begin();hp_it!=hp.end();hp_it++)
	{
		hp_book_it=hp_book_keeping.find((*hp_it).ID);
		if (hp_book_it==hp_book_keeping.end())
		{
			hp_book_keeping.insert(pair<int,int>((*hp_it).ID,-1));
		}
	}	
	//first associate the already tracked objects
	vector<ResultWindow> gt_to_match;
	vector<ResultWindow> hp_to_match;
	map<int,int>::iterator gt_book_it;
	for (vector<ResultWindow>::iterator gt_it=gt.begin();gt_it!=gt.end();gt_it++)
	{
		bool flag_matched=false;
		gt_book_it=gt_book_keeping.find((*gt_it).ID);
		if (gt_book_it!=gt_book_keeping.end())
		{
			int last_match=(*gt_book_it).second;
			if (last_match>=0)
			{
				for (vector<ResultWindow>::iterator hp_it=hp.begin();hp_it!=hp.end();hp_it++)
				{
					if ((*hp_it).ID==last_match && getDistance((*hp_it),(*gt_it))<1-IOU_threshold)// getDistance returns 1-IOU
					{
						//match found
						match_count++;
						error_sum+=getDistance((*hp_it),(*gt_it));
						matched_couple.push_back((*gt_it));
						matched_couple.push_back((*hp_it));
						hp.erase(hp_it);
						flag_matched=true;
						break;						
					}						
				}
			}
		}
		else//new object, add to book keeping
		{
			gt_book_keeping.insert(pair<int,int>((*gt_it).ID,-1));
		}
		if (!flag_matched)//not matched, add to matching list
		{
			gt_to_match.push_back((*gt_it));
		}
	}
	for (size_t i=0;i<hp.size();i++)
	{
		hp_to_match.push_back(hp[i]);
	}
	//make the matrix for hungarian algorithm
	int gt_size=gt_to_match.size();
	int hp_size=hp_to_match.size();
	if (gt_size*hp_size>0)
	{
		Matrix<double> matrix(gt_size, hp_size+gt_size);
		for (int i=0;i<matrix.rows();i++)
		{
			for (int j=0;j<matrix.columns();j++)
			{
				if (j<hp_size)
				{
					double dis=getDistance(gt_to_match[i],hp_to_match[j]);
					matrix(i,j)=dis<1-IOU_threshold ? dis:INFINITY;
				}
				else
				{
					matrix(i,j)=2.0;
				}
			}
		}
		//solve the bipartite matching
		Munkres m;
		m.solve(matrix);

		//arrange the outcome
		for (int i=0;i<matrix.rows();i++)
		{
			bool flag_matched=false;
			for (int j=0;j<hp_size;j++)
			{
				if (matrix(i,j)==0)
				{
					//match found
					flag_matched=true;
					match_count++;
					error_sum+=getDistance(gt_to_match[i],hp_to_match[j]);

					//check ID switch
					int gt_last_match=gt_book_keeping[gt_to_match[i].ID];
					if (!(gt_last_match==hp_to_match[j].ID || (gt_last_match==-1 && hp_book_keeping[hp_to_match[j].ID]==-1)))
					{
						IS_sum++;
						//cout<<"is"<<endl;
						//waitKey();
						switched_couple.push_back(gt_to_match[i]);
						switched_couple.push_back(hp_to_match[j]);
					}
					else
					{
						matched_couple.push_back(gt_to_match[i]);
						matched_couple.push_back(hp_to_match[j]);
					}
					gt_book_keeping[gt_to_match[i].ID]=hp_to_match[j].ID;
					hp_book_keeping[hp_to_match[j].ID]=gt_to_match[i].ID;
					break;
				}
			}
			if (!flag_matched)
			{
				M_sum++;
				//cout<<"ms"<<endl;
				//waitKey();
				miss_detection.push_back(gt_to_match[i]);
			}		
		}
		for (int j=0;j<hp_size;j++)
		{
			bool flag_matched=false;
			for (int i=0;i<matrix.rows();i++)
			{
				if (matrix(i,j)==0)
				{
					flag_matched=true;
					break;
				}
			}
			if (!flag_matched)
			{
				FP_sum++;
				//cout<<"fp"<<endl;
				//waitKey();
				false_alarm.push_back(hp_to_match[j]);
			}
		}
	}
	else
	{
		if (gt_size==0)
		{
			for (int j=0;j<hp_size;j++)
			{
				FP_sum++;
				//cout<<"fp"<<endl;
				//waitKey();
				false_alarm.push_back(hp_to_match[j]);
			}
		}
		else
		{
			for (int i=0;i<gt_size;i++)
			{
				M_sum++;
				//cout<<"missing"<<endl;
				//waitKey();
				miss_detection.push_back(gt_to_match[i]);
			}
		}
	}
}
MOTResult C_Mot::getMOT(bool is_det)
{
	MOTResult rt;
	rt.MOTP=match_count>0 ? 1-error_sum/match_count:0;
	if (!is_det)
		rt.MOTA=gt_count>0 ? 1-(M_sum+FP_sum+IS_sum)/gt_count:0;
	else
		rt.MOTA=gt_count>0 ? 1-(M_sum+FP_sum)/gt_count:0;
	rt.missing=(int)M_sum;
	rt.false_positive=(int)FP_sum;
	rt.ID_switch=(int)IS_sum;
	cout<<"MOTP: "<<rt.MOTP<<endl;
	cout<<"MOTA: "<<rt.MOTA<<endl;
	cout<<"missing: "<<rt.missing<<endl;
	cout<<"fp: "<<rt.false_positive<<endl;
	if (!is_det)
		cout<<"IS: "<<rt.ID_switch<<endl;
	else
	{
		cout<<"Precision: "<<match_count/(match_count+FP_sum)<<endl;
		cout<<"Recall: "<<match_count/gt_count<<endl;
	}
	return rt;
}
void C_Mot::paintFrame(Mat& frame)
{
	for (size_t i=0;i<matched_couple.size();i++)
	{
		rectangle(frame,matched_couple[i].window,Scalar(0,255,0),2);
		i++;
		rectangle(frame,matched_couple[i].window,Scalar(0,255,0),1);
	}
	for (size_t i=0;i<switched_couple.size();i++)
	{
		rectangle(frame,switched_couple[i].window,Scalar(0,255,255),2);
		i++;
		rectangle(frame,switched_couple[i].window,Scalar(0,255,255),1);
	}
	for (size_t i=0;i<miss_detection.size();i++)
	{
		rectangle(frame,miss_detection[i].window,Scalar(255,0,255),2);
	}
	for (size_t i=0;i<false_alarm.size();i++)
	{
		rectangle(frame,false_alarm[i].window,Scalar(0,0,255),1);
	}
}

/* ****** ****** */

ResultParser::ResultParser(const char* filename,double r,double h,double w)
	:ratio(r),h_ratio(h),w_ratio(w)
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
			/*if (xmlStrcmp(frame->name,BAD_CAST"dataset"))
			{
				cout<<"bad file"<<endl;
				open_success=false;
			}**/
			frame=frame->children;
			while (xmlStrcmp(frame->name,BAD_CAST"frame"))
			{
				frame=frame->next;
			}			
		}	
	}
vector<ResultWindow> ResultParser::readNextFrame()
{
	vector<ResultWindow> output;
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
			int ID;
			Rect res;
			double response;

			temp=xmlGetProp(object,BAD_CAST"confidence");
			if (temp!=NULL)
				response=string2float((char*)temp);
			else
				response=1;
			xmlFree(temp);

			temp=xmlGetProp(object,BAD_CAST"id");
			if(temp!=NULL)
				ID=string2int((char*)temp);
			else
				ID=-1;
			xmlFree(temp);



			xmlNodePtr box=object->children;
			while (xmlStrcmp(box->name,BAD_CAST"box") )
			{
				box=box->next;
			}
			temp=xmlGetProp(box,BAD_CAST"h");
			res.height=(int)(string2float((char*)temp)*ratio*h_ratio+0.5);
			xmlFree(temp);
			temp=xmlGetProp(box,BAD_CAST"w");
			res.width=(int)(string2float((char*)temp)*ratio*w_ratio+0.5);
			xmlFree(temp);
			temp=xmlGetProp(box,BAD_CAST"xc");
			res.x=(int)(string2float((char*)temp)*ratio-0.5*res.width+0.5);
			xmlFree(temp);
			temp=xmlGetProp(box,BAD_CAST"yc");
			res.y=(int)(string2float((char*)temp)*ratio-0.5*res.height+0.5);
			xmlFree(temp);

			output.push_back(ResultWindow(ID,res,response));
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
	return output;
}