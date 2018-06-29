/*
 *      C++ Main Header of Markov Chain for OpenCV (CvMC).
 *
 * Copyright (c) 2012 Omid B. Sakhi
 * All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef CVMC_H
#define CVMC_H

using namespace std;

class CvMC {
public:
	CvMC(){};

	void mutualAlign(const cv::Mat &seq,const cv::Mat &_TRANS,const cv::Mat &_EMIS, const cv::Mat &_INIT, double &logpseq, cv::Mat &PSTATES, cv::Mat &FORWARD, cv::Mat &BACKWARD, int cnt)
	{
	  /*
			seq 1xT array of sequences of observations (states are 0-M)
			TRANS NxN matrix of a model of HMM with N possible states
			EMIS NxM matrix, with N being number of decodable states and M being variability of observed states_
			INIT Nx1 - initial probabilities
			lospseg - irelevant
			PSTATES - NxT matrix with decoded probability for N possible states

		*/



			/* A Revealing Introduction to Hidden Markov Models, Mark Stamp */
			// 1. Initialization
		
		//cv::Mat seq = seqin.t();
		cv::Mat TRANS = _TRANS.clone();
		cv::Mat EMIS = _EMIS.clone();
		cv::Mat INIT = _INIT.clone();
		//correctModel(TRANS,EMIS,INIT);
		int T = cnt;//seq.cols; // number of element per sequence
		int C = seq.rows; // number of sequences
		int N = TRANS.rows; // number of states | also N = TRANS.cols | TRANS = A = {a_{i,j}} - NxN
		int M = EMIS.cols; // number of observations | EMIS = B = {b_{j}(k)} - NxM
		double intsum;
		int pos;
		// compute a_{0}
		FORWARD = cv::Mat(N,T+1,CV_64F);
		BACKWARD = cv::Mat(N,T+1,CV_64F);
		PSTATES = cv::Mat(N,T,CV_64F);
		cv::Mat S = cv::Mat(1,T+1,CV_64F);

		//Forward probabilities
		for (int count=0;count<T+1;count++)
		{
			//FORWARD.at<double>(0,count) = 0;
			if(count==0)
			{
				FORWARD.at<double>(0,count) = 1;
				S.at<double>(0,count) = 1;
			}
			else
			{
				for(int state=0;state<N;state++)
				{
					FORWARD.at<double>(state,count) = 0;
					intsum = 0;
					for(int substate=0;substate<N;substate++)
					{
						//cout << "cp1\n";
						intsum +=((double)FORWARD.at<double>(substate,count-1)) * ((double)TRANS.at<double>(substate,state));
						//cout << "cp2\n";
					}

					pos = (int)(seq.at<double>(0,count-1));
//cout << "cp3\n";
					FORWARD.at<double>(state,count) = EMIS.at<double>(state,pos)*intsum;
//cout << "cp4\n";

					//cout << ""<<state<<"\n";
					//cout << ""<<pos<<"\n";
					//cout << ""<<FORWARD.at<double>(state,count)<<"\n";
	                //cout << ""<<EMIS.at<double>(state,pos)<<"\n";
	                //cout << ""<<intsum<<"\n";
				}
				//cout << "\n";

	            intsum=0;
	            for(int substate=0;substate<N;substate++)
	            {
	                intsum +=FORWARD.at<double>(substate,count);
	            }

	            S.at<double>(0,count) = intsum;

	            //cout << ""<<S.at<double>(0,count)<<"\n";

	            for(int substate=0;substate<N;substate++)
	            {
	                FORWARD.at<double>(substate,count) =FORWARD.at<double>(substate,count)/S.at<double>(0,count);
	            }
			}
			//cout << "\n\n\n";
		}

		//Backward probabilities
		for (int count=T;count>0;count--)
		{
			//FORWARD.at<double>(0,count) = 0;
			if(count==T)
			{
			    for(int state=0;state<N;state++)
	                BACKWARD.at<double>(state,T) = 1;
			}
			else
			{
					for(int state=0;state<N;state++)
					{
						intsum=0;
						for(int substate=0;substate<N;substate++)
						{
							pos = (int)(seq.at<double>(0,count));
							intsum += ((double)TRANS.at<double>(state,substate))*((double)BACKWARD.at<double>(substate,count+1)) * EMIS.at<double>(substate,pos);
	                        //cout << "S"<<state<<"\n";
	                        //cout << "T"<<TRANS.at<double>(state,substate)<<"\n";
	                        //cout << "B"<<BACKWARD.at<double>(substate,count+1)<<"\n";
	                        //cout << "E"<<EMIS.at<double>(state,pos)<<"\n";
						}
						//cout << "S"<<intsum<<"\n";
	                    BACKWARD.at<double>(state,count) = (1/S.at<double>(count+1)) * intsum;
					}
			}
			//cout << "\n\n\n";
		}

	        for (int count=0;count<T;count++)
			{
					for(int state=0;state<N;state++)
						PSTATES.at<double>(state,count) = FORWARD.at<double>(state,count+1)*BACKWARD.at<double>(state,count+1);

					/*intsum=0;
					for(int substate=0;substate<N;substate++)
					{
						intsum +=PSTATES.at<double>(substate,count);
					}
					for(int substate=0;substate<N;substate++)
					{
						PSTATES.at<double>(substate,count) =PSTATES.at<double>(substate,count)/intsum;
					}*/
			}
		TRANS.release();
		EMIS.release();
		INIT.release();

	}

	static void printModel(const cv::Mat &TRANSb,const cv::Mat &TRANSa,const cv::Mat &INIT)
	{
		std::cout << "\nTRANSb: \n";
		for (int r=0;r<TRANSb.rows;r++)
		{
			for (int c=0;c<TRANSb.cols;c++)
				std::cout << TRANSb.at<double>(r,c) << " ";
			std::cout << "\n";
		}
		std::cout << "\nTRANSa: \n";
		for (int r=0;r<TRANSa.rows;r++)
		{
			for (int c=0;c<TRANSa.cols;c++)
				std::cout << TRANSa.at<double>(r,c) << " ";
			std::cout << "\n";
		}
		std::cout << "\nINIT: \n";
		for (int r=0;r<INIT.rows;r++)
		{
			for (int c=0;c<INIT.cols;c++)
				std::cout << INIT.at<double>(r,c) << " ";
			std::cout << "\n";
		}
		std::cout << "\n";
	}
};

#endif
