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

	void mutualAlign(const cv::Mat &seq,const cv::Mat &_TRANSb,const cv::Mat &_TRANSa, const cv::Mat &_INIT, double &logpseq, cv::Mat &PSTATES, int cnt)
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
		cv::Mat TRANSb  = _TRANSb.clone();
		cv::Mat TRANSa  = _TRANSa.clone();
		cv::Mat INIT    = _INIT.clone();

		int T = cnt;//seq.cols; // number of element per sequence
		int C = seq.rows; 	// number of sequences
		int N = TRANSb.rows; 	// number of states | also N = TRANS.cols | 
					// TRANS = A = {a_{i,j}} - NxN
		cv::Mat currStateProb = TRANSb.row(seq.at<double>(0,cnt));
		//cv::sortIdx(source, dst, CV_SORT_EVERY_ROW + CV_SORT_ASCENDING);


		TRANSb.release();
		TRANSa.release();
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
