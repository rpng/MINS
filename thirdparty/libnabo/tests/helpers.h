/*

Copyright (c) 2010--2011, Stephane Magnenat, ASL, ETHZ, Switzerland
You can contact the author at <stephane at magnenat dot net>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef __NABE_TEST_HELPERS_H
#define __NABE_TEST_HELPERS_H

#include "nabo/nabo.h"
#include <chrono>
#include <limits>
#include <iostream>
#include <fstream>

#include <cstdint>

using namespace std;
using namespace Nabo;

template<typename T>
typename NearestNeighbourSearch<T>::Matrix load(const char *fileName)
{
	typedef typename NearestNeighbourSearch<T>::Matrix Matrix;
	
	ifstream ifs(fileName);
	if (!ifs.good())
	{
		cerr << "Cannot open file "<< fileName << endl;
		exit(1);
	}
	
	vector<T> data;
	int dim(0);
	bool firstLine(true);
	
	while (!ifs.eof())
	{
		char line[1024];
		ifs.getline(line, sizeof(line));
		line[sizeof(line)-1] = 0;
		
		char *token = strtok(line, " \t,;");
		while (token)
		{
			if (firstLine)
				++dim;
			data.push_back(atof(token));
			token = strtok(NULL, " \t,;"); // FIXME: non reentrant, use strtok_r
		}
		firstLine = false;
	}
	
	return Matrix::Map(&data[0], dim, data.size() / dim);
}

template<typename T>
typename NearestNeighbourSearch<T>::Vector createQuery(const typename NearestNeighbourSearch<T>::Matrix& d, const NearestNeighbourSearch<T>& kdt, const int i, const int method)
{
	typedef typename NearestNeighbourSearch<T>::Vector VectorT;
	if (method < 0)
	{
		VectorT q = d.col(i % d.cols());
		T absBound = 0;
		for (int j = 0; j < q.size(); ++j)
			absBound += kdt.maxBound(j) - kdt.minBound(j);
		absBound /= (-method); // divided by -method
#ifdef EIGEN3_API
		if (i < d.cols())
			q.array() += absBound;
		else
			q.array() -= absBound;
#else // EIGEN3_API
		if (i < d.cols())
			q.cwise() += absBound;
		else
			q.cwise() -= absBound;
#endif // EIGEN3_API
		return q;
	}
	else
	{
		VectorT q(kdt.minBound.size());
		for (int j = 0; j < q.size(); ++j)
			q(j) = kdt.minBound(j) + T(rand()) * (kdt.maxBound(j) - kdt.minBound(j)) / T(RAND_MAX);
		return q;
	}
}

template<typename T>
typename NearestNeighbourSearch<T>::Matrix createQuery(const typename NearestNeighbourSearch<T>::Matrix& d, const int itCount, const int method)
{
	typedef typename NearestNeighbourSearch<T>::Matrix MatrixT;
	typedef Nabo::NearestNeighbourSearch<T> NNS;
	NNS* nns = NNS::create(d, d.rows(), typename NNS::SearchType(0));
	MatrixT q(d.rows(), itCount);
	for (int i = 0; i < itCount; ++i)
		q.col(i) = createQuery<T>(d, *nns, i, method);
	delete nns;
	return q;
}

struct timer
{
	using Clock = std::chrono::high_resolution_clock;
	using Time = Clock::time_point;

	timer():_start_time(curTime()){ }
	void restart() { _start_time = curTime(); }
	double elapsed() const                  // return elapsed time in seconds
	{ 
		using namespace std::chrono;
		return duration_cast<duration<double>>(curTime() - _start_time).count();
	}

private:
	Time curTime() const {
		return Clock::now();
	}
	Time _start_time;
};

#endif // __NABE_TEST_HELPERS_H

/* vim: set ts=8 sw=8 tw=0 noexpandtab cindent softtabstop=8 :*/
