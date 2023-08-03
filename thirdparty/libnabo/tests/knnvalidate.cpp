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

#include "nabo/nabo.h"
#include "helpers.h"
//#include "experimental/nabo_experimental.h"
#include <iostream>
#include <fstream>
#include <stdexcept>

using namespace std;
using namespace Nabo;

template<typename T, typename CloudType>
struct Loader
{
	void loadMatrix(const char *fileName)
	{
		data = load<T>(fileName);
	}
	CloudType getValue() const
	{
		return data;
	}
private:
	CloudType data;
};

template<typename T>
struct Loader<T, Eigen::Map<const Eigen::Matrix<T, 3, Eigen::Dynamic>, Eigen::Aligned> >
{
	void loadMatrix(const char *fileName)
	{
		data = load<T>(fileName);
	}
	Eigen::Map<const Eigen::Matrix<T, 3, Eigen::Dynamic>, Eigen::Aligned> getValue() const
	{
		return Eigen::Map<const Eigen::Matrix<T, 3, Eigen::Dynamic>, Eigen::Aligned>(data.data(), 3, data.cols());
	}

private:
	Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> data;
};

template<typename T, typename CloudType>
void validate(const char *fileName, const int K, const int dim, const int method, const T maxRadius)
{
	typedef Nabo::NearestNeighbourSearch<T, CloudType> NNS;
	typedef vector<NNS*> NNSV;
	typedef typename NNS::Matrix Matrix;
	typedef typename NNS::Vector Vector;
	typedef typename NNS::IndexMatrix IndexMatrix;
	
	Loader<T, CloudType> loader;
	loader.loadMatrix(fileName);

	// check if file is ok
	const CloudType d = loader.getValue();
	if (d.rows() != dim)
	{
		cerr << "Provided data has " << d.rows() << " dimensions, but the requested dimensions were " << dim << endl;
		exit(2);
	}
	if (K >= d.cols())
	{
		cerr << "Requested more nearest neighbour than points in the data set" << endl;
		exit(2);
	}
	
	// create different methods
	NNSV nnss;
	unsigned searchTypeCount(NNS::SEARCH_TYPE_COUNT);
	#ifndef HAVE_OPENCL
	searchTypeCount -= 3;
	#endif // HAVE_OPENCL
	for (unsigned i = 0; i < searchTypeCount; ++i)
		nnss.push_back(NNS::create(d, d.rows(), typename NNS::SearchType(i)));
	//nnss.push_back(new KDTreeBalancedPtInLeavesStack<T>(d, false));
	
	
	// check methods together
	const int itCount(method != -1 ? method : d.cols() * 2);
	
	/*
	// element-by-element search
	typedef typename NNS::IndexVector IndexVector;
	for (int i = 0; i < itCount; ++i)
	{
		const Vector q(createQuery<T>(d, *nnss[0], i, method));
		const IndexVector indexes_bf(nnss[0]->knn(q, K, 0, NNS::SORT_RESULTS));
		for (size_t j = 1; j < nnss.size(); ++j)
		{
			const IndexVector indexes_kdtree(nnss[j]->knn(q, K, 0, NNS::SORT_RESULTS));
			if (indexes_bf.size() != K)
			{
				cerr << "Different number of points found between brute force and request" << endl;
				exit(3);
			}
			if (indexes_bf.size() != indexes_kdtree.size())
			{
				cerr << "Different number of points found between brute force and NNS type "<< j  << endl;
				exit(3);
			}
			for (size_t k = 0; k < size_t(K); ++k)
			{
				Vector pbf(d.col(indexes_bf[k]));
				//cerr << indexes_kdtree[k] << endl;
				Vector pkdtree(d.col(indexes_kdtree[k]));
				if (fabsf((pbf-q).squaredNorm() - (pkdtree-q).squaredNorm()) >= numeric_limits<float>::epsilon())
				{
					cerr << "Method " << j << ", cloud point " << i << ", neighbour " << k << " of " << K << " is different between bf and kdtree (dist " << (pbf-pkdtree).norm() << ")\n";
					cerr << "* query:\n";
					cerr << q << "\n";
					cerr << "* indexes " << indexes_bf[k] << " (bf) vs " <<  indexes_kdtree[k] << " (kdtree)\n";
					cerr << "* coordinates:\n";
					cerr << "bf: (dist " << (pbf-q).norm() << ")\n";
					cerr << pbf << "\n";
					cerr << "kdtree (dist " << (pkdtree-q).norm() << ")\n";
					cerr << pkdtree << endl;
					exit(4);
				}
			}
		}
	}
	*/
	// create big query
	// check all-in-one query
	Matrix q(createQuery<T>(d, itCount, method));
	IndexMatrix indexes_bf(K, q.cols());
	Matrix dists2_bf(K, q.cols());
	nnss[0]->knn(q, indexes_bf, dists2_bf, K, 0, NNS::SORT_RESULTS, maxRadius);
	assert(indexes_bf.cols() == q.cols());
	for (size_t j = 1; j < nnss.size(); ++j)
	{
		IndexMatrix indexes_kdtree(K, q.cols());
		Matrix dists2_kdtree(K, q.cols());
		nnss[j]->knn(q, indexes_kdtree, dists2_kdtree, K, 0, NNS::SORT_RESULTS, maxRadius);
		if (indexes_bf.rows() != K)
		{
			cerr << "Different number of points found between brute force and request" << endl;
			exit(3);
		}
		if (indexes_bf.cols() != indexes_kdtree.cols())
		{
			cerr << "Different number of points found between brute force and NNS type "<< j  << endl;
			exit(3);
		}
		
		for (int i = 0; i < q.cols(); ++i)
		{
			for (size_t k = 0; k < size_t(K); ++k)
			{
				if (dists2_bf(k,i) == NNS::InvalidValue)
					continue;
				const int pbfi(indexes_bf(k,i));
				const Vector pbf(d.col(pbfi));
				const int pkdt(indexes_kdtree(k,i));
				if (pkdt < 0 || pkdt >= d.cols())
				{
					cerr << "Method " << j << ", query point " << i << ", neighbour " << k << " of " << K << " has invalid index " << pkdt << " out of range [0:" << d.cols() << "[" << endl;
					exit(4);
				}
				const Vector pkdtree(d.col(pkdt));
				const Vector pq(q.col(i));
				const T distDiff(fabsf((pbf-pq).squaredNorm() - (pkdtree-pq).squaredNorm()));
				if (distDiff > numeric_limits<T>::epsilon())
				{
					cerr << "Method " << j << ", query point " << i << ", neighbour " << k << " of " << K << " is different between bf and kdtree (dist2 " << distDiff << ")\n";
					cerr << "* query point:\n";
					cerr << pq << "\n";
					cerr << "* indexes " << pbfi << " (bf) vs " << pkdt << " (kdtree)\n";
					cerr << "* coordinates:\n";
					cerr << "bf: (dist " << (pbf-pq).norm() << ")\n";
					cerr << pbf << "\n";
					cerr << "kdtree (dist " << (pkdtree-pq).norm() << ")\n";
					cerr << pkdtree << endl;
					cerr << "* bf neighbours:\n";
					for (int l = 0; l < K; ++l)
						cerr << indexes_bf(l,i) << " (dist " << (d.col(indexes_bf(l,i))-pq).norm() << ")\n";
					cerr << "* kdtree neighbours:\n";
					for (int l = 0; l < K; ++l)
						cerr << indexes_kdtree(l,i) << " (dist " << (d.col(indexes_kdtree(l,i))-pq).norm() << ")\n";
					exit(4);
				}
			}
		}
	}
	
// 	cout << "\tstats kdtree: "
// 		<< kdt.getStatistics().totalVisitCount << " on "
// 		<< (long long)(itCount) * (long long)(d.cols()) << " ("
// 		<< (100. * double(kdt.getStatistics().totalVisitCount)) /  (double(itCount) * double(d.cols())) << " %"
// 		<< ")\n" << endl;
	
	// delete searches
	for (typename NNSV::iterator it(nnss.begin()); it != nnss.end(); ++it)
		delete (*it);
}

int main(int argc, char* argv[])
{
	if (argc < 4)
	{
		cerr << "Usage " << argv[0] << " DATA K DIM METHOD [MAX_RADIUS]" << endl;
		return 1;
	}
	
	const int K(atoi(argv[2]));
	const int dim(atoi(argv[3]));
	const int method(atoi(argv[4]));
	const float maxRadius(argc >= 6 ? float(atof(argv[5])) : numeric_limits<float>::infinity());
	
	if (dim == 3)
	{
		validate<float, Eigen::MatrixXf>(argv[1], K, dim, method, maxRadius);
		validate<float, Eigen::Matrix3Xf>(argv[1], K, dim, method, maxRadius);
		validate<float, Eigen::Map<const Eigen::Matrix3Xf, Eigen::Aligned> >(argv[1], K, dim, method, maxRadius);
	} else
	{
		validate<float, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> >(argv[1], K, dim, method, maxRadius);
	}
	//validate<double>(argv[1], K, method);
	
	return 0;
}
