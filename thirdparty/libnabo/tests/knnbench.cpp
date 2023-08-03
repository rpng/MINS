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

// currently disable FLANN
#undef HAVE_FLANN

#include "nabo/nabo.h"
#include "experimental/nabo_experimental.h"
#include "helpers.h"
#ifdef HAVE_ANN
	#include "ANN.h"
#endif // HAVE_ANN
#ifdef HAVE_FLANN
	#include "flann/flann.hpp"
#endif // HAVE_FLANN
#include <iostream>
#include <fstream>
#include <stdexcept>

using namespace std;
using namespace Nabo;

typedef Nabo::NearestNeighbourSearch<double>::Matrix MatrixD;
typedef Nabo::NearestNeighbourSearch<double>::Vector VectorD;
typedef Nabo::NearestNeighbourSearch<double>::Index IndexD;
typedef Nabo::NearestNeighbourSearch<double>::IndexVector IndexVectorD;
typedef Nabo::NearestNeighbourSearch<float>::Matrix MatrixF;
typedef Nabo::NearestNeighbourSearch<float>::Vector VectorF;
typedef Nabo::NearestNeighbourSearch<float>::Index IndexF;
typedef Nabo::NearestNeighbourSearch<float>::IndexVector IndexVectorF;
typedef Nabo::BruteForceSearch<double> BFSD;
typedef Nabo::BruteForceSearch<float> BFSF;
// typedef Nabo::KDTreeBalancedPtInNodesPQ<double> KDTD1;
// typedef Nabo::KDTreeBalancedPtInNodesStack<double> KDTD2;
// struct KDTD3: public Nabo::KDTreeBalancedPtInLeavesStack<double>
// {
// 	KDTD3(const Matrix& cloud):
// 		Nabo::KDTreeBalancedPtInLeavesStack<double>(cloud, true)
// 	{}
// };
// struct KDTD4: public Nabo::KDTreeBalancedPtInLeavesStack<double>
// {
// 	KDTD4(const Matrix& cloud):
// 		Nabo::KDTreeBalancedPtInLeavesStack<double>(cloud, false)
// 	{}
// };
// typedef Nabo::KDTreeUnbalancedPtInLeavesImplicitBoundsStack<double,IndexHeapSTL<int,double>> KDTD5A;
// typedef Nabo::KDTreeUnbalancedPtInLeavesImplicitBoundsStack<double,IndexHeapBruteForceVector<int,double>> KDTD5B;
// typedef Nabo::KDTreeUnbalancedPtInLeavesImplicitBoundsStackOpt<double,IndexHeapBruteForceVector<int,double>> KDTD5OB;
// typedef Nabo::KDTreeUnbalancedPtInLeavesImplicitBoundsStackOpt<double,IndexHeapSTL<int,double>> KDTD5OA;
// typedef Nabo::KDTreeUnbalancedPtInLeavesExplicitBoundsStack<double> KDTD6;




struct BenchResult
{
	double creationDuration;
	double executionDuration;
	double visitCount;
	double totalCount;
	
	BenchResult():
		creationDuration(0),
		executionDuration(0),
		visitCount(0),
		totalCount(0)
	{}
	
	void operator +=(const BenchResult& that)
	{
		creationDuration += that.creationDuration;
		executionDuration += that.executionDuration;
		visitCount += that.visitCount;
		totalCount += that.totalCount;
	}
	
	void operator /=(const double factor)
	{
		creationDuration /= factor;
		executionDuration /= factor;
		visitCount /= factor;
		totalCount /= factor;
	}
};
typedef vector<BenchResult> BenchResults;

// template<typename T>
// BenchResult doBench(const Matrix& d, const Matrix& q, const Index K, const int itCount)
// {
// 	BenchResult result;
// 	timer t;
// 	T nns(d);
// 	result.creationDuration = t.elapsed();
// 	
// 	t.restart();
// 	nns.knnM(q, K, 0, 0);
// 	result.executionDuration = t.elapsed();
// 	
// 	result.visitCount = double(nns.getStatistics().totalVisitCount);
// 	result.totalCount = double(itCount) * double(d.cols());
// 	
// 	return result;
// }

template<typename T>
BenchResult doBenchType(const typename NearestNeighbourSearch<T>::SearchType type, 
						const unsigned creationOptionFlags,
						const typename NearestNeighbourSearch<T>::Matrix& d,
						const typename NearestNeighbourSearch<T>::Matrix& q,
						const int K,
						const int /*itCount*/,
						const int searchCount)
{
	typedef NearestNeighbourSearch<T> nnsT;
	typedef typename NearestNeighbourSearch<T>::Matrix Matrix;
	typedef typename NearestNeighbourSearch<T>::IndexMatrix IndexMatrix;
	
	BenchResult result;
	timer t;
	nnsT* nns(nnsT::create(d, d.rows(), type, creationOptionFlags));
	result.creationDuration = t.elapsed();
	
	for (int s = 0; s < searchCount; ++s)
	{
		t.restart();
		IndexMatrix indices(K, q.cols());
		Matrix dists2(K, q.cols());
		const unsigned long visitCount = nns->knn(q, indices, dists2, K, 0, 0);
		result.executionDuration += t.elapsed();
		result.visitCount += double(visitCount);
	}
	result.executionDuration /= double(searchCount);
	result.visitCount /= double(searchCount);
	
	delete nns;
	
	result.totalCount = double(q.cols()) * double(d.cols());
	
	return result;
}

#ifdef HAVE_ANN

BenchResult doBenchANNStack(const MatrixD& d, const MatrixD& q, const int K, const int itCount, const int searchCount)
{
	BenchResult result;
	timer t;
	const int ptCount(d.cols());
	const double **pa = new const double *[d.cols()];
	for (int i = 0; i < ptCount; ++i)
		pa[i] = &d.coeff(0, i);
	ANNkd_tree* ann_kdt = new ANNkd_tree(const_cast<double**>(pa), ptCount, d.rows(), 8);
	result.creationDuration = t.elapsed();
	
	for (int s = 0; s < searchCount; ++s)
	{
		t.restart();
		ANNidx nnIdx[K];
		ANNdist dists[K];
		for (int i = 0; i < itCount; ++i)
		{
			const VectorD& tq(q.col(i));
			ANNpoint queryPt(const_cast<double*>(&tq.coeff(0)));
			ann_kdt->annkSearch(		// search
							queryPt,	// query point
							K,			// number of near neighbours
							nnIdx,		// nearest neighbours (returned)
							dists,		// distance (returned)
							0);			// error bound
		}
		result.executionDuration += t.elapsed();
	}
	result.executionDuration /= double(searchCount);
	
	return result;
}

BenchResult doBenchANNPriority(const MatrixD& d, const MatrixD& q, const int K, const int itCount, const int searchCount)
{
	BenchResult result;
	timer t;
	const int ptCount(d.cols());
	const double **pa = new const double *[d.cols()];
	for (int i = 0; i < ptCount; ++i)
		pa[i] = &d.coeff(0, i);
	ANNkd_tree* ann_kdt = new ANNkd_tree(const_cast<double**>(pa), ptCount, d.rows(), 8);
	result.creationDuration = t.elapsed();
	
	for (int s = 0; s < searchCount; ++s)
	{
		t.restart();
		ANNidx nnIdx[K];
		ANNdist dists[K];
		for (int i = 0; i < itCount; ++i)
		{
			const VectorD& tq(q.col(i));
			ANNpoint queryPt(const_cast<double*>(&tq.coeff(0)));
			ann_kdt->annkPriSearch(		// search
							queryPt,	// query point
							K,			// number of near neighbours
							nnIdx,		// nearest neighbours (returned)
							dists,		// distance (returned)
							0);			// error bound
		}
		result.executionDuration += t.elapsed();
	}
	result.executionDuration /= double(searchCount);
	
	return result;
}

#endif // HAVE_ANN

#ifdef HAVE_FLANN

template<typename T>
BenchResult doBenchFLANN(const Matrix& d, const Matrix& q, const Index K, const int itCount)
{
	BenchResult result;
	const int dimCount(d.rows());
	const int dPtCount(d.cols());
	const int qPtCount(itCount);
	
	flann::Matrix<T> dataset(new T[dPtCount*dimCount], dPtCount, dimCount);
	for (int point = 0; point < dPtCount; ++point)
		for (int dim = 0; dim < dimCount; ++dim)
			dataset[point][dim] = d(dim, point);
		flann::Matrix<T> query(new T[qPtCount*dimCount], qPtCount, dimCount);
	for (int point = 0; point < qPtCount; ++point)
		for (int dim = 0; dim < dimCount; ++dim)
			query[point][dim] = q(dim, point);
	
	flann::Matrix<int> indices(new int[query.rows*K], query.rows, K);
	flann::Matrix<float> dists(new float[query.rows*K], query.rows, K);
	
	// construct an randomized kd-tree index using 4 kd-trees
	timer t;
	flann::Index<T> index(dataset, flann::KDTreeIndexParams(4) /*flann::AutotunedIndexParams(0.9)*/); // exact search
	index.buildIndex();
	result.creationDuration = t.elapsed();
	
	t.restart();
	// do a knn search, using 128 checks
	index.knnSearch(query, indices, dists, int(K), flann::SearchParams(128)); // last parameter ignored because of autotuned
	result.executionDuration = t.elapsed();
	
	dataset.free();
	query.free();
	indices.free();
	dists.free();
	
	return result;
}

#endif // HAVE_FLANN


int main(int argc, char* argv[])
{
	if (argc != 6)
	{
		cerr << "Usage " << argv[0] << " DATA K METHOD RUN_COUNT SEARCH_COUNT" << endl;
		return 1;
	}
	
	const MatrixD dD(load<double>(argv[1]));
	const MatrixF dF(load<float>(argv[1]));
	const int K(atoi(argv[2]));
	const int method(atoi(argv[3]));
	const int itCount(method >= 0 ? method : dD.cols() * 2);
	const int runCount(atoi(argv[4]));
	const int searchCount(atoi(argv[5]));
	
	// compare KDTree with brute force search
	if (K >= dD.cols())
	{
		cerr << "Requested more nearest neighbour than points in the data set" << endl;
		return 2;
	}
	
	// create queries
	MatrixD qD(createQuery<double>(dD, itCount, method));
	MatrixF qF(createQuery<float>(dF, itCount, method));
	
	const char* benchLabels[] =
	{
		//doBench<KDTD1>("Nabo, pt in nodes, priority, balance variance",
		//doBench<KDTD2>("Nabo, pt in nodes, stack, balance variance",
		//doBench<KDTD3>("Nabo, balanced, stack, pt in leaves only, balance variance",
		//"Nabo, balanced, stack, pt in leaves only, balance cell aspect ratio",
		//"Nabo, unbalanced, stack, pt in leaves only, implicit bounds, ANN_KD_SL_MIDPT, STL heap",
		//"Nabo, unbalanced, stack, pt in leaves only, implicit bounds, ANN_KD_SL_MIDPT, brute-force vector heap",
		"Nabo, double, unbalanced, stack, pt in leaves only, implicit bounds, ANN_KD_SL_MIDPT, brute-force vector heap, opt",
		"Nabo, double, unbalanced, stack, pt in leaves only, implicit bounds, ANN_KD_SL_MIDPT, STL heap, opt",
		"Nabo, float, unbalanced, stack, pt in leaves only, implicit bounds, ANN_KD_SL_MIDPT, brute-force vector heap, opt",
		"Nabo, float, unbalanced, stack, pt in leaves only, implicit bounds, ANN_KD_SL_MIDPT, STL heap, opt",
		"Nabo, float, unbalanced, stack, pt in leaves only, implicit bounds, ANN_KD_SL_MIDPT, STL heap, opt, stats",
		#ifdef HAVE_OPENCL
		"Nabo, float, OpenCL, GPU, balanced, points in nodes, stack, implicit bounds, balance aspect ratio, stats",
		"Nabo, float, OpenCL, GPU, balanced, points in leaves, stack, implicit bounds, balance aspect ratio, stats",
		//"Nabo, float, OpenCL, GPU, brute force",
		#endif // HAVE_OPENCL
		//"Nabo, unbalanced, points in leaves, stack, explicit bounds, ANN_KD_SL_MIDPT",
		#ifdef HAVE_ANN
		"ANN stack, double",
		//"ANN priority",
		#endif // HAVE_ANN
		#ifdef HAVE_FLANN
		"FLANN, double",
		"FLANN, float",
		#endif // HAVE_FLANN
	};
	
	// do bench themselves, accumulate over several times
	size_t benchCount(sizeof(benchLabels) / sizeof(const char *));
	cout << "Doing " << benchCount << " different benches " << runCount << " times, with " << searchCount << " query per run" << endl;
	BenchResults results(benchCount);
	for (int run = 0; run < runCount; ++run)
	{
		size_t i = 0;
		//results.at(i++) += doBench<KDTD1>(d, q, K, itCount, searchCount);
		//results.at(i++) += doBench<KDTD2>(d, q, K, itCount, searchCount);
		//results.at(i++) += doBench<KDTD3>(d, q, K, itCount, searchCount);
		//results.at(i++) += doBench<KDTD4>(d, q, K, itCount, searchCount);
		//results.at(i++) += doBench<KDTD5A>(d, q, K, itCount, searchCount);
		//results.at(i++) += doBench<KDTD5B>(d, q, K, itCount, searchCount);
		results.at(i++) += doBenchType<double>(NNSearchD::KDTREE_LINEAR_HEAP, 0, dD, qD, K, itCount, searchCount);
		results.at(i++) += doBenchType<double>(NNSearchD::KDTREE_TREE_HEAP, 0, dD, qD, K, itCount, searchCount);
		results.at(i++) += doBenchType<float>(NNSearchF::KDTREE_LINEAR_HEAP, 0, dF, qF, K, itCount, searchCount);
		results.at(i++) += doBenchType<float>(NNSearchF::KDTREE_TREE_HEAP, 0, dF, qF, K, itCount, searchCount);
		results.at(i++) += doBenchType<float>(NNSearchF::KDTREE_TREE_HEAP, NNSearchF::TOUCH_STATISTICS, dF, qF, K, itCount, searchCount);
		#ifdef HAVE_OPENCL
		results.at(i++) += doBenchType<float>(NNSearchF::KDTREE_CL_PT_IN_NODES, NNSearchF::TOUCH_STATISTICS, dF, qF, K, itCount, searchCount);
		results.at(i++) += doBenchType<float>(NNSearchF::KDTREE_CL_PT_IN_LEAVES, NNSearchF::TOUCH_STATISTICS, dF, qF, K, itCount, searchCount);
		//results.at(i++) += doBenchType<float>(NNSearchF::BRUTE_FORCE_CL, dF, qF, K, itCount, searchCount);
		#endif // HAVE_OPENCL
		#ifdef HAVE_ANN
		results.at(i++) += doBenchANNStack(dD, qD, K, itCount, searchCount);
		//results.at(i++) += doBenchANNPriority(d, q, K, itCount);
		#endif // HAVE_ANN
		#ifdef HAVE_FLANN
		results.at(i++) += doBenchFLANN<double>(dD, qD, K, itCount, searchCount);
		results.at(i++) += doBenchFLANN<float>(dF, qF, K, itCount, searchCount);
		#endif // HAVE_FLANN
	}
	
	// print results
	cout << "Showing average over " << runCount << " runs\n\n";
	for (size_t i = 0; i < benchCount; ++i)
	{
		results[i] /= double(runCount);
		cout << "Method " << benchLabels[i] << ":\n";
		cout << "  creation duration: " << results[i].creationDuration << "\n";
		cout << "  execution duration: " << results[i].executionDuration << "\n";
		if (results[i].totalCount != 0)
		{
			cout << "  visit count: " << results[i].visitCount << "\n";
			cout << "  total count: " << results[i].totalCount << "\n";
			cout << "  precentage visit: " << (results[i].visitCount * 100.) / results[i].totalCount << "\n";
		}
		else
			cout << "  no stats for visits\n";
		cout << endl;
	}
	
	return 0;
}
