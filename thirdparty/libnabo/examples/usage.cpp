// This example is in the public domain

#include "nabo/nabo.h"
#include <iostream>

int main()
{
	using namespace Nabo;
	using namespace Eigen;
	using namespace std;
	
	// 100 points in 3D
	MatrixXf M = MatrixXf::Random(3, 100);
	// 50 query points
	MatrixXf q = MatrixXf::Random(3, 50);
	
	// Create a kd-tree for M, note that M must stay valid during the lifetime of the kd-tree.
	NNSearchF* nns = NNSearchF::createKDTreeLinearHeap(M);
	
	// The output of the query are a matrix of indices to columns of M and 
	// a matrix of squared distances corresponding to these indices.
	// These matrix must have the correct size when passed to the search function.
	MatrixXi indices;
	MatrixXf dists2;
	
	// Look for the 5 nearest neighbours of each query point, 
	// We do not want approximations but we want to sort by the distance,
	indices.resize(5, q.cols());
	dists2.resize(5, q.cols());
	nns->knn(q, indices, dists2, 5, 0, NNSearchF::SORT_RESULTS);
	
	// Look for the 3 nearest neighbours of each query point, use the data themselves for the query.
	// We do not want approximations but we want to sort by the distance,
	// and we want to allow self-match.
	indices.resize(3, M.cols());
	dists2.resize(3, M.cols());
	nns->knn(M, indices, dists2, 3, 0, NNSearchF::SORT_RESULTS | NNSearchF::ALLOW_SELF_MATCH);
	
	// Make sure that the closest return points correspond to the points from M.
	for (int i = 0; i < 100; ++i)
	{
		// The query is the data itself and we allow self-match.
		if (indices.coeff(0, i) != i)
			cerr << "Oups, something went wrong: " << indices.coeff(0, i) << " " <<  i << endl;
	}
	
	// Now look for the 2 nearset neighbours of each query point.
	// We do allow 10% approximation but do not want to allow self-match.
	// We do not care about sorting either.
	indices.resize(2, M.cols());
	dists2.resize(2, M.cols());
	nns->knn(M, indices, dists2, 2, 0.1, 0);
	for (int i = 0; i < 50; ++i)
	{
		// The query is the data itself but we forbide self-match.
		if (indices.coeff(0, i) == i)
			cerr << "Oups, something went wrong: " << indices.coeff(0, i) << " " <<  i << endl;
	}
	
	// Cleanup the kd-tree.
	delete nns;
	
	return 0;
}
