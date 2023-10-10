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

#include "nabo_experimental.h"
#include "../nabo/index_heap.h"
#include <iostream>
#include <stdexcept>
#include <limits>
#include <queue>
#include <algorithm>

namespace Nabo
{
	using namespace std;
	
	template<typename T, typename CloudType>
	size_t argMax(const typename NearestNeighbourSearch<T, CloudType>::Vector& v)
	{
		T maxVal(0);
		size_t maxIdx(0);
		for (int i = 0; i < v.size(); ++i)
		{
			if (v[i] > maxVal)
			{
				maxVal = v[i];
				maxIdx = i;
			}
		}
		return maxIdx;
	}

	template<typename T, typename CloudType>
	size_t KDTreeBalancedPtInNodes<T, CloudType>::getTreeSize(size_t elCount) const
	{
		// FIXME: 64 bits safe stuff, only work for 2^32 elements right now
		size_t count = 0;
		int i = 31;
		for (; i >= 0; --i)
		{
			if (elCount & (1 << i))
				break;
		}
		for (int j = 0; j <= i; ++j)
			count |= (1 << j);
		//cerr << "tree size " << count << " (" << elCount << " elements)\n";
		return count;
	}
	
	template<typename T, typename CloudType>
	typename KDTreeBalancedPtInNodes<T, CloudType>::IndexVector KDTreeBalancedPtInNodes<T, CloudType>::cloudIndexesFromNodesIndexes(const IndexVector& indexes) const
	{
		IndexVector cloudIndexes(indexes.size());
		for (int i = 0; i < indexes.size(); ++i)
			cloudIndexes.coeffRef(i) = nodes[indexes[i]].index;
		return cloudIndexes;
	}

	template<typename T, typename CloudType>
	void KDTreeBalancedPtInNodes<T, CloudType>::buildNodes(const BuildPointsIt first, const BuildPointsIt last, const size_t pos)
	{
		const size_t count(last - first);
		//cerr << count << endl;
		if (count == 1)
		{
			nodes[pos] = Node(first->pos, -1, first->index);
			return;
		}
		
		// estimate variance
		// get mean
		Vector mean(Vector::Zero(this->dim));
		for (BuildPointsCstIt it(first); it != last; ++it)
			mean += it->pos;
		mean /= last - first;
		// get sum of variance
		Vector var(Vector::Zero(this->dim));
		for (BuildPointsCstIt it(first); it != last; ++it)
			var += (it->pos - mean).cwise() * (it->pos - mean);
		// get dimension of maxmial variance
		const size_t cutDim = argMax<T>(var);
		
		// sort
		sort(first, last, CompareDim(cutDim));
		
		// set node
		const size_t recurseCount(count-1);
		const size_t rightCount(recurseCount/2);
		const size_t leftCount(recurseCount-rightCount);
		assert(last - rightCount == first + leftCount + 1);
		
		nodes[pos] = Node((first+leftCount)->pos, cutDim, (first+leftCount)->index);
		
		//cerr << pos << " cutting on " << cutDim << " at " << (first+leftCount)->pos[cutDim] << endl;
		
		// recurse
		if (count > 2)
		{
			buildNodes(first, first + leftCount, childLeft(pos));
			buildNodes(first + leftCount + 1, last, childRight(pos));
		}
		else
		{
			nodes[childLeft(pos)] = Node(first->pos, -1, first->index);
			nodes[childRight(pos)] = Node(Vector(), -2, 0);
		}
	}

	template<typename T, typename CloudType>
	void KDTreeBalancedPtInNodes<T, CloudType>::dump(const Vector minValues, const Vector maxValues, const size_t pos) const
	{
		const Node& node(nodes[pos]);
		
		if (node.dim >= -1)
		{
			if (this->dim == 2)
				cout << "<circle cx=\"" << 100*node.pos(0) << "\" cy=\"" << 100*node.pos(1) << "\" r=\"1\" stroke=\"black\" stroke-width=\"0.2\" fill=\"red\"/>" << endl;
			else
				cout << "pt at\n" << node.pos << endl;
		}
		if (node.dim >= 0)
		{
			//cerr << "in bounds:\n" << minValues << "\nto\n" << maxValues << endl;
			
			// update bounds for left
			Vector leftMaxValues(maxValues);
			leftMaxValues[node.dim] = node.pos[node.dim];
			// update bounds for right
			Vector rightMinValues(minValues);
			rightMinValues[node.dim] = node.pos[node.dim];
			
			// print line
			if (this->dim == 2)
				cout << "<line x1=\"" << 100*rightMinValues(0) << "\" y1=\"" << 100*rightMinValues(1) << "\" x2=\"" << 100*leftMaxValues(0) << "\" y2=\"" << 100*leftMaxValues(1) << "\" style=\"stroke:rgb(0,0,0);stroke-width:0.2\"/>" << endl;
			else
				cout << "cut from\n" << rightMinValues << "\nto\n" << leftMaxValues << endl;
			// recurs
			dump(minValues, leftMaxValues, childLeft(pos));
			dump(rightMinValues, maxValues, childRight(pos));
		}
	}

	template<typename T, typename CloudType>
	KDTreeBalancedPtInNodes<T, CloudType>::KDTreeBalancedPtInNodes(const CloudType& cloud):
		NearestNeighbourSearch<T, CloudType>::NearestNeighbourSearch(cloud)
	{
		// build point vector and compute bounds
		BuildPoints buildPoints;
		buildPoints.reserve(cloud.cols());
		for (int i = 0; i < cloud.cols(); ++i)
		{
			const Vector& v(cloud.col(i));
			buildPoints.push_back(BuildPoint(v, i));
			const_cast<Vector&>(this->minBound) = this->minBound.cwise().min(v);
			const_cast<Vector&>(this->maxBound) = this->maxBound.cwise().max(v);
		}
		
		// create nodes
		nodes.resize(getTreeSize(cloud.cols()));
		buildNodes(buildPoints.begin(), buildPoints.end(), 0);
		
		// dump nodes
		//dump(minBound, maxBound, 0);
	}
	
	// points in nodes, priority queue
	
	template<typename T, typename CloudType>
	KDTreeBalancedPtInNodesPQ<T, CloudType>::KDTreeBalancedPtInNodesPQ(const CloudType& cloud):
		KDTreeBalancedPtInNodes<T, CloudType>::KDTreeBalancedPtInNodes(cloud)
	{
	}

	template<typename T, typename CloudType>
	typename KDTreeBalancedPtInNodesPQ<T, CloudType>::IndexVector KDTreeBalancedPtInNodesPQ<T, CloudType>::knn(const Vector& query, const Index k, const T epsilon, const unsigned optionFlags)
	{
		typedef priority_queue<SearchElement> Queue;
		
		const T maxError(1 + epsilon);
		const bool allowSelfMatch(optionFlags & NearestNeighbourSearch<T>::ALLOW_SELF_MATCH);
		
		Queue queue;
		queue.push(SearchElement(0, 0));
		IndexHeapSTL<Index, T> heap(k);
		statistics.lastQueryVisitCount = 0;
		
		while (!queue.empty())
		{
			SearchElement el(queue.top());
			queue.pop();
			
			// nothing is closer, we found best
			if (el.minDist * maxError > heap.headValue())
				break;
			
			size_t n(el.index);
			while (1)
			{
				const Node& node(nodes[n]);
				assert (node.dim != -2);
				
				// TODO: optimise dist while going down
				const T dist(dist2<T>(node.pos, query));
				if ((dist < heap.headValue()) &&
					(allowSelfMatch || (dist > numeric_limits<T>::epsilon())))
					heap.replaceHead(n, dist);
				
				// if we are at leaf, stop
				if (node.dim < 0)
					break;
				
				const T offset(query.coeff(node.dim) - node.pos.coeff(node.dim));
				const T offset2(offset * offset);
				const T bestDist(heap.headValue());
				if (offset > 0)
				{
					// enqueue offside ?
					if (offset2 < bestDist && nodes[childLeft(n)].dim != -2)
						queue.push(SearchElement(childLeft(n), offset2));
					// continue onside
					if (nodes[childRight(n)].dim != -2)
						n = childRight(n);
					else
						break;
				}
				else
				{
					// enqueue offside ?
					if (offset2 < bestDist && nodes[childRight(n)].dim != -2)
						queue.push(SearchElement(childRight(n), offset2));
					// continue onside
					if (nodes[childLeft(n)].dim != -2)
						n = childLeft(n);
					else
						break;
				}
				++statistics.lastQueryVisitCount;
			}
		}
		statistics.totalVisitCount += statistics.lastQueryVisitCount;
		
		if (optionFlags & NearestNeighbourSearch<T>::SORT_RESULTS)
			heap.sort();
		
		return cloudIndexesFromNodesIndexes(heap.getIndexes());
	}

	template struct KDTreeBalancedPtInNodesPQ<float>;
	template struct KDTreeBalancedPtInNodesPQ<double>;
	template struct KDTreeBalancedPtInNodesPQ<float, Eigen::Matrix3Xf>;
	template struct KDTreeBalancedPtInNodesPQ<double, Eigen::Matrix3Xd>;
	template struct KDTreeBalancedPtInNodesPQ<float, Eigen::Map<const Eigen::Matrix3Xf, Eigen::Aligned> >;
	template struct KDTreeBalancedPtInNodesPQ<double, Eigen::Map<const Eigen::Matrix3Xd, Eigen::Aligned> >;
	
	// points in nodes, stack
	
	template<typename T, typename CloudType>
	KDTreeBalancedPtInNodesStack<T, CloudType>::KDTreeBalancedPtInNodesStack(const CloudType& cloud):
		KDTreeBalancedPtInNodes<T, CloudType>::KDTreeBalancedPtInNodes(cloud)
	{
	}
	
	template<typename T, typename CloudType>
	typename KDTreeBalancedPtInNodesStack<T, CloudType>::IndexVector KDTreeBalancedPtInNodesStack<T, CloudType>::knn(const Vector& query, const Index k, const T epsilon, const unsigned optionFlags)
	{
		const bool allowSelfMatch(optionFlags & NearestNeighbourSearch<T>::ALLOW_SELF_MATCH);
		
		assert(nodes.size() > 0);
		assert(nodes[0].pos.size() == query.size());
		Heap heap(k);
		Vector off(Vector::Zero(nodes[0].pos.size()));
		
		statistics.lastQueryVisitCount = 0;
		
		recurseKnn(query, 0, 0, heap, off, 1 + epsilon, allowSelfMatch);
		
		if (optionFlags & NearestNeighbourSearch<T>::SORT_RESULTS)
			heap.sort();
		
		statistics.totalVisitCount += statistics.lastQueryVisitCount;
		
		return cloudIndexesFromNodesIndexes(heap.getIndexes());
	}
	
	template<typename T, typename CloudType>
	void KDTreeBalancedPtInNodesStack<T, CloudType>::recurseKnn(const Vector& query, const size_t n, T rd, Heap& heap, Vector& off, const T maxError, const bool allowSelfMatch)
	{
		const Node& node(nodes[n]);
		const int cd(node.dim);
		
		++statistics.lastQueryVisitCount;
		
		if (cd == -2)
			return;
		
		const T dist(dist2<T, CloudType>(node.pos, query));
		if ((dist < heap.headValue()) &&
			(allowSelfMatch || (dist > numeric_limits<T>::epsilon()))
		)
			heap.replaceHead(n, dist);
		
		if (cd != -1)
		{
			const T old_off(off.coeff(cd));
			const T new_off(query.coeff(cd) - node.pos.coeff(cd));
			if (new_off > 0)
			{
				recurseKnn(query, childRight(n), rd, heap, off, maxError, allowSelfMatch);
				rd += - old_off*old_off + new_off*new_off;
				if (rd * maxError < heap.headValue())
				{
					off.coeffRef(cd) = new_off;
					recurseKnn(query, childLeft(n), rd, heap, off, maxError, allowSelfMatch);
					off.coeffRef(cd) = old_off;
				}
			}
			else
			{
				recurseKnn(query, childLeft(n), rd, heap, off, maxError, allowSelfMatch);
				rd += - old_off*old_off + new_off*new_off;
				if (rd * maxError < heap.headValue())
				{
					off.coeffRef(cd) = new_off;
					recurseKnn(query, childRight(n), rd, heap, off, maxError, allowSelfMatch);
					off.coeffRef(cd) = old_off;
				}
			}
		}
	}
	
	template struct KDTreeBalancedPtInNodesStack<float>;
	template struct KDTreeBalancedPtInNodesStack<double>;
	template struct KDTreeBalancedPtInNodesStack<float, Eigen::Matrix3Xf>;
	template struct KDTreeBalancedPtInNodesStack<double, Eigen::Matrix3Xd>;
	template struct KDTreeBalancedPtInNodesStack<float, Eigen::Map<const Eigen::Matrix3Xf, Eigen::Aligned> >;
	template struct KDTreeBalancedPtInNodesStack<double, Eigen::Map<const Eigen::Matrix3Xd, Eigen::Aligned> >;
	
	// NEW:
	
	template<typename T, typename CloudType>
	size_t KDTreeBalancedPtInLeavesStack<T, CloudType>::getTreeSize(size_t elCount) const
	{
		// FIXME: 64 bits safe stuff, only work for 2^32 elements right now
		assert(elCount > 0);
		elCount --;
		size_t count = 0;
		int i = 31;
		for (; i >= 0; --i)
		{
			if (elCount & (1 << i))
				break;
		}
		for (int j = 0; j <= i; ++j)
			count |= (1 << j);
		count <<= 1;
		count |= 1;
		return count;
	}
	
	template<typename T, typename CloudType>
	void KDTreeBalancedPtInLeavesStack<T, CloudType>::buildNodes(const BuildPointsIt first, const BuildPointsIt last, const size_t pos, const Vector minValues, const Vector maxValues, const bool balanceVariance)
	{
		const size_t count(last - first);
		//cerr << count << endl;
		if (count == 1)
		{
			const int dim = -2-(first->index);
			assert(pos < nodes.size());
			nodes[pos] = Node(dim);
			return;
		}
		
		size_t cutDim;
		if (balanceVariance)
		{
			// estimate variance
			// get mean
			Vector mean(Vector::Zero(this->dim));
			for (BuildPointsCstIt it(first); it != last; ++it)
				mean += it->pos;
			mean /= last - first;
			// get sum of variance
			Vector var(Vector::Zero(this->dim));
			for (BuildPointsCstIt it(first); it != last; ++it)
				var += (it->pos - mean).cwise() * (it->pos - mean);
			// get dimension of maxmial variance
			cutDim = argMax<T>(var);
		}
		else
		{
			// find the largest dimension of the box
			cutDim = argMax<T>(maxValues - minValues);
		}
		
		// compute number of elements
		const size_t rightCount(count/2);
		const size_t leftCount(count - rightCount);
		assert(last - rightCount == first + leftCount);
		
		// sort
		//sort(first, last, CompareDim(cutDim));
		nth_element(first, first + leftCount, last, CompareDim(cutDim));
		
		// set node
		const T cutVal((first+leftCount)->pos.coeff(cutDim));
		nodes[pos] = Node(cutDim, cutVal);
		
		//cerr << pos << " cutting on " << cutDim << " at " << (first+leftCount)->pos[cutDim] << endl;
		
		// update bounds for left
		Vector leftMaxValues(maxValues);
		leftMaxValues[cutDim] = cutVal;
		// update bounds for right
		Vector rightMinValues(minValues);
		rightMinValues[cutDim] = cutVal;
		
		// recurse
		buildNodes(first, first + leftCount, childLeft(pos), minValues, leftMaxValues, balanceVariance);
		buildNodes(first + leftCount, last, childRight(pos), rightMinValues, maxValues, balanceVariance);
	}

	template<typename T, typename CloudType>
	KDTreeBalancedPtInLeavesStack<T, CloudType>::KDTreeBalancedPtInLeavesStack(const CloudType& cloud, const bool balanceVariance):
		NearestNeighbourSearch<T, CloudType>::NearestNeighbourSearch(cloud)
	{
		// build point vector and compute bounds
		BuildPoints buildPoints;
		buildPoints.reserve(cloud.cols());
		for (int i = 0; i < cloud.cols(); ++i)
		{
			const Vector& v(cloud.col(i));
			buildPoints.push_back(BuildPoint(v, i));
			const_cast<Vector&>(minBound) = minBound.cwise().min(v);
			const_cast<Vector&>(maxBound) = maxBound.cwise().max(v);
		}
		
		// create nodes
		nodes.resize(getTreeSize(cloud.cols()));
		buildNodes(buildPoints.begin(), buildPoints.end(), 0, minBound, maxBound, balanceVariance);
		//for (size_t i = 0; i < nodes.size(); ++i)
		//	cout << i << ": " << nodes[i].dim << " " << nodes[i].cutVal << endl;
	}
	
	template<typename T, typename CloudType>
	typename KDTreeBalancedPtInLeavesStack<T, CloudType>::IndexVector KDTreeBalancedPtInLeavesStack<T, CloudType>::knn(const Vector& query, const Index k, const T epsilon, const unsigned optionFlags)
	{
		const bool allowSelfMatch(optionFlags & NearestNeighbourSearch<T, CloudType>::ALLOW_SELF_MATCH);
		
		assert(nodes.size() > 0);
		Heap heap(k);
		Vector off(Vector::Zero(query.size()));
		
		statistics.lastQueryVisitCount = 0;
		
		recurseKnn(query, 0, 0, heap, off, 1 + epsilon, allowSelfMatch);
		
		if (optionFlags & NearestNeighbourSearch<T>::SORT_RESULTS)
			heap.sort();
		
		statistics.totalVisitCount += statistics.lastQueryVisitCount;
		
		return heap.getIndexes();
	}
	
	template<typename T, typename CloudType>
	void KDTreeBalancedPtInLeavesStack<T, CloudType>::recurseKnn(const Vector& query, const size_t n, T rd, Heap& heap, Vector& off, const T maxError, const bool allowSelfMatch)
	{
		const Node& node(nodes[n]);
		const int cd(node.dim);
		
		++statistics.lastQueryVisitCount;
		
		if (cd < 0)
		{
			if (cd == -1)
				return;
			const int index(-(cd + 2));
			const T dist(dist2<T>(query, cloud.col(index)));
			if ((dist < heap.headValue()) &&
				(allowSelfMatch || (dist > numeric_limits<T>::epsilon()))
			)
				heap.replaceHead(index, dist);
		}
		else
		{
			const T old_off(off.coeff(cd));
			const T new_off(query.coeff(cd) - node.cutVal);
			if (new_off > 0)
			{
				recurseKnn(query, childRight(n), rd, heap, off, maxError, allowSelfMatch);
				rd += - old_off*old_off + new_off*new_off;
				if (rd * maxError < heap.headValue())
				{
					off.coeffRef(cd) = new_off;
					recurseKnn(query, childLeft(n), rd, heap, off, maxError, allowSelfMatch);
					off.coeffRef(cd) = old_off;
				}
			}
			else
			{
				recurseKnn(query, childLeft(n), rd, heap, off, maxError, allowSelfMatch);
				rd += - old_off*old_off + new_off*new_off;
				if (rd * maxError < heap.headValue())
				{
					off.coeffRef(cd) = new_off;
					recurseKnn(query, childRight(n), rd, heap, off, maxError, allowSelfMatch);
					off.coeffRef(cd) = old_off;
				}
			}
		}
	}
	
	template struct KDTreeBalancedPtInLeavesStack<float>;
	template struct KDTreeBalancedPtInLeavesStack<double>;
	template struct KDTreeBalancedPtInLeavesStack<float, Eigen::Matrix3Xf>;
	template struct KDTreeBalancedPtInLeavesStack<double, Eigen::Matrix3Xd>;
	template struct KDTreeBalancedPtInLeavesStack<float, Eigen::Map<const Eigen::Matrix3Xf, Eigen::Aligned> >;
	template struct KDTreeBalancedPtInLeavesStack<double, Eigen::Map<const Eigen::Matrix3Xd, Eigen::Aligned> >;
	
	
	
	template<typename T, typename Heap, typename CloudType>
	unsigned KDTreeUnbalancedPtInLeavesImplicitBoundsStack<T, Heap, CloudType>::buildNodes(const BuildPointsIt first, const BuildPointsIt last, const Vector minValues, const Vector maxValues)
	{
		const size_t count(last - first);
		const unsigned pos(nodes.size());
		
		//cerr << count << endl;
		if (count == 1)
		{
			nodes.push_back(Node(first->index));
			return pos;
		}
		
		// find the largest dimension of the box
		const unsigned cutDim = argMax<T>(maxValues - minValues);
		T cutVal((maxValues(cutDim) + minValues(cutDim))/2);
		
		// TODO: do only sort once
		// sort
		sort(first, last, CompareDim(cutDim));
		
		// TODO: optimise using binary search
		size_t rightStart(0);
		while (rightStart < count && (first+rightStart)->pos.coeff(cutDim) < cutVal)
			++rightStart;
		
		// prevent trivial splits
		if (rightStart == 0)
		{
			cutVal = first->pos.coeff(cutDim);
			rightStart = 1;
		}
		else if (rightStart == count)
		{
			rightStart = count - 1;
			cutVal = (first + rightStart)->pos.coeff(cutDim);
		}
		
		// update bounds for left
		Vector leftMaxValues(maxValues);
		leftMaxValues[cutDim] = cutVal;
		// update bounds for right
		Vector rightMinValues(minValues);
		rightMinValues[cutDim] = cutVal;
		
		// count for recursion
		const size_t rightCount(count - rightStart);
		const size_t leftCount(count - rightCount);
		
		// add this
		nodes.push_back(Node(cutDim, cutVal, 0));
		
		// recurse
		const unsigned __attribute__ ((unused)) leftChild = buildNodes(first, first + leftCount, minValues, leftMaxValues);
		assert(leftChild == pos + 1);
		const unsigned rightChild = buildNodes(first + leftCount, last, rightMinValues, maxValues);
		
		// write right child index and return
		nodes[pos].rightChild = rightChild;
		return pos;
	}

	template<typename T, typename Heap, typename CloudType>
	KDTreeUnbalancedPtInLeavesImplicitBoundsStack<T, Heap, CloudType>::KDTreeUnbalancedPtInLeavesImplicitBoundsStack(const CloudType& cloud):
		NearestNeighbourSearch<T, CloudType>::NearestNeighbourSearch(cloud)
	{
		// build point vector and compute bounds
		BuildPoints buildPoints;
		buildPoints.reserve(cloud.cols());
		for (int i = 0; i < cloud.cols(); ++i)
		{
			const Vector& v(cloud.col(i));
			buildPoints.push_back(BuildPoint(v, i));
			const_cast<Vector&>(minBound) = minBound.cwise().min(v);
			const_cast<Vector&>(maxBound) = maxBound.cwise().max(v);
		}
		
		// create nodes
		//nodes.resize(getTreeSize(cloud.cols()));
		buildNodes(buildPoints.begin(), buildPoints.end(), minBound, maxBound);
		//for (size_t i = 0; i < nodes.size(); ++i)
		//	cout << i << ": " << nodes[i].dim << " " << nodes[i].cutVal <<  " " << nodes[i].rightChild << endl;
	}
	
	template<typename T, typename Heap, typename CloudType>
	typename KDTreeUnbalancedPtInLeavesImplicitBoundsStack<T, Heap, CloudType>::IndexVector KDTreeUnbalancedPtInLeavesImplicitBoundsStack<T, Heap, CloudType>::knn(const Vector& query, const Index k, const T epsilon, const unsigned optionFlags)
	{
		const bool allowSelfMatch(optionFlags & NearestNeighbourSearch<T, CloudType>::ALLOW_SELF_MATCH);
		
		assert(nodes.size() > 0);
		Heap heap(k);
		Vector off(Vector::Zero(query.size()));
		
		statistics.lastQueryVisitCount = 0;
		
		recurseKnn(query, 0, 0, heap, off, 1+epsilon, allowSelfMatch);
		
		if (optionFlags & NearestNeighbourSearch<T, CloudType>::SORT_RESULTS)
			heap.sort();
		
		statistics.totalVisitCount += statistics.lastQueryVisitCount;
		
		return heap.getIndexes();
	}
	
	template<typename T, typename Heap, typename CloudType>
	typename KDTreeUnbalancedPtInLeavesImplicitBoundsStack<T, Heap, CloudType>::IndexMatrix KDTreeUnbalancedPtInLeavesImplicitBoundsStack<T, Heap, CloudType>::knnM(const Matrix& query, const Index k, const T epsilon, const unsigned optionFlags)
	{
		const bool allowSelfMatch(optionFlags & NearestNeighbourSearch<T, CloudType>::ALLOW_SELF_MATCH);
		assert(nodes.size() > 0);
		
		assert(nodes.size() > 0);
		Heap heap(k);
		Vector off(query.rows());
		
		IndexMatrix result(k, query.cols());
		const int colCount(query.cols());
		
		for (int i = 0; i < colCount; ++i)
		{
			const Vector& q(query.col(i));
			
			off.setZero();
			heap.reset();
			
			statistics.lastQueryVisitCount = 0;
			
			recurseKnn(q, 0, 0, heap, off, 1+epsilon, allowSelfMatch);
			
			if (optionFlags & NearestNeighbourSearch<T>::SORT_RESULTS)
				heap.sort();
			
			result.col(i) = heap.getIndexes();
			
			statistics.totalVisitCount += statistics.lastQueryVisitCount;
		}
		
		return result;
	}
	
	template<typename T, typename Heap, typename CloudType>
	void KDTreeUnbalancedPtInLeavesImplicitBoundsStack<T, Heap, CloudType>::recurseKnn(const Vector& query, const unsigned n, T rd, Heap& heap, Vector& off, const T maxError, const bool allowSelfMatch)
	{
		const Node& node(nodes[n]);
		//++statistics.lastQueryVisitCount;
		
		if (node.rightChild == Node::INVALID_CHILD)
		{
			const unsigned index(node.ptIndex);
			//const T dist(dist2<T>(query, cloud.col(index)));
			//const T dist((query - cloud.col(index)).squaredNorm());
			T dist(0);
			const T* qPtr(&query.coeff(0));
			const T* dPtr(&cloud.coeff(0, index));
			const int dim(query.size());
			for (int i = 0; i < dim; ++i)
			{
				const T diff(*qPtr - *dPtr);
				dist += diff*diff;
				qPtr++; dPtr++;
			}
			if ((dist < heap.headValue()) &&
				(allowSelfMatch || (dist > numeric_limits<T>::epsilon()))
			)
				heap.replaceHead(index, dist);
		}
		else
		{
			const unsigned cd(node.dim);
			const T old_off(off.coeff(cd));
			const T new_off(query.coeff(cd) - node.cutVal);
			if (new_off > 0)
			{
				recurseKnn(query, node.rightChild, rd, heap, off, maxError, allowSelfMatch);
				rd += - old_off*old_off + new_off*new_off;
				if (rd * maxError < heap.headValue())
				{
					off.coeffRef(cd) = new_off;
					recurseKnn(query, n + 1, rd, heap, off, maxError, allowSelfMatch);
					off.coeffRef(cd) = old_off;
				}
			}
			else
			{
				recurseKnn(query, n+1, rd, heap, off, maxError, allowSelfMatch);
				rd += - old_off*old_off + new_off*new_off;
				if (rd * maxError < heap.headValue())
				{
					off.coeffRef(cd) = new_off;
					recurseKnn(query, node.rightChild, rd, heap, off, maxError, allowSelfMatch);
					off.coeffRef(cd) = old_off;
				}
			}
		}
	}
	
	template struct KDTreeUnbalancedPtInLeavesImplicitBoundsStack<float,IndexHeapSTL<int,float>>;
	template struct KDTreeUnbalancedPtInLeavesImplicitBoundsStack<float,IndexHeapBruteForceVector<int,float>>;
	template struct KDTreeUnbalancedPtInLeavesImplicitBoundsStack<double,IndexHeapSTL<int,double>>;
	template struct KDTreeUnbalancedPtInLeavesImplicitBoundsStack<double,IndexHeapBruteForceVector<int,double>>;

	template struct KDTreeUnbalancedPtInLeavesImplicitBoundsStack<float,IndexHeapSTL<int,float>,Eigen::Matrix3Xf>;
	template struct KDTreeUnbalancedPtInLeavesImplicitBoundsStack<float,IndexHeapBruteForceVector<int,float>,Eigen::Matrix3Xf>;
	template struct KDTreeUnbalancedPtInLeavesImplicitBoundsStack<double,IndexHeapSTL<int,double>,Eigen::Matrix3Xd>;
	template struct KDTreeUnbalancedPtInLeavesImplicitBoundsStack<double,IndexHeapBruteForceVector<int,double>,Eigen::Matrix3Xd>;

	template struct KDTreeUnbalancedPtInLeavesImplicitBoundsStack<float,IndexHeapSTL<int,float>,Eigen::Map<const Eigen::Matrix3Xf, Eigen::Aligned> >;
	template struct KDTreeUnbalancedPtInLeavesImplicitBoundsStack<float,IndexHeapBruteForceVector<int,float>,Eigen::Map<const Eigen::Matrix3Xf, Eigen::Aligned> >;
	template struct KDTreeUnbalancedPtInLeavesImplicitBoundsStack<double,IndexHeapSTL<int,double>,Eigen::Map<const Eigen::Matrix3Xd, Eigen::Aligned> >;
	template struct KDTreeUnbalancedPtInLeavesImplicitBoundsStack<double,IndexHeapBruteForceVector<int,double>,Eigen::Map<const Eigen::Matrix3Xd, Eigen::Aligned> >;
	
	template<typename T, typename CloudType>
	unsigned KDTreeUnbalancedPtInLeavesExplicitBoundsStack<T, CloudType>::buildNodes(const BuildPointsIt first, const BuildPointsIt last, const Vector minValues, const Vector maxValues)
	{
		const size_t count(last - first);
		const unsigned pos(nodes.size());
		
		//cerr << count << endl;
		if (count == 1)
		{
			const int dim = -1-(first->index);
			nodes.push_back(Node(dim));
			return pos;
		}
		
		// find the largest dimension of the box
		const int cutDim = argMax<T>(maxValues - minValues);
		T cutVal((maxValues(cutDim) + minValues(cutDim))/2);
		
		// TODO: do only sort once
		// sort
		sort(first, last, CompareDim(cutDim));
		
		// TODO: optimise using binary search
		size_t rightStart(0);
		while (rightStart < count && (first+rightStart)->pos.coeff(cutDim) < cutVal)
			++rightStart;
		
		// prevent trivial splits
		if (rightStart == 0)
		{
			cutVal = first->pos.coeff(cutDim);
			rightStart = 1;
		}
		else if (rightStart == count)
		{
			rightStart = count - 1;
			cutVal = (first + rightStart)->pos.coeff(cutDim);
		}
		
		// update bounds for left
		Vector leftMaxValues(maxValues);
		leftMaxValues[cutDim] = cutVal;
		// update bounds for right
		Vector rightMinValues(minValues);
		rightMinValues[cutDim] = cutVal;
		
		// count for recursion
		const size_t rightCount(count - rightStart);
		const size_t leftCount(count - rightCount);
		
		// add this
		nodes.push_back(Node(cutDim, cutVal, minValues.coeff(cutDim), maxValues.coeff(cutDim)));
		
		// recurse
		const unsigned __attribute__ ((unused)) leftChild = buildNodes(first, first + leftCount, minValues, leftMaxValues);
		assert(leftChild == pos + 1);
		const unsigned rightChild = buildNodes(first + leftCount, last, rightMinValues, maxValues);
		
		// write right child index and return
		nodes[pos].rightChild = rightChild;
		return pos;
	}

	template<typename T, typename CloudType>
	KDTreeUnbalancedPtInLeavesExplicitBoundsStack<T, CloudType>::KDTreeUnbalancedPtInLeavesExplicitBoundsStack(const CloudType& cloud):
		NearestNeighbourSearch<T, CloudType>::NearestNeighbourSearch(cloud)
	{
		// build point vector and compute bounds
		BuildPoints buildPoints;
		buildPoints.reserve(cloud.cols());
		for (int i = 0; i < cloud.cols(); ++i)
		{
			const Vector& v(cloud.col(i));
			buildPoints.push_back(BuildPoint(v, i));
			const_cast<Vector&>(minBound) = minBound.cwise().min(v);
			const_cast<Vector&>(maxBound) = maxBound.cwise().max(v);
		}
		
		// create nodes
		//nodes.resize(getTreeSize(cloud.cols()));
		buildNodes(buildPoints.begin(), buildPoints.end(), minBound, maxBound);
		//for (size_t i = 0; i < nodes.size(); ++i)
		//	cout << i << ": " << nodes[i].dim << " " << nodes[i].cutVal <<  " " << nodes[i].rightChild << endl;
	}
	
	template<typename T, typename CloudType>
	typename KDTreeUnbalancedPtInLeavesExplicitBoundsStack<T, CloudType>::IndexVector KDTreeUnbalancedPtInLeavesExplicitBoundsStack<T, CloudType>::knn(const Vector& query, const Index k, const T epsilon, const unsigned optionFlags)
	{
		const bool allowSelfMatch(optionFlags & NearestNeighbourSearch<T, CloudType>::ALLOW_SELF_MATCH);
		
		assert(nodes.size() > 0);
		Heap heap(k);
		
		statistics.lastQueryVisitCount = 0;
		
		recurseKnn(query, 0, 0, heap, 1+epsilon, allowSelfMatch);
		
		if (optionFlags & NearestNeighbourSearch<T>::SORT_RESULTS)
			heap.sort();
		
		statistics.totalVisitCount += statistics.lastQueryVisitCount;
		
		return heap.getIndexes();
	}
	
	template<typename T, typename CloudType>
	void KDTreeUnbalancedPtInLeavesExplicitBoundsStack<T, CloudType>::recurseKnn(const Vector& query, const size_t n, T rd, Heap& heap, const T maxError, const bool allowSelfMatch)
	{
		const Node& node(nodes[n]);
		const int cd(node.dim);
		
		++statistics.lastQueryVisitCount;
		
		if (cd < 0)
		{
			const int index(-(cd + 1));
			const T dist(dist2<T>(query, cloud.col(index)));
			if ((dist < heap.headValue()) &&
				(allowSelfMatch || (dist > numeric_limits<T>::epsilon()))
			)
				heap.replaceHead(index, dist);
		}
		else
		{
			const T q_val(query.coeff(cd));
			const T cut_diff(q_val - node.cutVal);
			if (cut_diff < 0)
			{
				recurseKnn(query, n+1, rd, heap, maxError, allowSelfMatch);
				
				T box_diff = node.lowBound - q_val;
				if (box_diff < 0)
					box_diff = 0;
				
				rd += cut_diff*cut_diff - box_diff*box_diff;
				
				if (rd * maxError < heap.headValue())
					recurseKnn(query, node.rightChild, rd, heap, maxError, allowSelfMatch);
			}
			else
			{
				recurseKnn(query, node.rightChild, rd, heap, maxError, allowSelfMatch);
				
				T box_diff = q_val - node.highBound;
				if (box_diff < 0)
					box_diff = 0;
				
				rd += cut_diff*cut_diff - box_diff*box_diff;
				
				if (rd * maxError < heap.headValue())
					recurseKnn(query, n + 1, rd, heap, maxError, allowSelfMatch);
			}
		}
	}
	
	template struct KDTreeUnbalancedPtInLeavesExplicitBoundsStack<float>;
	template struct KDTreeUnbalancedPtInLeavesExplicitBoundsStack<double>;
	template struct KDTreeUnbalancedPtInLeavesExplicitBoundsStack<float, Eigen::Matrix3Xf>;
	template struct KDTreeUnbalancedPtInLeavesExplicitBoundsStack<double, Eigen::Matrix3Xd>;
	template struct KDTreeUnbalancedPtInLeavesExplicitBoundsStack<float, Eigen::Map<const Eigen::Matrix3Xf, Eigen::Aligned> >;
	template struct KDTreeUnbalancedPtInLeavesExplicitBoundsStack<double, Eigen::Map<const Eigen::Matrix3Xd, Eigen::Aligned> >;
}
