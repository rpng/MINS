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

#ifndef __NABO_EXPERIMENTAL_H
#define __NABO_EXPERIMENTAL_H

#include "../nabo/nabo_private.h"
#include "../nabo/index_heap.h"

namespace Nabo
{
	// KDTree, balanced, points in nodes
	template<typename T, typename CloudType>
	struct KDTreeBalancedPtInNodes:public NearestNeighbourSearch<T, CloudType>
	{
		typedef typename NearestNeighbourSearch<T, CloudType>::Vector Vector;
		typedef typename NearestNeighbourSearch<T, CloudType>::Matrix Matrix;
		typedef typename NearestNeighbourSearch<T, CloudType>::Index Index;
		typedef typename NearestNeighbourSearch<T, CloudType>::IndexVector IndexVector;
		
	protected:
		struct BuildPoint
		{
			Vector pos;
			size_t index;
			BuildPoint(const Vector& pos =  Vector(), const size_t index = 0): pos(pos), index(index) {}
		};
		typedef std::vector<BuildPoint> BuildPoints;
		typedef typename BuildPoints::iterator BuildPointsIt;
		typedef typename BuildPoints::const_iterator BuildPointsCstIt;
		
		struct CompareDim
		{
			size_t dim;
			CompareDim(const size_t dim):dim(dim){}
			bool operator() (const BuildPoint& p0, const BuildPoint& p1) { return p0.pos(dim) < p1.pos(dim); }
		};
		
		struct Node
		{
			Vector pos;
			int dim; // -1 == leaf, -2 == invalid
			Index index;
			Node(const Vector& pos = Vector(), const int dim = -2, const Index index = 0):pos(pos), dim(dim), index(index) {}
		};
		typedef std::vector<Node> Nodes;
		
		Nodes nodes;
		
		inline size_t childLeft(size_t pos) const { return 2*pos + 1; }
		inline size_t childRight(size_t pos) const { return 2*pos + 2; }
		inline size_t parent(size_t pos) const { return (pos-1)/2; }
		size_t getTreeSize(size_t size) const;
		IndexVector cloudIndexesFromNodesIndexes(const IndexVector& indexes) const;
		void buildNodes(const BuildPointsIt first, const BuildPointsIt last, const size_t pos);
		void dump(const Vector minValues, const Vector maxValues, const size_t pos) const;
		
	protected:
		KDTreeBalancedPtInNodes(const CloudType& cloud);
	};
	
	// KDTree, balanced, points in nodes, priority queue
	template<typename T, typename CloudType>
	struct KDTreeBalancedPtInNodesPQ: public KDTreeBalancedPtInNodes<T, CloudType>
	{
		typedef typename NearestNeighbourSearch<T, CloudType>::Vector Vector;
		typedef typename NearestNeighbourSearch<T, CloudType>::Matrix Matrix;
		typedef typename NearestNeighbourSearch<T, CloudType>::Index Index;
		typedef typename NearestNeighbourSearch<T, CloudType>::IndexVector IndexVector;
		typedef typename KDTreeBalancedPtInNodes<T, CloudType>::Node Node;
		typedef typename KDTreeBalancedPtInNodes<T, CloudType>::Nodes Nodes;
		
		using NearestNeighbourSearch<T, CloudType>::statistics;
		using KDTreeBalancedPtInNodes<T, CloudType>::nodes;
		using KDTreeBalancedPtInNodes<T, CloudType>::childLeft;
		using KDTreeBalancedPtInNodes<T, CloudType>::childRight;
		
	protected:
		struct SearchElement
		{
			size_t index;
			T minDist;
			
			SearchElement(const size_t index, const T minDist): index(index), minDist(minDist) {}
			// invert test as std::priority_queue shows biggest element at top
			friend bool operator<(const SearchElement& e0, const SearchElement& e1) { return e0.minDist > e1.minDist; }
		};
		
	public:
		KDTreeBalancedPtInNodesPQ(const CloudType& cloud);
		virtual IndexVector knn(const Vector& query, const Index k, const T epsilon, const unsigned optionFlags);
	};
	
	// KDTree, balanced, points in nodes, stack
	template<typename T, typename CloudType>
	struct KDTreeBalancedPtInNodesStack: public KDTreeBalancedPtInNodes<T, CloudType>
	{
		typedef typename NearestNeighbourSearch<T, CloudType>::Vector Vector;
		typedef typename NearestNeighbourSearch<T, CloudType>::Matrix Matrix;
		typedef typename NearestNeighbourSearch<T, CloudType>::Index Index;
		typedef typename NearestNeighbourSearch<T, CloudType>::IndexVector IndexVector;
		typedef typename KDTreeBalancedPtInNodes<T, CloudType>::Node Node;
		typedef typename KDTreeBalancedPtInNodes<T, CloudType>::Nodes Nodes;
		
		using NearestNeighbourSearch<T, CloudType>::statistics;
		using KDTreeBalancedPtInNodes<T, CloudType>::nodes;
		using KDTreeBalancedPtInNodes<T, CloudType>::childLeft;
		using KDTreeBalancedPtInNodes<T, CloudType>::childRight;
		
		typedef IndexHeapSTL<Index, T> Heap;
		
	protected:
		void recurseKnn(const Vector& query, const size_t n, T rd, Heap& heap, Vector& off, const T maxError, const bool allowSelfMatch);
		
	public:
		KDTreeBalancedPtInNodesStack(const CloudType& cloud);
		virtual IndexVector knn(const Vector& query, const Index k, const T epsilon, const unsigned optionFlags);
	};
	
	
	//  KDTree, balanced, points in leaves, stack
	template<typename T, typename CloudType>
	struct KDTreeBalancedPtInLeavesStack: public NearestNeighbourSearch<T, CloudType>
	{
		typedef typename NearestNeighbourSearch<T, CloudType>::Vector Vector;
		typedef typename NearestNeighbourSearch<T, CloudType>::Matrix Matrix;
		typedef typename NearestNeighbourSearch<T, CloudType>::Index Index;
		typedef typename NearestNeighbourSearch<T, CloudType>::IndexVector IndexVector;
		
		using NearestNeighbourSearch<T, CloudType>::statistics;
		using NearestNeighbourSearch<T, CloudType>::cloud;
		using NearestNeighbourSearch<T, CloudType>::minBound;
		using NearestNeighbourSearch<T, CloudType>::maxBound;
		
	protected:
		struct BuildPoint
		{
			Vector pos;
			size_t index;
			BuildPoint(const Vector& pos =  Vector(), const size_t index = 0): pos(pos), index(index) {}
		};
		typedef std::vector<BuildPoint> BuildPoints;
		typedef typename BuildPoints::iterator BuildPointsIt;
		typedef typename BuildPoints::const_iterator BuildPointsCstIt;
		
		struct CompareDim
		{
			size_t dim;
			CompareDim(const size_t dim):dim(dim){}
			bool operator() (const BuildPoint& p0, const BuildPoint& p1) { return p0.pos(dim) < p1.pos(dim); }
		};
		
		typedef IndexHeapSTL<Index, T> Heap;
		
		struct Node
		{
			int dim; // -1 == invalid, <= -2 = index of pt
			T cutVal;
			Node(const int dim = -1, const T cutVal = 0):
				dim(dim), cutVal(cutVal) {}
		};
		typedef std::vector<Node> Nodes;
		
		Nodes nodes;
		
		inline size_t childLeft(size_t pos) const { return 2*pos + 1; }
		inline size_t childRight(size_t pos) const { return 2*pos + 2; }
		inline size_t parent(size_t pos) const { return (pos-1)/2; }
		size_t getTreeSize(size_t size) const;
		void buildNodes(const BuildPointsIt first, const BuildPointsIt last, const size_t pos, const Vector minValues, const Vector maxValues, const bool balanceVariance);
		void recurseKnn(const Vector& query, const size_t n, T rd, Heap& heap, Vector& off, const T maxError, const bool allowSelfMatch);
		
	public:
		KDTreeBalancedPtInLeavesStack(const CloudType& cloud, const bool balanceVariance);
		virtual IndexVector knn(const Vector& query, const Index k, const T epsilon, const unsigned optionFlags);
	};
	
	//  KDTree, unbalanced, points in leaves, stack, implicit bounds, ANN_KD_SL_MIDPT
	template<typename T, typename Heap, typename CloudType>
	struct KDTreeUnbalancedPtInLeavesImplicitBoundsStack: public NearestNeighbourSearch<T, CloudType>
	{
		typedef typename NearestNeighbourSearch<T, CloudType>::Vector Vector;
		typedef typename NearestNeighbourSearch<T, CloudType>::Matrix Matrix;
		typedef typename NearestNeighbourSearch<T, CloudType>::Index Index;
		typedef typename NearestNeighbourSearch<T, CloudType>::IndexVector IndexVector;
		typedef typename NearestNeighbourSearch<T, CloudType>::IndexMatrix IndexMatrix;
		
		using NearestNeighbourSearch<T, CloudType>::statistics;
		using NearestNeighbourSearch<T, CloudType>::cloud;
		using NearestNeighbourSearch<T, CloudType>::minBound;
		using NearestNeighbourSearch<T, CloudType>::maxBound;
		
	protected:
		struct BuildPoint
		{
			Vector pos;
			size_t index;
			BuildPoint(const Vector& pos =  Vector(), const size_t index = 0): pos(pos), index(index) {}
		};
		typedef std::vector<BuildPoint> BuildPoints;
		typedef typename BuildPoints::iterator BuildPointsIt;
		typedef typename BuildPoints::const_iterator BuildPointsCstIt;
		
		struct CompareDim
		{
			size_t dim;
			CompareDim(const size_t dim):dim(dim){}
			bool operator() (const BuildPoint& p0, const BuildPoint& p1) { return p0.pos(dim) < p1.pos(dim); }
		};
		
		struct Node
		{
			enum
			{
				INVALID_CHILD = 0xffffffff,
				INVALID_PT = 0xffffffff
			};
			unsigned dim;
			unsigned rightChild;
			union
			{
				T cutVal;
				unsigned ptIndex;
			};
			
			Node(const int dim, const T cutVal, unsigned rightChild):
				dim(dim), rightChild(rightChild), cutVal(cutVal) {}
			Node(const unsigned ptIndex = INVALID_PT):
				dim(0), rightChild(INVALID_CHILD), ptIndex(ptIndex) {}
		};
		typedef std::vector<Node> Nodes;
		
		Nodes nodes;
		
		unsigned buildNodes(const BuildPointsIt first, const BuildPointsIt last, const Vector minValues, const Vector maxValues);
		void recurseKnn(const Vector& query, const unsigned n, T rd, Heap& heap, Vector& off, const T maxError, const bool allowSelfMatch);
		
	public:
		KDTreeUnbalancedPtInLeavesImplicitBoundsStack(const CloudType& cloud);
		virtual IndexVector knn(const Vector& query, const Index k, const T epsilon, const unsigned optionFlags);
		virtual IndexMatrix knnM(const Matrix& query, const Index k, const T epsilon, const unsigned optionFlags);
	};
	
	//  KDTree, unbalanced, points in leaves, stack, explicit bounds, ANN_KD_SL_MIDPT
	template<typename T, typename CloudType>
	struct KDTreeUnbalancedPtInLeavesExplicitBoundsStack: public NearestNeighbourSearch<T, CloudType>
	{
		typedef typename NearestNeighbourSearch<T, CloudType>::Vector Vector;
		typedef typename NearestNeighbourSearch<T, CloudType>::Matrix Matrix;
		typedef typename NearestNeighbourSearch<T, CloudType>::Index Index;
		typedef typename NearestNeighbourSearch<T, CloudType>::IndexVector IndexVector;
		
		
		using NearestNeighbourSearch<T, CloudType>::statistics;
		using NearestNeighbourSearch<T, CloudType>::cloud;
		using NearestNeighbourSearch<T, CloudType>::minBound;
		using NearestNeighbourSearch<T, CloudType>::maxBound;
		
	protected:
		struct BuildPoint
		{
			Vector pos;
			size_t index;
			BuildPoint(const Vector& pos =  Vector(), const size_t index = 0): pos(pos), index(index) {}
		};
		typedef std::vector<BuildPoint> BuildPoints;
		typedef typename BuildPoints::iterator BuildPointsIt;
		typedef typename BuildPoints::const_iterator BuildPointsCstIt;
		
		struct CompareDim
		{
			size_t dim;
			CompareDim(const size_t dim):dim(dim){}
			bool operator() (const BuildPoint& p0, const BuildPoint& p1) { return p0.pos(dim) < p1.pos(dim); }
		};
		
		typedef IndexHeapSTL<Index, T> Heap;
		
		struct Node
		{
			int dim; // <= -1 = index of pt
			unsigned rightChild;
			T cutVal;
			T lowBound;
			T highBound;
			Node(const int dim = -1, const T cutVal = 0, const T lowBound = 0, const T highBound = 0, unsigned rightChild = 0):
				dim(dim), rightChild(rightChild), cutVal(cutVal), lowBound(lowBound), highBound(highBound) {}
		};
		typedef std::vector<Node> Nodes;
		
		Nodes nodes;
		
		unsigned buildNodes(const BuildPointsIt first, const BuildPointsIt last, const Vector minValues, const Vector maxValues);
		void recurseKnn(const Vector& query, const size_t n, T rd, Heap& heap, const T maxError, const bool allowSelfMatch);
		
	public:
		KDTreeUnbalancedPtInLeavesExplicitBoundsStack(const CloudType& cloud);
		virtual IndexVector knn(const Vector& query, const Index k, const T epsilon, const unsigned optionFlags);
	};
}

#endif // __NABO_H
