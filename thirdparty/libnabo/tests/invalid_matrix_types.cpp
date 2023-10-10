/*

Copyright (c) 2015, Simon Lynen, Google Inc.
You can contact the author at <slynen at google dot com>

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

/*
 * These tests ensure that the instantiation of Nabo Trees with non-matching
 * matrix types is not allowed.
 */
template <typename MatrixType>
int testFunction()
{
	MatrixType M = MatrixType::Random(3, 100);
	MatrixType q = MatrixType::Random(3, 5);

#ifdef NABO_TYPE_CREATE
	Nabo::NNSearchF* nns = Nabo::NNSearchF::create(M);
#endif  // NABO_TYPE_CREATE

#ifdef NABO_TYPE_BRUTE_FORCE
	Nabo::NNSearchF* nns = Nabo::NNSearchF::createBruteForce(M);
#endif  // NABO_TYPE_BRUTE_FORCE

#ifdef NABO_TYPE_LINEAR_HEAP
	Nabo::NNSearchF* nns = Nabo::NNSearchF::createKDTreeLinearHeap(M);
#endif  // NABO_TYPE_TREE_HEAP

#ifdef NABO_TYPE_TREE_HEAP
	Nabo::NNSearchF* nns = Nabo::NNSearchF::createKDTreeTreeHeap(M);
#endif  // NABO_TYPE_TREE_HEAP
	delete nns;
	return 0;
}
int main(int /*argc*/, char** /*argv*/)
{
  // Ensure that the test as such compiles.
#ifdef NABO_EIGEN_DYNAMIC_TYPE
	int value = testFunction<Eigen::MatrixXf>();
#endif  // NABO_EIGEN_DYNAMIC_TYPE

#ifdef NABO_EIGEN_SEMI_DYNAMIC_TYPE
	int value = testFunction<Eigen::Matrix<float, 5, Eigen::Dynamic> >();
#endif  // NABO_EIGEN_SEMI_DYNAMIC_TYPE
	return value;
}
