from pynabo import *
import numpy as np

x = np.array([[0.,3.], [1,2], [4,5]])
print(x)

nns = NearestNeighbourSearch(x)
q = np.array([[1.1, 2.]])
print(q)

res = nns.knn(q, 2, 0, SearchOptionFlags.ALLOW_SELF_MATCH)
print(res[0])
print(res[1])
