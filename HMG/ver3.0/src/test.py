import numpy as np
import math

cx = np.arange(0, 50, 0.5)
cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]

print(cx)
print(cy)




def plus