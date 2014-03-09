import numpy as np
import sys
from operator import add

sensor1Pos = np.dot(1, [-np.sqrt(3)/4, 3/4., -1/2.])
sensor2Pos = np.dot(1, [np.sqrt(3)/2, 0, -1/2.])
sensor3Pos = np.dot(1, [-np.sqrt(3)/4, -3/4., -1/2.])
topOfSphere = np.dot(1, [0, 0, 1])

x1, y1, z1 = sensor1Pos
x2, y2, z2 = sensor2Pos
x3, y3, z3 = sensor2Pos


A = np.array([0, z1, -y1, -z1, 0, x1, y1, -x1, 0, 0, z2, -y2, -z2, 0, x2,
              y2, -x2, 0]).reshape([6,3])

A3 = np.array([0, z1, -y1, -z1, 0, x1, y1, -x1, 0, 0, z2, -y2, -z2, 0, x2,
              y2, -x2, 0, 0, z3, -y3, -z3, 0, x3, y3, -x3,
               0]).reshape([9,3])

U, s, V = np.linalg.svd(A)
U3, s3, V3 = np.linalg.svd(A3)

print "U3=", U3
print "s3=", s3
#s3 = [s3[2], s3[0], s3[1]]
print "s3new=", s3
print "V3=", V3

inv = lambda i: 0 if i == 0 else 1./i
sInv = [inv(i) for i in s]
sInv3 = [inv(i) for i in s3]

f = file(sys.argv[1])

lines = f.readlines()
# skip first line - human description
lines = lines[1:]

sumX = [0, 0, 0]
sumY = [0, 0, 0]
sumW = [0, 0, 0]
sumCross = [0, 0, 0]

for line in lines:
    split = line.split()
    if len(split) < 1:
        break
    x1 = int(split[2])
    y1 = int(split[3])
    x2 = int(split[4])
    y2 = int(split[5])
    x3 = int(split[6])
    y3 = int(split[7])
    b = np.array([x1, 0, y1, x2, 0, y2, x3, 0, y3]).reshape(9, 1)
    #w =  U3[:,0].dot(b) * V3[:,0] * sInv3[0] + U3[:,1].dot(b) * V3[:,1] * sInv3[1] + U3[:,2].dot(b) * V3[:,2] * sInv3[2]
    #w =  U3[:,0].dot(b) * V3[0] * sInv3[0] + U3[:,1].dot(b) * V3[1] * sInv3[1] + U3[:,2].dot(b) * V3[2] * sInv3[2]
    w =  U3[:,0].dot(b) * V3[:, 0] * sInv3[0] + U3[:,1].dot(b) * V3[:, 1] * sInv3[1] + U3[:, 2].dot(b) * V3[:, 2] * sInv3[2]
    print x1, y1, x2, y2, x3, y3
    print w
    print np.cross(w, topOfSphere)

    sumX = map(add, sumX, [x1, x2, x3])
    sumY = map(add, sumX, [y1, y2, y3])
    sumW = map(add, sumW, w)
    sumCross = map(add, sumCross, np.cross(w, topOfSphere))

print "SumX: ", sumX
print "SumY: ", sumY
print "SumW: ", sumW
print "SumCross: ", sumCross


