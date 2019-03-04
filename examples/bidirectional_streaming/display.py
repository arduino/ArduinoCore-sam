import struct
import numpy as np
import matplotlib.pyplot as mp

with open('test.dat', 'r') as f:
    r = f.read()
h = struct.unpack("<4I", r[0:16])
n = h[0]
tx = np.array(struct.unpack("<%ih"%n, r[16:16+n*2]))
rx = np.array(struct.unpack("<%ih"%(2*n), r[16+n*2:16+n*3*2]))
e = ord(struct.unpack("c", r[16+n*3*2])[0])
s = r[16+n*3*2+1:16+n*3*2+20]

rx1 = rx[np.arange(0, 2*n-1, 2)]
rx2 = rx[np.arange(1, 2*n, 2)]

print "header", h
print "error", e
print "timestamp", s

mp.plot(tx)
mp.plot(rx1)
mp.plot(rx2)

mp.show()
