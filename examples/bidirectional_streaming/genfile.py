
import numpy as np
import struct

nS = 1000000
d = np.arange(nS)%4096
#d = np.random.randint(0, 2**12, nS)
adcF = 42000000
clkDiv = 42
nchan = 2
txd = struct.pack("{}h".format(len(d)), *d)
h = struct.pack("<4I", nS, adcF, clkDiv, nchan)
rxdet = struct.pack("{}h".format(nS*2+10), *np.zeros(nS*2+10))
s = "" + h + txd + rxdet
with open("test.dat", 'wb') as f:
    f.write(s)
