
import numpy as np
import serial
import struct
import time
import glob
import hashlib

currentport = glob.glob('/dev/ttyACM*')[0]
print "Port: ", currentport

port = serial.Serial(currentport, baudrate=0, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, bytesize = serial.EIGHTBITS, timeout=0)

savepath = '/users/nsr/barbour/junk/'

# Just for testing purposes.
testdata = np.ones(1000000)
for i in range(len(testdata)):
    testdata[i] = i%4096

def blockingWrite(data):
    n = len(data)
    while n:
        n -= port.write(data)

def blockingRead(n):
    r = ''
    while n:
        s = port.read(n)
        r += s
        n -= len(s)
    return r

class sound:
    def __init__(self, data=testdata, clkDiv=84, adcF=21000000):
        self.nS = len(data)
        self.clkDiv = clkDiv
        self.adcF = adcF
        self.txd = struct.pack("<%sH"%(len(data)), *data)
        # These are to regulate flow through the port. It seems it can
        # block if too many data are written while reading.
        self.maxOut = 65536
        self.maxIn = self.maxOut
        self.maxAdvance = 65536
        self.hash = hashlib.md5(self.txd).hexdigest()
        print "Sound has hash: ", self.hash
    def episode(self):
        # Number of bytes to transmit.
        nb = len(self.txd)
	tstamp = time.strftime("%Y-%m-%d_%H:%M:%S", time.localtime())
        print tstamp
        print self.hash
        h = struct.pack("<3I", self.nS, self.adcF, self.clkDiv)
        blockingWrite(h)
        # DAC and ADC...
        fname = "DnA_"+tstamp+".dat"
        # Leave as string for immediate writing.
        nchan = blockingRead(4)
        preamble = fname + h + nchan
        # Later we need the number.
        nchan = struct.unpack("<I", nchan)[0]
        error = False
        # Cursors
        txc = 0
        rxc = 0
        first = True
	with open(savepath+fname, 'wb') as f:
            f.write(preamble)
            f.write(self.txd)
            while txc < nb or rxc < nchan*nb:
                if txc < nb and txc - rxc/nchan < self.maxAdvance:
                    nt = min(nb-txc, self.maxOut)
                    w = port.write(self.txd[txc:txc+nt])
                    txc += w
                if rxc < nchan*nb:
                    if first:
                        first = False
                        print "Started."
                    nr = min(self.maxIn*nchan, port.inWaiting(), nb*nchan-rxc)
                    if nr:
                        s = port.read(nr)
                        f.write(s)
                        rxc += len(s)
            error = blockingRead(1)
            f.write(error)
            endtstamp = time.strftime("%Y-%m-%d_%H:%M:%S", time.localtime())
            f.write(endtstamp)
        if ord(error):
            print "An error was reported in ", tstamp
        else:
            print "OK."
            print ""







