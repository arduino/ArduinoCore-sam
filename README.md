Streaming data over the native USB to the Arduino Due was disappointingly slow, about 60kb/s, yet transmission from Arduino to host computer could run at Mb/s, suggesting that the USB connection was not limiting. Inspection of the code showed that data was received byte by byte through a chain of functions with massively redundant checks.

This repository is a fork of the official code and contains optimisations of the USB code. With them, it is possible to perform round-trip streaming of data to and from the Arduino Due at 1.8Mb/s at least and probably much faster (the Arduino code was not limiting in these tests driven by a simple python script on the host computer).

A simple example application making use of this code is included in the "examples" directory. A simple (linux) python script streams arbitrary blocks of data from the computer to the Arduino DAC (using DMA) and simultaneously receives two channels of ADC data (again using DMA). The parameters in the scripts set a 500kHz clock for both DAC and ADC (each channel), but the code could stream at twice the rate without difficulty.

Changes to the code
===================

A (non-blocking) overloaded read function that accepts as parameters a buffer and size is now provided as a member of the SerialUSB class. If neither of the read functions is used, for instance during a DMA application, the user may need to call "SerialUSB.accept()" periodically if there is a danger that the buffer will sometimes be too full to accept a full FIFO (512 bytes) of data upon interrupt, as this will cause reception to block. The CDC_SERIAL_BUFFER_SIZE can be increased from the original 512 bytes to reduce this risk.

The SerialUSB (Serial_) class has been modified to remove all mention of the RingBuffer used elsewhere in the Arduino code but NOT here. This was confusing at best.

The ring_buffer that IS used in SerialUSB has been made a member of the class. This enables access to it during DMA applications (e.g. streaming to DAC), eliminating needless copy operations.

The implementation of the ring_buffer has been altered slightly for easy compatibility with DMA applications; head and tail are now ever-increasing 64-bit integers.

The Arduino Due code uses a poor-man's interrupts, scheduling them between loop iterations. Proper USB interrupts can now be enabled via new member functions and interrupt-driven code can handle most, and in some scenarios all, data reception.

The code has been reworked to remove the need for locking even with interrupts enabled. This is achieved by using the FIFO signals for synchronisation.

Block transfers are now used throughout the reception chain and their overhead has been minimised. The accept function has been rewritten.

The changes to the code are under the same licences as the original files.



