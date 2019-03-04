// g++ -O3 -o bidi bidi.cpp
// Useful resource! https://en.wikibooks.org/wiki/Serial_Programming/termios
// Also used example code here: https://www.tldp.org/HOWTO/text/Serial-Programming-HOWTO
#include <iostream>
#include <string>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>

/*
File format
uint32_t nS number of DAC samples (parameter)
uint32_t adcF ADC base clock frequency (max 42000000?) (parameter)
uint32_t clkDiv divisor of base clock for sample frequency (min 42?) (parameter)
uint32_t nchan (supplied from arduino)
array of nS shorts DAC data
array of nchan*nS shorts ADC data
bool error
string end-timestamp (e.g. 2019-03-02_13:58:11)

Parameters
string path to file in correct format; nchan, ADC data, error and end-timestamp to be filled in.

Call directly or from python using os.execl(executable, datafile, port); python use is untested.
 */

#define MAX_TRANSFER 4096

int blocking_write(int fd, char *b, int n) {
  int w = 0;
  while (w < n) {
    ssize_t s = write(fd, &b[w], n-w);
    if (s >= 0) w += s;
  }
}

int blocking_read(int fd, char *b, int n) {
  int r = 0;
  while (r < n) {
    ssize_t s = read(fd, &b[r], n-r);
    if (s >= 0) r += s;
  }
}



int main(int argc, char *argv[]) {
  // Open file.
  std::string data(argv[1]);
  int f = open(data.c_str(), O_RDWR);
  std::cout << data << std::endl;
  if (f < 0) {
    std::cerr << "Error opening file " << data << std::endl;
    return(1);
  }
  
  int h[4];
  blocking_read(f, (char*) &h, 16);
  int nS = h[0];
  int adcF = h[1];
  int clkDiv = h[2];
  int nchan = h[3];
  std::cout << nS << " " << adcF << " " << clkDiv << " " << nchan << std::endl;
  // Open port.
  int p = open(argv[2], O_RDWR | O_NOCTTY | O_NDELAY);
  if (p == -1) {
    std::cerr << "Error opening port " << argv[2] << std::endl;
    return(1);
  }
  // Flush any prior garbage.
  tcflush(p, TCIOFLUSH);
  // Transmit header information.
  blocking_write(p, (char*)&h, 12);
  // Read nchan. Already supplied...
  blocking_read(p, (char*)&h[3], 4);
  // Read data offset.
  off_t rdo = 16 + nS * 2;
  // Error offset.
  off_t eo = rdo + nS*nchan*2;

  char * addr = NULL;
  // Error and timestamp add 20 bytes.
  addr = (char*)mmap(NULL, eo+20, PROT_READ | PROT_WRITE, MAP_SHARED, f, 0);
  // Write nchan to file.
  memcpy(addr+12, &h[3], 4);

  // Write and read counters.
  int w = 0;
  int r = 0;
  fd_set readfds, writefds;
  int maxfd = p + 1;
  while((w < nS*2) || (r < nS*nchan*2)) {
    if (w < nS*2) {
      FD_SET(p, &writefds);
    } else {
      FD_ZERO(&writefds);
    }
    if (r < nS*nchan*2) {
      FD_SET(p, &readfds);
    } else {
      FD_ZERO(&readfds);
    }
    select(maxfd, &readfds, &writefds, NULL, NULL);
    if (w < nS*2) {
      ssize_t n = write(p, addr+16+w, std::min(nS*2 - w, MAX_TRANSFER));
      if (n >= 0) w += n;
    }
    if (r < nS*nchan*2) {
      ssize_t n = read (p, addr+rdo+r, std::min(nS*nchan*2 - r, MAX_TRANSFER));
      if (n >= 0) r += n;
    }
  }
  // Write error flag.
  bool e;
  blocking_read(p, (char*)&e, 1);
  if (e) std::cerr << "Arduino error" << std::endl;
  std::memcpy(addr+eo, &e, 1);
  // Write end timestamp.
  time_t rawtime;
  struct tm * timeinfo;
  char tbuffer [20];
  time (&rawtime);
  timeinfo = localtime (&rawtime);
  strftime (tbuffer, 20, "%Y-%m-%d_%H:%M:%S", timeinfo);
  puts(tbuffer);
  std::memcpy(addr+eo+1, tbuffer, 19);
  munmap(addr, eo+20);
  close(f);
  return(0);
}
