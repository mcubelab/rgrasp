
// helper functions
template<typename T1, typename T>
void setArrayFromScalars(T1& a, T x0,T x1,T x2,T x3,T x4,T x5,T x6,T x7,T x8)
{a[0] = x0; a[1] = x1;  a[2] = x2;  a[3] = x3;  a[4] = x4;  a[5] = x5; 
 a[6] = x6; a[7] = x7;  a[8] = x8; }
template<typename T1, typename T>
void setArrayFromScalars(T1& a, T x0,T x1,T x2,T x3,T x4,T x5,T x6,T x7,T x8,T x9,T x10,T x11)
{a[0] = x0; a[1] = x1;  a[2] = x2;  a[3] = x3;  a[4] = x4;   a[5] = x5; 
 a[6] = x6; a[7] = x7;  a[8] = x8;  a[9] = x9;  a[10] = x10; a[11] = x11;}

#include <sys/time.h>
#include <iomanip>
double tt_tic=0;

double tic() {
  struct timeval t;
  gettimeofday(&t, NULL);
  tt_tic = ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
  return tt_tic;
}

double toc(double t) {
  double s = tic();
  return s-t;
}

void print_toc(double t, std::string title) {
  std::cout << "[" << title << "] Elapsed time: " << std::setprecision(5) << toc(t) << "sec" << std::endl;
}
