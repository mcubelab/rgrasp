all:
	g++ -c -fPIC pcprocess.cpp -o pcprocess.o -I/usr/include/python2.7 -O2
	g++ -shared -Wl,-soname,libpcprocess.so -o libpcprocess.so  pcprocess.o -O2
