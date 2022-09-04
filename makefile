export

debugflags = -ggdb
releaseflags = -O3 -s -march=native -fomit-frame-pointer
cppflags = --std=c++20 -Wall -Wextra -pedantic ${releaseflags}

otto: main.o
	g++ $$cppflags -o $@ $<

%.o: %.cpp makefile
	g++ $$cppflags -o $@ $< -c

clean:
	rm -f *.o otto
