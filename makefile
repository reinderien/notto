export

debugflags = -Og -ggdb
releaseflags = -Ofast -s -march=native
cppflags = --std=c++20 -Wall -Wextra -pedantic ${releaseflags}

otto: main.o
	g++ $$cppflags -o $@ $<

%.o: %.cpp makefile
	g++ $$cppflags -o $@ $< -c

clean:
	rm -f *.o otto
