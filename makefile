export

debugflags = -Og -ggdb
releaseflags = -Ofast -s -march=native
cppflags = --std=c++20 -Wall -Wextra -pedantic ${releaseflags}

otto: main.o
	g++ $$cppflags -o $@ $<

%.o: %.cpp makefile
	g++ $$cppflags -o $@ $< -c

callgrind.svg: callgrind.out
	python -m gprof2dot -f callgrind $< | dot -Tsvg -o $@

callgrind.out: generated/big.txt otto
	valgrind --tool=callgrind --callgrind-out-file=$@ ./otto < $<

generated/%.txt:
	./generate.py

clean:
	rm -f *.o *.out *.svg otto
	rm -rf generated
