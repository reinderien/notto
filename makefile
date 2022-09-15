export

debugflags = -O0 -ggdb
releaseflags = -Ofast -march=native -DNDEBUG -s
cgrindflags = -Ofast -march=native -DNDEBUG -mno-avx512f
cppflags = --std=c++20 -Wall -Wextra -pedantic ${releaseflags}
pyflags = -OO

pycache = __pycache__/main.cpython-310.opt-2.pyc

all: otto ${pycache}

otto: main.o
	g++ $$cppflags -o $@ $<

%.o: %.cpp makefile
	g++ $$cppflags -o $@ $< -c

callgrind.svg: callgrind.out
	python -m gprof2dot -wf callgrind $< | dot -Tsvg -o $@

callgrind.out: generated/big.txt otto
	valgrind --tool=callgrind --callgrind-out-file=$@ ./otto < $<

generated/%.txt:
	./generate.py

${pycache}: main.py makefile
	python $$pyflags -m py_compile $<

clean:
	rm -f *.o *.out *.svg otto
	rm -rf generated __pycache__
