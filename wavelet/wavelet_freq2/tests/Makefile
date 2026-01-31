CC=g++
CFLAGS=-I.. -I. -O2
DEPS = ../phase_estimator.h
OBJ = test_phase_estimator.o test_pll_convergence.o ../phase_estimator.o

all: test_phase_estimator test_pll_convergence

%.o: %.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

test_phase_estimator: test_phase_estimator.o ../phase_estimator.o
	$(CC) -o $@ $^ $(CFLAGS)

test_pll_convergence: test_pll_convergence.o ../phase_estimator.o
	$(CC) -o $@ $^ $(CFLAGS)

clean:
	rm -f *.o test_phase_estimator test_pll_convergence
