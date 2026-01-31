CC=g++
CFLAGS=-I.. -I. -O2
DEPS = ../phase_estimator.h
OBJ = test_phase_estimator.o ../phase_estimator.o

%.o: %.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

test_phase_estimator: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)

clean:
	rm -f *.o test_phase_estimator
