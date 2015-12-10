ifdef ICC
CXX = icpc
WARN_FLAGS = -O3 -ipo -g -Wall -wd981 -wd383 -wd2259 -Werror # -Weffc++
else
CXX = g++
WARN_FLAGS = -O3 -g -Wall -Wextra -Wabi -Wctor-dtor-privacy -Wnon-virtual-dtor -Wreorder -Wstrict-null-sentinel -Woverloaded-virtual -Wshadow -Wcast-align -Wpointer-arith -Wwrite-strings -Wundef -Wredundant-decls -Werror # -Weffc++
endif

BIN = test
OBJECTS = OptionParser.o test.o

$(BIN): $(OBJECTS)
	$(CXX) -o $@ $(OBJECTS) $(WARN_FLAGS) $(LINKFLAGS)

%.o: %.cpp OptionParser.h
	$(CXX) $(WARN_FLAGS) $(CXXFLAGS) -c $< -o $@

.PHONY: clean

clean:
	rm -f *.o $(BIN)
