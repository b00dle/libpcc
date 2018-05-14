

CXX = c++
CXXFLAGS = -g -std=c++0x -DLINUX -Wall -O3 -I../include -fPIC -lz -fopenmp

LDDFLAGS = -shared


SOURCES = $(wildcard $(SRCDIR)*.cpp)
OBJECTS = $(patsubst $(SRCDIR)%.cpp, %.o, $(SOURCES))
HEADERS = $(wildcard $(SRCDIR)*.hpp)

TARGET = ../lib/libpcc.so

default: $(TARGET)
	@echo built $(TARGET)




$(TARGET): $(OBJECTS) $(HEADERS) Makefile
	$(CXX) $(LDDFLAGS) -o $@ $(OBJECTS)

%.o: %.cpp Makefile
	$(CXX) -c $< $(CXXFLAGS)

clean:
	@rm -f $(OBJECTS)
	@echo cleaned

realclean: clean
	@rm -f $(TARGET) *~
	@echo realcleaned

