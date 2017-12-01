

CXX = c++
CXXFLAGS = -std=c++0x -g -DLINUX -Wall -O3 -I../include \
	-L../lib -lpcc -lzmq -Wl,-rpath,../lib


SOURCES = $(wildcard *.cpp)
OBJECTS = $(patsubst %.cpp, %.o, $(SOURCES))
TARGETS = $(patsubst %.cpp, %, $(SOURCES))


default:
	cd ../src && make
	make examples

examples: $(TARGETS)
	@echo built $(TARGETS)

%: %.cpp Makefile
	$(CXX) $< $(CXXFLAGS) -o $@

clean:
	cd ../src && make clean
	@rm -f $(TARGETS)
	@echo cleaned

realclean: clean
	cd ../src && make realclean
	@rm -f *~
	@echo realcleaned

