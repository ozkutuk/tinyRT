all: raytracer

CXXFLAGS := -Wall -Wextra -Wnon-virtual-dtor -pedantic -O3 -std=c++17
LDFLAGS := -lpthread 

raytracer: main.cpp parser.cpp ppmimage.cpp tinyxml2.cpp tinymath.cpp parser.h tinyxml2.h tinymath.h ppmimage.h
	$(CXX) $(CXXFLAGS) main.cpp parser.cpp ppmimage.cpp tinyxml2.cpp tinymath.cpp $(LDFLAGS) -o raytracer
