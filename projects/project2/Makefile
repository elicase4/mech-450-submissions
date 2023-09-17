# Change -O2 to -g when you are debugging your code
CXXFLAGS      = -O2 -std=c++14

# Include directories
# This includes OMPL's include directory and Eigen3's (a required dependency)
# pkg-config finds the needed parameters for OMPL and Eigen3.
INCLUDE_FLAGS = `pkg-config --cflags ompl eigen3`

# Linker options and libraries
LD_FLAGS      = `pkg-config --libs ompl` \
                -lse2ez_plan -lse2ez_core \
                -lyaml-cpp -lBox2D -lorocos-kdl \
                -lboost_system -lboost_filesystem -lboost_program_options

# The C++ compiler to invoke (must have support for C++11)
CXX           = c++

PROGRAMS      = se2ez-benchmark se2ez-plan

.PHONY : all
all : $(PROGRAMS)

# Remove all programs
clean :
	$(RM) $(PROGRAMS)

# Compile the object files into the main program
% :
	$(CXX) $(INCLUDE_FLAGS) $(CXXFLAGS) -o $@ src/$@.cpp $(LD_FLAGS)
