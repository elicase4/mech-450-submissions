CXX_FLAGS=-std=c++11 -O2 -Wall -Wextra

INCLUDE_FLAGS=`pkg-config --cflags ompl eigen3`
# Linker options
LD_FLAGS=-lcollisionchecking `pkg-config --libs ompl`

# The c++ compiler to invoke
CXX=c++        # g++ -Og -g i used this for gdb debugging
all: Project4Pendulum Project4Car

clean:
	rm -f *.o
	rm -f Project4Pendulum Project4Car

%.o: src/%.cpp
	$(CXX) -c $(CXX_FLAGS) $(INCLUDE_FLAGS) $< -o $@

Project4Pendulum: Project4Pendulum.o RG-RRT.o
	$(CXX) $(CXX_FLAGS) $(INCLUDE_FLAGS) -Lsrc/. -o $@ $^ $(LD_FLAGS)

Project4Car: Project4Car.o RG-RRT.o
	$(CXX) $(CXX_FLAGS) $(INCLUDE_FLAGS) -Lsrc/. -o $@ $^ $(LD_FLAGS)
