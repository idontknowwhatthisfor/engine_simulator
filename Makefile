CXX      ?= g++
CXXFLAGS ?= -std=c++20 -O2 -Wall -Wextra
LDLIBS   += -lsfml-audio -lsfml-graphics -lsfml-window -lsfml-system

ifeq ($(shell uname -s),Darwin)
SFML     ?= /opt/homebrew
CPPFLAGS += -DGL_SILENCE_DEPRECATION
LDLIBS   += -framework OpenGL
else
SFML     ?= /usr/local
LDLIBS   += -lGL
endif

CPPFLAGS += -DGL_GLEXT_PROTOTYPES -I$(SFML)/include
LDFLAGS  += -L$(SFML)/lib -Wl,-rpath,$(SFML)/lib

I4   = engines/inline4_fit
V4   = engines/ducati_v4
BMW  = engines/bmw_s1000rr
MESHES = $(I4)/engine.mesh $(V4)/v4.mesh $(BMW)/s1000rr.mesh

# one translation unit: engine_sim.cpp includes src/ and engines/*/ headers
HEADERS = $(wildcard src/*.hpp) $(wildcard engines/*/*.hpp)

all: engine_sim $(MESHES)

engine_sim: engine_sim.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $< -o $@ $(LDFLAGS) $(LDLIBS)

$(I4)/engine.mesh: $(I4)/extract_parts.py $(I4)/scene.gltf $(I4)/scene.bin
	python3 -I $(I4)/extract_parts.py

$(V4)/v4.mesh: $(V4)/build_v4.py tools/meshlib.py $(I4)/engine.mesh
	python3 -I $(V4)/build_v4.py

$(BMW)/s1000rr.mesh: $(BMW)/build_s1000rr.py tools/meshlib.py $(I4)/engine.mesh
	python3 -I $(BMW)/build_s1000rr.py

run: all
	./engine_sim --engine-type inline4

run-v4: all
	./engine_sim --engine-type v4

run-s1000rr: all
	./engine_sim --engine-type s1000rr

# kinematics and physics checks for every engine (headless)
test: all
	@for t in inline4 v4 s1000rr; do \
		printf "%-8s selftest " $$t; ./engine_sim --engine-type $$t --selftest | tail -1; \
		printf "%-8s phystest " $$t; ./engine_sim --engine-type $$t --phystest | tail -1; \
	done

clean:
	rm -f engine_sim

.PHONY: all run run-v4 run-s1000rr test clean
