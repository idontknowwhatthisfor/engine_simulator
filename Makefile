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
H2   = engines/kawasaki_h2
MESHES = $(I4)/engine.mesh $(V4)/v4.mesh $(BMW)/s1000rr.mesh $(H2)/h2.mesh
# the turbocharger (--turbo yes); its scene.bin is an optional release asset (assets.json)
TURBO_BIN = $(I4)/turbocharger/scene.bin
MESHES += $(I4)/turbo.mesh

# one translation unit: engine_sim.cpp includes src/ and engines/*/ headers
HEADERS = $(wildcard src/*.hpp) $(wildcard engines/*/*.hpp)

all: engine_sim $(MESHES)

engine_sim: engine_sim.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $< -o $@ $(LDFLAGS) $(LDLIBS)

$(I4)/scene.bin: assets.json scripts/fetch_assets.py
	python3 -I scripts/fetch_assets.py

$(I4)/engine.mesh: $(I4)/extract_parts.py $(I4)/scene.gltf $(I4)/scene.bin
	python3 -I $(I4)/extract_parts.py

# skipped (and retried by the next make) while the turbo model is not downloaded
$(I4)/turbo.mesh: $(I4)/build_turbo.py $(I4)/turbocharger/scene.gltf assets.json $(wildcard $(TURBO_BIN))
	@test -f $(TURBO_BIN) || python3 -I scripts/fetch_assets.py
	@if [ -f $(TURBO_BIN) ]; then python3 -I $(I4)/build_turbo.py; \
	else echo "turbo model not downloaded: --turbo yes is unavailable (see assets.json)"; fi

$(V4)/v4.mesh: $(V4)/build_v4.py tools/meshlib.py $(I4)/engine.mesh
	python3 -I $(V4)/build_v4.py

$(BMW)/s1000rr.mesh: $(BMW)/build_s1000rr.py tools/meshlib.py $(I4)/engine.mesh
	python3 -I $(BMW)/build_s1000rr.py

# one run writes both the engine and the supercharger
$(H2)/h2.mesh: $(H2)/build_h2.py tools/meshlib.py $(I4)/engine.mesh
	python3 -I $(H2)/build_h2.py

run: all
	./engine_sim --engine-type inline4

run-v4: all
	./engine_sim --engine-type v4

run-s1000rr: all
	./engine_sim --engine-type s1000rr

run-h2: all
	./engine_sim --engine-type h2

# kinematics and physics checks for every engine (headless)
test: all
	@for t in inline4 v4 s1000rr h2; do \
		printf "%-8s selftest " $$t; ./engine_sim --engine-type $$t --selftest | tail -1; \
		printf "%-8s phystest " $$t; ./engine_sim --engine-type $$t --phystest | tail -1; \
	done
	@if [ -f $(I4)/turbo.mesh ]; then printf "%-8s phystest " "turbo"; ./engine_sim --turbo yes --phystest | tail -1; fi

clean:
	rm -f engine_sim

.PHONY: all run run-v4 run-s1000rr run-h2 test clean

assets:
	python3 -I scripts/fetch_assets.py

.PHONY: assets
