# the Lexus LFA's 1LR-GUE V10; included by the Makefile
ENGINE_HEADERS += engines/lexus_lfa/lexus_lfa.hpp
MESHES += engines/lexus_lfa/lfa.mesh

engines/lexus_lfa/lfa.mesh: engines/lexus_lfa/build_lfa.py tools/meshlib.py engines/inline4_fit/engine.mesh
	python3 -I engines/lexus_lfa/build_lfa.py
