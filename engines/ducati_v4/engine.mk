# the Ducati V4; included by the Makefile
ENGINE_HEADERS += engines/ducati_v4/ducati_v4.hpp
MESHES += engines/ducati_v4/v4.mesh

engines/ducati_v4/v4.mesh: engines/ducati_v4/build_v4.py tools/meshlib.py engines/inline4_fit/engine.mesh
	python3 -I engines/ducati_v4/build_v4.py
