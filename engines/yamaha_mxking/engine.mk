# the engine's part of the Makefile (the Makefile includes every engines/*/engine.mk, except in
# folders starting with _, like this template): its registering header, its meshes and their rule
ENGINE_HEADERS += engines/yamaha_mxking/mxking.hpp
MESHES += engines/yamaha_mxking/mxking.mesh

# rebuilt when the builder, meshlib or the inline-4 mesh it takes parts from changes
engines/yamaha_mxking/mxking.mesh: engines/yamaha_mxking/build_mxking.py tools/meshlib.py engines/inline4_fit/engine.mesh
	python3 -I engines/yamaha_mxking/build_mxking.py
