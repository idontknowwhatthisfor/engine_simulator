# the engine's part of the Makefile (the Makefile includes every engines/*/engine.mk, except in
# folders starting with _, like this template): its registering header, its meshes and their rule
ENGINE_HEADERS += engines/_template/myengine.hpp
MESHES += engines/_template/myengine.mesh

# rebuilt when the builder, meshlib or the inline-4 mesh it takes parts from changes
engines/_template/myengine.mesh: engines/_template/build_myengine.py tools/meshlib.py engines/inline4_fit/engine.mesh
	python3 -I engines/_template/build_myengine.py
