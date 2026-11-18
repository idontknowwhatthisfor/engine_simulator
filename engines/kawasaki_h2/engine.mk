# the Kawasaki Ninja H2; included by the Makefile
ENGINE_HEADERS += engines/kawasaki_h2/kawasaki_h2.hpp
MESHES += engines/kawasaki_h2/h2.mesh

# one run writes both the engine and the supercharger
engines/kawasaki_h2/h2.mesh: engines/kawasaki_h2/build_h2.py tools/meshlib.py engines/inline4_fit/engine.mesh
	python3 -I engines/kawasaki_h2/build_h2.py
