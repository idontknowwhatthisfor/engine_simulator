# the 2014 Ford Mustang V6's 3.7 L Cyclone; included by the Makefile
ENGINE_HEADERS += engines/ford_mustang_v6/mustang_v6.hpp
MESHES += engines/ford_mustang_v6/mustang_v6.mesh

# one run writes both the engine and the tuned car's twin-screw blower (mustang_v6_blower.mesh)
engines/ford_mustang_v6/mustang_v6.mesh: engines/ford_mustang_v6/build_mustang_v6.py tools/meshlib.py engines/inline4_fit/engine.mesh
	python3 -I engines/ford_mustang_v6/build_mustang_v6.py
