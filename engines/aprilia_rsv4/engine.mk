# the Aprilia RSV4 RR; included by the Makefile
ENGINE_HEADERS += engines/aprilia_rsv4/aprilia_rsv4.hpp
MESHES += engines/aprilia_rsv4/rsv4.mesh

engines/aprilia_rsv4/rsv4.mesh: engines/aprilia_rsv4/build_rsv4.py tools/meshlib.py engines/inline4_fit/engine.mesh
	python3 -I engines/aprilia_rsv4/build_rsv4.py
