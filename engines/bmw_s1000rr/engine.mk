# the BMW S 1000 RR; included by the Makefile
ENGINE_HEADERS += engines/bmw_s1000rr/bmw_s1000rr.hpp
MESHES += engines/bmw_s1000rr/s1000rr.mesh

engines/bmw_s1000rr/s1000rr.mesh: engines/bmw_s1000rr/build_s1000rr.py tools/meshlib.py engines/inline4_fit/engine.mesh
	python3 -I engines/bmw_s1000rr/build_s1000rr.py
