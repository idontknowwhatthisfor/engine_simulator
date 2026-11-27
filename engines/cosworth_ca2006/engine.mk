# the Cosworth CA2006 2.4 L F1 V8 (Williams FW28); included by the Makefile
ENGINE_HEADERS += engines/cosworth_ca2006/cosworth_ca2006.hpp
MESHES += engines/cosworth_ca2006/ca2006.mesh

engines/cosworth_ca2006/ca2006.mesh: engines/cosworth_ca2006/build_ca2006.py tools/meshlib.py engines/inline4_fit/engine.mesh
	python3 -I engines/cosworth_ca2006/build_ca2006.py
