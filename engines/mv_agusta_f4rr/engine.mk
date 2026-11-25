# the MV Agusta F4 RR; included by the Makefile
ENGINE_HEADERS += engines/mv_agusta_f4rr/mv_agusta_f4rr.hpp
MESHES += engines/mv_agusta_f4rr/f4rr.mesh

engines/mv_agusta_f4rr/f4rr.mesh: engines/mv_agusta_f4rr/build_f4rr.py tools/meshlib.py engines/inline4_fit/engine.mesh
	python3 -I engines/mv_agusta_f4rr/build_f4rr.py
