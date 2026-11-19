# the Audi RS 3's 2.5 TFSI inline-5; included by the Makefile
ENGINE_HEADERS += engines/audi_rs3/audi_rs3.hpp
MESHES += engines/audi_rs3/rs3.mesh

engines/audi_rs3/rs3.mesh: engines/audi_rs3/build_rs3.py tools/meshlib.py engines/inline4_fit/engine.mesh
	python3 -I engines/audi_rs3/build_rs3.py

# the procedural turbo drawn when the downloaded turbo model (engines/inline4_fit/turbo.mesh) is not there
MESHES += engines/audi_rs3/rs3_turbo.mesh

engines/audi_rs3/rs3_turbo.mesh: engines/audi_rs3/build_rs3_turbo.py tools/meshlib.py
	python3 -I engines/audi_rs3/build_rs3_turbo.py
