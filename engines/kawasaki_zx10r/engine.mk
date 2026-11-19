# the Kawasaki Ninja ZX-10R; included by the Makefile
ENGINE_HEADERS += engines/kawasaki_zx10r/kawasaki_zx10r.hpp
MESHES += engines/kawasaki_zx10r/zx10r.mesh

engines/kawasaki_zx10r/zx10r.mesh: engines/kawasaki_zx10r/build_zx10r.py tools/meshlib.py engines/inline4_fit/engine.mesh
	python3 -I engines/kawasaki_zx10r/build_zx10r.py
