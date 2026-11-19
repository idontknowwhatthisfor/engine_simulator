# the Yamaha MX King 150; included by the Makefile
ENGINE_HEADERS += engines/yamaha_mxking/mxking.hpp
MESHES += engines/yamaha_mxking/mxking.mesh engines/yamaha_mxking/mxking_race.mesh

# one run writes both the stock engine and the race one (--tuned yes); rebuilt when the builder,
# meshlib or the inline-4 mesh it takes parts from changes
engines/yamaha_mxking/mxking.mesh: engines/yamaha_mxking/build_mxking.py tools/meshlib.py engines/inline4_fit/engine.mesh
	python3 -I engines/yamaha_mxking/build_mxking.py

# written with the stock mesh; built on its own only when it is missing
engines/yamaha_mxking/mxking_race.mesh: engines/yamaha_mxking/mxking.mesh
	@test -f $@ -a $@ -nt $< || python3 -I engines/yamaha_mxking/build_mxking.py
