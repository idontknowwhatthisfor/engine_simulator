# the inline-4 (the original model) and its optional turbocharger; included by the Makefile.
# The other engines are built from this one's parts (they depend on engine.mesh).
ENGINE_HEADERS += engines/inline4_fit/inline4_fit.hpp
MESHES += engines/inline4_fit/engine.mesh engines/inline4_fit/turbo.mesh

engines/inline4_fit/scene.bin: assets.json scripts/fetch_assets.py
	python3 -I scripts/fetch_assets.py

engines/inline4_fit/engine.mesh: engines/inline4_fit/extract_parts.py tools/meshlib.py engines/inline4_fit/scene.gltf engines/inline4_fit/scene.bin
	python3 -I engines/inline4_fit/extract_parts.py

# the turbocharger (the tuned inline-4's, --tuned yes, and the RS 3's): its scene.bin is an optional release asset (assets.json). Skipped
# (and retried by the next make) while it is not downloaded
engines/inline4_fit/turbo.mesh: engines/inline4_fit/build_turbo.py engines/inline4_fit/turbocharger/scene.gltf assets.json $(wildcard engines/inline4_fit/turbocharger/scene.bin)
	@test -f engines/inline4_fit/turbocharger/scene.bin || python3 -I scripts/fetch_assets.py
	@if [ -f engines/inline4_fit/turbocharger/scene.bin ]; then python3 -I engines/inline4_fit/build_turbo.py; \
	else echo "turbo model not downloaded: the RS 3 and the tuned inline-4 draw the procedural turbo (see assets.json)"; fi
