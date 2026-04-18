.PHONY: build test

configure:
	cmake -S . -B build \
	  -DCMAKE_BUILD_TYPE=Debug \
	  -DCONFIG_FILE=$(CURDIR)/config_sim.h \
	  -DBUILD_TESTS=ON

build: configure
	cmake --build build --parallel $$(nproc)

test: build
	ctest --test-dir build --output-on-failure
