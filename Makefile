TIME_COMMAND=/usr/bin/time --format "buildtime: real=%e user=%U sys=%S [ %C ]"

BUILD_DIR=build

RELEASE_BUILD_TYPE=RelWithDebInfo

RELEASE_DIR=$(shell echo "${RELEASE_BUILD_TYPE}" | tr '[:upper:]' '[:lower:]')

NUM_JOBS := $(shell nproc --all)
BUILD_COMMAND := $(MAKE) --jobs $(NUM_JOBS) --no-print-directory

# The list of files which should trigger a re-cmake, find all CMakeLists.txt and .cmake files in the
# repro, excluding the build, install and .git folder
CMAKE_DEPS := $(shell find * \( -name CMakeLists.txt -or -name '*.cmake' \) -not \( -path "${BUILD_DIR}/*" -o -path "${INSTALL_DIR}/*" -o -path ".git/*" \) | sort)

# If the .ONESHELL special target appears anywhere in the makefile then all recipe lines for each
# target will be provided to a single invocation of the shell.
.ONESHELL:
SHELL=/bin/bash

# Default to release build
all: release

#################### RELEASE
${BUILD_DIR}/Makefile: $(CMAKE_DEPS)
	@ $(eval LOGFILE=$(shell echo cmake-`date +%Y%m%dT%H%M%S`.log))
	@ $(info build: configuring ${RELEASE_BUILD_TYPE} build via cmake [log: ${BUILD_DIR}/${LOGFILE}]...)
	@ mkdir -p ${BUILD_DIR}
	@ cd ${BUILD_DIR}
	@ $(TIME_COMMAND) cmake ../ -DCMAKE_BUILD_TYPE="${RELEASE_BUILD_TYPE}" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON > ${LOGFILE} 2>&1 && (echo "release cmake SUCCEEDED") || (/usr/bin/tail -25 ${LOGFILE}; echo "release cmake FAILED"; exit 1)

.PHONY: release
release: ${BUILD_DIR}/Makefile
	@ $(eval LOGFILE=$(shell echo build-`date +%Y%m%dT%H%M%S`.log))
	@ $(info build: building ${RELEASE_BUILD_TYPE} [log: ${BUILD_DIR}/${LOGFILE}]...)
	@ set -e -o pipefail
	@ $(TIME_COMMAND) $(BUILD_COMMAND) -C "${BUILD_DIR}" 2>&1 && (echo "release build SUCCEEDED") || (echo "release build FAILED"; exit 1) | tee ${BUILD_DIR}/build-`date +%Y%m%dT%H%M%S`.log
	@ cd ${BUILD_DIR}

.PHONY: clean
clean:
	@ $(TIME_COMMAND) rm -rf "${BUILD_DIR}" && (echo "clean SUCCEEDED") || (echo "clean FAILED"; exit 1)
	@ $(info build: outputs cleaned.)
