TIME_COMMAND=/usr/bin/time --format "buildtime: real=%e user=%U sys=%S [ %C ]"

BUILD_DIR=build

RELEASE_BUILD_TYPE=RelWithDebInfo
DEBUG_BUILD_TYPE=Debug

RELEASE_DIR=$(shell echo "${RELEASE_BUILD_TYPE}" | tr '[:upper:]' '[:lower:]')
DEBUG_DIR=$(shell echo "${DEBUG_BUILD_TYPE}" | tr '[:upper:]' '[:lower:]')

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
all: debug

#################### DEBUG
${BUILD_DIR}/${DEBUG_DIR}/Makefile: $(CMAKE_DEPS)
	@ $(eval LOGFILE=$(shell echo cmake-`date +%Y%m%dT%H%M%S`.log))
	@ $(info build: configuring ${DEBUG_BUILD_TYPE} build via cmake [log: ${BUILD_DIR}/${LOGFILE}]...)
	@ mkdir -p ${BUILD_DIR}/${DEBUG_DIR}
	@ cd ${BUILD_DIR}/${DEBUG_DIR}
	@ $(TIME_COMMAND) cmake ../../ -DCMAKE_BUILD_TYPE="${DEBUG_BUILD_TYPE}" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON > ${LOGFILE} 2>&1 && (echo "release cmake SUCCEEDED") || (/usr/bin/tail -25 ${LOGFILE}; echo "release cmake FAILED"; exit 1)

.PHONY: debug
debug: ${BUILD_DIR}/${DEBUG_DIR}/Makefile
	@ $(eval LOGFILE=$(shell echo build-`date +%Y%m%dT%H%M%S`.log))
	@ $(info build: building ${RELEASE_BUILD_TYPE} [log: ${BUILD_DIR}/${LOGFILE}]...)
	@ set -e -o pipefail
	@ $(TIME_COMMAND) $(BUILD_COMMAND) -C "${BUILD_DIR}/${DEBUG_DIR}" 2>&1 && (echo "release build SUCCEEDED") || (echo "release build FAILED"; exit 1) | tee ${BUILD_DIR}/build-`date +%Y%m%dT%H%M%S`.log
	@ cd ${BUILD_DIR}
	@ rm -f latest
	@ /bin/ln -s "${DEBUG_DIR}" latest

#################### RELEASE
${BUILD_DIR}/${RELEASE_DIR}/Makefile: $(CMAKE_DEPS)
	@ $(eval LOGFILE=$(shell echo cmake-`date +%Y%m%dT%H%M%S`.log))
	@ $(info build: configuring ${RELEASE_BUILD_TYPE} build via cmake [log: ${BUILD_DIR}/${LOGFILE}]...)
	@ mkdir -p ${BUILD_DIR}/${RELEASE_DIR}
	@ cd ${BUILD_DIR}/${RELEASE_DIR}
	@ $(TIME_COMMAND) cmake ../../ -DCMAKE_BUILD_TYPE="${RELEASE_BUILD_TYPE}" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON > ${LOGFILE} 2>&1 && (echo "release cmake SUCCEEDED") || (/usr/bin/tail -25 ${LOGFILE}; echo "release cmake FAILED"; exit 1)

.PHONY: release
release: ${BUILD_DIR}/${RELEASE_DIR}/Makefile
	@ $(eval LOGFILE=$(shell echo build-`date +%Y%m%dT%H%M%S`.log))
	@ $(info build: building ${RELEASE_BUILD_TYPE} [log: ${BUILD_DIR}/${LOGFILE}]...)
	@ set -e -o pipefail
	@ $(TIME_COMMAND) $(BUILD_COMMAND) -C "${BUILD_DIR}/${RELEASE_DIR}" 2>&1 && (echo "release build SUCCEEDED") || (echo "release build FAILED"; exit 1) | tee ${BUILD_DIR}/build-`date +%Y%m%dT%H%M%S`.log
	@ cd ${BUILD_DIR}
	@ rm -f latest
	@ /bin/ln -s "${RELEASE_DIR}" latest

.PHONY: clean
clean:
	@ $(TIME_COMMAND) rm -rf "${BUILD_DIR}" && (echo "clean SUCCEEDED") || (echo "clean FAILED"; exit 1)
	@ $(info build: outputs cleaned.)
