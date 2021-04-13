#!/bin/bash
set -e

TOP_DIR="$(pwd)"

sudo apt-get update

. ./ci/lib.sh

build_cppcheck() {
    . ./build/cppcheck.sh
}

build_astyle() {
    export ASTYLE_EXT_LIST=".cpp .hpp"
    . ./build/astyle.sh
}

build_${BUILD_TYPE}
