#!/bin/bash
set -e

sudo apt-get update

. ./ci/travis/lib.sh

build_cppcheck() {
    . ./build/cppcheck.sh
}

build_astyle() {
    export ASTYLE_EXT_LIST=".cpp .hpp"
    . ./build/astyle.sh
}

build_${BUILD_TYPE}
