#!/bin/bash
set -e

. ./ci/travis/lib.sh

build_default() {
    . ./build/cppcheck.sh
}

build_astyle() {
    export ASTYLE_EXT_LIST=".cpp .hpp"
    . ./build/astyle.sh
}

build_${BUILD_TYPE:-default}
