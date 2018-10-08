#!/bin/bash
set -e

build_default() {
    . ./ci/scripts/cppcheck.sh
}

build_astyle() {
    . ./ci/scripts/astyle.sh
}

build_${BUILD_TYPE:-default}
