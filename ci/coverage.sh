#!/bin/bash
# Run this script from the ADCore root directory
# Make sure we exit on any error
set -e

export ADCORE_DIR=`pwd`
export COVERAGE_DIR=${ADCORE_DIR}/ci

#export GCOV_PREFIX=${COVERAGE_DIR}
#export GCOV_PREFIX_STRIP=5

lcov --no-external --capture \
                   --directory ${ADCORE_DIR}/ADApp/ADSrc/ \
                   --directory ${ADCORE_DIR}/ADApp/pluginSrc/ \
                   --output-file ${COVERAGE_DIR}/coverage.info \
                   || { echo "Running lcov failed" && exit 2; }

# generate a readable html report
genhtml --output-directory ${COVERAGE_DIR}/html \
        --demangle-cpp ${COVERAGE_DIR}/coverage.info \
        --title "ADCore plugin-test" \
        || { echo "Running lcov genhtml failed" && exit 2; }

lcov --list ${COVERAGE_DIR}/coverage.info

