#
# Makefile
# Callum McColl, 2019-06-05 11:58
#

.unexport SAY

.ifndef TARGET

STDS=98 03 11 14 17
STD?=17

.for std in ${STDS}
.unexport USE_CXX${std}
.endfor

.ifdef STD
.  if ${STD} == 03 || ${STD} == 98
CXXSTDFL=-std=c++${STD}
.  else
.    include "../../mk/c++${STD}.mk"
.  endif
.else
CXXSTDFL=-std=c++98
.endif
.endif

ALL_TARGETS=test robot-local

C_SRCS!=ls *.c
CC_SRCS!=ls *.cc
ALL_HDRS!=ls *.h *.hpp
DOC_HDRS=${ALL_HDRS}
SPECIFIC_LIBS+=-lm
SPECIFIC_LIBS+=-lguunits
SPECIFIC_LIBS+=-lgucoordinates
LOCAL=_LOCAL

${MODULE_BASE}_HDRS=${ALL_HDRS}
PKGCONFIG_NAME=gunavigation
PKGCONFIG_VERSION=1.0
PKGCONFIG_DESCRIPTION=Provides methods for calculating trajectories used in navigation.

#WFLAGS=-Weverything wn-warning-option ${WERROR}
WFLAGS=-Wall -Wno-unknown-pragmas -Wno-unknown-warning-option -Wno-c99-extensions -Wno-padded -Werror
.ifndef TARGET
WFLAGS+=-Weverything -Wno-poison-system-directories
.endif
CPP03_EXTRA_WFLAGS=-Wno-c++98-compat -Wno-c++98-compat-pedantic -Wc++98-compat-extra-semi
CPP11_EXTRA_WFLAGS=-Wno-c++98-compat -Wno-c++98-compat-pedantic -Wc++98-compat-extra-semi
CPP14_EXTRA_WFLAGS=-Wno-c++98-compat -Wno-c++98-compat-pedantic -Wc++98-compat-extra-semi
CPP17_EXTRA_WFLAGS=-Wno-c++98-compat -Wno-c++98-compat-pedantic -Wc++98-compat-extra-semi

all:	all-real

.ifndef TARGET
.ifndef COVERAGE
generate-coverage-report:
	${MAKE} generate-coverage-report COVERAGE=yes

c-coverage:
	${MAKE} c-coverage COVERAGE=yes
.else
CODE_COVERAGE=yes
SPECIFIC_CPPFLAGS+=-fprofile-arcs -ftest-coverage
SPECIFIC_LDFLAGS+=--coverage

LCOV!=which lcov &2>/dev/null
GENHTML!=which genhtml &2>/dev/null

generate-coverage-report:
.if empty(LCOV)
	${SAY} "You must install lcov in order to generate a code coverage report."
.endif
.if empty(GENHTML)
	${SAY} "You must install genhtml in order to generate a code coverage report."
.endif
.if !empty(LCOV) && !empty(GENHTML)
	rm -rf ${COVERAGE_BUILD_DIR}
	mkdir -p ${COVERAGE_BUILD_DIR}
	cp ${TEST_BUILD_DIR}/*.gc* ${COVERAGE_BUILD_DIR}/ || true 
	cp build.host-local/*.gc* ${COVERAGE_BUILD_DIR}/ || true
	cd ${COVERAGE_BUILD_DIR} && lcov --directory . --base-directory ${TEST_BUILD_DIR} --gcov-tool gcov --capture -o cov.info && genhtml --title "${COVERAGE_TITLE}" cov.info -o coverage || cd ${SRCDIR}
.endif


c-coverage:
	$E${MAKE} generate-coverage-report TEST_BUILD_DIR="${SRCDIR}/ctests/build.host" COVERAGE_BUILD_DIR="coverage/c" COVERAGE_TITLE="C Tests Coverage"
.endif

cpp-coverage:
.for std in ${STDS}
	bmake single-cpp-coverage STD=${std}
.endfor

.ifdef STD
single-cpp-coverage:
	bmake generate-coverage-report TEST_BUILD_DIR="${SRCDIR}/tests/build.host-${STD}" COVERAGE_BUILD_DIR="coverage/${STD}" COVERAGE_TITLE="C++${STD} Tests Coverage"
.endif

.ifdef IGNORE_TESTS

test:

.else

host-local:
	$Ebmake host-local IGNORE_TESTS=yes

robot-local:
	$Ebmake robot-local IGNORE_TESTS=yes

upload-robot:
	$Ebmake upload-robot IGNORE_TESTS=yes

test: ctest cpptest

coverage-test:
	$E${MAKE} test COVERAGE=yes STD=${STD}

build-lib:
	$Eenv ${MAKE} host-local MAKEFLAGS= IGNORE_TESTS=yes TESTING=yes

run-cpp-test:
.ifndef TARGET
	${SAY} "Testing C++ Implementation with C++${STD} Standard."
	$Ecd ${SRCDIR}/tests && ${MAKE} build-test STD=${STD} EXTRA_WFLAGS="${CPP${STD}_EXTRA_WFLAGS}" BUILDDIR=build.host-${STD} LOCAL= MAKEFLAGS= SDIR=${SRCDIR} TESTLIBDIR=${SRCDIR}/build.host-local && cd ${SRCDIR} && ./tests/build.host-${STD}/tests
.endif

ctest:
.ifndef TARGET
	$E${MAKE} clean
	${SAY} "Building Implementation with C99 Standard."
	$E${MAKE} build-lib
	${SAY} "Testing C Implementation."
	$Ecd ${SRCDIR}/ctests && ${MAKE} build-test BUILDDIR=build.host LOCAL= MAKEFLAGS= SDIR=${SRCDIR} TESTLIBDIR=${SRCDIR}/build.host-local && cd ${SRCDIR} && ./ctests/build.host/ctests
.ifdef COVERAGE
	$Ebmake c-coverage
.endif
.endif

coverage-ctest:
	$E${MAKE} ctest COVERAGE=yes


.for std in ${STDS}
STD_TARGETS+=cpp${std}test
.endfor

cpptest: ${STD_TARGETS}

coverage-cpptest:
	$E${MAKE} cpptest COVERAGE=yes

.for std in ${STDS}
cpp${std}test: build-lib
	$E${MAKE} clean
	${SAY} "Building Implementation with C++${std} Standard."
	$E${MAKE} build-lib STD=${std}
	$E${MAKE} run-cpp-test TESTING=yes STD=${std}
.ifdef COVERAGE
	$E${MAKE} single-cpp-coverage STD=${std}
.endif

cpp${std}coverage:
	$E${MAKE} cpp${std}test COVERAGE=yes
.endfor

.endif

.ifdef TESTING
HOST_LOCAL_OUTFLAGS=${HOST_LOCAL_SOFLAGS} ${COMMON_OFLAGS}
.endif
.endif

.include "../../mk/prefs.mk"

${MODULE_BASE}_HDRS=${ALL_HDRS}

.include "../../mk/mipal.mk"
# vim:ft=make
#

