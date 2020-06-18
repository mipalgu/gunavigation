#
# Makefile
# Callum McColl, 2019-06-05 11:58
#

ALL_TARGETS=test robot-local

C_SRCS!=ls *.c
CPP_SRCS!=ls *.cpp
ALL_HDRS!=ls *.h
SPECIFIC_LIBS=-lguunits -lguvision_utils
LOCAL=_LOCAL

.include "../../mk/c++17.mk"

${MODULE_BASE}_HDRS=${ALL_HDRS}
PKGCONFIG_NAME=gunavigation
PKGCONFIG_VERSION=1.0
PKGCONFIG_DESCRIPTION=Simple utilities used for calculating arcs and trajectories when navigating.

all:	all-real

.ifndef IGNORE_TESTS

host-local:
	$Ebmake host-local IGNORE_TESTS=yes

robot-local:
	$Ebmake robot-local IGNORE_TESTS=yes

upload-robot:
	$Ebmake upload-robot IGNORE_TESTS=yes

test:
.ifndef TARGET
	$Eenv ${MAKE} host-local MAKEFLAGS= IGNORE_TESTS=yes TESTING=yes
	$Ecd ${SRCDIR}/tests && ${MAKE} build-test BUILDDIR=build.host LOCAL= MAKEFLAGS= SDIR=${SRCDIR} TESTLIBDIR=${SRCDIR}/build.host-local && cd ${SRCDIR} && ./tests/build.host/tests || cd ${SRCDIR} && env ${MAKE} host-local MAKEFLAGS= IGNORE_TESTS=yes
.endif
.endif

.ifdef TESTING
HOST_LOCAL_OUTFLAGS=${HOST_LOCAL_SOFLAGS} ${COMMON_OFLAGS}
.endif

.include "../../mk/mipal.mk"
# vim:ft=make
#
