ALL_TARGETS=build-test

SDIR?=.

HDRS!=ls *.h 2>/dev/null || :
CC_SRCS!=ls *.cc 2>/dev/null || :
CPP_SRCS!=ls *.cpp 2>/dev/null || :
CXXFLAGS+=-I${SDIR} -I../../../../Common -I../../../gusimplewhiteboard
TESTLIBDIR?=${SDIR}/../build.host-local
SPECIFIC_LIBS=-L${TESTLIBDIR} -lgunavigation -L/usr/local/lib -lgtest -lgtest_main -lguunits -Xlinker -rpath ${TESTLIBDIR}
WFLAGS=

all:	all-real

build-test: clean host

test:

.include "../../../mk/c++17.mk"
.include "../../../mk/mipal.mk"

LDFLAGS=
