WFLAGS=-Wall -Weverything -Wno-unknown-pragmas -Wno-unknown-warning-option -Wno-global-constructors -Werror -Wno-padded -Wno-weak-vtables -Wno-c99-extensions -Wno-variadic-macros -Wno-used-but-marked-unused ${EXTRA_WFLAGS} -Wno-poison-system-directories
.ifdef STD
.  if ${STD} == 03 || ${STD} == 98
CXXSTDFL=-std=c++${STD}
.  else
.    include "../../../mk/c++${STD}.mk"
.  endif
.else
CXXSTDFL=-std=c++98
.endif

ALL_TARGETS=build-test

SDIR?=.

HDRS!=ls *.h 2>/dev/null || :
C_SRCS!=ls *.c 2>/dev/null || :
CC_SRCS!=ls *.cc 2>/dev/null || :
CPP_SRCS!=ls *.cpp 2>/dev/null || :
CXXFLAGS+=-I${SDIR} -I../../../../Common -I../../../gusimplewhiteboard
TESTLIBDIR?=${SDIR}/../build.host-local
SPECIFIC_LIBS=-L${TESTLIBDIR} -lgunavigation -L/usr/local/lib -lgtest -lgtest_main -lguunits -lgucoordinates -rpath ${TESTLIBDIR} --coverage

CODE_COVERAGE=yes

all:	all-real

build-test: clean-build-dir host

test:

clean-build-dir:
	rm -rf ${BUILDDIR}

.include "../../../mk/mipal.mk"

LDFLAGS=
