#
# Makefile
# Callum McColl, 2019-06-05 11:58
#

ALL_TARGETS=host-local robot-local

C_SRCS!=ls *.c
ALL_HDRS!=ls *.h
SPECIFIC_LIBS=-lguunits
LOCAL=_LOCAL

.include "../../mk/c++17.mk"

${MODULE_BASE}_HDRS=${ALL_HDRS}
PKGCONFIG_NAME=gunavigation
PKGCONFIG_VERSION=1.0
PKGCONFIG_DESCRIPTION=Simple utilities used for calculating arcs and trajectories when navigating.

.include "../../mk/mipal.mk"
# vim:ft=make
#
