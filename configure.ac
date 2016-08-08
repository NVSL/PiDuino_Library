# Library Info
AC_INIT([libpiDuino], [1.0], [jgarzagu@ucsd.edu])

# Use C++ compiler
AC_PROG_CXX

# Add AR Archiver packaging executable objects (*.o) into a library (*.a or *.so)
AM_PROG_AR

# Add libtools
AC_CONFIG_MACRO_DIR([m4])
AC_PROG_LIBTOOL
LT_INIT

# Checks for header files.
AC_CHECK_HEADERS([pthread.h math.h])

# Enable "automake" to simplify creating makefiles
AM_INIT_AUTOMAKE([-Wall foreign])
AC_CONFIG_FILES([Makefile])

# Do final output
AC_OUTPUT