# Library Info
AC_INIT([libpiDuino], [1.0], [jgarzagu@ucsd.edu])
AM_PROG_AR

# Look for C++ compiler
AC_PROG_CXX

# Tell to autotools that a library is built in this package
AC_PROG_RANLIB

# Checks for libraries.
#AX_PTHREAD([AC_MSG_RESULT([yes]), [AC_MSG_ERROR([requires pthread])])
#AX_PTHREAD()
##PTHREAD_CFLAGS
##PKG_CHECK_MODULES([GLIB], [glib-2.0], [have_libglib=yes], [have_libglib=no])
##AM_CONDITIONAL([GLIB],  [test "$have_libglib" = "yes"])

# Checks for header files.
#AC_CHECK_HEADERS([fcntl.h stdint.h stdlib.h string.h sys/ioctl.h termios.h unistd.h])

# Checks for typedefs, structures, and compiler characteristics.
#AC_CHECK_HEADER_STDBOOL
#AC_C_INLINE
#AC_TYPE_INT8_T
#AC_TYPE_SIZE_T
#AC_TYPE_UINT16_T
#AC_TYPE_UINT32_T
#AC_TYPE_UINT8_T

# Checks for library functions.
#AC_FUNC_MALLOC
#AC_FUNC_MMAP
#AC_FUNC_REALLOC
#AC_CHECK_FUNCS([clock_gettime isascii memmove memset pow strchr strcspn strerror strrchr strspn strstr])

# Compiler Flags
#LIBS="$PTHREAD_LIBS $LIBS"
#LDFLAGS="-lpthread $LD_FLAGS"
#CFLAGS="$CFLAGS $PTHREAD_CFLAGS"
#CXXFLAGS="$PTHREAD_CC"

# Enable "automake" to simplify creating makefiles
AM_INIT_AUTOMAKE([-Wall foreign])
AC_CONFIG_FILES([Makefile])

# Do final output
AC_OUTPUT