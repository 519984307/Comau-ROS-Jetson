#ifndef ACTION_GLOBAL_H
#define ACTION_GLOBAL_H

#include <QtCore/qglobal.h>

#ifndef BUILD_STATIC
# if defined(ACTION_LIB)
#  define ACTION_EXPORT Q_DECL_EXPORT
# else
#  define ACTION_EXPORT Q_DECL_IMPORT
# endif
#else
# define ACTION_EXPORT
#endif

#endif // ACTION_GLOBAL_H
