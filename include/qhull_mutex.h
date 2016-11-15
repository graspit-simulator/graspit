#ifndef _qhull_mutex_h_
#define _qhull_mutex_h_

/*!	\file
    This is the mutex that should be used to constrain access to qhull, 
    which is not thread-safe. Any file using qhull should include this 
	header and use the defined mutex.
*/

#include <qmutex.h>

//! The global mutex used to synchronize access to QHull
extern QMutex qhull_mutex;

#endif
