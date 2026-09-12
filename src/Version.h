/*
 * Version.h
 *
 *  Created on: 25 Dec 2016
 *      Author: David
 */

#ifndef SRC_VERSION_H_
#define SRC_VERSION_H_

#include <ecv_duet3d.h>

#ifndef VERSION
// Note: the complete VERSION string must be in standard version number format and must not contain spaces! This is so that DWC can parse it.
# define MAIN_VERSION	"3.7.0-beta.1"
// Fork identity. Without this a running board is indistinguishable from stock Duet firmware in
// M115 and in DWC, which matters when several machines run different local builds. Bump
// FORK_VERSION whenever a build is flashed to a machine that others may need to identify.
# define FORK_VERSION	"+unlayered.9"
# ifdef USE_CAN0
#  define VERSION_SUFFIX	FORK_VERSION "(CAN0)"
# elif defined(NO_S_CURVE)
#  define VERSION_SUFFIX	FORK_VERSION "(no 3rd order motion)"
# else
#  define VERSION_SUFFIX	FORK_VERSION
# endif
# define VERSION MAIN_VERSION VERSION_SUFFIX
#endif

extern const char *_ecv_array const DateText;
extern const char *_ecv_array const TimeSuffix;

#define AUTHORS "reprappro, dc42, chrishamm, t3p3, dnewman, printm3d"

#endif /* SRC_VERSION_H_ */
