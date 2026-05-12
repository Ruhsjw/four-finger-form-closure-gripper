#ifndef PID_SUPPORT_H
#define PID_SUPPORT_H

#include "pid_types.h"

#include <stdint.h>

#ifdef  __cplusplus
extern "C" {
#endif /* __cplusplus */

pid_ticks_t pid_ticks_per_second(void);
pid_ticks_t pid_ticks_get(void);

#ifdef  __cplusplus
}
#endif /* __cplusplus */

#endif /* PID_SUPPORT_H */
