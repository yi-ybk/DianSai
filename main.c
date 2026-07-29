/* For usleep() */
#include <stddef.h>
#include <stdint.h>
#include <unistd.h>

/* POSIX Header files */
#include <pthread.h>

#include "ti_msp_dl_config.h"

#include "robot.h"

void mainThread(void)
{
    /* 初始化机器人 */
    robotInit();
}
 