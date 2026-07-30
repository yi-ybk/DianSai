/*
 *  ======== main_freertos.c ========
 */
#include <stdint.h>

#ifdef __ICCARM__
#include <DLib_Threads.h>
#endif
/* POSIX 头文件 */
#include <pthread.h>

/* FreeRTOS 头文件 */
#include <FreeRTOS.h>
#include <task.h>

#include "ti_msp_dl_config.h"

#include "robot.h"

/* 主线程栈大小，单位为字节 */
#define THREADSTACKSIZE configMINIMAL_STACK_SIZE * 4

/* 启动时读取一次并保存，便于调试复位原因；RSTCAUSE 寄存器读取后会清零。 */
volatile uint32_t g_resetCause;

/*
 *  ======== 主程序入口 ========
 */
int main(void)
{
    g_resetCause = (uint32_t)DL_SYSCTL_getResetCause();

    /* 初始化 IAR C 库锁 */
#ifdef __ICCARM__
    __iar_Initlocks();
#endif

    /* 初始化由 SysConfig 生成的底层硬件配置 */
    SYSCFG_DL_init();

    robotInit();

    /* 启动 FreeRTOS 调度器；成功启动后不会返回 */
    vTaskStartScheduler();

    return 0;
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook(void)
{
    /* 动态内存分配失败钩子：关闭中断后停止运行。 */
    taskDISABLE_INTERRUPTS();
    for (;;)
        ;
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook(void)
{
    /* 空闲任务钩子：此处不能执行阻塞操作。 */
}
/*-----------------------------------------------------------*/

#if (configCHECK_FOR_STACK_OVERFLOW)
/*
     *  ======== 栈溢出钩子 ========
     *  启用栈溢出检测后，若应用未自行实现钩子函数，则使用此弱定义。
     */
#if defined(__IAR_SYSTEMS_ICC__)
__weak void vApplicationStackOverflowHook(
    TaskHandle_t pxTask, char *pcTaskName)
#elif (defined(__TI_COMPILER_VERSION__))
#pragma WEAK(vApplicationStackOverflowHook)
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
#elif (defined(__GNUC__) || defined(__ti_version__))
void __attribute__((weak))
vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
#endif
{
    (void) pcTaskName;
    (void) pxTask;

    /* 检测到栈溢出后关闭中断并停止运行。 */
    taskDISABLE_INTERRUPTS();
    for (;;)
        ;
}
#endif