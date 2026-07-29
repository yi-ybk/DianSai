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

#define ELECTROMAGNET_MAIN_TEST_ENABLED (1)
#define NRF24_STAGE1_TEST_ENABLED       (0)
#define HC12_MAIN_TX_TEST_ENABLED       (0)

#if ELECTROMAGNET_MAIN_TEST_ENABLED
#include "electromagnet.h"
#define ELECTROMAGNET_STARTUP_DELAY_CYCLES (80000000U)
#elif HC12_MAIN_TX_TEST_ENABLED
#include "hc12_uart.h"
#elif NRF24_STAGE1_TEST_ENABLED
#include "nrf24l01_test.h"
#else
#include "robot.h"
#endif

/* 主线程栈大小，单位为字节 */
#define THREADSTACKSIZE configMINIMAL_STACK_SIZE * 4

/* 启动时读取一次并保存，便于调试复位原因；RSTCAUSE 寄存器读取后会清零。 */
volatile uint32_t g_resetCause;

/*
 *  ======== 主程序入口 ========
 */
int main(void)
{
#if HC12_MAIN_TX_TEST_ENABLED
    static const uint8_t test_data[] =
        "HC12 TEST: ABCDEFGHIJKLMNOPQRSTUVWXYZ 0123456789\r\n";
#endif

    g_resetCause = (uint32_t)DL_SYSCTL_getResetCause();

#ifdef __ICCARM__
    __iar_Initlocks();
#endif

    /* 初始化 SysConfig 时钟、引脚及外设；
     * 电磁铁 PB1 在使能输出前会预置为低电平。 */
    SYSCFG_DL_init();

#if ELECTROMAGNET_MAIN_TEST_ENABLED

    /*
     * PB1 高电平持续吸合测试。
     * 上电后先保持关闭 1 秒，然后持续吸合，直到断电或复位。
     */
    ElectromagnetInit();
    delay_cycles(ELECTROMAGNET_STARTUP_DELAY_CYCLES);
    ElectromagnetOn();

    for (;;)
        __NOP();

#elif HC12_MAIN_TX_TEST_ENABLED

    /*
     * 初始化 HC-12 软件驱动、接收缓冲区和 UART3 NVIC 中断。
     * 此处未启动 FreeRTOS，所以使用 Hc12Init()，
     * 不使用 Hc12RtosInit()。
     */
    Hc12Init();

    /* 等待 HC-12 上电稳定，80 MHz 下约为 200 ms */
    delay_cycles(16000000U);

    for (;;)
    {
        if (!Hc12Write(test_data, sizeof(test_data) - 1U))
        {
            /* 初始化或发送发生异常 */
            for (;;)
                __NOP();
        }

        /* 80 MHz 下约为 100 ms */
        delay_cycles(8000000U);
    }

#elif NRF24_STAGE1_TEST_ENABLED

    Nrf24Stage1TestRun();

    for (;;)
        __NOP();

#else

    robotInit();
    vTaskStartScheduler();

#endif

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
