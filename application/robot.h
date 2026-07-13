#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

void robotInit(void);
void ImuParseTask(void *argument);

void TestTask(void *argument);