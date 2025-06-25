#include "mcu_config.h"
#include "driver.h"
#include "task.h"
#include "dm_ctrl.h"
#include "remote_control.h"
#include "arm_control.h"

void MCU_Init(void)
{
    task_init();
    remote_control_init();
    can_driver_init();
    ArmControl_Init();
}