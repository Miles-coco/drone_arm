#include "mcu_config.h"
#include "driver.h"
#include "task.h"
#include "dm_ctrl.h"
#include "remote_control.h"

void MCU_init(void)
{
    task_init();
    remote_control_init();
    can_driver_init();
    dm4310_motor_init();

}