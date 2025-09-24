#include "rt_cli.h"
#include "rt_setting.h"

int CMD_PMUXCONFIG(rt_cli_t cli, int argc, char **argv)
{
    rt_setting_t app = rt_setting_find(SETTING_APP_NAME);
    int pmux[10];
    
    // Read PMUX array
    rt_param_get_int_array("PMUX", pmux, app);
    
    // Print all PMUX configurations
    for(int i = 0; i < 10; i++)
    {
        rt_kprintf("PMUX%d=%d\n", i, pmux[i]);
    }
    
    return RT_EOK;
}
