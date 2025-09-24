#include <rtthread.h>
#include <rtdevice.h>
#include "rt_cli.h"
#include "rt_setting.h"
#include "nl.h"

/**
 * @brief Parse coordinate string "x,y,z" into float array
 */

int SETBASELINE(rt_cli_t cli, int argc, char **argv)
{
    nl_t anta[3], antb[3], tmp[3];
    rt_setting_t app = rt_setting_find(SETTING_APP_NAME);

    if (argc < 3)
    {
        return RT_ERROR;
    }

    // Get current ANTA and ANTB coordinates
    rt_param_get_float_array("ANTA", anta, app);
    rt_param_get_float_array("ANTB", antb, app);
    
    if (!rt_strcmp("ANTA", argv[1]))
    {
        if (sscanf(argv[2], "%f,%f,%f", &tmp[0], &tmp[1], &tmp[2]) != 3)
        {
            return RT_ERROR;
        }
        
        v3copy(anta, tmp);
    }
    else if (!rt_strcmp("ANTB", argv[1]))
    {
        if (sscanf(argv[2], "%f,%f,%f", &tmp[0], &tmp[1], &tmp[2]) != 3)
        {
            return RT_ERROR;
        }
        
        v3copy(antb, tmp);
    }
    else
    {
        return RT_ERROR;
    }

    // Save updated coordinates
    rt_param_set_float_array("ANTA", anta, 3, app);
    rt_param_set_float_array("ANTB", antb, 3, app);
    
    rt_setting_save(app);
    return RT_EOK;
}

