#include <rtdevice.h>
#include "drv_common.h"
#include "rt_vcom.h"
#include "rt_cli.h"
#include "rt_setting.h"
#include "hipnuc_reg.h"
#include "bl_api.h"


int REBOOT(rt_cli_t cli, int argc, char **argv)
{
    if(argc < 2)
    {
        hipnuc_wr_reg(HIREG_ADDR_CTRL, HIREG_CTRL_REBOOT);
    }
    
    if(argc == 2 && !rt_strcmp("BL", argv[1]))
    {
        hipnuc_wr_reg(HIREG_ADDR_CTRL, HIREG_CTRL_ENTER_BL);
    }
    
    return RT_EOK;
}

int SAVECONFIG(rt_cli_t cli, int argc, char **argv)
{
    hipnuc_wr_reg(HIREG_ADDR_CTRL, HIREG_CTRL_SAVECONFIG);
    return RT_EOK;
}

/* Extract COM index from device name - support 0-99 */
static int extract_com_index(const char *name)
{
    if (!name) return -1;
    
    int len = rt_strlen(name);
    int num = 0;
    int found_digit = 0;
    
    // Scan from end to find trailing digits
    for (int i = len - 1; i >= 0; i--) {
        if (name[i] >= '0' && name[i] <= '9') {
            found_digit = 1;
            num = (name[i] - '0') + num * 10;
        } else if (found_digit) {
            break;  // Hit non-digit after finding digits
        }
    }
    
    return (found_digit && num <= 99) ? num : -1;
}

int UNLOGALL(rt_cli_t cli, int argc, char **argv)
{
    rt_setting_t app = rt_setting_find(SETTING_APP_NAME);
    
    // Find VCOM associated with current CLI
    rt_lsvr_t current_lsvr = cli->sock->host;
    struct rt_eobj_information *vcom_info = rt_eobj_get_information(RT_EPIC_Class_VCOM);
    if (!vcom_info) return RT_ERROR;
    
    rt_slist_t *node;
    rt_slist_for_each(node, &(vcom_info->list)) {
        rt_vcom_t vcom = (rt_vcom_t)rt_slist_entry(node, struct rt_eobj, list);
        if (vcom->lsvr == current_lsvr) {
            // Extract COM index from VCOM name
            int com_idx = extract_com_index(vcom->parent.name);
            if (com_idx >= 0 && com_idx <= 99) {
                char param_pattern[16];
                rt_snprintf(param_pattern, sizeof(param_pattern), "CM%d/*", com_idx);
                rt_param_set_int(param_pattern, 0, app);
                rt_vcom_set_time(vcom, 0);
                return RT_EOK;
            }
            break;
        }
    }
    
    return RT_ERROR;
}

int CMD_VERSION(rt_cli_t cli, int argc, char **argv)
{
    rt_setting_t app = rt_setting_find(SETTING_APP_NAME);
    uint32_t uuid[2];
    
    get_cpu_uuid(uuid);

    rt_kprintf("PNAME=%s\n", rt_param_get_string("PNAME", app));
    rt_kprintf("BUILD=%s\n", __DATE__);
    rt_kprintf("UUID=%08X%08X\n", uuid[0], uuid[1]);
    rt_kprintf("APP_VER=%d\n", APP_VERSION);
    rt_kprintf("BL_VER=%d\n", bl_get_version());
    
    return RT_EOK;
}

int SERIALCONFIG(rt_cli_t cli, int argc, char **argv)
{
    int baud;
    rt_vcom_t vcom = RT_NULL;
    int arg_offset = 1;
    
    // Check if first argument is a COM specification
    if (argc >= 2) {
        rt_vcom_t test_vcom = rt_vcom_find(argv[1]);
        if (test_vcom) {
            vcom = test_vcom;
            arg_offset = 2;
        }
    }
    
    // If no VCOM specified, find VCOM associated with current CLI
    if (!vcom) {
        rt_lsvr_t current_lsvr = cli->sock->host;
        struct rt_eobj_information *vcom_info = rt_eobj_get_information(RT_EPIC_Class_VCOM);
        if (vcom_info) {
            rt_slist_t *node;
            rt_slist_for_each(node, &(vcom_info->list)) {
                rt_vcom_t test_vcom = (rt_vcom_t)rt_slist_entry(node, struct rt_eobj, list);
                if (test_vcom->lsvr == current_lsvr) {
                    vcom = test_vcom;
                    break;
                }
            }
        }
        arg_offset = 1;
    }
    
    if (!vcom) return RT_ERROR;
    
    // Parse baud rate
    if (argc < arg_offset + 1 || sscanf(argv[arg_offset], "%d", &baud) != 1) {
        return RT_ERROR;
    }
    
    // Extract COM index dynamically
    int com_idx = extract_com_index(vcom->parent.name);
    if (com_idx < 0 || com_idx > 99) {
        rt_kprintf("ERROR: Unsupported VCOM\n");
        return RT_ERROR;
    }
    
    rt_setting_t app = rt_setting_find(SETTING_APP_NAME);
    
    // Set parameter and save
    char param_name[16];
    rt_snprintf(param_name, sizeof(param_name), "CM%d_BAUD", com_idx);
    rt_param_set_int(param_name, baud, app);
    rt_setting_save(app);
    
    // Rest of the function remains the same...
    rt_lsvr_clear(vcom->lsvr);
    rt_kprintf("OK\r\n");
    rt_thread_mdelay(5);
    
    struct rt_device *device = vcom->lsvr->backend_dev;
    struct rt_serial_device *serial = (struct rt_serial_device *)device;
    serial->config.baud_rate = baud;
    rt_device_control(device, RT_DEVICE_CTRL_CONFIG, &serial->config);
    
    return RT_EFULL;
}

int CMD_COMCONFIG(rt_cli_t cli, int argc, char **argv)
{
    rt_setting_t app = rt_setting_find(SETTING_APP_NAME);
    rt_vcom_t vcom = RT_NULL;
    int com_idx = -1;
    
    // Expected format: COM
    if(argc < 1)
    {
        return RT_ERROR;
    }
    
    // Find VCOM by name
    vcom = rt_vcom_find(argv[0]);
    if(!vcom)
    {
        rt_kprintf("ERROR: No VCOM available\n");
        return RT_ERROR;
    }
    
    // Extract COM index from name
    char *name = vcom->parent.name;
    int len = rt_strlen(name);
    
    // Find last digit
    if (len > 0 && name[len-1] >= '0' && name[len-1] <= '9') {
        com_idx = name[len-1] - '0';
        
        // Check for two digits
        if (len > 1 && name[len-2] >= '0' && name[len-2] <= '9') {
            com_idx = (name[len-2] - '0') * 10 + com_idx;
        }
    }
    
    // Print COM configuration
    rt_kprintf("COM=%s\n", vcom->parent.name);

    char baud_param[16];
    rt_snprintf(baud_param, sizeof(baud_param), "CM%d_BAUD", com_idx);
    rt_kprintf("BAUD=%d\n", rt_param_get_int(baud_param, app));
    
    // List all messages
    rt_slist_t *node_child;
    rt_slist_for_each(node_child, &(vcom->list))
    {
        rt_vmsg_t vmsg = (rt_vmsg_t)rt_slist_entry(node_child, struct rt_eobj, list);
        
        if(rt_vmsg_get_type(vmsg) == RT_VMSG_TYPE_PERIODIC)
        {
            rt_kprintf("MSG=%s, ONTIME=%d\n", vmsg->parent.name, vmsg->interval_ms);
        }
            
        if(rt_vmsg_get_type(vmsg) == RT_VMSG_TYPE_ONE_SHOT)
        {
            rt_kprintf("MSG=%s, ONMARK\n", vmsg->parent.name);
        }
    }
    
    return RT_EOK;
}

int CMD_MSG_CONFIG(rt_cli_t cli, int argc, char **argv)
{
    int com_idx = -1, ret = RT_ERROR, on_time_ms;
    rt_vcom_t vcom = RT_NULL;
    rt_setting_t app = rt_setting_find(SETTING_APP_NAME);
    
    // Expected format: COM MSG_NAME ONTIME/ONMARK VALUE
    if(argc < 4)
    {
        return RT_ERROR;
    }

    // Find VCOM by name (first parameter is always COM)
    vcom = rt_vcom_find(argv[0]);
    if(!vcom)
    {
        return RT_ERROR;
    }
    
    // Extract COM index from name
    char *name = vcom->parent.name;
    int len = rt_strlen(name);
    
    // Find last digit
    if (len > 0 && name[len-1] >= '0' && name[len-1] <= '9') {
        com_idx = name[len-1] - '0';
        
        // Check for two digits
        if (len > 1 && name[len-2] >= '0' && name[len-2] <= '9') {
            com_idx = (name[len-2] - '0') * 10 + com_idx;
        }
    }

    // Find message
    rt_vmsg_t msg = rt_vmsg_find(argv[1], vcom);
    
    if(msg)
    {
        // Parse time value
        on_time_ms = strtof(argv[3], 0) * 1000;
        
        if(!rt_strcmp("ONTIME", argv[2]))
        {
            rt_vmsg_set_time(msg, on_time_ms);
            rt_vmsg_set_type(msg, RT_VMSG_TYPE_PERIODIC);
            on_time_ms &= ~0x80000000;
        }
        else if(!rt_strcmp("ONMARK", argv[2]))
        {
            rt_vmsg_set_type(msg, RT_VMSG_TYPE_ONE_SHOT);
            on_time_ms |= 0x80000000;
            
            if(!rt_strcmp("ONCE", argv[3]))
            {
                rt_vcom_output_once(vcom);
                return RT_EEMPTY;
            }
        }
        else
        {
            // Invalid configuration type
            return RT_ERROR;
        }

        // Save parameter
        char param_path[32];
        rt_snprintf(param_path, sizeof(param_path), "CM%d/%s", com_idx, msg->parent.name);
        rt_param_set_int(param_path, on_time_ms, app);
        
        ret = RT_EOK;
    }

    return ret;
}
