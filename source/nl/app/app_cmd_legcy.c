#include "app_def.h"
#include "hipnuc_reg.h"

int LOG(rt_cli_t cli, int argc, char **argv);
int FCAL(rt_cli_t cli, int argc, char **argv);
int CONFIG(rt_cli_t cli, int argc, char **argv);

/* AT command support */
static int atinfo(rt_cli_t cli, int argc, char * argv[])
{
    rt_setting_t app = rt_setting_find(SETTING_APP_NAME);

    rt_kprintf("\r\n%s %d.%d.%d build %s\r\n", rt_param_get_string("PNAME", app), APP_VERSION / 100, (APP_VERSION % 100)/10, APP_VERSION % 10, __DATE__);
    rt_kprintf("%-16s%d\r\n",        "PROFILE:", (rt_param_get_int("PROFILE", app)));
    
    uint32_t uuid[2];
    get_cpu_uuid(uuid);
    
    rt_kprintf("%-16s%08X%08X\r\n",     "UUID:", uuid[0], uuid[1]);
    return RT_EOK;
}


static int ateout(rt_cli_t cli, int argc, char * argv[])
{
    int val;
    
    sscanf(argv[0], "AT+EOUT=%d", &val);
    
    struct rt_eobj_information *vcom_info = rt_eobj_get_information(RT_EPIC_Class_VCOM);
    if (!vcom_info) return RT_ERROR;
    
    rt_slist_t *node;
    rt_slist_for_each(node, &(vcom_info->list))
    {
        rt_vcom_t vcom = (rt_vcom_t)rt_slist_entry(node, struct rt_eobj, list);
        if(vcom->lsvr == cli->sock->host)
        {
            rt_vcom_enable(vcom, val);
            return RT_EOK;
        }
    }
    
    return RT_EOK;
}

static int atrst(rt_cli_t cli, int argc, char *argv[])
{
    hipnuc_wr_reg(HIREG_ADDR_CTRL, HIREG_CTRL_REBOOT);
    return RT_EOK;
}

static int atfcall(rt_cli_t cli, int argc, char **argv)
{
    int rotation_deg;
    char buf[16];

    char *cmds0[] = {"LOG", "DISABLE"};
    LOG(cli, ARRAY_SIZE(cmds0), cmds0);

    char *cmds1[] = {"FCAL", "FCA"};
    FCAL(cli, ARRAY_SIZE(cmds1), cmds1);

    char *cmds2[] = {"FCAL", "FCG", buf};
    sscanf(argv[0], "AT+FCALL=%d", &rotation_deg);
    sprintf(buf, "%d", rotation_deg);

    FCAL(cli, ARRAY_SIZE(cmds2), cmds2);
    return RT_EOK;
}


void app_cli_cmd_legcy_init()
{
    // Command table for easier maintenance
    static const struct {
        const char *name;
        int (*handler)(rt_cli_t cli, int argc, char **argv);
    } cmd_table[] = {
        {"AT+INFO", atinfo},
        {"AT+RST", atrst},
        {"AT+EOUT=", ateout},
        {"AT+FCALL=", atfcall},
    };
    
    struct rt_eobj_information *cli_info = rt_eobj_get_information(RT_EPIC_Class_CLI);
    if (!cli_info) return;
    
    rt_slist_t *node;
    rt_slist_for_each(node, &(cli_info->list))
    {
        rt_cli_t cli = (rt_cli_t)rt_slist_entry(node, struct rt_eobj, list);
        
        // Register all commands for each CLI
        for (int i = 0; i < ARRAY_SIZE(cmd_table); i++) {
            rt_cli_cmd_create(cmd_table[i].name, cmd_table[i].handler, cli);
        }
    }
}
