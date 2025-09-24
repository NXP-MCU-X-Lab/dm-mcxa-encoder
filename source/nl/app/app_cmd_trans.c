#include <rtthread.h>
#include <rtdevice.h>
#include "rt_cli.h"
#include "rt_lsvr.h"

/*
 * TRANS command: establish bidirectional transparent transmission between two LSVRs
 * Usage:
 *   TRANS COM1 COM2   -> enable transparent forwarding between COM1 and COM2
 *   TRANS OFF/STOP    -> disable current transparent forwarding
 *
 * Note:
 *   Arguments are LSVR names (e.g., "COM1", "COM2").
 *   This implementation directly uses rt_lsvr_find; no rt_vcom dependency.
 */

static rt_lsock_t trans_sock_a = RT_NULL;
static rt_lsock_t trans_sock_b = RT_NULL;

static void trans_forward_cb(rt_lsock_t lsock, rt_size_t size, void *parameter)
{
    rt_lsock_t target_sock = (rt_lsock_t)parameter;
    if (!lsock || !target_sock) return;

    static uint8_t buf[1500];
    
    while (size)
    {
        rt_size_t to_read = (size > sizeof(buf)) ? sizeof(buf) : size;
        rt_size_t read_len = rt_lsock_read(lsock, buf, to_read);
        if (!read_len) break;
        
        rt_lsock_write(target_sock, buf, read_len);
        size -= read_len;
    }
}

int TRANS(rt_cli_t cli, int argc, char **argv)
{
    rt_lsvr_t l1 = RT_NULL;
    rt_lsvr_t l2 = RT_NULL;
    
    if (argc == 3)
    {
        l1 = rt_lsvr_find(argv[1]);
        l2 = rt_lsvr_find(argv[2]);

        if (!l1 || !l2)
        {
            return RT_ERROR;
        }

        /* Clear any existing transparent forwarding */
        if (trans_sock_a) { rt_lsock_delete(trans_sock_a); trans_sock_a = RT_NULL; }
        if (trans_sock_b) { rt_lsock_delete(trans_sock_b); trans_sock_b = RT_NULL; }

        trans_sock_a = rt_lsock_create("TRANS_A", 1500, l1);
        trans_sock_b = rt_lsock_create("TRANS_B", 1500, l2);

        if (!trans_sock_a || !trans_sock_b)
        {
            if (trans_sock_a) { rt_lsock_delete(trans_sock_a); trans_sock_a = RT_NULL; }
            if (trans_sock_b) { rt_lsock_delete(trans_sock_b); trans_sock_b = RT_NULL; }
            return RT_ERROR;
        }

        rt_lsock_set_rx_indicate(trans_sock_a, trans_forward_cb, trans_sock_b);
        rt_lsock_set_rx_indicate(trans_sock_b, trans_forward_cb, trans_sock_a);

        rt_kprintf("TRANS=%s<->%s\n", argv[1], argv[2]);
        
        rt_thread_mdelay(300);
        
        /* Disable all other socks on both LSVRs, keep TRANS socks enabled */
        rt_lsvr_all_socks_enable(l1, 0);
        rt_lsvr_all_socks_enable(l2, 0);
        rt_lsock_enable(trans_sock_a, 1);
        rt_lsock_enable(trans_sock_b, 1);

        return RT_EOK;
    }
    else if (argc >= 2 && !rt_strcmp(argv[1], "STOP"))
    {
        if (trans_sock_a) { rt_lsock_delete(trans_sock_a); trans_sock_a = RT_NULL; }
        if (trans_sock_b) { rt_lsock_delete(trans_sock_b); trans_sock_b = RT_NULL; }
        /* Re-enable all socks on both LSVRs */

        rt_kprintf("TRANS=OFF\n");
        return RT_EOK;
    }

    return RT_ERROR;
}

