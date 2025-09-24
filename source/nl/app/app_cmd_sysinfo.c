#include "rt_cli.h"

static long list_thread(void)
{
    struct rt_thread *thread;
    struct rt_list_node *node;

    struct rt_object_information *info;
    
    info = rt_object_get_information(RT_Object_Class_Thread);
    if (!info) return 0;

    rt_kprintf("name pri size used\n");

    for (node = info->object_list.next; node != &(info->object_list); node = node->next)
    {
        thread = rt_list_entry(node, struct rt_thread, list);
        
        rt_uint8_t *ptr = (rt_uint8_t *)thread->stack_addr;
        while (*ptr == '#')ptr ++;
        rt_uint8_t usage = (thread->stack_size - ((rt_ubase_t)ptr - (rt_ubase_t)thread->stack_addr)) 
                          * 100 / thread->stack_size;
        
        rt_kprintf("%-4.4s %3d %4d %3d%%\n", 
                  thread->name,
                  thread->current_priority,
                  thread->stack_size,
                  usage);
    }
    
    return 0;
}

int CMD_SYSINFO(rt_cli_t cli, int argc, char **argv)
{
    rt_size_t total = 0, used = 0, max_used = 0;
    rt_uint8_t major, minor;

    void cpu_usage_get(rt_uint8_t * major, rt_uint8_t * minor);
    cpu_usage_get(&major, &minor);
    rt_kprintf("usage=%d.%d%%\n", major, minor);
    rt_kprintf("time=%d\n", rt_tick_get_millisecond());

    rt_memory_info(&total, &used, &max_used);

    rt_kprintf("mem_total=%dKB\n", total / 1024);
    rt_kprintf("max_used=%dKB\n", max_used / 1024);
    rt_kprintf("mem_free=%dKB\n", (total - used) / 1024);

    list_thread();

    return RT_EOK;
}
