#include <finsh.h>
#include "nl.h"
#include "rtklib.h"
#include <ulog.h>




uint32_t nl_ut_ntrip(int argc, char** argv)
{

    char buf[256];
    uint32_t len;

    len = reqntrip_c(buf, "qxxrli003", "b6984cd", "AUTO");
    printf("%s\r\n", buf);
  
}

