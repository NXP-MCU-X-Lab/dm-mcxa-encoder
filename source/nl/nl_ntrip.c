#include "nl.h"


#define TINTACT             200         /* period for stream active (ms) */
#define SERIBUFFSIZE        4096        /* serial buffer size (bytes) */
#define TIMETAGH_LEN        64          /* time tag file header length */
#define MAXCLI              32          /* max client connection for tcp svr */
#define MAXSTATMSG          32          /* max length of status message */
#define DEFAULT_MEMBUF_SIZE 4096        /* default memory buffer size (bytes) */

#define NTRIP_AGENT         "RTKLIB"
#define NTRIP_CLI_PORT      2101        /* default ntrip-client connection port */
#define NTRIP_SVR_PORT      80          /* default ntrip-server connection port */
#define NTRIP_MAXRSP        32768       /* max size of ntrip response */
#define NTRIP_MAXSTR        256         /* max length of mountpoint string */
#define NTRIP_RSP_OK_CLI    "ICY 200 OK\r\n" /* ntrip response: client */
#define NTRIP_RSP_OK_SVR    "OK\r\n"    /* ntrip response: server */
#define NTRIP_RSP_SRCTBL    "SOURCETABLE 200 OK\r\n" /* ntrip response: source table */
#define NTRIP_RSP_TBLEND    "ENDSOURCETABLE"
#define NTRIP_RSP_HTTP      "HTTP/"     /* ntrip response: http */
#define NTRIP_RSP_ERROR     "ERROR"     /* ntrip response: error */
#define NTRIP_RSP_UNAUTH    "HTTP/1.0 401 Unauthorized\r\n"
#define NTRIP_RSP_ERR_PWD   "ERROR - Bad Pasword\r\n"
#define NTRIP_RSP_ERR_MNTP  "ERROR - Bad Mountpoint\r\n"
 
 
 
typedef struct
{
    uint32_t        lenght;
    char const *    string;
} ntrip_http_string_t;


const ntrip_http_string_t ntrip_client_connect_caster_string[] = 
{
	{5, "GET /"},
	{64, " HTTP/1.0\r\nUser-Agent: NTRIP Client/1.0.0\r\nAuthorization: Basic "},
	{4, "\r\n\r\n"},
};

const ntrip_http_string_t ntrip_server_connect_caster_string[] = 
{
	{7, "SOURCE "},
	{2, " /"},
	{38, "\r\nSource-Agent: NTRIP Server/1.0.0\r\n\r\n"},
};

const ntrip_http_string_t ntrip_reply_string[] = 
{
	{20, "SOURCETABLE 200 OK\r\n"},
	{16, "ENDSOURCETABLE\r\n"},
	{12, "ICY 200 OK\r\n"},
	{22, "ERROR - Bad Password\r\n"},
	{25, "ERROR - Account Invalid\r\n"},
};




int encbase64(char *str, const uint8_t *byte, int n)
{
    const char table[]=
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    int i,j,k,b;
    

    for (i=j=0;i/8<n;) {
        for (k=b=0;k<6;k++,i++) {
            b<<=1; if (i/8<n) b|=(byte[i/8]>>(7-i%8))&0x1;
        }
        str[j++]=table[b];
    }
    while (j&0x3) str[j++]='=';
    str[j]='\0';

    return j;
}






/* create NTRIP client request */
int reqntrip_c(char *buf, char *user, char *passwd, char *mntpnt)
{
    char base64buf[128], *p=buf;
    
    
    p+=sprintf(p,"GET /%s HTTP/1.0\r\n", mntpnt);
    p+=sprintf(p,"User-Agent: NTRIP %s\r\n", NTRIP_AGENT);
    
    if (!*user) {
        p+=sprintf(p,"Accept: */*\r\n");
        p+=sprintf(p,"Connection: close\r\n");
    }
    else {
        sprintf(base64buf,"%s:%s", user, passwd);
        p+=sprintf(p,"Authorization: Basic ");
        p+=encbase64(p, (uint8_t *)base64buf, strlen(base64buf));
        p+=sprintf(p,"\r\n");
    }
    p+=sprintf(p,"\r\n");
    return p-buf;
}







