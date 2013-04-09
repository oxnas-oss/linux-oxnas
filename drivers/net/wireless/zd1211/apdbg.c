#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <netinet/in.h>

#include <linux/sockios.h>

#define	ZDAPIOCTL	(SIOCDEVPRIVATE)

struct zdap_ioctl
{
        unsigned short cmd;                /* Command to run */
        unsigned long addr;                /* Length of the data buffer */
        unsigned long value;               /* Pointer to the data buffer */
        unsigned char data[0x100];
};

/* Declaration of macro and function for handling WEP Keys */

#if 0

#define SKIP_ELEM { \
    while(isxdigit(*p)) \
        p++; \
}

#define SKIP_DELIMETER { \
    if(*p == ':' || *p == ' ') \
        p++; \
}

#endif

char hex(char);
unsigned char asctohex(char *str);

#define ZD_IOCTL_REG_READ			0x01
#define ZD_IOCTL_REG_WRITE			0x02
#define ZD_IOCTL_MEM_DUMP			0x03
#define ZD_IOCTL_RATE       			0x04
#define ZD_IOCTL_SNIFFER    			0x05
#define ZD_IOCTL_CAM_DUMP   			0x06
#define ZD_IOCTL_DUMP_PHY   			0x07
#define ZD_IOCTL_CARD_SETTING 			0x08
#define ZD_IOCTL_HASH_DUMP			0x09
#define ZD_IOCTL_RFD_DUMP			0x0A
#define ZD_IOCTL_MEM_READ			0x0B
#define ZD_IOCTL_MEM_WRITE			0x0C

//for STA
#define ZD_IOCTL_TX_RATE			0x0D
#define ZD_IOCTL_EEPROM				0x0E

#define ZD_IOCTL_BCN				0x10
#define ZD_IOCTL_REG_READ16			0x11
#define ZD_IOCTL_REG_WRITE16			0x12

//for CAM Test
#define ZD_IOCTL_CAM_READ			0x13
#define ZD_IOCTL_CAM_WRITE			0x14
#define ZD_IOCTL_CAM_RESET			0x15
#define ZD_IOCTL_READ_PHY			0x16
#define ZD_IOCTL_WRITE_PHY			0x17
#define ZD_IOCTL_CONT_TX            0x18
#define ZD_IOCTL_SET_MIC_CNT_ENABLE 0x19
#define ZD_IOCTL_GET_MIC_CNT_ENABLE 0x1A
#define ZD_IOCTL_DEBUG_FLAG  0x21

char *prgname;

int set_ioctl(int sock, struct ifreq *req)
{
        if (ioctl(sock, ZDAPIOCTL, req) < 0)
        {
                fprintf(stderr, "%s: ioctl(SIOCGIFMAP): %s\n",
                        prgname, strerror(errno));
                return -1;
        }

        return 0;
}


int read_reg(int sock, struct ifreq *req)
{
        struct zdap_ioctl *zdreq = 0;

        if (!set_ioctl(sock, req))
                return -1;

        //zdreq = (struct zdap_ioctl *)req->ifr_data;
        //printf( "reg = %4x, value = %4x\n", zdreq->addr, zdreq->value);

        return 0;
}


int read_mem(int sock, struct ifreq *req)
{
        struct zdap_ioctl *zdreq = 0;
        int i;

        if (!set_ioctl(sock, req))
                return -1;

        /*zdreq = (struct zdap_ioctl *)req->ifr_data;
        printf( "dump mem from %x, length = %x\n", zdreq->addr, zdreq->value);

        for (i=0; i<zdreq->value; i++) {
            printf("%02x", zdreq->data[i]);
            printf(" ");

            if ((i>0) && ((i+1)%16 == 0))
                printf("\n");
        }*/

        return 0;
}


int main(int argc, char **argv)
{
        int sock;
        int addr, value;
        struct ifreq req;
        char *action = NULL;
        struct zdap_ioctl zdreq;
        int mode = 0;

        prgname = argv[0];

        if (argc < 5) {
                printf("Usage:\n");
                printf("   %s<ifname> [<operation>] [<address>(0)] [<value>(0)]\n\n",  prgname );
                printf("\n");
                printf("valid operation: mem, phy\n");
                printf("camdump, card, rfds, rmem, wmem, rate\n");
                printf("hash, eep, rdcam, wrcam, rstcam\n\n");
                printf("##### Hardware Memory Operation ######\n");
                printf(" rdphy         Read Phy Register\n");
                printf(" wrphy         Write Phy Register\n");
                printf(" read          Read MAC Register\n");
                printf(" write         Write MAC Register\n");
                printf("##### Feature Control Operation #####\n");
                printf(" g_mic_cnt     Get If WPA counter measure enabled.\n");
                printf(" s_mic_cnt     Set WPA counter measure feature\n");
                printf(" txrate        Set Fixed Transmission Rate\n");
                printf("##### Production Operation #####\n");
                printf("tx_cont        Activate Continous TX\n");
                printf("##### For CAM function\n");
                printf("camdump	       <Addr> <Length>\n");
                printf("rdcam          <Addr>\n");
                printf("wrcam          <Addr> <Data>\n");
                exit(1);
        }

        strcpy(req.ifr_name, argv[1]);
        zdreq.addr = 0;
        zdreq.value = 0;

        /* a silly raw socket just for ioctl()ling it */
        sock = socket(AF_INET, SOCK_RAW, IPPROTO_RAW);
        if (sock < 0) {
                fprintf(stderr, "%s: socket(): %s\n", argv[0], strerror(errno));
                exit(1);
        }

        sscanf(argv[3], "%x", &addr);
        sscanf(argv[4], "%x", &value);
        zdreq.addr = addr;
        zdreq.value = value;

        if (!strcmp(argv[2], "read")) {
                zdreq.cmd = ZD_IOCTL_REG_READ;
                goto just_set;
        } else if (!strcmp(argv[2], "mem")) {
                zdreq.cmd = ZD_IOCTL_MEM_DUMP;
                goto just_set;
        } else if (!strcmp(argv[2], "write")) {
                zdreq.cmd = ZD_IOCTL_REG_WRITE;
                goto just_set;
        } else if (!strcmp(argv[2], "sniffer")) {
                zdreq.cmd = ZD_IOCTL_SNIFFER;
                goto just_set;
        } else if (!strcmp(argv[2], "camdump")) {
                zdreq.cmd = ZD_IOCTL_CAM_DUMP;
                goto just_set;
        } else if (!strcmp(argv[2], "phy")) {
                zdreq.cmd = ZD_IOCTL_DUMP_PHY;
                goto just_set;
        } else if (!strcmp(argv[2], "rdphy")) {
                zdreq.cmd = ZD_IOCTL_READ_PHY;
                goto just_set;
        } else if (!strcmp(argv[2], "wrphy")) {
                zdreq.cmd = ZD_IOCTL_WRITE_PHY;
                goto just_set;
        }
        else if (!strcmp(argv[2], "card")) {
                zdreq.cmd = ZD_IOCTL_CARD_SETTING;
                goto just_set;
        } else if (!strcmp(argv[2], "hash")) {
                zdreq.cmd = ZD_IOCTL_HASH_DUMP;
                goto just_set;
        } else if (!strcmp(argv[2], "rfd")) {
                zdreq.cmd = ZD_IOCTL_RFD_DUMP;
                goto just_set;
        } else if (!strcmp(argv[2], "rmem")) {
                zdreq.cmd = ZD_IOCTL_MEM_READ;
                goto just_set;
        } else if (!strcmp(argv[2], "wmem")) {
                zdreq.cmd = ZD_IOCTL_MEM_WRITE;
                goto just_set;
        } else if (!strcmp(argv[2], "txrate")) {
                zdreq.cmd = ZD_IOCTL_TX_RATE;
                goto just_set;
        } else if (!strcmp(argv[2], "eep")) {
                zdreq.cmd = ZD_IOCTL_EEPROM;
                goto just_set;
        } else if (!strcmp(argv[2], "rate")) {
                zdreq.cmd = ZD_IOCTL_RATE;
                goto just_set;
        } else if(!strcmp(argv[2], "cont_tx")) {
                zdreq.cmd = ZD_IOCTL_CONT_TX;
                goto just_set;
        } else if (!strcmp(argv[2], "bcn")) {
                zdreq.cmd = ZD_IOCTL_BCN;
                goto just_set;
        } else if (!strcmp(argv[2], "read16")) {
                zdreq.cmd = ZD_IOCTL_REG_READ16;
                goto just_set;
        } else if (!strcmp(argv[2], "write16")) {
                zdreq.cmd = ZD_IOCTL_REG_WRITE16;
                goto just_set;
        } else if (!strcmp(argv[2], "rdcam")) {
                mode = 1;
                zdreq.cmd = ZD_IOCTL_CAM_READ;
                goto just_set;
        } else if (!strcmp(argv[2], "wrcam")) {
                mode = 1;
                zdreq.cmd = ZD_IOCTL_CAM_WRITE;
                goto just_set;
        } else if (!strcmp(argv[2], "rstcam")) {
                zdreq.cmd = ZD_IOCTL_CAM_RESET;
                goto just_set;
        } else if (!strcmp(argv[2], "g_mic_cnt")) {
                zdreq.cmd = ZD_IOCTL_GET_MIC_CNT_ENABLE;
                goto just_set;
        } else if (!strcmp(argv[2], "s_mic_cnt")) {
                zdreq.cmd = ZD_IOCTL_SET_MIC_CNT_ENABLE;
                goto just_set;
        } else if (!strcmp(argv[2], "debugflag")) {
                zdreq.cmd = ZD_IOCTL_DEBUG_FLAG;
                goto just_set;
        } else {
                fprintf(stderr, "error action\n");
                exit(1);
        }

just_set:

        if(mode == 0)
                sscanf(argv[3], "%x", &addr);
        else
                sscanf(argv[3], "%d", &addr);

        sscanf(argv[4], "%x", &value);

        zdreq.addr = addr;
        zdreq.value = value;
        req.ifr_data = (char *)&zdreq;
        set_ioctl(sock, &req);

fail:
        exit(0);
}

unsigned char asctohex(char *str)
{
        unsigned char value;

        value = hex(*str) & 0x0f;
        value = value << 4;
        str++;
        value |= hex(*str) & 0x0f;

        return value;
}

char hex(char v)
{
        if(isdigit(v))
                return v - '0';
        else if(isxdigit(v))
                return (tolower(v) - 'a' + 10);
        else
                return 0;
}
