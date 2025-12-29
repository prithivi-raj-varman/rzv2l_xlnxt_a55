/**
 * @file    main.c
 * @brief   Main function and RPMSG example application.
 * @date    2020.10.27
 * @author  Copyright (c) 2020, eForce Co., Ltd. All rights reserved.
 * @license SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************
 * @par     History
 *          - rev 1.0 (2019.10.23) nozaki
 *            Initial version.
 *          - rev 1.1 (2020.01.28) Imada
 *            Modification for OpenAMP 2018.10.
 *          - rev 1.2 (2020.10.27) Imada
 *            Added the license description.
 ****************************************************************************
 */
#include <atomic>
#include "cancomm/CanComm.h"
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include "metal/alloc.h"
#include "metal/utilities.h"
#include "openamp/open_amp.h"
#include "platform_info.h"
#include "rsc_table.h"

#define SHUTDOWN_MSG    (0xEF56A55A)

#ifndef max
#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     (_a > _b) ? _a : _b; })
#endif

#ifndef ARRAY_SIZE
    #define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))
#endif

extern "C" {
    int init_system(void);
    void cleanup_system(void);
}

/* Payload definition */
// struct _payload {
//     unsigned long num;
//     unsigned long size;
//     unsigned char data[];
// };

#pragma pack(push, 1)
struct rpmsg_can_pkt {
    uint32_t can_id;
    uint8_t  dlc;
    uint8_t  data[8];
};
#pragma pack(pop)


/* Payload information */
struct payload_info {
    int minnum;
    int maxnum;
    int num;
};

/* Internal functions */
static void rpmsg_service_bind(struct rpmsg_device *rdev, const char *name, uint32_t dest);
static void rpmsg_service_unbind(struct rpmsg_endpoint *ept);
static int rpmsg_service_cb0(struct rpmsg_endpoint *rp_ept, void *data, size_t len, uint32_t src, void *priv);
static int payload_init(struct rpmsg_device *rdev, struct payload_info *pi);
static void register_handler(int signum, void(* handler)(int));
static void stop_handler(int signum);
static void init_cond(void);
static void show_menu(int argc);
static int wait_input(int argc, char *argv[]);
static void launch_communicate(int pattern);
static void *communicate(void* arg);

/* Globals */
int sockfd = -1;
bool response = false;
std::atomic<bool> runCanThread(true);
CanComm can;
static struct can_frame send_frame;   /* Single frame buffer for sending */
struct rpmsg_can_pkt tx_pkt;
static struct can_frame recv_frame;   /* Buffer for received echoed frame */

static struct rpmsg_endpoint rp_ept = { 0 };
static struct _payload *i_payload;
static int rnum = 0;
static int err_cnt = 0;
static char *svc_name = NULL;
int force_stop = 0;
pthread_cond_t cond;
pthread_mutex_t mutex, rsc_mutex;

struct comm_arg ids[] = {
    {NULL, 0},
    {NULL, 1},
};

/* External functions */
extern int init_system(void);
extern void cleanup_system(void);

#if 0
/* Application entry point */
static int app (struct rpmsg_device *rdev, struct remoteproc *priv, unsigned long svcno)
{
    int ret = 0;
    int shutdown_msg = SHUTDOWN_MSG;
    int expect_rnum = 0;
    int i;
    static int sighandled = 0;

    LPRINTF("RPMsg CAN frame echo test started.\n");
    LPRINTF("Sending classic CAN frames to remote core, expecting echo back.\n");

    /* Prepare a sample CAN frame (you can modify this or add logic to send multiple/different frames) */
    // send_frame.can_id  = 0x123;      /* Example standard CAN ID */
    // send_frame.can_dlc = 8;          /* Full 8 bytes data */
    // memset(send_frame.data, 0xA5, 8); /* Fill data with pattern 0xA5 for easy integrity check */

    tx_pkt.can_id = 0x123;
    tx_pkt.dlc = 8;
    memset(tx_pkt.data, 0xA5, 8);


    /* Create RPMsg endpoint */
    if (svcno == 0) {
        svc_name = (const char *)CFG_RPMSG_SVC_NAME0;
    } else {
        svc_name = (const char *)CFG_RPMSG_SVC_NAME1;
    }
    
    pthread_mutex_lock(&rsc_mutex);
    ret = rpmsg_create_ept(&rp_ept, rdev, svc_name, APP_EPT_ADDR,
                   RPMSG_ADDR_ANY,
                   rpmsg_service_cb0, rpmsg_service_unbind);
    pthread_mutex_unlock(&rsc_mutex);
    if (ret) {
        LPERROR("Failed to create RPMsg endpoint.");
        return ret;
    }
    LPRINTF("RPMSG endpoint created: %p\n", &rp_ept);

    if (!sighandled) {
        sighandled = 1;
        register_handler(SIGINT, stop_handler);
        register_handler(SIGTERM, stop_handler);
    }

    while (!force_stop && !is_rpmsg_ept_ready(&rp_ept))
        platform_poll(priv);

    if (force_stop) {
        LPRINTF("\nForce stopped.\n");
        goto error;
    }

    LPRINTF("RPMSG channel ready. Starting CAN frame transmission loop.\n");

    /* Example: send 100 frames continuously (or adjust as needed) */
    for (i = 0; i < 100 && !force_stop; i++) {
        /* Optionally modify frame for each iteration, e.g. increment ID or change data */
        send_frame.can_id = 0x123 + i;  /* Vary ID for testing */

        // LPRINTF("Sending CAN frame: ID=0x%03lX, DLC=%d\n",
        //         send_frame.can_id & 0x7FF, send_frame.can_dlc);
        LPRINTF("Sending CAN frame: ID=0x%03lX, DLC=%d, Data: "
        "%02X %02X %02X %02X %02X %02X %02X %02X\n",
        send_frame.can_id & 0x7FF, send_frame.can_dlc,
        send_frame.data[0], send_frame.data[1], send_frame.data[2], send_frame.data[3],
        send_frame.data[4], send_frame.data[5], send_frame.data[6], send_frame.data[7]);

        ret = rpmsg_send(&rp_ept, &send_frame, sizeof(struct can_frame));
        if (ret < 0) {
            LPERROR("Failed to send frame: %d\n", ret);
            break;
        }

        expect_rnum++;

        /* Wait for echo response */
        while (!force_stop && (rnum < expect_rnum) && !err_cnt) {
            platform_poll(priv);
        }

        if (err_cnt) {
            LPERROR("Echo validation failed!\n");
        }

        usleep(10000);  /* Small delay between frames */
    }

    LPRINTF("************************************");
    LPRINTF(" Test Results: Error count = %d ", err_cnt);
    LPRINTF("************************************");

error:
    /* Send shutdown message to remote */
    rpmsg_send(&rp_ept, &shutdown_msg, sizeof(int));
    sleep(1);
    LPRINTF("Quitting application - CAN echo test end\n");

    return 0;
}
#endif

static int app(struct rpmsg_device *rdev,
               struct remoteproc *priv,
               unsigned long svcno)
{
    int ret;
    int i;
    static int sighandled = 0;
    uint32_t shutdown_msg = SHUTDOWN_MSG;

    struct rpmsg_can_pkt tx_pkt;

    LPRINTF("RPMsg CAN test started\n");

    /* Select service name based on channel */
    if (svcno == 0)
        svc_name = (char *)CFG_RPMSG_SVC_NAME0;
    else
        svc_name = (char *)CFG_RPMSG_SVC_NAME1;

    /* Create RPMsg endpoint */
    pthread_mutex_lock(&rsc_mutex);
    ret = rpmsg_create_ept(&rp_ept,
                           rdev,
                           svc_name,
                           APP_EPT_ADDR,
                           RPMSG_ADDR_ANY,
                           rpmsg_service_cb0,
                           rpmsg_service_unbind);
    pthread_mutex_unlock(&rsc_mutex);

    if (ret) {
        LPERROR("Failed to create RPMsg endpoint\n");
        return ret;
    }

    LPRINTF("RPMsg endpoint created\n");

    /* Register signal handlers once */
    if (!sighandled) {
        sighandled = 1;
        register_handler(SIGINT, stop_handler);
        register_handler(SIGTERM, stop_handler);
    }

    /* Wait until endpoint is ready */
    while (!force_stop && !is_rpmsg_ept_ready(&rp_ept))
        platform_poll(priv);

    if (force_stop) {
        LPRINTF("Force stop before start\n");
        return 0;
    }

    LPRINTF("RPMsg channel ready, starting transmission\n");

    /* Initialize TX packet */
    memset(&tx_pkt, 0, sizeof(tx_pkt));
    tx_pkt.dlc = 8;
    memset(tx_pkt.data, 0xA5, 8);

    /* Send 100 CAN packets */
    for (i = 0; i < 100 && !force_stop; i++) {

        tx_pkt.can_id = 0x123 + i;

        LPRINTF("TX CAN: ID=0x%03X DLC=%d Data=%02X %02X %02X %02X %02X %02X %02X %02X\n",
                tx_pkt.can_id,
                tx_pkt.dlc,
                tx_pkt.data[0], tx_pkt.data[1],
                tx_pkt.data[2], tx_pkt.data[3],
                tx_pkt.data[4], tx_pkt.data[5],
                tx_pkt.data[6], tx_pkt.data[7]);

        ret = rpmsg_send(&rp_ept,
                         &tx_pkt,
                         sizeof(struct rpmsg_can_pkt));
        if (ret < 0) {
            LPERROR("rpmsg_send failed: %d\n", ret);
            break;
        }

        /* Allow RPMsg RX processing */
        platform_poll(priv);
        usleep(10000);
    }

    LPRINTF("Transmission finished: RX count=%d errors=%d\n",
            rnum, err_cnt);

    /* Notify remote core to shutdown */
    rpmsg_send(&rp_ept, &shutdown_msg, sizeof(shutdown_msg));
    sleep(1);

    LPRINTF("RPMsg CAN test end\n");
    return 0;
}


static void rpmsg_service_bind(struct rpmsg_device *rdev, const char *name, uint32_t dest)
{
    LPRINTF("new endpoint notification is received.");
    if (strcmp(name, svc_name)) {
        LPERROR("Unexpected name service %s.", name);
    }
    else
        (void)rpmsg_create_ept(&rp_ept, rdev, svc_name,
                       APP_EPT_ADDR, dest,
                       rpmsg_service_cb0,
                       rpmsg_service_unbind);
    return ;
}

static void rpmsg_service_unbind(struct rpmsg_endpoint *ept)
{
    (void)ept;
    /* service 0 */
    rpmsg_destroy_ept(&rp_ept);
    memset(&rp_ept, 0x0, sizeof(struct rpmsg_endpoint));
    return ;
}

#if 0
static int rpmsg_service_cb0(struct rpmsg_endpoint *ept,
                             void *data,
                             size_t len,
                             uint32_t src,
                             void *priv)
{
    (void)ept;
    (void)src;
    (void)priv;

    if (len != sizeof(struct rpmsg_can_pkt)) {
        LPRINTF("RX unexpected length: %zu\n", len);
        err_cnt++;
        return RPMSG_SUCCESS;
    }

    struct rpmsg_can_pkt *rx = (struct rpmsg_can_pkt *)data;

    LPRINTF("RX CAN: ID=0x%03X DLC=%d\n",
            rx->can_id, rx->dlc);

    rnum++;
    return RPMSG_SUCCESS;
}
#endif
//struct rpmsg_can_pkt *rx = (struct rpmsg_can_pkt *)data;

static int rpmsg_service_cb0(struct rpmsg_endpoint *ept,
                             void *data,
                             size_t len,
                             uint32_t src,
                             void *priv)
{
    (void)ept;
    (void)src;
    (void)priv;

    if (len != sizeof(struct rpmsg_can_pkt)) {
        LPRINTF("RX unexpected length: %zu\n", len);
        return RPMSG_SUCCESS;
    }

    struct rpmsg_can_pkt *rx = (struct rpmsg_can_pkt *)data;

    /* ---- Map RPMsg packet to CAN frame ---- */
    CanComm::Command cmd;

    cmd.nodeID  = (rx->can_id >> NODE_SHIFTER) & 0xFF;
    cmd.command = rx->can_id & 0x3F;

    /* Convert data bytes to uint64 (same way SendCanMessage expects) */
    uint64_t payload = 0;
    for (int i = 0; i < rx->dlc; i++) {
        payload |= ((uint64_t)rx->data[i] << (i * 8));
    }
    cmd.data = payload;

    /* ---- Send to CAN bus ---- */
    can.SendCanMessage(cmd);

    LPRINTF("RPMSG->CAN: ID=0x%03X DLC=%d\n",
            rx->can_id, rx->dlc);

    return RPMSG_SUCCESS;
}


static int payload_init(struct rpmsg_device *rdev, struct payload_info *pi) {
    int rpmsg_buf_size = 0;

    /* Get the maximum buffer size of a rpmsg packet */
    if ((rpmsg_buf_size = rpmsg_virtio_get_buffer_size(rdev)) <= 0) {
        return rpmsg_buf_size;
    }

    pi->minnum = 1;
    pi->maxnum = rpmsg_buf_size - 24;
    pi->num = pi->maxnum / pi->minnum;

    i_payload =
        (struct _payload *)metal_allocate_memory(2 * sizeof(unsigned long) +
                      pi->maxnum);
    if (!i_payload) {
        LPERROR("memory allocation failed.");
        return -ENOMEM;
    }

    return 0;
}

static void init_cond(void)
{
#ifdef __linux__
    pthread_mutex_init(&mutex, NULL);
    pthread_mutex_init(&rsc_mutex, NULL);
    pthread_cond_init(&cond, NULL);
#endif
}

void CanWorkerThread()
{
    if (can.init() != 0) {
        std::cerr << "[CAN] Initialization failed!" << std::endl;
        return;
    }

    while (runCanThread) {
        can.ReceiveCanMessage();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

int main(int argc, char *argv[])
{
    int pattern;
    unsigned long proc_id;
    unsigned long rsc_id;
    int i;
    int ret = 0;

    /* Initialize HW system components */
    init_system();
    init_cond();
    std::thread canThread(CanWorkerThread);
    /* Initialize platform */
    
    for (i = 0; i < ARRAY_SIZE(ids); i++) {
        proc_id = rsc_id = ids[i].channel;
    
        ret = platform_init(proc_id, rsc_id, &ids[i].platform);
        if (ret) {
            LPERROR("Failed to initialize platform.");
            ret = 1;
            goto error_return;
        }
    }

    while (!force_stop) {
        show_menu(argc);
        pattern = wait_input(argc, argv);

        if (!pattern) break;

        launch_communicate(pattern);

        if (argc >= 2) break;
    }

    for (i = 0; i < ARRAY_SIZE(ids); i++) {
        platform_cleanup(ids[i].platform);
        ids[i].platform = NULL;
    }
    cleanup_system();

error_return:
    return ret;
}

/**
 * @fn communicate
 * @brief perform test communication
 * @param arg - test conditions
 */
static void *communicate(void* arg) {
    struct comm_arg *p = (struct comm_arg*)arg;
    struct rpmsg_device *rpdev;
    unsigned long proc_id = p->channel;

    LPRINTF("thread start ");

    pthread_mutex_lock(&rsc_mutex);
    rpdev = platform_create_rpmsg_vdev(p->platform, 0,
                      VIRTIO_DEV_MASTER,
                      NULL,
                      rpmsg_service_bind);
    pthread_mutex_unlock(&rsc_mutex);
    if (!rpdev) {
        LPERROR("Failed to create rpmsg virtio device.");
    } else {
        (void)app(rpdev, p->platform, proc_id);
        platform_release_rpmsg_vdev(p->platform, rpdev);
    }
    LPRINTF("Stopping application...");

    return NULL;
}

/**
 * @fn launch_communicate
 * @brief Launch test threads according to test patterns
 */
static void launch_communicate(int pattern)
{
    pthread_t th = 0;

    pattern--;
    if ((pattern < 0) || (ARRAY_SIZE(ids) <= max(0, pattern - 1))) return;

    pthread_create(&th, NULL, communicate, &ids[pattern]);

    if (th) pthread_join(th, NULL);
}

static void register_handler(int signum, void(* handler)(int)) {
    if (signal(signum, handler) == SIG_ERR) {
        LPRINTF("register sig:%d failed.", signum);
    } else {
        LPRINTF("register sig:%d succeeded.", signum);
    }
}

static void stop_handler(int signum) {
    force_stop = 1;
    (void)signum;

    pthread_mutex_lock(&mutex);
    pthread_cond_signal(&cond);
    pthread_mutex_unlock(&mutex);
}

/**
 * @fn show_menu
 * @brief Display a menu if no command line arguments are specified.
 * @param Number of command line arguments
 */
static void show_menu(int argc)
{
    const char *menu = R"(
******************************************
*   rpmsg communication sample program   *
******************************************

1. communicate with RZ/V2L CM33 ch0
2. communicate with RZ/V2L CM33 ch1

e. exit

please input
> )";

    if (argc < 2)
        printf("%s ", menu);
}

/**
 * @fn wait_input
 * @brief Accept menu selection in dialogue format
 * @param argc - number of command line arguments
 * @param argv - command line arguments
 */
static int wait_input(int argc, char *argv[])
{
    char inbuf[3] = {0};
    char selected[3] = {0};
    int pattern;

    if (argc >= 2) {
        pattern = strtoul(argv[1], NULL, 0) + 1;

        /***************************************
        * rpmsg_sample_client 0   -> pattern 1
        * rpmsg_sample_client 1   -> pattern 2
        **************************************/
    } else {
        fgets(inbuf, sizeof(inbuf), stdin);
        sscanf(inbuf, "%c", selected);
        
        if ('e' == selected[0]) {
            pattern = 0;
        } else {
            selected[2] = '\0';
            pattern = atoi(selected);
        }
    }

    return pattern;
}
