/**
 * @file    main.c
 * @brief   Main function and RPMSG example application modified to send classic CAN frames.
 * @date    2020.10.27 (modified 2025)
 * @author  Original: Copyright (c) 2020, eForce Co., Ltd. All rights reserved.
 * @license SPDX-License-Identifier: BSD-3-Clause
 *
 * Modification: Replaced the variable-size echo test with sending fixed classic CAN frames
 *               (11-bit standard ID, up to 8 bytes data). The remote side is expected to echo
 *               the frame back unchanged. Integrity is checked on reception.
 */

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

/* Classic CAN frame structure */
struct can_frame {
    uint32_t can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags (here only 11-bit standard ID) */
    uint8_t  can_dlc; /* Data length code (0-8) */
    uint8_t  data[8] __attribute__((aligned(4)));
};

#ifndef max
#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     (_a > _b) ? _a : _b; })
#endif

#ifndef ARRAY_SIZE
    #define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))
#endif

/* Internal functions */
static void rpmsg_service_bind(struct rpmsg_device *rdev, const char *name, uint32_t dest);
static void rpmsg_service_unbind(struct rpmsg_endpoint *ept);
static int rpmsg_service_cb0(struct rpmsg_endpoint *rp_ept, void *data, size_t len, uint32_t src, void *priv);
static void register_handler(int signum, void(* handler)(int));
static void stop_handler(int signum);
static void init_cond(void);
static void show_menu(int argc);
static int wait_input(int argc, char *argv[]);
static void launch_communicate(int pattern);
static void *communicate(void* arg);

/* Globals */
static struct rpmsg_endpoint rp_ept = { 0 };
static struct can_frame send_frame;   /* Single frame buffer for sending */
static struct can_frame recv_frame;   /* Buffer for received echoed frame */
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

/* Application entry point - modified for CAN frame transmission */
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
    send_frame.can_id  = 0x123;      /* Example standard CAN ID */
    send_frame.can_dlc = 8;          /* Full 8 bytes data */
    memset(send_frame.data, 0xA5, 8); /* Fill data with pattern 0xA5 for easy integrity check */

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

/* Callback remains mostly the same - validates the echoed CAN frame */
static int rpmsg_service_cb0(struct rpmsg_endpoint *cb_rp_ept, void *data, size_t len, uint32_t src, void *priv)
{
    (void)cb_rp_ept;
    (void)src;
    (void)priv;

    if (len != sizeof(struct can_frame)) {
        LPERROR("Invalid frame size received: %zu (expected %zu)\n", len, sizeof(struct can_frame));
        err_cnt++;
        return -1;
    }

    memcpy(&recv_frame, data, sizeof(struct can_frame));

    //LPRINTF("Received echoed CAN frame: ID=0x%03lX, DLC=%d\n",
            //recv_frame.can_id & 0x7FF, recv_frame.can_dlc);
            LPRINTF("Received echoed CAN frame: ID=0x%03lX, DLC=%d, Data: "
        "%02X %02X %02X %02X %02X %02X %02X %02X\n",
        recv_frame.can_id & 0x7FF, recv_frame.can_dlc,
        recv_frame.data[0], recv_frame.data[1], recv_frame.data[2], recv_frame.data[3],
        recv_frame.data[4], recv_frame.data[5], recv_frame.data[6], recv_frame.data[7]);

    /* Validate integrity */
    if (recv_frame.can_id != send_frame.can_id ||
        recv_frame.can_dlc != send_frame.can_dlc ||
        memcmp(recv_frame.data, send_frame.data, recv_frame.can_dlc) != 0) {
        LPERROR("Frame corruption detected!\n");
        err_cnt++;
        return -1;
    }

    rnum++;  /* Increment received count */
    return 0;
}

/* Rest of the functions unchanged */
static void rpmsg_service_bind(struct rpmsg_device *rdev, const char *name, uint32_t dest)
{
    LPRINTF("New endpoint notification received.\n");
    if (strcmp(name, svc_name)) {
        LPERROR("Unexpected name service %s.\n", name);
    } else {
        (void)rpmsg_create_ept(&rp_ept, rdev, svc_name,
                       APP_EPT_ADDR, dest,
                       rpmsg_service_cb0,
                       rpmsg_service_unbind);
    }
}

static void rpmsg_service_unbind(struct rpmsg_endpoint *ept)
{
    (void)ept;
    rpmsg_destroy_ept(&rp_ept);
    memset(&rp_ept, 0x0, sizeof(struct rpmsg_endpoint));
}

static void init_cond(void)
{
#ifdef __linux__
    pthread_mutex_init(&mutex, NULL);
    pthread_mutex_init(&rsc_mutex, NULL);
    pthread_cond_init(&cond, NULL);
#endif
}

int main(int argc, char *argv[])
{
    int pattern;
    unsigned long proc_id;
    unsigned long rsc_id;
    int i;
    int ret = 0;

    init_system();
    init_cond();

    for (i = 0; i < ARRAY_SIZE(ids); i++) {
        proc_id = rsc_id = ids[i].channel;
    
        ret = platform_init(proc_id, rsc_id, &ids[i].platform);
        if (ret) {
            LPERROR("Failed to initialize platform.\n");
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

static void *communicate(void* arg) {
    struct comm_arg *p = (struct comm_arg*)arg;
    struct rpmsg_device *rpdev;
    unsigned long proc_id = p->channel;

    LPRINTF("Thread start\n");

    pthread_mutex_lock(&rsc_mutex);
    rpdev = platform_create_rpmsg_vdev(p->platform, 0,
                      VIRTIO_DEV_MASTER,
                      NULL,
                      rpmsg_service_bind);
    pthread_mutex_unlock(&rsc_mutex);
    if (!rpdev) {
        LPERROR("Failed to create rpmsg virtio device.\n");
    } else {
        (void)app(rpdev, p->platform, proc_id);
        platform_release_rpmsg_vdev(p->platform, rpdev);
    }
    LPRINTF("Stopping application...\n");

    return NULL;
}

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
        LPRINTF("register sig:%d failed.\n", signum);
    } else {
        LPRINTF("register sig:%d succeeded.\n", signum);
    }
}

static void stop_handler(int signum) {
    force_stop = 1;
    (void)signum;

    pthread_mutex_lock(&mutex);
    pthread_cond_signal(&cond);
    pthread_mutex_unlock(&mutex);
}

static void show_menu(int argc)
{
    const char *menu = R"(
******************************************
*   rpmsg CAN frame sample program      *
******************************************

1. communicate with RZ/V2L CM33 ch0
2. communicate with RZ/V2L CM33 ch1

e. exit

please input
> )";

    if (argc < 2)
        printf("%s ", menu);
}

static int wait_input(int argc, char *argv[])
{
    char inbuf[3] = {0};
    char selected[3] = {0};
    int pattern;

    if (argc >= 2) {
        pattern = strtoul(argv[1], NULL, 0) + 1;
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