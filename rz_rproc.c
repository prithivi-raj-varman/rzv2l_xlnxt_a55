/*
 * Copyright (c) 2014, Mentor Graphics Corporation
 * All rights reserved.
 * Copyright (c) 2017 Xilinx, Inc.
 * Copyright (c) 2020, eForce Co., Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**************************************************************************
 * FILE NAME
 *
 *       rz_rproc.c
 *
 * DESCRIPTION
 *
 *       This file defines RZ/G2 CA5X/CR7 remoteproc implementation.
 *
 * @par  History
 *       - rev 1.0 (2020.10.27) Imada
 *         Initial version.
 *
 **************************************************************************/

#include <pthread.h>
#include <metal/atomic.h>
#include <metal/assert.h>
#include <metal/device.h>
#include <metal/irq.h>
#include <metal/list.h>
#include <metal/utilities.h>
#include <openamp/rpmsg_virtio.h>
#include "platform_info.h"

extern struct ipi_info ipi;
extern struct shm_info shm;

/** to stop execution until received an interrupt. */
extern pthread_mutex_t mutex;
extern pthread_cond_t cond;

/** to avoid second initialization of the common resource */
static int initialized = 0;

/** share memories */
extern struct vring_info vrinfo[CFG_RPMSG_SVCNO];

/** flag SIGINT or SIGTERM have been received */
extern int force_stop;

/* Inline functions to add accessing address check to corresponding 
 * libmetal functions to avoid accessing a reserved region. 
 * They are mainly required because of larger (uio) mmap size due the 
 * 4KB page size on Verified Linux. They are also utilized for uC3 because 
 * accessing a reserved area of registers can cause system failure.
 */
static inline void metal_io_read32_with_check(struct metal_io_region *io, unsigned long offset, unsigned int *ret)
{
    if ((MBX_REG_START <= offset) && (offset < MBX_REG_END)) {
        *ret = metal_io_read32(io, offset);
    }
    else {
        LPERROR("Offset value larger than the register size in metal_io_read32.");
    }

    return ;
}

static inline void metal_io_write32_with_check(struct metal_io_region *io, unsigned long offset, uint64_t val)
{
    if ((MBX_REG_START <= offset) && (offset < MBX_REG_END)) {
        metal_io_write32(io, offset, val);
    }
    else {
        LPERROR("Offset value larger than the register size in metal_io_write32.");
    }
    return ;
}

static int init_memory_device_individual(struct shm_info *info){
    struct metal_device *dev;
    metal_phys_addr_t mem_pa;
    int ret;

    ret = metal_device_open(info->bus_name, info->name, &dev);
    if (ret) {
        LPERROR("Failed to open uio device %s: %d.", info->name, ret);
        goto err;
    }
    LPRINTF("Successfully open uio device: %s.", info->name);
    
    info->dev = dev;
    info->io = metal_device_io_region(dev, 0x0U);
    if (!info->io) {
        LPRINTF("info->io is 0");
        goto err;
    }
    
    mem_pa = metal_io_phys(info->io, 0x0U);
    info->mem = metal_allocate_memory(sizeof(*info->mem));
    memset(info->mem, 0, sizeof(*info->mem));
    remoteproc_init_mem(info->mem, info->name, mem_pa, mem_pa,
                metal_io_region_size(info->io),
                info->io);
    LPRINTF("Successfully added memory device %s.", info->name);

    return 0;
err:
    if(info->dev)
        metal_device_close(info->dev);
    return ret;
}

/**
 * @fn add_memory_device
 * @param rproc - platform resource
 * @param info - memory management info
 * @param base - memory management template
 */
static void add_memory_device(struct remoteproc *rproc, struct shm_info* info, struct shm_info *base) {
    memcpy(info, base, sizeof(struct shm_info));
    info->mem = metal_allocate_memory(sizeof(*info->mem));
    memcpy(info->mem, base->mem, sizeof(*info->mem));
    memset(&info->mem->node, 0, sizeof(struct metal_list));
    remoteproc_add_mem(rproc, info->mem);
}

/**
 * @fn init_memory_device
 * @param rproc - platform resource
 * @return 0(normal) else(failed)
 */
static int init_memory_device(struct remoteproc *rproc) {
    struct remoteproc_priv *prproc = rproc->priv;
    int ret = -1;
    int memory_initialized;

    if (!prproc || !prproc->vr_info) {
        LPRINTF("vring is null");
        goto error_return;
    }

    if ((prproc->notify_id < 0) || (1 < prproc->notify_id)) {
        LPRINTF("rscid is invalid");
        goto error_return;
    }

    memory_initialized = prproc->notify_id + prproc->mbx_chn_id;
    if (!memory_initialized) {
        ret = init_memory_device_individual(&vrinfo[0].rsc);
        ret |= init_memory_device_individual(&vrinfo[0].ctl);
        ret |= init_memory_device_individual(&vrinfo[0].shm);
        ret |= init_memory_device_individual(&vrinfo[1].ctl);
        ret |= init_memory_device_individual(&vrinfo[1].shm);
        ret |= init_memory_device_individual(&shm);
        if (ret) {
            LPRINTF("init_memory_device failed.");
            goto error_return;
        }
    } else {
        ret = 0;
    }

    metal_list_init(&rproc->mems);
    add_memory_device(rproc, &prproc->vr_info[0], &vrinfo[0].rsc);
    add_memory_device(rproc, &prproc->vr_info[1], &vrinfo[prproc->notify_id].ctl);
    add_memory_device(rproc, &prproc->vr_info[2], &vrinfo[prproc->notify_id].shm);
    add_memory_device(rproc, &prproc->vr_info[3], &shm);

error_return:
    return ret;
}

/**
 * @fn create_vrinfo
 * @brief Create memory management information
 * @return 0(normal) else(failed)
 */
static int create_vrinfo(struct remoteproc* rproc) {
    int ret = -1;
    struct remoteproc_priv* prproc;
    size_t size;

    if (!rproc || !rproc->priv) {
        LPRINTF("priv is null");
        goto error_return;
    }
    prproc = rproc->priv;
    size = sizeof(struct shm_info) * VRING_MAX;

    prproc->vr_info = metal_allocate_memory(size);
    if (!(prproc->vr_info)) {
        LPRINTF("vr_info allocate memory failed.");
        goto error_return;
    }
    if (memset(prproc->vr_info, 0, size) == NULL) {
        LPRINTF("vr_info memset failed.");
        goto error_return;
    }

    ret = init_memory_device(rproc);

error_return:
    return ret;
}

/**
 * @fn remoteproc_remove_mem
 * @brief discard the holding available memory address information
 * @param pmem - memory information to be discarded
 */
static inline void remoteproc_remove_mem(struct remoteproc_mem **pmem)
{
    if (pmem && *pmem) {
        struct remoteproc_mem* mem = *pmem;
        if (mem->node.prev) {
            metal_list_del(&mem->node);
            mem->node.prev = NULL;
            mem->node.next = NULL;
        }
        metal_free_memory(mem);
        *pmem = NULL;
    }
}

/**
 * @fn deinit_memory_device_individual
 * @param info - shared memory information to be discarded.
 */
static void deinit_memory_device_individual(struct shm_info *info)
{
    if (info) {
        remoteproc_remove_mem(&info->mem);
        if (info->dev) {
            metal_device_close(info->dev);
            LPRINTF("%s closed", info->name);
            info->dev = NULL;
        }
    }
}

/**
 * @fn deinit_memory_device
 * @brief close and release memory device
 * @param rproc - platform resource
 */
static void deinit_memory_device(struct remoteproc *rproc)
{
    int i;
    struct remoteproc_priv* prproc;

    if (!rproc) goto error_return;
    prproc = rproc->priv;

    if (!prproc || !prproc->vr_info) goto error_return;

    deinit_memory_device_individual(&vrinfo[0].rsc);
    deinit_memory_device_individual(&vrinfo[0].ctl);
    deinit_memory_device_individual(&vrinfo[0].shm);
    deinit_memory_device_individual(&vrinfo[1].ctl);
    deinit_memory_device_individual(&vrinfo[1].shm);
    deinit_memory_device_individual(&shm);

    if (prproc->vr_info) {
        for (i = 0; i < VRING_MAX; i++) {
            if (prproc->vr_info[i].mem) {
                metal_free_memory(prproc->vr_info[i].mem);
                prproc->vr_info[i].mem = NULL;
            }
        }
        metal_free_memory(prproc->vr_info);
        prproc->vr_info = NULL;
    }

error_return:
    return;
}

static int rz_proc_irq_handler(int vect_id, void *data)
{
    unsigned int val = 0U;

    (void)vect_id;
    (void)data;

    /* Clear the interrupt */
    metal_io_write32_with_check(ipi.io, MBX_REMOTE_INT_CLR_REG(MBX_NO), 0x1U);

    /* Get a massage from the mailbox */
    metal_io_read32_with_check(shm.io, SHM_REMOTE_OFFSET(MBX_NO), &val);

    if (val >= RPVDEV_MAX_NUM) { /* val should have the notify_id of the sender */
        return METAL_IRQ_NOT_HANDLED; /* Invalid message arrived */
    }

#ifdef __linux__
    ipi.notify_id = val;
    atomic_flag_clear(&ipi.sync);
    pthread_mutex_lock(&mutex);
    pthread_cond_signal(&cond);
    pthread_mutex_unlock(&mutex);
#else /* uC3 */
    if (ipi.ipi_sem_id[val] != E_ID) {
        isig_sem(ipi.ipi_sem_id[val]);
    }
    else
        return METAL_IRQ_NOT_HANDLED;
#endif

    return METAL_IRQ_HANDLED;
}

static int rz_enable_interrupt(struct remoteproc *rproc, struct metal_device *ipi_dev)
{
    unsigned int irq_vect;
    int ret = 0;

    if (!ipi.registered) {
        /* Register interrupt handler and enable interrupt for RZ/G2 CA5X or CR7 */
        irq_vect = (uintptr_t)ipi_dev->irq_info;
        ret = metal_irq_register((int)irq_vect, rz_proc_irq_handler, ipi_dev, rproc);
        if (ret) {
            LPRINTF("metal_irq_register() failed with %d", ret);
            return ret;
        }
        metal_irq_enable(irq_vect);
    }

    return ret;
}

static void rz_disable_interrupt(struct remoteproc *rproc)
{
    (void)rproc;
    struct metal_device *dev;
    int ret;

    dev = ipi.dev;
    if (dev) {
        metal_irq_disable((uintptr_t)dev->irq_info);
        ret = metal_irq_unregister((int)dev->irq_info, NULL, NULL, NULL);
        if (ret) {
            LPRINTF("metal_irq_unregister() failed with %d", ret);
            return ;
        }
        metal_device_close(dev);
        ipi.registered = 0;
    }
}

static struct remoteproc *
rz_proc_init(struct remoteproc *rproc,
            struct remoteproc_ops *ops, void *arg)
{
    struct remoteproc_priv *prproc = arg;
    struct metal_device *dev;
    int ret;

    if ((!rproc) || (!prproc) || (!ops))
        return NULL;
    rproc->priv = prproc;
    rproc->ops = ops;

    if (!ipi.registered) {
        /* Get an IPI device (Mailbox) */
        ret = metal_device_open(ipi.bus_name, ipi.name, &dev);
        if (ret) {
            LPERROR("Failed to open ipi device: %d.", ret);
            return NULL;
        }
        ipi.dev = dev;
        ipi.io = metal_device_io_region(dev, 0x0U);
        if (!ipi.io)
            goto err1;
#ifdef __linux__
        atomic_flag_test_and_set(&ipi.sync);
#endif
        LPRINTF("Successfully probed IPI device");

    }
    ret = rz_enable_interrupt(rproc, ipi.dev);
    if (ret) {
        LPERROR("Failed to register the interrupt handler.");
        goto err1;
    }
    ipi.registered++;

    /* Get the resource table device */
    if (create_vrinfo(rproc)){
        goto err1;
    }

    return rproc;
err1:
    metal_device_close(ipi.dev);
    return NULL;
}

static void rz_proc_remove(struct remoteproc *rproc)
{
    if (!rproc)
        return;

    if (ipi.registered > 1) {
        ipi.registered--;
        return;
    }

    deinit_memory_device(rproc);
    /* Disable interrupts */
    rz_disable_interrupt(rproc);

    return;
}

static int rz_proc_notify(struct remoteproc *rproc, uint32_t id)
{
    struct remoteproc_priv *prproc = rproc->priv;
    unsigned int val = 0U;

    /* Put a message saying "This is the notify_id of mine!" */
    metal_io_write32_with_check(shm.io, SHM_LOCAL_OFFSET(MBX_NO), (uint64_t)prproc->notify_id);

    /* Check interrupt status: Has the previous message been received? */
    do {
        metal_io_read32_with_check(ipi.io, MBX_LOCAL_INT_STS_REG(MBX_NO), &val);
    } while (0 != val && !force_stop);

    /* Send notification */
    metal_io_write32_with_check(ipi.io, MBX_LOCAL_INT_SET_REG(MBX_NO), 0x1U);

    return 0;
}

#ifdef __linux__
static void *
rz_proc_mmap(struct remoteproc *rproc,
            metal_phys_addr_t *pa, metal_phys_addr_t *da, size_t size,
            unsigned int attribute, struct metal_io_region **io)
{
    metal_phys_addr_t lpa, lda;
    struct metal_io_region *tmpio;
    struct remoteproc_priv *prproc;
    (void)attribute;
    (void)size;
    
    if (!rproc) {
        LPRINTF("rproc is null");
        return NULL;
    }
    prproc = rproc->priv;
    
    lpa = *pa;
    lda = *da;

    if ((lpa == METAL_BAD_PHYS) && (lda == METAL_BAD_PHYS))
        return NULL;
    if (lpa == METAL_BAD_PHYS)
        lpa = lda;
    if (lda == METAL_BAD_PHYS)
        lda = lpa;
    tmpio = prproc->vr_info[VRING_RSC].io; /* We consider the resource table device only */
    if (!tmpio) {
        LPRINTF("tmpio is null");
        return NULL;
    }

    *pa = lpa;
    *da = lda;
    if (io)
        *io = tmpio;

    return metal_io_phys_to_virt(tmpio, lpa);
}
#endif

/* processor operations in rz. It defines
 * notification operation and remote processor managementi operations. */
struct remoteproc_ops rz_proc_ops = {
    .init = rz_proc_init,
    .remove = rz_proc_remove,
#ifdef __linux__
    .mmap = rz_proc_mmap,
#else
    .mmap = NULL,
#endif
    .notify = rz_proc_notify,
    .start = NULL,
    .stop = NULL,
    .shutdown = NULL,
};
