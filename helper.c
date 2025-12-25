/*
 * Copyright (c) 2014, Mentor Graphics Corporation
 * All rights reserved.
 *
 * Copyright (c) 2015 Xilinx, Inc. All rights reserved.
 * Copyright (c) 2020 eForce Co., Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 ****************************************************************************
 * @par     History
 *          - rev 1.0 (2019.09.17) nozaki
 *            Initial version.
 ****************************************************************************
 */

#include <string.h>
#include "metal/sys.h"
#include "metal/shmem.h"
#include "metal/device.h"
#include "metal/io.h"


int init_system(void)
{
    struct metal_init_params metal_param = METAL_INIT_DEFAULTS;

    return metal_init(&metal_param);
}

void cleanup_system(void)
{
    metal_finish();
}

