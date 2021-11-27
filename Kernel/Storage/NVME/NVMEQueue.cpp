/*
* Copyright (c) 2021, Pankaj Raghav <pankydev8@gmail.com>
*
* SPDX-License-Identifier: BSD-2-Clause
 */
#include <Kernel/Storage/NVME/NVMEController.h>

namespace Kernel {
NonnullRefPtr<NVMEQueue> NVMEQueue::create(const NVMEController& controller, u16 qid)
{
    return adopt_ref(*new NVMEQueue(controller, qid));
}
}
