/*
 * Copyright (c) 2021, Pankaj Raghav <pankydev8@gmail.com>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#pragma once

#include "AK/kmalloc.h"
#include <AK/NonnullRefPtr.h>
#include <AK/NonnullRefPtrVector.h>
#include <AK/OwnPtr.h>
#include <AK/RefCounted.h>
#include <AK/RefPtr.h>
#include <AK/Types.h>
#include <Kernel/Locking/Spinlock.h>
#include <Kernel/Storage/NVME/NVMEDefinitions.h>
#include <Kernel/Storage/NVME/NVMEQueue.h>
#include <Kernel/Storage/StorageDevice.h>

namespace Kernel {
class NVMENameSpace : public StorageDevice
{
  AK_MAKE_ETERNAL

public:
    static NonnullRefPtr<NVMENameSpace> create(NonnullRefPtrVector<NVMEQueue> queues, u16 nsid, size_t storage_size, size_t lba_size);
    explicit NVMENameSpace(NonnullRefPtrVector<NVMEQueue> queues, size_t storage_size, size_t lba_size, size_t major_number, size_t minor_number, u16 nsid, NonnullOwnPtr<KString> early_device_name);

    CommandSet command_set() const override { return CommandSet::NVMe; };
    void start_request(AsyncBlockDeviceRequest& request) override;
    void test_rw();

private:
    u16 m_nsid;
    u32 m_lba_size;
    u64 m_max_addresable_block;
    NonnullRefPtrVector<NVMEQueue> m_queues;
};

}
