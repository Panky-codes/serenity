/*
 * Copyright (c) 2021, Pankaj Raghav <pankydev8@gmail.com>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#include <AK/NonnullOwnPtr.h>
#include <Kernel/Storage/NVME/NVMENameSpace.h>
#include <Kernel/Storage/StorageManagement.h>

namespace Kernel {

NonnullRefPtr<NVMENameSpace> NVMENameSpace::create(NonnullRefPtrVector<NVMEQueue> queues, u16 nsid, size_t storage_size, size_t lba_size)
{
    auto minor_number = StorageManagement::minor_number();
    auto major_number = StorageManagement::major_number();
    // TODO: For now we are assuming only one controller is there. Name format nvme<ctrlid>n<nsid>
    auto device_name = String::formatted("nvme0n{:d}", nsid);
    auto device_name_kstring = KString::must_create(device_name.view());
    return adopt_ref(*new NVMENameSpace(queues, storage_size, lba_size, major_number, minor_number, nsid, move(device_name_kstring)));
}

NVMENameSpace::NVMENameSpace(NonnullRefPtrVector<NVMEQueue> queues, size_t max_addresable_block, size_t lba_size, size_t major_number, size_t minor_number, u16 nsid, NonnullOwnPtr<KString> dev_name)
    : StorageDevice(major_number, minor_number, lba_size, max_addresable_block, move(dev_name))
    , m_nsid(nsid)
    , m_lba_size(lba_size)
    , m_max_addresable_block(max_addresable_block)
    , m_queues(queues)
{
}

void NVMENameSpace::start_request([[maybe_unused]] AsyncBlockDeviceRequest& request)
{
}

}