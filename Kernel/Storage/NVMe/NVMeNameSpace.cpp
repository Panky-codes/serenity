/*
 * Copyright (c) 2021, Pankaj R <pankydev8@gmail.com>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include "NVMeNameSpace.h"
#include <AK/NonnullOwnPtr.h>
#include <Kernel/Devices/DeviceManagement.h>
#include <Kernel/Storage/NVMe/NVMeNameSpace.h>
#include <Kernel/Storage/StorageManagement.h>

namespace Kernel {

NonnullRefPtr<NVMeNameSpace> NVMeNameSpace::create(NonnullRefPtrVector<NVMeQueue> queues, u16 nsid, size_t storage_size, size_t lba_size)
{
    auto minor_number = StorageManagement::minor_number();
    auto major_number = StorageManagement::major_number();
    auto device_name = String::formatted("nvme0n{:d}", nsid);
    auto device_name_kstring = KString::must_create(device_name.view());
    auto device_or_error = DeviceManagement::try_create_device<NVMeNameSpace>(queues, storage_size, lba_size, major_number, minor_number, nsid, move(device_name_kstring));
    VERIFY(!device_or_error.is_error());

    return device_or_error.release_value();
}

NVMeNameSpace::NVMeNameSpace(NonnullRefPtrVector<NVMeQueue> queues, size_t max_addresable_block, size_t lba_size, size_t major_number, size_t minor_number, u16 nsid, NonnullOwnPtr<KString> dev_name)
    : StorageDevice(major_number, minor_number, lba_size, max_addresable_block, move(dev_name))
    , m_nsid(nsid)
    , m_lba_size(lba_size)
    , m_max_addresable_block(max_addresable_block)
    , m_queues(queues)
{
}

void NVMeNameSpace::start_request(AsyncBlockDeviceRequest& request)
{
    auto index = Processor::current_id();
    auto& queue = m_queues.at(index);
    // TODO: For now we support only IO transfers of size PAGE_SIZE (Going along with the current constraint in the block layer)
    // Eventually remove this constraint by using the PRP2 field in the submission struct and remove block layer constraint for NVMe driver.
    VERIFY(request.block_count() <= (PAGE_SIZE / block_size()));

    if (request.request_type() == AsyncBlockDeviceRequest::Read) {
        queue.read(request, m_nsid, request.block_index(), request.block_count());
    } else {
        queue.write(request, m_nsid, request.block_index(), request.block_count());
    }
}
}
