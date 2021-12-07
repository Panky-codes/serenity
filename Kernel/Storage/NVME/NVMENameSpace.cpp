/*
 * Copyright (c) 2021, Pankaj Raghav <pankydev8@gmail.com>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#include <AK/NonnullOwnPtr.h>
#include <Kernel/Storage/NVME/NVMENameSpace.h>
#include <Kernel/Storage/StorageManagement.h>
#include "NVMENameSpace.h"


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
    test_rw();
}

void NVMENameSpace::start_request(AsyncBlockDeviceRequest& request)
{
    // TODO: qid 0 is allocated to admin queue
    auto qid = Processor::current_id() + 1;
    auto& queue = m_queues.at(qid);

    if (request.request_type() == AsyncBlockDeviceRequest::Read) {
        queue.read(request, m_nsid, request.block_index(), request.block_count());
    } else {
        queue.write(request, m_nsid, request.block_index(), request.block_count());
    }
}
void NVMENameSpace::test_rw()
{
    for (int i = 0; i < 100; i++) {
        // TODO: Remove this before testing it with actual EXT2 filesystem
        {
            AK::Array<u8, 4096> buf {};
            auto index = i;
            buf[1] = 9;
            auto uk_buf = UserOrKernelBuffer::for_kernel_buffer(buf.data());
            auto read_request_or_error = try_make_request<AsyncBlockDeviceRequest>(AsyncBlockDeviceRequest::Write, index, 1, uk_buf, 512);
            if (read_request_or_error.is_error()) {
                dbgln("NVMENS::read_block({}): try_make_request failed", index);
                return;
            }
            auto read_request = read_request_or_error.release_value();
            switch (read_request->wait().request_result()) {
            case AsyncDeviceRequest::Success:
                break;
            case AsyncDeviceRequest::Failure:
                dbgln("NVMENS::read_block({}) IO error", index);
                break;
            case AsyncDeviceRequest::MemoryFault:
                dbgln("NVMENS::read_block({}) EFAULT", index);
                break;
            case AsyncDeviceRequest::Cancelled:
                dbgln("NVMENS::read_block({}) cancelled", index);
                break;
            default:
                VERIFY_NOT_REACHED();
            }
        }
        {
            AK::Array<u8, 4096> buf {};
            auto index = i;
            auto uk_buf = UserOrKernelBuffer::for_kernel_buffer(buf.data());
            auto read_request_or_error = try_make_request<AsyncBlockDeviceRequest>(AsyncBlockDeviceRequest::Read, index, 1, uk_buf, 512);
            if (read_request_or_error.is_error()) {
                dbgln("NVMENS::read_block({}): try_make_request failed", index);
                return;
            }
            auto read_request = read_request_or_error.release_value();
            switch (read_request->wait().request_result()) {
            case AsyncDeviceRequest::Success:
                break;
            case AsyncDeviceRequest::Failure:
                dbgln("NVMENS::read_block({}) IO error", index);
                break;
            case AsyncDeviceRequest::MemoryFault:
                dbgln("NVMENS::read_block({}) EFAULT", index);
                break;
            case AsyncDeviceRequest::Cancelled:
                dbgln("NVMENS::read_block({}) cancelled", index);
                break;
            default:
                VERIFY_NOT_REACHED();
            }
            VERIFY(buf[1] == 9);
        }
    }
    return;
}

}