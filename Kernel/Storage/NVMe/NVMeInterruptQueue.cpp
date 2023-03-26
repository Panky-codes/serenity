/*
 * Copyright (c) 2022, Pankaj R <pankydev8@gmail.com>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <Kernel/Devices/BlockDevice.h>
#include <Kernel/Storage/NVMe/NVMeDefinitions.h>
#include <Kernel/Storage/NVMe/NVMeInterruptQueue.h>
#include <Kernel/WorkQueue.h>

namespace Kernel {

UNMAP_AFTER_INIT NVMeInterruptQueue::NVMeInterruptQueue(NonnullOwnPtr<Memory::Region> rw_dma_region, Memory::PhysicalPage const& rw_dma_page, u16 qid, u8 irq, u32 q_depth, OwnPtr<Memory::Region> cq_dma_region, Vector<NonnullRefPtr<Memory::PhysicalPage>> cq_dma_page, OwnPtr<Memory::Region> sq_dma_region, Vector<NonnullRefPtr<Memory::PhysicalPage>> sq_dma_page, Memory::TypedMapping<DoorbellRegister volatile> db_regs)
    : NVMeQueue(move(rw_dma_region), rw_dma_page, qid, q_depth, move(cq_dma_region), cq_dma_page, move(sq_dma_region), sq_dma_page, move(db_regs))
    , IRQHandler(irq)
{
    enable_irq();
}

bool NVMeInterruptQueue::handle_irq(RegisterState const&)
{
    SpinlockLocker lock(m_request_lock);
    return process_cq() ? true : false;
}

void NVMeInterruptQueue::submit_sqe(NVMeSubmission& sub)
{
    NVMeQueue::submit_sqe(sub);
}

void NVMeInterruptQueue::complete_current_request(u16 cmdid, u16 status)
{
    auto work_item_creation_result = g_io_work->try_queue([this, cmdid, status]() {
        SpinlockLocker lock(m_request_lock);
        auto& request_pdu = m_requests.get(cmdid).release_value();
        auto current_request = request_pdu.request;
        request_pdu.used = false;
        AsyncDeviceRequest::RequestResult req_result = AsyncDeviceRequest::Success;

        // There can be submission without any request associated with it such as with
        // admin queue commands during init. If there is no request, we are done
        if (!current_request)
            goto endio_handler;

        if (status) {
            req_result = AsyncBlockDeviceRequest::Failure;
            goto complete_request;
        }

        if (current_request->request_type() == AsyncBlockDeviceRequest::RequestType::Read) {
            if (auto result = current_request->write_to_buffer(current_request->buffer(), m_rw_dma_region->vaddr().as_ptr(), current_request->buffer_size()); result.is_error()) {
                req_result = AsyncBlockDeviceRequest::MemoryFault;
                goto complete_request;
            }
        }

    complete_request:
        lock.unlock();
        current_request->complete(req_result);
    endio_handler:
        if (request_pdu.end_io_handler)
            request_pdu.end_io_handler(status);
        return;
    });

    if (work_item_creation_result.is_error()) {
        SpinlockLocker lock(m_request_lock);
        auto& request_pdu = m_requests.get(cmdid).release_value();
        auto current_request = request_pdu.request;
        request_pdu.used = false;

        if (request_pdu.end_io_handler)
            request_pdu.end_io_handler(status);

        lock.unlock();
        current_request->complete(AsyncDeviceRequest::OutOfMemory);
    }
}
}
