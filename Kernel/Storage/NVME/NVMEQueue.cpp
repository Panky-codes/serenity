/*
 * Copyright (c) 2021, Pankaj Raghav <pankydev8@gmail.com>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#include "NVMEQueue.h"
#include "Kernel/StdLib.h"
#include <Kernel/Arch/x86/IO.h>
#include <Kernel/Storage/NVME/NVMEController.h>
#include <Kernel/Storage/NVME/NVMEQueue.h>
#include <Kernel/WorkQueue.h>

namespace Kernel {
// TODO: Move this to generally memory manager
ErrorOr<NonnullOwnPtr<Memory::Region>> dma_alloc_buffer(size_t size, AK::StringView name, Memory::Region::Access access, RefPtr<Memory::PhysicalPage>& dma_buffer_page)
{
    dma_buffer_page = MM.allocate_supervisor_physical_page();
    if (dma_buffer_page.is_null())
        return ENOMEM;
    auto region_or_error = MM.allocate_kernel_region(dma_buffer_page->paddr(), size, name, access);
    return region_or_error;
}

NonnullRefPtr<NVMEQueue> NVMEQueue::create(const NVMEController& controller, u16 qid, u8 irq)
{
    return adopt_ref(*new NVMEQueue(controller, qid, irq));
}

NVMEQueue::NVMEQueue(const NVMEController& controller, u16 qid, u8 irq)
    : IRQHandler(irq)
    , qid(qid)
    , cq_valid_phase(1)
    , sq_tail(0)
    , cq_head(0)
    , m_admin_queue(qid == 0)
    , m_irq(irq)
    , m_qdepth(0)
    , m_controller(controller)
    , m_current_request(nullptr)

{

    if (m_admin_queue) {
        m_qdepth = m_controller->get_admin_q_dept();
        setup_admin_queue();
    } else {
        m_qdepth = IO_QUEUE_SIZE; // Needs to be configurable
        setup_io_queue();
    }
    // DMA region for RW operation. For now the requests don't exceed more than 4096 bytes(Storage device takes of it)
    if (auto buffer = dma_alloc_buffer(PAGE_SIZE, "Admin CQ queue", Memory::Region::Access::ReadWrite, m_rw_dma_page); buffer.is_error()) {
        dmesgln("Failed to allocate memory for ADMIN CQ queue");
        VERIFY_NOT_REACHED();
    } else {
        m_rw_dma_region = buffer.release_value();
    }
}
void NVMEQueue::setup_admin_queue()
{
    auto cq_size = round_up_to_power_of_two(CQ_SIZE(m_qdepth), 4096);
    auto sq_size = round_up_to_power_of_two(SQ_SIZE(m_qdepth), 4096);
    m_controller->reset_controller();

    if (auto buffer = dma_alloc_buffer(cq_size, "Admin CQ queue", Memory::Region::Access::ReadWrite, m_cq_dma_page); buffer.is_error()) {
        dmesgln("Failed to allocate memory for ADMIN CQ queue");
        VERIFY_NOT_REACHED();
    } else {
        m_cq_dma_region = buffer.release_value();
    }

    if (auto buffer = dma_alloc_buffer(sq_size, "Admin SQ queue", Memory::Region::Access::ReadWrite, m_sq_dma_page); buffer.is_error()) {
        dmesgln("Failed to allocate memory for ADMIN CQ queue");
        VERIFY_NOT_REACHED();
    } else {
        m_sq_dma_region = buffer.release_value();
    }

    // TODO: Should this be written as little endian?
    m_controller->write64_controller_regs(CC_ACQ, reinterpret_cast<u64>(m_cq_dma_page->paddr().as_ptr()));
    m_controller->write64_controller_regs(CC_ASQ, reinterpret_cast<u64>(m_sq_dma_page->paddr().as_ptr()));

    m_controller->start_controller();
    m_controller->set_admin_queue_ready_flag();
    enable_irq();
}
void NVMEQueue::setup_io_queue()
{
    nvme_submission sub {};
    auto cq_size = round_up_to_power_of_two(CQ_SIZE(m_qdepth), 4096);
    auto sq_size = round_up_to_power_of_two(SQ_SIZE(m_qdepth), 4096);

    VERIFY(sizeof(nvme_submission) == (1 << SQ_WIDTH));
    if (auto buffer = dma_alloc_buffer(cq_size, "IO CQ queue", Memory::Region::Access::ReadWrite, m_cq_dma_page); buffer.is_error()) {
        dmesgln("Failed to allocate memory for ADMIN CQ queue");
        VERIFY_NOT_REACHED();
    } else {
        m_cq_dma_region = buffer.release_value();
    }

    if (auto buffer = dma_alloc_buffer(sq_size, "IO SQ queue", Memory::Region::Access::ReadWrite, m_sq_dma_page); buffer.is_error()) {
        dmesgln("Failed to allocate memory for ADMIN CQ queue");
        VERIFY_NOT_REACHED();
    } else {
        m_sq_dma_region = buffer.release_value();
    }
    {
        sub.op = OP_ADMIN_CREATE_COMPLETION_QUEUE;
        sub.data_ptr.prp1 = reinterpret_cast<u64>(AK::convert_between_host_and_little_endian(m_cq_dma_page->paddr().as_ptr()));
        sub.cdw10 = AK::convert_between_host_and_little_endian((IO_QUEUE_SIZE << 16 | qid));
        auto flags = QUEUE_IRQ_ENABLED | QUEUE_PHY_CONTIGUOUS;
        // TODO: For now using pin based interrupts. Clear the first 16 bits
        // to use pin-based interrupts. NVMe spec 1.4, section 5.3
        sub.cdw11 = AK::convert_between_host_and_little_endian(flags & 0xFFFF);
        m_controller->submit_admin_command(sub);
    }
    {
        sub.op = OP_ADMIN_CREATE_SUBMISSION_QUEUE;
        sub.data_ptr.prp1 = reinterpret_cast<u64>(AK::convert_between_host_and_little_endian(m_sq_dma_page->paddr().as_ptr()));
        sub.cdw10 = AK::convert_between_host_and_little_endian((IO_QUEUE_SIZE << 16 | qid));
        auto flags = QUEUE_IRQ_ENABLED | QUEUE_PHY_CONTIGUOUS;
        // The qid used below points to the completion queue qid NVMe spec 1.4, section 5.4
        sub.cdw11 = AK::convert_between_host_and_little_endian(qid << 16 | flags);
        m_controller->submit_admin_command(sub);
    }

    enable_irq();
}
bool NVMEQueue::cqe_available()
{
    auto* completion_arr = reinterpret_cast<nvme_completion*>(m_cq_dma_region->vaddr().as_ptr());
    return PHASE_TAG(completion_arr[cq_head].status) == cq_valid_phase;
}
void NVMEQueue::update_cqe_head()
{
    SpinlockLocker lock(m_cq_lock);
    // To prevent overflow, use a temp variable
    u32 temp_cq_head = cq_head + 1;
    if (cq_head == m_qdepth) {
        cq_head = 0;
        cq_valid_phase ^= 1;
    } else {
        cq_head = temp_cq_head;
    }
}
bool NVMEQueue::handle_irq(const RegisterState&)
{
    u32 nr_of_processed_cqes = 0;
    while (cqe_available()) {
        u16 status;
        auto* completion_arr = reinterpret_cast<nvme_completion*>(m_cq_dma_region->vaddr().as_ptr());
        ++nr_of_processed_cqes;
        status = CQ_STATUS_FIELD(completion_arr[cq_head].status);
        dbgln("CQ_HEAD: {} in handle irq", cq_head);
        dbgln("Status field is: {:x}", status);
        if (m_current_request)
            complete_current_request(status);
        update_cqe_head();
    }
    if (nr_of_processed_cqes) {
        m_controller->update_cq_doorbell(cq_head, qid);
    }
    return nr_of_processed_cqes ? true : false;
}
void NVMEQueue::submit_sqe(const struct nvme_submission& sub)
{
    SpinlockLocker lock(m_sq_lock);
    AK::dbgln("CID in submit_sqe is: {}", sub.cmdid);
    auto* submission_arr = reinterpret_cast<nvme_submission*>(m_sq_dma_region->vaddr().as_ptr());

    memcpy(&submission_arr[sq_tail], &sub, sizeof(nvme_submission));
    {
        auto temp_sq_tail = sq_tail + 1;
        if (temp_sq_tail == IO_QUEUE_SIZE)
            sq_tail = 0;
        else
            sq_tail = temp_sq_tail;
    }
    AK::full_memory_barrier();
    m_controller->update_sq_doorbell(sq_tail, qid);
}

u16 NVMEQueue::submit_sync_sqe(nvme_submission& sub)
{
    auto* completion_arr = reinterpret_cast<nvme_completion*>(m_cq_dma_region->vaddr().as_ptr());
    // TODO: Reading does not need a lock I guess?
    // For now let's use sq tail as a unique command id.
    u16 cqe_cid;
    sub.cmdid = sq_tail;
    u16 cid = sq_tail;
    full_memory_barrier();
    submit_sqe(sub);
    // FIXME: Probably a sloppy way of doing sync requests. Need to find a better way
    // Like having a list of callbacks associated with cid
    do {
        cqe_cid = completion_arr[cq_head - 1].command_id;
    } while (cid != cqe_cid);

    auto status = CQ_STATUS_FIELD(completion_arr[cq_head].status);
    return status;
}

void NVMEQueue::read(AsyncBlockDeviceRequest& request, u16 nsid, u64 index, u32 count)
{
    // TODO: count should be 16 bits Figure 372 1.4 Spec
    nvme_submission sub {};
    m_current_request = request;
    sub.op = OP_NVME_READ;
    sub.nsid = nsid;
    sub.cdw10 = AK::convert_between_host_and_little_endian(index & 0xFFFFFFFF);
    sub.cdw11 = AK::convert_between_host_and_little_endian(index >> 32);
    // No. of lbas is 0 based
    sub.cdw12 = AK::convert_between_host_and_little_endian((count - 1) & 0xFFFF);
    sub.data_ptr.prp1 = reinterpret_cast<u64>(AK::convert_between_host_and_little_endian(m_rw_dma_page->paddr().as_ptr()));

    AK::full_memory_barrier();
    submit_sqe(sub);
}

void NVMEQueue::write(AsyncBlockDeviceRequest& request, u16 nsid, u64 index, u32 count)
{
    // TODO: count should be 16 bits Figure 372 1.4 Spec
    nvme_submission sub {};
    m_current_request = request;

    if (auto result = m_current_request->read_from_buffer(m_current_request->buffer(), m_rw_dma_region->vaddr().as_ptr(), 512 * m_current_request->block_count()); result.is_error()) {
        complete_current_request(AsyncDeviceRequest::MemoryFault);
        return;
    }
    sub.op = OP_NVME_WRITE;
    sub.nsid = nsid;
    sub.cdw10 = AK::convert_between_host_and_little_endian(index & 0xFFFFFFFF);
    sub.cdw11 = AK::convert_between_host_and_little_endian(index >> 32);
    // No. of lbas is 0 based
    sub.cdw12 = AK::convert_between_host_and_little_endian((count - 1) & 0xFFFF);
    sub.data_ptr.prp1 = reinterpret_cast<u64>(AK::convert_between_host_and_little_endian(m_rw_dma_page->paddr().as_ptr()));

    AK::full_memory_barrier();
    submit_sqe(sub);
}

void NVMEQueue::complete_current_request(u16 status)
{
    g_io_work->queue([this, status]() {
        auto current_request = m_current_request;
        m_current_request.clear();
        if (status)
            current_request->complete(AsyncBlockDeviceRequest::Failure);
        if (current_request->request_type() == AsyncBlockDeviceRequest::RequestType::Read) {
            if (auto result = current_request->write_to_buffer(current_request->buffer(), m_rw_dma_region->vaddr().as_ptr(), 512 * current_request->block_count()); result.is_error()) {
                current_request->complete(AsyncDeviceRequest::MemoryFault);
                return;
            }
        }
        current_request->complete(AsyncDeviceRequest::Success);
    });
}
}
