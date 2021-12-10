/*
 * Copyright (c) 2021, Pankaj Raghav <pankydev8@gmail.com>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#include "NVMEController.h"
#include "AK/Format.h"
#include <AK/RefPtr.h>
#include <AK/Types.h>
#include <Kernel/Arch/x86/IO.h>
#include <Kernel/Arch/x86/Processor.h>
#include <Kernel/Bus/PCI/API.h>
#include <Kernel/Devices/Device.h>
#include <Kernel/FileSystem/ProcFS.h>
#include <Kernel/Sections.h>

namespace Kernel {

NonnullRefPtr<NVMEController> NVMEController::initialize(const Kernel::PCI::DeviceIdentifier& device_identifier)
{
    return adopt_ref(*new NVMEController(device_identifier));
}

NVMEController::NVMEController(const PCI::DeviceIdentifier& device_identifier)
    : PCI::Device(device_identifier.address())
    , m_pci_device_id(device_identifier)
    , m_admin_queue_ready(false)
    , m_device_count(0)
{
    // Nr of queues = one queue per core
    auto nr_of_queues = Processor::count();
    auto irq = device_identifier.interrupt_line().value();

    PCI::enable_memory_space(device_identifier.address());
    PCI::enable_bus_mastering(device_identifier.address());
    auto bar = PCI::get_BAR0(device_identifier.address()) & 0xFFFFFFF0;
    auto bar_len = round_up_to_power_of_two(CTRL_REG_SIZE(nr_of_queues), 4096);
    auto region_or_error = MM.allocate_kernel_region(PhysicalAddress(bar), bar_len, "PCI NVMe BAR", Memory::Region::Access::ReadWrite);
    if (region_or_error.is_error())
        dmesgln("Failed to map NVMe BAR0");
    // Queues need access to the doorbell registers, hence leaking the pointer
    m_controller_regs = region_or_error.release_value().leak_ptr();
    void* ptr = m_controller_regs->vaddr().as_ptr();
    VERIFY((*static_cast<u16*>(ptr)) == 2047); // TODO: Remove this

    create_admin_queue(irq);
    VERIFY(m_admin_queue_ready == true);
    for (u32 cpuid = 0; cpuid < nr_of_queues; ++cpuid) {
        // qid is zero is used for admin queue
        create_io_queue(irq, cpuid + 1);
    }

    identify_and_init_namespaces();
}

void NVMEController::reset_controller()
{
    volatile u32 cc, csts;
    u8* ptr = m_controller_regs->vaddr().as_ptr();
    csts = *reinterpret_cast<u32*>(ptr + CSTS_REG);
    VERIFY((csts & (1 << CSTS_RDY_BIT)) == 0x1);

    cc = *reinterpret_cast<u32*>(ptr + CC_REG);
    cc = cc & ~(1 << CC_EN_BIT);

    *reinterpret_cast<u32*>(ptr + CC_REG) = cc;

    IO::delay(100);
    AK::full_memory_barrier();

    csts = *reinterpret_cast<u32*>(ptr + CSTS_REG);
    VERIFY((csts & (1 << CSTS_RDY_BIT)) == 0x0);
}

void NVMEController::start_controller()
{
    volatile u32 cc, csts;
    u8* ptr = m_controller_regs->vaddr().as_ptr();
    csts = *reinterpret_cast<u32*>(ptr + CSTS_REG);
    VERIFY((csts & (1 << CSTS_RDY_BIT)) == 0x0);

    cc = *reinterpret_cast<u32*>(ptr + CC_REG);

    cc = cc | (1 << CC_EN_BIT);
    cc = cc | (CQ_WIDTH << CC_IOCQES_BIT);
    cc = cc | (SQ_WIDTH << CC_IOSQES_BIT);

    *reinterpret_cast<u32*>(ptr + CC_REG) = cc;

    IO::delay(100);
    AK::full_memory_barrier();
    csts = *reinterpret_cast<u32*>(ptr + CSTS_REG);
    VERIFY((csts & (1 << CSTS_RDY_BIT)) == 0x1);
}
u32 NVMEController::get_admin_q_dept()
{
    u32 aqa = *reinterpret_cast<u32*>(m_controller_regs->vaddr().as_ptr() + CC_AQA);
    // Queue depth is 0 based
    return min(ACQ_SIZE(aqa), ASQ_SIZE(aqa)) + 1;
}
void NVMEController::identify_and_init_namespaces()
{

    RefPtr<Memory::PhysicalPage> prp_dma_buffer;
    OwnPtr<Memory::Region> prp_dma_region;
    AK::Array<u8, NVME_IDENTIFY_SIZE> namespace_data_struct;
    u32 active_namespace_list[NVME_IDENTIFY_SIZE / sizeof(u32)];
    // TODO: Without this print stuff breaks in **KVM** mode
    dbgln("Identify namespaces");

    if (auto buffer = dma_alloc_buffer(NVME_IDENTIFY_SIZE, "Identify PRP", Memory::Region::Access::ReadWrite, prp_dma_buffer); buffer.is_error()) {
        dmesgln("Failed to allocate memory for ADMIN CQ queue");
        // TODO:Need to figure out better error propagation
        VERIFY_NOT_REACHED();
    } else {
        prp_dma_region = buffer.release_value();
    }
    // Get the active namespace
    {
        nvme_submission sub {};
        u16 status = 0;
        sub.op = OP_ADMIN_IDENTIFY;
        sub.data_ptr.prp1 = reinterpret_cast<u64>(AK::convert_between_host_and_little_endian(prp_dma_buffer->paddr().as_ptr()));
        sub.cdw10 = NVME_CNS_ID_ACTIVE_NS & 0xff;
        status = submit_admin_command(sub, true);
        if (status) {
            dmesgln("Failed identify active namespace command");
            VERIFY_NOT_REACHED();
        }
        if (void* fault_at; !safe_memcpy(active_namespace_list, prp_dma_region->vaddr().as_ptr(), NVME_IDENTIFY_SIZE, fault_at)) {
            VERIFY_NOT_REACHED();
        }
    }
    // Get the NAMESPACE attributes
    {
        nvme_submission sub {};
        u16 status = 0;
        for (auto nsid : active_namespace_list) {
            memset(prp_dma_region->vaddr().as_ptr(), 0, NVME_IDENTIFY_SIZE);
            // Invalid NS
            if (nsid == 0)
                break;
            sub.op = OP_ADMIN_IDENTIFY;
            sub.data_ptr.prp1 = reinterpret_cast<u64>(AK::convert_between_host_and_little_endian(prp_dma_buffer->paddr().as_ptr()));
            sub.cdw10 = NVME_CNS_ID_NS & 0xff;
            sub.nsid = nsid;
            status = submit_admin_command(sub, true);
            if (status) {
                dmesgln("Failed identify active namespace command");
                VERIFY_NOT_REACHED();
            }
            if (void* fault_at; !safe_memcpy(namespace_data_struct.data(), prp_dma_region->vaddr().as_ptr(), NVME_IDENTIFY_SIZE, fault_at)) {
                VERIFY_NOT_REACHED();
            }
            auto val = get_ns_features(namespace_data_struct);
            auto block_counts = val.get<0>();
            auto block_size = 1 << val.get<1>();

            dbgln_if(NVME_DEBUG, "NVMe: Block count is {} and Block size is {}", block_counts, block_size);

            m_namespaces.append(NVMENameSpace::create(m_queues, nsid, val.get<0>(), block_size));
            m_device_count++;
            dbgln_if(NVME_DEBUG, "NVMe: Initialized namespace with NSID: {}", nsid);
        }
    }
}
Tuple<u64, u8> NVMEController::get_ns_features(Array<u8, 4096>& identify_data_struct)
{
    auto flbas = identify_data_struct[26] & 0xf;
    // FIXME: Is there a better way of retreiving the information instead of casts?
    auto namespace_size = *reinterpret_cast<u64*>(identify_data_struct.data());
    auto lba_format = *reinterpret_cast<u32*>((identify_data_struct.data() + 128 + (4 * flbas)));

    auto lba_size = (lba_format & 0x00ff0000) >> 16;
    return Tuple<u64, u8>(namespace_size, lba_size);
}
RefPtr<StorageDevice> NVMEController::device(u32 index) const
{
    return m_namespaces.at(index);
}

size_t NVMEController::devices_count() const
{
    return m_device_count;
}
bool NVMEController::reset()
{
    TODO();
    return false;
}
bool NVMEController::shutdown()
{
    TODO();
    return false;
}
void NVMEController::complete_current_request([[maybe_unused]] AsyncDeviceRequest::RequestResult result)
{
    VERIFY_NOT_REACHED();
}
void NVMEController::test_rw_functionality()
{
    auto& ns = m_namespaces.at(0);
    ns.test_read();
}
void NVMEController::create_admin_queue(u8 irq)
{
    auto qdepth = get_admin_q_dept();
    OwnPtr<Memory::Region> cq_dma_region;
    RefPtr<Memory::PhysicalPage> cq_dma_page;
    OwnPtr<Memory::Region> sq_dma_region;
    RefPtr<Memory::PhysicalPage> sq_dma_page;
    auto cq_size = round_up_to_power_of_two(CQ_SIZE(qdepth), 4096);
    auto sq_size = round_up_to_power_of_two(SQ_SIZE(qdepth), 4096);
    reset_controller();

    if (auto buffer = dma_alloc_buffer(cq_size, "Admin CQ queue", Memory::Region::Access::ReadWrite, cq_dma_page); buffer.is_error()) {
        dmesgln("Failed to allocate memory for ADMIN CQ queue");
        VERIFY_NOT_REACHED();
    } else {
        cq_dma_region = buffer.release_value();
    }

    if (auto buffer = dma_alloc_buffer(sq_size, "Admin SQ queue", Memory::Region::Access::ReadWrite, sq_dma_page); buffer.is_error()) {
        dmesgln("Failed to allocate memory for ADMIN CQ queue");
        VERIFY_NOT_REACHED();
    } else {
        sq_dma_region = buffer.release_value();
    }
    m_admin_queue = (NVMEQueue::create(0, irq, qdepth, move(cq_dma_region), cq_dma_page, move(sq_dma_region), sq_dma_page, m_controller_regs));

    // TODO: Should this be written as little endian?
    write64_controller_regs(CC_ACQ, reinterpret_cast<u64>(cq_dma_page->paddr().as_ptr()));
    write64_controller_regs(CC_ASQ, reinterpret_cast<u64>(sq_dma_page->paddr().as_ptr()));

    start_controller();
    set_admin_queue_ready_flag();
    m_admin_queue->enable_interrupts();
    dbgln_if(NVME_DEBUG, "NVMe: Admin queue created");
}
void NVMEController::create_io_queue(u8 irq, u8 qid)
{
    nvme_submission sub {};
    OwnPtr<Memory::Region> cq_dma_region;
    RefPtr<Memory::PhysicalPage> cq_dma_page;
    OwnPtr<Memory::Region> sq_dma_region;
    RefPtr<Memory::PhysicalPage> sq_dma_page;
    auto cq_size = round_up_to_power_of_two(CQ_SIZE(IO_QUEUE_SIZE), 4096);
    auto sq_size = round_up_to_power_of_two(SQ_SIZE(IO_QUEUE_SIZE), 4096);

    VERIFY(sizeof(nvme_submission) == (1 << SQ_WIDTH));
    if (auto buffer = dma_alloc_buffer(cq_size, "IO CQ queue", Memory::Region::Access::ReadWrite, cq_dma_page); buffer.is_error()) {
        dmesgln("Failed to allocate memory for IO CQ queue");
        VERIFY_NOT_REACHED();
    } else {
        cq_dma_region = buffer.release_value();
    }
    // Phase bit is important to determine completion, so zero out the space
    // so that we don't get any garbage phase bit value
    memset(cq_dma_region->vaddr().as_ptr(), 0, cq_size);

    if (auto buffer = dma_alloc_buffer(sq_size, "IO SQ queue", Memory::Region::Access::ReadWrite, sq_dma_page); buffer.is_error()) {
        dmesgln("Failed to allocate memory for IO CQ queue");
        VERIFY_NOT_REACHED();
    } else {
        sq_dma_region = buffer.release_value();
    }

    {
        sub.op = OP_ADMIN_CREATE_COMPLETION_QUEUE;
        sub.data_ptr.prp1 = reinterpret_cast<u64>(AK::convert_between_host_and_little_endian(cq_dma_page->paddr().as_ptr()));
        // The queue size is 0 based
        sub.cdw10 = AK::convert_between_host_and_little_endian(((IO_QUEUE_SIZE - 1) << 16 | qid));
        auto flags = QUEUE_IRQ_ENABLED | QUEUE_PHY_CONTIGUOUS;
        // TODO: For now using pin based interrupts. Clear the first 16 bits
        // to use pin-based interrupts. NVMe spec 1.4, section 5.3
        sub.cdw11 = AK::convert_between_host_and_little_endian(flags & 0xFFFF);
        submit_admin_command(sub);
    }
    {
        sub.op = OP_ADMIN_CREATE_SUBMISSION_QUEUE;
        sub.data_ptr.prp1 = reinterpret_cast<u64>(AK::convert_between_host_and_little_endian(sq_dma_page->paddr().as_ptr()));
        // The queue size is 0 based
        sub.cdw10 = AK::convert_between_host_and_little_endian(((IO_QUEUE_SIZE - 1) << 16 | qid));
        auto flags = QUEUE_IRQ_ENABLED | QUEUE_PHY_CONTIGUOUS;
        // The qid used below points to the completion queue qid NVMe spec 1.4, section 5.4
        sub.cdw11 = AK::convert_between_host_and_little_endian(qid << 16 | flags);
        submit_admin_command(sub);
    }

    m_queues.append(NVMEQueue::create(qid, irq, IO_QUEUE_SIZE, move(cq_dma_region), cq_dma_page, move(sq_dma_region), sq_dma_page, m_controller_regs));
    // TODO: Interrupts should be enabled towards the end i.e after ns creation?
    m_queues.last().enable_interrupts();
    dbgln_if(NVME_DEBUG, "NVMe: Created IO Queue with QID{}", m_queues.size());
}
}
