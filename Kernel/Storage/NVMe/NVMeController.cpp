/*
 * Copyright (c) 2021, Pankaj R <pankydev8@gmail.com>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include "NVMeController.h"
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

ErrorOr<NonnullRefPtr<NVMeController>> NVMeController::try_initialize(const Kernel::PCI::DeviceIdentifier& device_identifier)
{
    auto controller = TRY(adopt_nonnull_ref_or_enomem(new NVMeController(device_identifier)));
    TRY(controller->initialize());
    return controller;
}

NVMeController::NVMeController(const PCI::DeviceIdentifier& device_identifier)
    : PCI::Device(device_identifier.address())
    , m_pci_device_id(device_identifier)
{
}

ErrorOr<void> NVMeController::initialize()
{
    // Nr of queues = one queue per core
    auto nr_of_queues = Processor::count();
    auto irq = m_pci_device_id.interrupt_line().value();

    PCI::enable_memory_space(m_pci_device_id.address());
    PCI::enable_bus_mastering(m_pci_device_id.address());
    auto bar = PCI::get_BAR0(m_pci_device_id.address()) & 0xFFFFFFF0;
    auto bar_len = round_up_to_power_of_two(CTRL_REG_SIZE(nr_of_queues), 4096);
    auto region_or_error = TRY(MM.allocate_kernel_region(PhysicalAddress(bar), bar_len, "PCI NVMe BAR", Memory::Region::Access::ReadWrite));
    // Queues need access to the doorbell registers, hence leaking the pointer
    m_controller_regs = region_or_error.leak_ptr();

    TRY(create_admin_queue(irq));
    VERIFY(m_admin_queue_ready == true);

    VERIFY(IO_QUEUE_SIZE < get_ctrl_regs<u16>(CC_CAP));
    dbgln_if(NVME_DEBUG, "NVMe: IO queue depth is: {}", IO_QUEUE_SIZE);

    // Create an IO queue per core
    for (u32 cpuid = 0; cpuid < nr_of_queues; ++cpuid) {
        // m_qid is zero is used for admin queue
        TRY(create_io_queue(irq, cpuid + 1));
    }
    TRY(identify_and_init_namespaces());
    return {};
}

// NVMe Spec 1.4: Section 7.3.2
bool NVMeController::reset_controller()
{
    volatile u32 cc, csts;
    csts = get_ctrl_regs<u32>(CSTS_REG);
    if ((csts & (1 << CSTS_RDY_BIT)) != 0x1)
        return false;

    cc = get_ctrl_regs<u32>(CC_REG);
    cc = cc & ~(1 << CC_EN_BIT);

    set_ctrl_regs(CC_REG, cc);

    IO::delay(10);
    full_memory_barrier();

    csts = get_ctrl_regs<u32>(CSTS_REG);
    if ((csts & (1 << CSTS_RDY_BIT)) != 0x0)
        return false;

    return true;
}

// NVMe Spec 1.4: Section 7.3.2
bool NVMeController::start_controller()
{
    volatile u32 cc, csts;
    csts = get_ctrl_regs<u32>(CSTS_REG);
    if ((csts & (1 << CSTS_RDY_BIT)) != 0x0)
        return false;

    cc = get_ctrl_regs<u32>(CC_REG);

    cc = cc | (1 << CC_EN_BIT);
    cc = cc | (CQ_WIDTH << CC_IOCQES_BIT);
    cc = cc | (SQ_WIDTH << CC_IOSQES_BIT);

    set_ctrl_regs(CC_REG, cc);

    IO::delay(10);
    full_memory_barrier();
    csts = get_ctrl_regs<u32>(CSTS_REG);
    if ((csts & (1 << CSTS_RDY_BIT)) != 0x1)
        return false;

    return true;
}

u32 NVMeController::get_admin_q_dept()
{
    u32 aqa = get_ctrl_regs<u32>(CC_AQA);
    // Queue depth is 0 based
    u32 q_depth = min(ACQ_SIZE(aqa), ASQ_SIZE(aqa)) + 1;
    dbgln_if(NVME_DEBUG, "NVMe: Admin queue depth is {}", q_depth);
    return q_depth;
}

ErrorOr<void> NVMeController::identify_and_init_namespaces()
{

    RefPtr<Memory::PhysicalPage> prp_dma_buffer;
    OwnPtr<Memory::Region> prp_dma_region;
    auto namespace_data_struct = ByteBuffer::create_zeroed(NVMe_IDENTIFY_SIZE).release_value();
    u32 active_namespace_list[NVMe_IDENTIFY_SIZE / sizeof(u32)];

    {
        auto buffer = TRY(MM.dma_allocate_buffer(NVMe_IDENTIFY_SIZE, "Identify PRP", Memory::Region::Access::ReadWrite, prp_dma_buffer));
        prp_dma_region = move(buffer);
    }

    // Get the active namespace
    {
        NVMeSubmission sub {};
        u16 status = 0;
        sub.op = OP_ADMIN_IDENTIFY;
        sub.data_ptr.prp1 = reinterpret_cast<u64>(AK::convert_between_host_and_little_endian(prp_dma_buffer->paddr().as_ptr()));
        sub.cdw10 = NVMe_CNS_ID_ACTIVE_NS & 0xff;
        status = submit_admin_command(sub, true);
        if (status) {
            dmesgln("Failed to identify active namespace command");
            return EFAULT;
        }
        if (void* fault_at; !safe_memcpy(active_namespace_list, prp_dma_region->vaddr().as_ptr(), NVMe_IDENTIFY_SIZE, fault_at)) {
            return EFAULT;
        }
    }
    // Get the NAMESPACE attributes
    {
        NVMeSubmission sub {};
        u16 status = 0;
        for (auto nsid : active_namespace_list) {
            memset(prp_dma_region->vaddr().as_ptr(), 0, NVMe_IDENTIFY_SIZE);
            // Invalid NS
            if (nsid == 0)
                break;
            sub.op = OP_ADMIN_IDENTIFY;
            sub.data_ptr.prp1 = reinterpret_cast<u64>(AK::convert_between_host_and_little_endian(prp_dma_buffer->paddr().as_ptr()));
            sub.cdw10 = NVMe_CNS_ID_NS & 0xff;
            sub.nsid = nsid;
            status = submit_admin_command(sub, true);
            if (status) {
                dmesgln("Failed identify namespace with nsid {}", nsid);
                return EFAULT;
            }
            if (void* fault_at; !safe_memcpy(namespace_data_struct.data(), prp_dma_region->vaddr().as_ptr(), NVMe_IDENTIFY_SIZE, fault_at)) {
                return EFAULT;
            }
            auto val = get_ns_features(namespace_data_struct);
            auto block_counts = val.get<0>();
            auto block_size = 1 << val.get<1>();

            dbgln_if(NVME_DEBUG, "NVMe: Block count is {} and Block size is {}", block_counts, block_size);

            m_namespaces.append(TRY(NVMeNameSpace::try_create(m_queues, nsid, val.get<0>(), block_size)));
            m_device_count++;
            dbgln_if(NVME_DEBUG, "NVMe: Initialized namespace with NSID: {}", nsid);
        }
    }
    return {};
}

Tuple<u64, u8> NVMeController::get_ns_features(ByteBuffer& identify_data_struct)
{
    auto flbas = identify_data_struct[FLBA_SIZE_INDEX] & FLBA_SIZE_MASK;
    auto namespace_size = *reinterpret_cast<u64*>(identify_data_struct.offset_pointer(0));
    auto lba_format = *reinterpret_cast<u32*>((identify_data_struct.offset_pointer(LBA_FORMAT_SUPPORT_INDEX + (4 * flbas))));

    auto lba_size = (lba_format & LBA_SIZE_MASK) >> 16;
    return Tuple<u64, u8>(namespace_size, lba_size);
}
RefPtr<StorageDevice> NVMeController::device(u32 index) const
{
    return m_namespaces.at(index);
}

size_t NVMeController::devices_count() const
{
    return m_device_count;
}

bool NVMeController::reset()
{
    if (!reset_controller())
        return false;
    if (!start_controller())
        return false;
    return true;
}

bool NVMeController::shutdown()
{
    TODO();
    return false;
}

void NVMeController::complete_current_request([[maybe_unused]] AsyncDeviceRequest::RequestResult result)
{
    VERIFY_NOT_REACHED();
}

ErrorOr<void> NVMeController::create_admin_queue(u8 irq)
{
    auto qdepth = get_admin_q_dept();
    OwnPtr<Memory::Region> cq_dma_region;
    RefPtr<Memory::PhysicalPage> cq_dma_page;
    OwnPtr<Memory::Region> sq_dma_region;
    RefPtr<Memory::PhysicalPage> sq_dma_page;
    auto cq_size = round_up_to_power_of_two(CQ_SIZE(qdepth), 4096);
    auto sq_size = round_up_to_power_of_two(SQ_SIZE(qdepth), 4096);
    if (!reset_controller()) {
        dmesgln("Failed to reset the NVMe controller");
        return EFAULT;
    }
    {
        auto buffer = TRY(MM.dma_allocate_buffer(cq_size, "Admin CQ queue", Memory::Region::Access::ReadWrite, cq_dma_page));
        cq_dma_region = move(buffer);
    }

    // Phase bit is important to determine completion, so zero out the space
    // so that we don't get any garbage phase bit value
    memset(cq_dma_region->vaddr().as_ptr(), 0, cq_size);

    {
        auto buffer = TRY(MM.dma_allocate_buffer(sq_size, "Admin SQ queue", Memory::Region::Access::ReadWrite, sq_dma_page));
        sq_dma_region = move(buffer);
    }

    m_admin_queue = TRY(NVMeQueue::try_create(0, irq, qdepth, move(cq_dma_region), cq_dma_page, move(sq_dma_region), sq_dma_page, m_controller_regs));

    write64_controller_regs(CC_ACQ, reinterpret_cast<u64>(AK::convert_between_host_and_little_endian(cq_dma_page->paddr().as_ptr())));
    write64_controller_regs(CC_ASQ, reinterpret_cast<u64>(AK::convert_between_host_and_little_endian(sq_dma_page->paddr().as_ptr())));

    if (!start_controller()) {
        dmesgln("Failed to restart the NVMe controller");
        return EFAULT;
    }
    set_admin_queue_ready_flag();
    m_admin_queue->enable_interrupts();
    dbgln_if(NVME_DEBUG, "NVMe: Admin queue created");
    return {};
}

ErrorOr<void> NVMeController::create_io_queue(u8 irq, u8 qid)
{
    NVMeSubmission sub {};
    OwnPtr<Memory::Region> cq_dma_region;
    RefPtr<Memory::PhysicalPage> cq_dma_page;
    OwnPtr<Memory::Region> sq_dma_region;
    RefPtr<Memory::PhysicalPage> sq_dma_page;
    auto cq_size = round_up_to_power_of_two(CQ_SIZE(IO_QUEUE_SIZE), 4096);
    auto sq_size = round_up_to_power_of_two(SQ_SIZE(IO_QUEUE_SIZE), 4096);

    static_assert(sizeof(NVMeSubmission) == (1 << SQ_WIDTH));

    {
        auto buffer = TRY(MM.dma_allocate_buffer(cq_size, "IO CQ queue", Memory::Region::Access::ReadWrite, cq_dma_page));
        cq_dma_region = move(buffer);
    }

    // Phase bit is important to determine completion, so zero out the space
    // so that we don't get any garbage phase bit value
    memset(cq_dma_region->vaddr().as_ptr(), 0, cq_size);

    {
        auto buffer = TRY(MM.dma_allocate_buffer(sq_size, "IO SQ queue", Memory::Region::Access::ReadWrite, sq_dma_page));
        sq_dma_region = move(buffer);
    }

    {
        sub.op = OP_ADMIN_CREATE_COMPLETION_QUEUE;
        sub.data_ptr.prp1 = reinterpret_cast<u64>(AK::convert_between_host_and_little_endian(cq_dma_page->paddr().as_ptr()));
        // The queue size is 0 based
        sub.cdw10 = AK::convert_between_host_and_little_endian(((IO_QUEUE_SIZE - 1) << 16 | qid));
        auto flags = QUEUE_IRQ_ENABLED | QUEUE_PHY_CONTIGUOUS;
        // TODO: Eventually move to MSI.
        // For now using pin based interrupts. Clear the first 16 bits
        // to use pin-based interrupts. NVMe spec 1.4, section 5.3.
        sub.cdw11 = AK::convert_between_host_and_little_endian(flags & 0xFFFF);
        submit_admin_command(sub, true);
    }
    {
        sub.op = OP_ADMIN_CREATE_SUBMISSION_QUEUE;
        sub.data_ptr.prp1 = reinterpret_cast<u64>(AK::convert_between_host_and_little_endian(sq_dma_page->paddr().as_ptr()));
        // The queue size is 0 based
        sub.cdw10 = AK::convert_between_host_and_little_endian(((IO_QUEUE_SIZE - 1) << 16 | qid));
        auto flags = QUEUE_IRQ_ENABLED | QUEUE_PHY_CONTIGUOUS;
        // The m_qid used below points to the completion queue m_qid NVMe spec 1.4, section 5.4
        sub.cdw11 = AK::convert_between_host_and_little_endian(qid << 16 | flags);
        submit_admin_command(sub, true);
    }

    m_queues.append(TRY(NVMeQueue::try_create(qid, irq, IO_QUEUE_SIZE, move(cq_dma_region), cq_dma_page, move(sq_dma_region), sq_dma_page, m_controller_regs)));
    m_queues.last().enable_interrupts();
    dbgln_if(NVME_DEBUG, "NVMe: Created IO Queue with QID{}", m_queues.size());
    return {};
}
}
