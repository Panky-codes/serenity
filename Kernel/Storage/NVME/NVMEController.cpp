/*
 * Copyright (c) 2021, Pankaj Raghav <pankydev8@gmail.com>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#include "NVMEController.h"
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
    // Nr of queues = one queue per core + one admin queue
    auto nr_of_queues = Processor::count() + 1;
    PCI::enable_memory_space(device_identifier.address());
    PCI::enable_bus_mastering(device_identifier.address());
    auto bar = PCI::get_BAR0(device_identifier.address()) & 0xFFFFFFF0;
    auto bar_len = round_up_to_power_of_two(CTRL_REG_SIZE(nr_of_queues), 4096);
    auto region_or_error = MM.allocate_kernel_region(PhysicalAddress(bar), bar_len, "PCI NVMe BAR", Memory::Region::Access::ReadWrite);
    if (region_or_error.is_error())
        dbgln("Failed to map NVMe BAR");
    // Queues need access to the doorbell registers, hence leaking the pointer
    m_controller_regs = region_or_error.release_value().leak_ptr();
    void* ptr = m_controller_regs->vaddr().as_ptr();
    VERIFY((*static_cast<u16*>(ptr)) == 2047); // TODO: Remove this

    // TODO: For now determining admin queue based on qid. Create a enum flag
    create_admin_queue(device_identifier.interrupt_line().value());
//        for (u32 cpuid = 0; cpuid < nr_of_queues; ++cpuid) {
//            if (cpuid != 0) {
//                VERIFY(m_admin_queue_ready == true);
//            }
//            m_queues.append(NVMEQueue::create(cpuid, device_identifier.interrupt_line().value()));
//        }
    //    VERIFY(m_admin_queue_ready == true);
    //    // Identify Namespace attributes
    //    identify_and_init_namespaces();
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
    //    u8 namespace_data_struct[NVME_IDENTIFY_SIZE];

    if (auto buffer = dma_alloc_buffer(NVME_IDENTIFY_SIZE, "Admin CQ queue", Memory::Region::Access::ReadWrite, prp_dma_buffer); buffer.is_error()) {
        dmesgln("Failed o allocate memory for ADMIN CQ queue");
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
        for (auto ns : active_namespace_list) {
            memset(prp_dma_region->vaddr().as_ptr(), 0, NVME_IDENTIFY_SIZE);
            // Invalid NS
            if (ns == 0)
                break;
            sub.op = OP_ADMIN_IDENTIFY;
            sub.data_ptr.prp1 = reinterpret_cast<u64>(AK::convert_between_host_and_little_endian(prp_dma_buffer->paddr().as_ptr()));
            sub.cdw10 = NVME_CNS_ID_NS & 0xff;
            sub.nsid = ns;
            status = submit_admin_command(sub, true);
            if (status) {
                dmesgln("Failed identify active namespace command");
                VERIFY_NOT_REACHED();
            }
            if (void* fault_at; !safe_memcpy(namespace_data_struct.data(), prp_dma_region->vaddr().as_ptr(), NVME_IDENTIFY_SIZE, fault_at)) {
                VERIFY_NOT_REACHED();
            }
            auto val = get_ns_features(namespace_data_struct);
            auto lba = 1 << val.get<1>();
            dbgln("Nsize is {} and lba is {}", val.get<0>(), lba);

            m_namespaces.append(NVMENameSpace::create(m_queues, ns, val.get<0>(), lba));
            m_device_count++;
        }
    }
    // Get the device attributes of the active namespaces & init namespace
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
    ns.test_rw();
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
    m_admin_queue  = (NVMEQueue::create(0, irq, move(cq_dma_region), cq_dma_page, move(sq_dma_region), sq_dma_page, m_controller_regs));

    // TODO: Should this be written as little endian?
    write64_controller_regs(CC_ACQ, reinterpret_cast<u64>(cq_dma_page->paddr().as_ptr()));
    write64_controller_regs(CC_ASQ, reinterpret_cast<u64>(sq_dma_page->paddr().as_ptr()));

    start_controller();
    set_admin_queue_ready_flag();
    m_admin_queue->enable_interrupts();
}
}