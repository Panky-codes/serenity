/*
 * Copyright (c) 2021, Pankaj Raghav <pankydev8@gmail.com>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#include <AK/RefPtr.h>
#include <AK/Types.h>
#include <Kernel/Arch/x86/IO.h>
#include <Kernel/Arch/x86/Processor.h>
#include <Kernel/Bus/PCI/API.h>
#include <Kernel/FileSystem/ProcFS.h>
#include <Kernel/Sections.h>
#include <Kernel/Storage/NVME/NVMEController.h>

namespace Kernel {

NonnullRefPtr<NVMEController> NVMEController::initialize(const Kernel::PCI::DeviceIdentifier& device_identifier)
{
    return adopt_ref(*new NVMEController(device_identifier));
}
NVMEController::NVMEController(const PCI::DeviceIdentifier& device_identifier)
    : PCI::Device(device_identifier.address())
    , m_pci_device_id(device_identifier)
    , m_admin_queue_ready(false)
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
    m_controller_regs = region_or_error.release_value();
    void* ptr = m_controller_regs->vaddr().as_ptr();
    VERIFY((*static_cast<u16*>(ptr)) == 2047); // TODO: Remove this

    // Need to create one extra queue for admin
    // TODO: For now determining admin queue based on qid. Create a enum flag
    for (u32 cpuid = 0; cpuid < nr_of_queues; ++cpuid) {
        if (cpuid != 0) {
            VERIFY(m_admin_queue_ready == true);
        }
        m_queues.append(NVMEQueue::create(*this, cpuid, device_identifier.interrupt_line().value()));
    }
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
}