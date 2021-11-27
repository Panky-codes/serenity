/*
 * Copyright (c) 2021, Pankaj Raghav <pankydev8@gmail.com>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#include <AK/RefPtr.h>
#include <AK/Types.h>
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
{
    PCI::enable_memory_space(device_identifier.address());
    auto bar = PCI::get_BAR0(device_identifier.address()) & 0xFFFFFFF0;
    dbgln("BAR0 value: {}", bar);
    auto region_or_error = MM.allocate_kernel_region(PhysicalAddress(bar), 0x1000, "PCI NVMe BAR", Memory::Region::Access::ReadWrite);
    if (region_or_error.is_error())
        dbgln("Failed to map NVMe BAR");
    m_controller_regs = region_or_error.release_value();
    void* ptr = m_controller_regs->vaddr().as_ptr();
    VERIFY((*static_cast<u16*>(ptr)) == 2047); // TODO: Remove this

    auto nr_of_cpus = Processor::count();
    // Need to create one extra queue for admin
    for (u32 cpuid = 0; cpuid < nr_of_cpus + 1; ++cpuid)
        m_queues.append(NVMEQueue::create(*this, cpuid));
}
}