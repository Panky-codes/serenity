/*
 * Copyright (c) 2020, Liav A. <liavalb@hotmail.co.il>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <AK/AnyOf.h>
#include <Kernel/Arch/Interrupts.h>
#include <Kernel/Bus/PCI/API.h>
#include <Kernel/Bus/PCI/Device.h>
#include <Kernel/Memory/TypedMapping.h>

namespace Kernel::PCI {

Device::Device(DeviceIdentifier const& pci_identifier)
    : m_pci_identifier(pci_identifier)
{
    m_pci_identifier->initialize();
    m_interrupts.m_irq = m_pci_identifier->interrupt_line().value();
    m_interrupts.m_nr_of_irqs = 1;
}

bool Device::is_msi_capable() const
{
    return AK::any_of(
        m_pci_identifier->capabilities(),
        [](auto const& capability) { return capability.id().value() == PCI::Capabilities::ID::MSI; });
}
bool Device::is_msix_capable() const
{
    return m_pci_identifier->is_msix_capable();
}

void Device::enable_pin_based_interrupts() const
{
    PCI::enable_interrupt_line(m_pci_identifier);
}
void Device::disable_pin_based_interrupts() const
{
    PCI::disable_interrupt_line(m_pci_identifier);
}

void Device::enable_message_signalled_interrupts()
{
    TODO();
}
void Device::disable_message_signalled_interrupts()
{
    TODO();
}
void Device::enable_extended_message_signalled_interrupts()
{
    for (auto& capability : m_pci_identifier->capabilities()) {
        if (capability.id().value() == PCI::Capabilities::ID::MSIX) {
            capability.write16(2, capability.read16(2) | (1 << 15));
        }
    }
}
void Device::disable_extended_message_signalled_interrupts()
{
    TODO();
}

// Returns numbers of irqs reserved for this device. zero is returned
// if only pin based interrupts was available
ErrorOr<InterruptType> Device::reserve_irqs(u8 number_of_irqs)
{
    // Let us not allow partial allocation of irqs for MSIx.
    if (is_msix_capable()) {
        m_interrupts.m_irq = TRY(reserve_interrupt_handler(number_of_irqs));
        m_interrupts.m_nr_of_irqs = number_of_irqs;
        m_interrupts.m_type = InterruptType::MSIX;
        // If MSIx is available, disable the pin based interrupts
        disable_pin_based_interrupts();
        enable_extended_message_signalled_interrupts();
    }
    // Only pin based interrupts are supported
    return m_interrupts.m_type;
}

// This function is used to allocate an irq at an index and returns
// the actual IRQ that was programmed at that index. This function is
// mainly useful for MSI/MSIx based interrupt mechanism where the driver
// needs to program. If the PCI device doesn't support MSIx interrupts, then
// this function will just return the irq used for pin based interrupt
ErrorOr<u8> Device::allocate_irq(u8 index)
{
    if (is_msix_capable()) {
        VERIFY(index < m_interrupts.m_nr_of_irqs);
        //auto table_bar_ptr = PCI::get_BAR(device_identifier(), static_cast<PCI::HeaderType0BaseRegister>(m_pci_identifier->get_msix_table_bar()));
        auto table_bar_ptr = PCI::get_BAR0(device_identifier()) & 0xFFFFFFF0;
        auto table_offset = m_pci_identifier->get_msix_table_offset();
        auto entry_ptr = TRY(Memory::map_typed_writable<msix_table_entry volatile>(PhysicalAddress(table_bar_ptr + table_offset + (index * 16))));

        entry_ptr->data = m_interrupts.m_irq + index + IRQ_VECTOR_BASE;
        // TODO BEFORE PR: Move this to arch specific code
        // TODO: we map all the IRQs to cpu 0 by default. We could attach
        // cpu affinity in the future where specific LAPIC can id can be used.
        u64 addr = 0xfee00000 | (0 << 12);
        entry_ptr->addr_low = addr & 0xffffffff;
        entry_ptr->addr_high = addr >> 32;
        u32 vector_ctrl = entry_ptr->vector_ctlr;
        vector_ctrl &= ~(0x0001);
        entry_ptr->vector_ctlr = vector_ctrl;
        return m_interrupts.m_irq + index;
    }
    // For pin based interrupts, we share the irq
    return m_interrupts.m_irq;
}

}
