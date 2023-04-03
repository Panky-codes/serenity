/*
 * Copyright (c) 2020, Liav A. <liavalb@hotmail.co.il>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <AK/AnyOf.h>
#include <Kernel/Bus/PCI/API.h>
#include <Kernel/Bus/PCI/Device.h>

namespace Kernel::PCI {

Device::Device(DeviceIdentifier const& pci_identifier)
    : m_pci_identifier(pci_identifier)
{
}

bool Device::is_msi_capable() const
{
    return AK::any_of(
        m_pci_identifier->capabilities(),
        [](auto const& capability) {
        if (capability.id().value() == PCI::Capabilities::ID::MSI) {
            dbgln("PCI MSI message control: {}", (capability.read32(0) >> 16));
            return true;
        }
        return false ; });
}
bool Device::is_msix_capable()
{
    for (auto& capability : m_pci_identifier->capabilities()) {
        if (capability.id().value() == PCI::Capabilities::ID::MSIX) {
            msix_bir_bar = (capability.read8(4) & 0x7);
            msix_bir_offset = (capability.read32(4) & 0xfff8);
            msix_count = (capability.read16(2) & 0x7FF) + 1;
            return true;
        }
    }
    return false;
}

void Device::enable_pin_based_interrupts() const
{
    PCI::enable_interrupt_line(m_pci_identifier);
}
void Device::disable_pin_based_interrupts() const
{
    PCI::disable_interrupt_line(m_pci_identifier);
}

void Device::enable_msi()
{
    TODO();
}
void Device::disable_msi()
{
    TODO();
}
void Device::enable_msix()
{
    for (auto& capability : m_pci_identifier->capabilities()) {
        if (capability.id().value() == PCI::Capabilities::ID::MSIX) {
            capability.write16(2, capability.read16(2) | (1 << 15));
        }
    }
}
void Device::disable_msix()
{
    TODO();
}
}
