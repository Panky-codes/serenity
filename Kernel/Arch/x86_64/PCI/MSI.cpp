/*
 * Copyright (c) 2023, Pankaj R <dev@pankajraghav.com>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <Kernel/Arch/Interrupts.h>
#include <Kernel/Arch/PCIMSI.h>
#include <Kernel/Arch/x86_64/PCI/MSI.h>
#include <Kernel/InterruptDisabler.h>
#include <Kernel/Arch/x86_64/Interrupts/APIC.h>


namespace Kernel {
u64 msi_address_register(u8 destination_id, bool redirection_hint, bool destination_mode)
{
    u64 flags = 0;
    if (redirection_hint) {
        flags |= MSI_REDIRECTION_HINT;
        if (destination_mode)
            flags |= MSI_DESTINATION_MODE_LOGICAL;
    }
    return (MSI_ADDRESS_BASE | (destination_id << MSI_DESTINATION_SHIFT) | flags);
}

u32 msi_data_register(u8 vector, [[maybe_unused]] bool level_trigger, [[maybe_unused]] bool assert)
{
    u32 flags = 0;

    if (level_trigger) {
        flags |= MSI_TRIGGER_MODE_LEVEL;
        if (assert)
            flags |= MSI_LEVEL_ASSERT;
    }
    return ((vector + IRQ_VECTOR_BASE) & MSI_DATA_VECTOR_MASK) | flags;
}

u32 msi_vector_control_register(u32 vector_control, bool mask)
{
    if (!mask)
        return (vector_control & ~(0x0001));
    return (vector_control | 0x0001);
}
void msi_signal_eoi() {
    InterruptDisabler disabler;
    APIC::the().eoi();
}


}
