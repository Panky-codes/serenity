/*
 * Copyright (c) 2018-2020, Andreas Kling <kling@serenityos.org>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <Kernel/Arch/InterruptManagement.h>
#include <Kernel/Arch/x86_64/Interrupts/APIC.h>
#include <Kernel/Debug.h>
#include <Kernel/InterruptDisabler.h>
#include <Kernel/Interrupts/MSI.h>

namespace Kernel {

MSIHandler::MSIHandler(u8 irq)
    : GenericInterruptHandler(irq)
{
    register_interrupt_handler();
}

MSIHandler::~MSIHandler() = default;

bool MSIHandler::eoi()
{
    // FIXME: Maybe some sanity checks on irq?
    InterruptDisabler disabler;
    APIC::the().eoi();
    return true;
}

}
