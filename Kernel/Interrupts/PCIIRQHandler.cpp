/*
 * Copyright (c) 2018-2020, Andreas Kling <kling@serenityos.org>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <Kernel/Arch/InterruptManagement.h>
#include <Kernel/Debug.h>
#include <Kernel/Interrupts/PCIIRQHandler.h>
#include <Kernel/Arch/PCIMSI.h>


namespace Kernel {

PCIIRQHandler::PCIIRQHandler(PCI::Device& device, u8 irq)
    : GenericInterruptHandler(irq)
    , device(device)
{
    auto type = device.get_interrupt_type();

    if (type == PCI::InterruptType::PIN)
        m_responsible_irq_controller = (InterruptManagement::the().get_responsible_irq_controller(irq));
    else
        m_message_signalled_interrupts = true;

    if (is_registered())
        disable_irq();
}

PCIIRQHandler::~PCIIRQHandler() = default;

bool PCIIRQHandler::eoi()
{
    dbgln_if(IRQ_DEBUG, "EOI IRQ {}", interrupt_number());
    if (!m_shared_with_others) {
        if (!m_message_signalled_interrupts)
            m_responsible_irq_controller->eoi(*this);
        else
            msi_signal_eoi();
        return true;
    }
    return false;
}

void PCIIRQHandler::enable_irq()
{
    dbgln_if(IRQ_DEBUG, "Enable IRQ {}", interrupt_number());
    if (!is_registered())
        register_interrupt_handler();
    m_enabled = true;
    if (!m_shared_with_others) {
        if (!m_message_signalled_interrupts)
            m_responsible_irq_controller->enable(*this);
        else
            device.enable_interrupt(interrupt_number());
    }
}

void PCIIRQHandler::disable_irq()
{
    dbgln_if(IRQ_DEBUG, "Disable IRQ {}", interrupt_number());
    m_enabled = false;

    if (!m_shared_with_others) {
        if (!m_message_signalled_interrupts)
            m_responsible_irq_controller->disable(*this);
        else
            device.disable_interrupt(interrupt_number());
    }
}

}
