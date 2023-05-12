/*
 * Copyright (c) 2021-2022, Liav A. <liavalb@hotmail.co.il>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <Kernel/Devices/Device.h>
#include <Kernel/Interrupts/IRQHandler.h>
#include <Kernel/Library/LockRefPtr.h>
#include <Kernel/Locking/Mutex.h>
#include <Kernel/Memory/PhysicalPage.h>
#include <Kernel/PhysicalAddress.h>
#include <Kernel/Random.h>
#include <Kernel/Sections.h>
#include <Kernel/Storage/ATA/AHCI/Controller.h>
#include <Kernel/Storage/ATA/AHCI/Port.h>
#include <Kernel/Storage/StorageDevice.h>
#include <Kernel/WaitQueue.h>

namespace Kernel {

class AsyncBlockDeviceRequest;

class AHCIController;
class AHCIPort;
class AHCIInterruptHandler final {
    friend class AHCIController;

public:
    static ErrorOr<NonnullOwnPtr<AHCIInterruptHandler>> create(AHCIController&, u8 irq, AHCI::MaskedBitField taken_ports);
    virtual ~AHCIInterruptHandler();

    bool is_responsible_for_port_index(u32 port_index) const { return m_taken_ports.is_set_at(port_index); }

private:
    AHCIInterruptHandler(AHCIController&, u8 irq, AHCI::MaskedBitField taken_ports);

    void allocate_resources_and_initialize_ports();

    //^ IRQHandler
    virtual bool handle_irq(RegisterState const&);

    enum class Direction : u8 {
        Read,
        Write,
    };

    AHCI::MaskedBitField create_pending_ports_interrupts_bitfield() const;

    // Data members
    NonnullLockRefPtr<AHCIController> m_parent_controller;
    AHCI::MaskedBitField m_taken_ports;
    AHCI::MaskedBitField m_pending_ports_interrupts;
    RefPtr<IRQHandler> m_interrupt_handler;
};
}
