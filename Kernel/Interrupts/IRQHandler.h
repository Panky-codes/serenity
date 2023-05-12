/*
 * Copyright (c) 2018-2020, Andreas Kling <kling@serenityos.org>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <AK/RefPtr.h>
#include <AK/Types.h>
#include <Kernel/Arch/IRQController.h>
#include <Kernel/Interrupts/GenericInterruptHandler.h>
#include <Kernel/Library/LockRefPtr.h>

namespace Kernel {

class IRQHandler final : public GenericInterruptHandler
    , public AtomicRefCounted<IRQHandler> {
private:
    using CallbackType = AK::Function<bool(RegisterState const&)>;

public:
    virtual ~IRQHandler();

    virtual bool handle_interrupt(RegisterState const& regs) override { return m_callback(regs); }
    //    virtual bool handle_irq(RegisterState const&);

    void enable_irq();
    void disable_irq();

    virtual bool eoi() override;

    virtual HandlerType type() const override { return HandlerType::IRQHandler; }
    virtual StringView purpose() const override { return m_purpose; }
    virtual StringView controller() const override { return m_responsible_irq_controller->model(); }

    virtual size_t sharing_devices_count() const override { return 0; }
    virtual bool is_shared_handler() const override { return false; }
    void set_shared_with_others(bool status) { m_shared_with_others = status; }
    explicit IRQHandler(u8 irq);
    IRQHandler(u8 irq, CallbackType callback, StringView purpose);

protected:
    void change_irq_number(u8 irq);

private:
    bool m_shared_with_others { false };
    bool m_enabled { false };
    NonnullLockRefPtr<IRQController> m_responsible_irq_controller;
    CallbackType m_callback { nullptr };
    StringView m_purpose { "IRQ Handler"sv };
};

}
