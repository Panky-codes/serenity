/*
* Copyright (c) 2023, Pankaj R <dev@pankajraghav.com>
*
* SPDX-License-Identifier: BSD-2-Clause
*/

#pragma once

#include <AK/Types.h>
#include <Kernel/Arch/IRQController.h>
#include <Kernel/Interrupts/GenericInterruptHandler.h>

namespace Kernel {

class MSIHandler : public GenericInterruptHandler {
public:
   virtual ~MSIHandler();

   virtual bool handle_interrupt(RegisterState const& regs) override { return handle_irq(regs); }
   virtual bool handle_irq(RegisterState const&) = 0;

   virtual bool eoi() override;

   virtual HandlerType type() const override { return HandlerType::IRQHandler; }
   virtual StringView purpose() const override { return "IRQ Handler"sv; }
   virtual StringView controller() const override { return "LAPIC"sv; }

   virtual size_t sharing_devices_count() const override { return 0; }
   virtual bool is_shared_handler() const override { return false; }
   virtual bool is_sharing_with_others() const override { return m_shared_with_others; }

protected:
   explicit MSIHandler(u8 irq);

private:
   bool m_shared_with_others { false };
   bool m_enabled { false };
};

}
