/*
 * Copyright (c) 2020, Liav A. <liavalb@hotmail.co.il>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <AK/Format.h>
#include <AK/NonnullRefPtr.h>
#include <AK/StringBuilder.h>
#include <AK/Types.h>
#include <Kernel/Bus/PCI/Definitions.h>

namespace Kernel::PCI {

enum class InterruptType {
    PIN,
    MSIX
};

struct Interrupts {
    u8 m_irq { 0 };
    u8 m_nr_of_irqs { 0 };
    InterruptType m_type { InterruptType::PIN };
};

struct [[gnu::packed]] msix_table_entry {
    u32 addr_low;
    u32 addr_high;
    u32 data;
    u32 vector_ctlr;
};

class Device {
public:
    DeviceIdentifier const& device_identifier() const { return *m_pci_identifier; };

    virtual ~Device() = default;

    virtual StringView device_name() const = 0;

    void enable_pin_based_interrupts() const;
    void disable_pin_based_interrupts() const;

    bool is_msi_capable() const;
    bool is_msix_capable() const;

    void enable_message_signalled_interrupts();
    void disable_message_signalled_interrupts();

    void enable_extended_message_signalled_interrupts();
    void disable_extended_message_signalled_interrupts();
    ErrorOr<InterruptType> reserve_irqs(u8 number_of_irqs);
    ErrorOr<u8> allocate_irq(u8 index);

protected:
    explicit Device(DeviceIdentifier const& pci_identifier);

private:
    NonnullRefPtr<DeviceIdentifier const> const m_pci_identifier;
    Interrupts m_interrupts;
};

template<typename... Parameters>
void dmesgln_pci(Device const& device, AK::CheckedFormatString<Parameters...>&& fmt, Parameters const&... parameters)
{
    AK::StringBuilder builder;
    if (builder.try_append("{}: {}: "sv).is_error())
        return;
    if (builder.try_append(fmt.view()).is_error())
        return;
    AK::VariadicFormatParams<AK::AllowDebugOnlyFormatters::Yes, StringView, Address, Parameters...> variadic_format_params { device.device_name(), device.device_identifier().address(), parameters... };
    vdmesgln(builder.string_view(), variadic_format_params);
}

}
