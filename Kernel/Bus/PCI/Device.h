/*
 * Copyright (c) 2020, Liav A. <liavalb@hotmail.co.il>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <AK/Format.h>
#include <AK/NonnullRefPtr.h>
#include <AK/StringBuilder.h>
#include <AK/Tuple.h>
#include <AK/Types.h>
#include <Kernel/Bus/PCI/Definitions.h>

namespace Kernel::PCI {

class Device {
public:
    DeviceIdentifier const& device_identifier() const { return *m_pci_identifier; };

    virtual ~Device() = default;

    virtual StringView device_name() const = 0;

    void enable_pin_based_interrupts() const;
    void disable_pin_based_interrupts() const;

    bool is_msi_capable() const;
    bool is_msix_capable();

    void enable_msi();
    void disable_msi();

    void enable_msix();
    void disable_msix();
    u16 count_msix_interrupts() { return msix_count; }
    void get_msix_bir_and_offset(u32* off, u8* bar)
    {
        *off = msix_bir_offset;
        *bar = msix_bir_bar;
    };

protected:
    explicit Device(DeviceIdentifier const& pci_identifier);

private:
    NonnullRefPtr<DeviceIdentifier const> m_pci_identifier;
    u32 msix_bir_offset = 0;
    u8 msix_bir_bar = 0;
    u32 msix_count = 0;
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
