/*
 * Copyright (c) 2020, Liav A. <liavalb@hotmail.co.il>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <AK/AnyOf.h>
#include <AK/Error.h>
#include <AK/NonnullRefPtr.h>
#include <AK/RefPtr.h>
#include <Kernel/Bus/PCI/Definitions.h>

namespace Kernel::PCI {

ErrorOr<NonnullRefPtr<DeviceIdentifier>> DeviceIdentifier::from_enumerable_identifier(EnumerableDeviceIdentifier const& other_identifier)
{
    return adopt_nonnull_ref_or_enomem(new (nothrow) DeviceIdentifier(other_identifier));
}

void DeviceIdentifier::initialize() const
{
    for (auto cap : capabilities()) {
        if (cap.id() == PCI::Capabilities::ID::MSIX) {
            auto msix_bir_bar = (cap.read8(4) & 0x7);
            auto msix_bir_offset = (cap.read32(4) & 0xfff8);
            auto msix_count = (cap.read16(2) & 0x7FF) + 1;
            dbgln("MSIx: bar {} offset{} count{}", msix_bir_bar, msix_bir_offset, msix_count);
            m_msix_info = MSIxInfo(cap, msix_count, msix_bir_bar, msix_bir_offset);
        }
    }
}

}
