/*
 * Copyright (c) 2021, Pankaj Raghav <pankydev8@gmail.com>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#pragma once

#include <AK/NonnullRefPtrVector.h>
#include <AK/OwnPtr.h>
#include <AK/RefPtr.h>
#include <AK/Types.h>
#include <Kernel/Bus/PCI/Device.h>
#include <Kernel/Locking/Spinlock.h>
#include <Kernel/Storage/StorageController.h>
#include <Kernel/Storage/NVME/NVMEQueue.h>
#include <Kernel/Storage/NVME/NVMEDefinitions.h>


namespace Kernel {
//TODO: Change this to Storage controller later
class NonsenseBaseClass : public AK::RefCounted<NonsenseBaseClass>
{};

class NVMEController : public PCI::Device
    , public NonsenseBaseClass {
      AK_MAKE_ETERNAL
public:
    static NonnullRefPtr<NVMEController> initialize(PCI::DeviceIdentifier const&);
    NVMEController(PCI::DeviceIdentifier const&);

private:
    PCI::DeviceIdentifier m_pci_device_id;
    NonnullRefPtrVector<NVMEQueue> m_queues;
    OwnPtr<Memory::Region> m_controller_regs;
};

}
