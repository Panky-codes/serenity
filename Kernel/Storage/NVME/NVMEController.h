/*
 * Copyright (c) 2021, Pankaj Raghav <pankydev8@gmail.com>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#pragma once

#include <AK/NonnullRefPtrVector.h>
#include <AK/OwnPtr.h>
#include <AK/RefPtr.h>
#include <AK/Tuple.h>
#include <AK/Types.h>
#include <Kernel/Bus/PCI/Device.h>
#include <Kernel/Locking/Spinlock.h>
#include <Kernel/Storage/NVME/NVMEDefinitions.h>
#include <Kernel/Storage/NVME/NVMENameSpace.h>
#include <Kernel/Storage/NVME/NVMEQueue.h>
#include <Kernel/Storage/StorageController.h>

namespace Kernel {
// TODO: Change this to Storage controller later
class NonsenseBaseClass : public AK::RefCounted<NonsenseBaseClass> {
};

class NVMEController : public PCI::Device
    , public NonsenseBaseClass {
    AK_MAKE_ETERNAL
public:
    static NonnullRefPtr<NVMEController> initialize(PCI::DeviceIdentifier const&);
    explicit NVMEController(PCI::DeviceIdentifier const&);

    void reset_controller();
    void start_controller();
    u32 get_admin_q_dept();
    void write64_controller_regs(u32 offset, u64 value) { *reinterpret_cast<u64*>(m_controller_regs->vaddr().as_ptr() + offset) = value; };
    void update_cq_doorbell(u16 cq_head, u16 qid)
    {
        u32 addr = REG_SQ0TDBL_START + ((2 * qid + 1) * (4 << CAP_DSTRD));
        *reinterpret_cast<u16*>(m_controller_regs->vaddr().as_ptr() + addr) = cq_head;
    }

    void update_sq_doorbell(u16 sq_head, u16 qid)
    {
        u32 addr = REG_SQ0TDBL_START + ((2 * qid) * (4 << CAP_DSTRD));
        *reinterpret_cast<u16*>(m_controller_regs->vaddr().as_ptr() + addr) = sq_head;
    }
    u16 submit_admin_command(struct nvme_submission& sub, bool sync = false)
    {
        // First queue is always the admin queue
        auto& admin_queue = m_queues.first();
        if (sync) {
            return admin_queue.submit_sync_sqe(sub);
        }
        admin_queue.submit_sqe(sub);
        return 0;
    }

    bool is_admin_queue_ready() { return m_admin_queue_ready; };
    void set_admin_queue_ready_flag() { m_admin_queue_ready = true; };

private:
    void identify_and_init_namespaces();
    Tuple<u64, u8> get_ns_features(Array<u8, NVME_IDENTIFY_SIZE>& identify_data_struct);

private:
    PCI::DeviceIdentifier m_pci_device_id;
    NonnullRefPtrVector<NVMEQueue> m_queues;
    NonnullRefPtrVector<NVMENameSpace> m_namespaces;
    OwnPtr<Memory::Region> m_controller_regs;
    bool m_admin_queue_ready;
};
}
