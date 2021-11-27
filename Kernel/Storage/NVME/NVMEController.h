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
//#include <Kernel/Storage/NVME/NVMEQueue.h>

namespace Kernel {
class NVMEController;

class NVMEQueue : public RefCounted<NVMEQueue> {
public:
    static NonnullRefPtr<NVMEQueue> create(const NVMEController&, u16 qid);
    explicit NVMEQueue(const NVMEController& controller, u16 qid)
        : qid(qid)
        , cq_valid_phase(1)
        , sq_tail(0)
        , cq_head(0)
        , m_admin_queue(qid == 0)
        , m_controller(controller)
    {
    }
    bool is_admin_queue() { return m_admin_queue; };

private:
    u16 qid;
    u8 cq_valid_phase;
    u16 sq_tail;
    u16 cq_head;
    bool m_admin_queue;
    Spinlock m_cq_lock;
    Spinlock m_sq_lock;
    NonnullRefPtr<NVMEController> m_controller;
};
struct nvme_completion {
    LittleEndian<u32> cmd_spec;
    LittleEndian<u32> res;

    LittleEndian<u16> sq_head; /* how much of this queue may be reclaimed */
    LittleEndian<u16> sq_id;   /* submission queue that generated this entry */

    u16 command_id;           /* of the command which completed */
    LittleEndian<u16> status; /* did the command fail, and if so, why? */
};

struct data_ptr_t {
    LittleEndian<u64> prp1;
    LittleEndian<u64> prp2;
};

struct nvme_submission {
    LittleEndian<u8> op;
    LittleEndian<u8> flags;
    LittleEndian<u16> cmdid;
    LittleEndian<u32> nsid;
    LittleEndian<u64> rsvd;
    LittleEndian<u64> meta_ptr;
    struct data_ptr_t data_ptr;
    LittleEndian<u32> cdw10;
    LittleEndian<u32> cdw11;
    LittleEndian<u32> cdw12;
    LittleEndian<u32> cdw13;
    LittleEndian<u32> cdw14;
    LittleEndian<u32> cdw15;
};


class NVMEController : public PCI::Device
    , public RefCounted<NVMEController> {
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
