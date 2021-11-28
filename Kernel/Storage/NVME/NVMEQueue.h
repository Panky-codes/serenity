/*
 * Copyright (c) 2021, Pankaj Raghav <pankydev8@gmail.com>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#pragma once

#include <AK/NonnullRefPtrVector.h>
#include <AK/OwnPtr.h>
#include <AK/RefCounted.h>
#include <AK/RefPtr.h>
#include <AK/Types.h>
#include <Kernel/Bus/PCI/Device.h>
#include <Kernel/Interrupts/IRQHandler.h>
#include <Kernel/Locking/Spinlock.h>
#include <Kernel/Storage/NVME/NVMEDefinitions.h>

namespace Kernel {

class NVMEController;

class NVMEQueue : public RefCounted<NVMEQueue>
    , public IRQHandler {
public:
    static NonnullRefPtr<NVMEQueue> create(const NVMEController&, u16 qid, u8 irq);
    explicit NVMEQueue(const NVMEController& controller, u16 qid, u8 irq);
    bool is_admin_queue() { return m_admin_queue; };
    bool handle_irq(const RegisterState&) override;
    void submit_sqe(struct nvme_submission const&);

private:
    void setup_admin_queue();
    void setup_io_queue();
    bool cqe_available();
    void update_cqe_head();

private:
    u16 qid;
    u8 cq_valid_phase;
    u16 sq_tail;
    u16 cq_head;
    bool m_admin_queue;
    // TODO: need to find a better way of getting this
    u8 m_irq;
    u32 m_qdepth;
    Spinlock m_cq_lock { LockRank::Interrupts };
    Spinlock m_sq_lock { LockRank::Interrupts };
    RefPtr<NVMEController> m_controller;
    OwnPtr<Memory::Region> m_cq_dma_region;
    RefPtr<Memory::PhysicalPage> m_cq_dma_page;
    OwnPtr<Memory::Region> m_sq_dma_region;
    RefPtr<Memory::PhysicalPage> m_sq_dma_page;
};
}