/*
* Copyright (c) 2021, Pankaj Raghav <pankydev8@gmail.com>
*
* SPDX-License-Identifier: BSD-2-Clause
*/
#pragma once

#include <AK/RefPtr.h>
#include <AK/Types.h>
#include <AK/RefCounted.h>
#include <Kernel/Locking/Spinlock.h>
#include <AK/NonnullRefPtrVector.h>
#include <AK/OwnPtr.h>
#include <AK/RefPtr.h>
#include <AK/Types.h>
#include <Kernel/Bus/PCI/Device.h>
#include <Kernel/Locking/Spinlock.h>

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
}