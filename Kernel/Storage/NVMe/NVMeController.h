/*
 * Copyright (c) 2021, Pankaj R <pankydev8@gmail.com>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <AK/NonnullRefPtr.h>
#include <AK/NonnullRefPtrVector.h>
#include <AK/OwnPtr.h>
#include <AK/RefPtr.h>
#include <AK/Tuple.h>
#include <AK/Types.h>
#include <Kernel/Bus/PCI/Device.h>
#include <Kernel/Locking/Spinlock.h>
#include <Kernel/Storage/NVMe/NVMeDefinitions.h>
#include <Kernel/Storage/NVMe/NVMeNameSpace.h>
#include <Kernel/Storage/NVMe/NVMeQueue.h>
#include <Kernel/Storage/StorageController.h>

namespace Kernel {

class NVMeController : public PCI::Device
    , public StorageController {
    AK_MAKE_ETERNAL
public:
    static NonnullRefPtr<NVMeController> initialize(PCI::DeviceIdentifier const&);
    explicit NVMeController(PCI::DeviceIdentifier const&);
    RefPtr<StorageDevice> device(u32 index) const override;
    size_t devices_count() const override;

protected:
    bool reset() override;
    bool shutdown() override;
    void complete_current_request(AsyncDeviceRequest::RequestResult result) override;
    template<typename T>
    void set_ctrl_regs(u32 offset, T val)
    {
        u8* ptr = m_controller_regs->vaddr().as_ptr();
        *reinterpret_cast<T*>(ptr + offset) = val;
    }
    template<typename T>
    T get_ctrl_regs(u32 offset)
    {
        u8* ptr = m_controller_regs->vaddr().as_ptr();
        return *reinterpret_cast<T*>(ptr + offset);
    }

public:
    bool reset_controller();
    bool start_controller();
    u32 get_admin_q_dept();
    void write64_controller_regs(u32 offset, u64 value) { *reinterpret_cast<u64*>(m_controller_regs->vaddr().as_ptr() + offset) = value; };

    u16 submit_admin_command(struct NVMeSubmission& sub, bool sync = false)
    {
        // First queue is always the admin queue
        if (sync) {
            return m_admin_queue->submit_sync_sqe(sub);
        }
        m_admin_queue->submit_sqe(sub);
        return 0;
    }

    bool is_admin_queue_ready() { return m_admin_queue_ready; };
    void set_admin_queue_ready_flag() { m_admin_queue_ready = true; };

private:
    void identify_and_init_namespaces();
    Tuple<u64, u8> get_ns_features(ByteBuffer& identify_data_struct);
    void create_admin_queue(u8 irq);
    void create_io_queue(u8 irq, u8 qid);

private:
    PCI::DeviceIdentifier m_pci_device_id;
    RefPtr<NVMeQueue> m_admin_queue;
    NonnullRefPtrVector<NVMeQueue> m_queues;
    NonnullRefPtrVector<NVMeNameSpace> m_namespaces;
    Memory::Region* m_controller_regs;
    bool m_admin_queue_ready;
    size_t m_device_count;
};
}
