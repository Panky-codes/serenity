/*
 * Copyright (c) 2021, Pankaj Raghav <pankydev8@gmail.com>
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
#include <Kernel/Storage/NVME/NVMEDefinitions.h>
#include <Kernel/Storage/NVME/NVMENameSpace.h>
#include <Kernel/Storage/NVME/NVMEQueue.h>
#include <Kernel/Storage/StorageController.h>

namespace Kernel {

class NVMEController : public PCI::Device
    , public StorageController {
    AK_MAKE_ETERNAL
public:
    static NonnullRefPtr<NVMEController> initialize(PCI::DeviceIdentifier const&);
    explicit NVMEController(PCI::DeviceIdentifier const&);
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
    void reset_controller();
    void start_controller();
    u32 get_admin_q_dept();
    void write64_controller_regs(u32 offset, u64 value) { *reinterpret_cast<u64*>(m_controller_regs->vaddr().as_ptr() + offset) = value; };

    u16 submit_admin_command(struct nvme_submission& sub, bool sync = false)
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
    void test_rw_functionality();
    void create_admin_queue(u8 irq);
    void create_io_queue(u8 irq, u8 qid);

private:
    PCI::DeviceIdentifier m_pci_device_id;
    RefPtr<NVMEQueue> m_admin_queue;
    NonnullRefPtrVector<NVMEQueue> m_queues;
    NonnullRefPtrVector<NVMENameSpace> m_namespaces;
    Memory::Region* m_controller_regs;
    bool m_admin_queue_ready;
    size_t m_device_count;
};
}
