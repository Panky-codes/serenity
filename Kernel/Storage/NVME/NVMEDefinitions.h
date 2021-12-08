/*
 * Copyright (c) 2021, Pankaj Raghav <pankydev8@gmail.com>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#pragma once
#include <AK/Types.h>
#include <AK/Endian.h>
struct nvme_completion;
struct nvme_submission;

// NVMe spec 1.4, section 3.1
#define REG_SQ0TDBL_START 0x1000
#define REG_SQ0TDBL_END 0x1003
// Doorbell stride. NVMe spec 8.6
#define CAP_DSTRD 0 // TODO: Need to be configurable
#define CTRL_REG_SIZE(NR_OF_QUEUES) (REG_SQ0TDBL_END + ((NR_OF_QUEUES) * (4 << CAP_DSTRD)))

// CC – Controller Configuration
#define CC_REG 0x14   // Offset 14h: CC – Controller Configuration
#define CSTS_REG 0x1C // Offset 1Ch: CSTS – Controller Status
#define CC_ASQ 0x28   // Offset 28h: ASQ – Admin Submission Queue Base Address
#define CC_ACQ 0x30   // Offset 30h: ACQ – Admin Completion Queue Base Address
#define CC_AQA 0x24   // Offset 24h: AQA – Admin Queue Attributes

#define CC_EN_BIT 0x0
#define CSTS_RDY_BIT 0x0
#define CC_IOSQES_BIT 16
#define CC_IOCQES_BIT 20

#define CC_AQA_MASK (0xfff)
#define ACQ_SIZE(x) ((x) >> 16 & CC_AQA_MASK)
#define ASQ_SIZE(x) ((x)&CC_AQA_MASK)

#define CQ_WIDTH 4 // CQ is 16 bytes(2^4) in size. NVMe spec: 4.6
#define SQ_WIDTH 6 // SQ size is 64 bytes(2^6) in size. NVMe spec: 4.2
#define CQ_SIZE(q_depth) ((q_depth) * sizeof(struct nvme_completion))
#define SQ_SIZE(q_depth) ((q_depth) << SQ_WIDTH)
#define PHASE_TAG(x) ((x)&0x1)
#define CQ_STATUS_FIELD_MASK 0xfffe
#define CQ_STATUS_FIELD(x) (((x)&CQ_STATUS_FIELD_MASK) >> 1)

#define NR_IO_QUEUES 1 // TODO: Need to be configurable w module param
#define NR_ADMIN_QUEUE 1
#define TOT_NR_OF_QUEUES (NR_IO_QUEUES + NR_ADMIN_QUEUE)
#define NR_IRQ 2
#define IO_QUEUE_SIZE 64 // TODO:Need to be configurable

// IDENTIFY
//  NVMe spec 1.4: 5.15.1
#define NVME_IDENTIFY_SIZE 4096
#define NVME_CNS_ID_ACTIVE_NS 0x2
#define NVME_CNS_ID_NS 0x0

// OPCODES
// NVME spec 1.4 Section 5 ADMIN COMMAND SET
#define OP_ADMIN_CREATE_COMPLETION_QUEUE 0x5
#define OP_ADMIN_CREATE_SUBMISSION_QUEUE 0x1
#define OP_ADMIN_IDENTIFY 0x6
#define OP_ADMIN_SET_FEATURES 0x9
// IO opcodes
#define OP_NVME_WRITE 0x1
#define OP_NVME_READ 0x2

// FLAGS
#define QUEUE_PHY_CONTIGUOUS (1 << 0)
#define QUEUE_IRQ_ENABLED (1 << 1)

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
