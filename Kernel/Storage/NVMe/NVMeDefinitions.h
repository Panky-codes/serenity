/*
 * Copyright (c) 2021, Pankaj R <pankydev8@gmail.com>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <AK/Endian.h>
#include <AK/Types.h>
struct NVMeCompletion;
struct NVMeSubmission;

// NVMe spec 1.4, section 3.1
static constexpr u32 REG_SQ0TDBL_START = 0x1000;
static constexpr u32 REG_SQ0TDBL_END = 0x1003;
// Doorbell stride. NVMe spec 8.6
static constexpr u8 CAP_DSTRD = 0; // TODO: Need to be configurable
static constexpr u32 CTRL_REG_SIZE(u16 NR_OF_QUEUES)
{
    return REG_SQ0TDBL_END + (NR_OF_QUEUES * (4 << CAP_DSTRD));
}
// CC – Controller Configuration
static constexpr u8 CC_CAP = 0x0;    // Offset 14h: CC – Controller Capabilities
static constexpr u8 CC_REG = 0x14;   // Offset 14h: CC – Controller Configuration
static constexpr u8 CSTS_REG = 0x1C; // Offset 1Ch: CSTS – Controller Status
static constexpr u8 CC_ASQ = 0x28;   // Offset 28h: ASQ – Admin Submission Queue Base Address
static constexpr u8 CC_ACQ = 0x30;   // Offset 30h: ACQ – Admin Completion Queue Base Address
static constexpr u8 CC_AQA = 0x24;   // Offset 24h: AQA – Admin Queue Attributes

static constexpr u8 CC_EN_BIT = 0x0;
static constexpr u8 CSTS_RDY_BIT = 0x0;
static constexpr u8 CC_IOSQES_BIT = 16;
static constexpr u8 CC_IOCQES_BIT = 20;

static constexpr u16 CC_AQA_MASK = (0xfff);
static constexpr u16 ACQ_SIZE(u32 x)
{
    return (x >> 16) & CC_AQA_MASK;
}
static constexpr u16 ASQ_SIZE(u32 x)
{
    return x & CC_AQA_MASK;
}
static constexpr u8 CQ_WIDTH = 4; // CQ is 16 bytes(2^4) in size. NVMe spec: 4.6
static constexpr u8 SQ_WIDTH = 6; // SQ size is 64 bytes(2^6) in size. NVMe spec: 4.2
static constexpr u16 CQ_SIZE(u16 q_depth)
{
    return q_depth << CQ_WIDTH;
}
static constexpr u16 SQ_SIZE(u16 q_depth)
{
    return q_depth << SQ_WIDTH;
}
static constexpr u8 PHASE_TAG(u16 x)
{
    return x & 0x1;
}
static constexpr u16 CQ_STATUS_FIELD_MASK = 0xfffe;
static constexpr u16 CQ_STATUS_FIELD(u16 x)
{
    return (x & CQ_STATUS_FIELD_MASK) >> 1;
}

static constexpr u16 IO_QUEUE_SIZE = 64; // TODO:Need to be configurable

// IDENTIFY
//  NVMe spec 1.4: 5.15.1
static constexpr u16 NVMe_IDENTIFY_SIZE = 4096;
static constexpr u8 NVMe_CNS_ID_ACTIVE_NS = 0x2;
static constexpr u8 NVMe_CNS_ID_NS = 0x0;
static constexpr u8 FLBA_SIZE_INDEX = 26;
static constexpr u8 FLBA_SIZE_MASK = 0xf;
static constexpr u8 LBA_FORMAT_SUPPORT_INDEX = 128;
static constexpr u32 LBA_SIZE_MASK = 0x00ff0000;


// OPCODES
// NVMe spec 1.4 Section 5 ADMIN COMMAND SET
enum AdminCommandOpCode {
    OP_ADMIN_CREATE_COMPLETION_QUEUE = 0x5,
    OP_ADMIN_CREATE_SUBMISSION_QUEUE = 0x1,
    OP_ADMIN_IDENTIFY = 0x6,
};

// IO opcodes
enum IOCommandOpcode {
    OP_NVME_WRITE = 0x1,
    OP_NVME_READ = 0x2
};

// FLAGS
static constexpr u8 QUEUE_PHY_CONTIGUOUS = (1 << 0);
static constexpr u8 QUEUE_IRQ_ENABLED = (1 << 1);

struct NVMeCompletion {
    LittleEndian<u32> cmd_spec;
    LittleEndian<u32> res;

    LittleEndian<u16> sq_head; /* how much of this queue may be reclaimed */
    LittleEndian<u16> sq_id;   /* submission queue that generated this entry */

    u16 command_id;           /* of the command which completed */
    LittleEndian<u16> status; /* did the command fail, and if so, why? */
};

struct DataPtr {
    LittleEndian<u64> prp1;
    LittleEndian<u64> prp2;
};

struct NVMeSubmission {
    LittleEndian<u8> op;
    LittleEndian<u8> flags;
    LittleEndian<u16> cmdid;
    LittleEndian<u32> nsid;
    LittleEndian<u64> rsvd;
    LittleEndian<u64> meta_ptr;
    struct DataPtr data_ptr;
    LittleEndian<u32> cdw10;
    LittleEndian<u32> cdw11;
    LittleEndian<u32> cdw12;
    LittleEndian<u32> cdw13;
    LittleEndian<u32> cdw14;
    LittleEndian<u32> cdw15;
};
