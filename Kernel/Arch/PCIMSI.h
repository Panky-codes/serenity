/*
 * Copyright (c) 2023, Pankaj R <dev@pankajraghav.com>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once
#include <AK/Types.h>

namespace Kernel {
#if ARCH(X86_64)
u64 msi_address_register(u8 destination_id, bool redirection_hint, bool destination_mode);
u32 msi_data_register(u8 vector, bool level_trigger, bool assert);
u32 msi_vector_control_register(u32 vector_control, bool mask);
void msi_signal_eoi();
#elif ARCH(AARCH64)
static u64 msi_address_register(u8 destination_id, bool redirection_hint, bool destination_mode)
{
    return 0;
}
static u32 msi_data_register(u8 vector, bool level_trigger, bool assert) { return 0; }
static u32 msi_vector_control_register(u32 vector_control, bool mask) { return 0; }
#endif
}