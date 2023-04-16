/*
* Copyright (c) 2023, Pankaj R <dev@pankajraghav.com>
*
* SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

// Address register
#define MSI_ADDRESS_BASE 0xfee00000
#define MSI_DESTINATION_SHIFT 12
#define MSI_REDIRECTION_HINT 0x00000008
#define MSI_DESTINATION_MODE_LOGICAL 0x00000004


// Data register
#define MSI_DATA_VECTOR_MASK 0xFF
#define MSI_TRIGGER_MODE_LEVEL 0x00008000
#define MSI_LEVEL_ASSERT 0x00004000