/*
 * arch/arm/mach-at91/include/mach/memory.h
 *
 *  Copyright (C) 2004 SAN People
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

#include <mach/hardware.h>

#ifdef CONFIG_SPARSEMEM

#if defined(CONFIG_ARCH_AT91SAM9G45)
#define MAX_PHYSMEM_BITS        31
#define SECTION_SIZE_BITS       27

/* bank page offsets */
#define PAGE_OFFSET1    0xc8000000

#define __phys_to_virt(phys)                                            \
        ((phys) >= 0x70000000 ? (phys) - 0x70000000 + PAGE_OFFSET1 :    \
         (phys) + PAGE_OFFSET)

#define __virt_to_phys(virt)                                            \
         ((virt) >= PAGE_OFFSET1 ? (virt) - PAGE_OFFSET1 + 0x70000000 : \
          (virt) - PAGE_OFFSET)

#endif

#endif

#endif
