// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2017 Tuomas Tynkkynen
 * Copyright (c) 2026 Adam Lackorzynski
 *
 * BSP
 */

#include <config.h>
#include <cpu_func.h>
#include <dm.h>
#include <env.h>
#include <fdtdec.h>
#include <init.h>
#include <log.h>
#include <virtio_types.h>
#include <virtio.h>

#include <linux/sizes.h>

#include <asm/armv8/mmu.h>

static struct mm_region my_mem_map[] = {
	 {
		/* RAM */
		.virt = 0UL, // filled out later
		.phys = 0UL, // filled out later
		.size = 0, // filled out later
		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
			 PTE_BLOCK_INNER_SHARE
	},
	{ 0, }, // lower mmio
	{ 0, }, // higher mmio
	{
		/* List terminator */
		0,
	}
};

struct mm_region *mem_map = my_mem_map;

int board_late_init(void)
{
	/*
	 * Make sure virtio bus is enumerated so that peripherals
	 * on the virtio bus can be discovered by their drivers
	 */
	virtio_init();

	/* Set env vars here because our start of RAM is dynamic */
	unsigned long ramstart = gd->dram[0].start;
	env_set_hex("ramstart", ramstart);

	env_set_hex("scriptaddr",     ramstart + 0x00200000);
	env_set_hex("pxefile_addr_r", ramstart + 0x00300000);
	env_set_hex("kernel_addr_r",  ramstart + 0x00400000);
	env_set_hex("ramdisk_addr_r", ramstart + 0x04000000);

	return 0;
}

int dram_init(void)
{
	if (fdtdec_setup_mem_size_base() != 0)
		return -EINVAL;

	my_mem_map[0].virt = gd->ram_base;
	my_mem_map[0].phys = gd->ram_base;
	my_mem_map[0].size = gd->ram_size;

	// We are generous with the device memory areas, as a compromise
	// of not scanning the whole DT for MMIO areas.
	int idx = 1;
	if (gd->ram_base > 0) {
		my_mem_map[idx].virt = 0;
		my_mem_map[idx].phys = 0;
		my_mem_map[idx].size = gd->ram_base;
		my_mem_map[idx].attrs = PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
			                PTE_BLOCK_NON_SHARE |
			                PTE_BLOCK_PXN | PTE_BLOCK_UXN;
		idx++;
	}

	unsigned long b = gd->ram_base + gd->ram_size;
	my_mem_map[idx].virt = b;
	my_mem_map[idx].phys = b;
	my_mem_map[idx].size = (64ull << 30);
	my_mem_map[idx].attrs = PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
	                        PTE_BLOCK_NON_SHARE |
	                        PTE_BLOCK_PXN | PTE_BLOCK_UXN;

	return 0;
}

int dram_init_banksize(void)
{
	return fdtdec_setup_memory_banksize();
}

/*
 * Saved address of DTB
 *
 * Must not be in BSS, as bss must not be written before relocations are
 * done -- see arch/arm/cpu/u-boot.lds
 */
unsigned long virt_arm_saved_dtb = 1;

asm(
".global save_boot_params	\n"
"save_boot_params:	\n"
"	adrp x9, virt_arm_saved_dtb\n"
"	add  x9, x9, #:lo12:virt_arm_saved_dtb\n"
"	str x0, [x9]\n"
"	b save_boot_params_ret\n"
);

int board_fdt_blob_setup(void **fdtp)
{
	*fdtp = (void *)virt_arm_saved_dtb;
	return 0;
}

void enable_caches(void)
{
	 icache_enable();
	 dcache_enable();
}
