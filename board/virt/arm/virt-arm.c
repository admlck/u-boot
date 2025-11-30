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
#include <env_internal.h>
#include <fdtdec.h>
#include <fdt_support.h>
#include <fs.h>
#include <init.h>
#include <log.h>
#include <part.h>
#include <sort.h>
#include <virtio_types.h>
#include <virtio.h>

#include <linux/sizes.h>
#include <linux/kernel.h>

#include <asm/armv8/mmu.h>

#define MAX_MEM_MAP_REGIONS	32
#define MAX_DT_RANGES		64

/*
 * Filled in by dram_init() before relocation, so it must not be in BSS,
 * see arch/arm/cpu/u-boot.lds. The last entry stays zero as terminator.
 */
static struct mm_region my_mem_map[MAX_MEM_MAP_REGIONS + 1] __section(".data");

struct mm_region *mem_map = my_mem_map;

struct addr_range {
	u64 start, end; /* end is exclusive */
};

struct range_list {
	struct addr_range r[MAX_DT_RANGES];
	int cnt;
};

static void range_add(struct range_list *l, u64 start, u64 size)
{
	if (!size || start == OF_BAD_ADDR)
		return;

	if (l->cnt >= MAX_DT_RANGES) {
		log_warning("virt-arm: too many address ranges in the DT\n");
		return;
	}

	l->r[l->cnt].start = start;
	l->r[l->cnt].end = start + size;
	l->cnt++;
}

static int range_cmp(const void *a, const void *b)
{
	const struct addr_range *ra = a, *rb = b;

	if (ra->start == rb->start)
		return 0;
	return ra->start < rb->start ? -1 : 1;
}

/* Sort the ranges and merge overlapping and adjacent ones */
static void range_merge(struct range_list *l)
{
	int i, n = 0;

	if (!l->cnt)
		return;

	qsort(l->r, l->cnt, sizeof(l->r[0]), range_cmp);

	for (i = 1; i < l->cnt; i++) {
		if (l->r[i].start <= l->r[n].end)
			l->r[n].end = max(l->r[n].end, l->r[i].end);
		else
			l->r[++n] = l->r[i];
	}
	l->cnt = n + 1;
}

/* Add the CPU side of the windows of a PCI host bridge */
static void add_pci_windows(const void *blob, int node, struct range_list *l)
{
	int parent = fdt_parent_offset(blob, node);
	int pna, na, ns, len;
	const fdt32_t *p;

	if (parent < 0)
		return;

	p = fdt_getprop(blob, node, "ranges", &len);
	if (!p)
		return;

	na = fdt_address_cells(blob, node);
	pna = fdt_address_cells(blob, parent);
	ns = fdt_size_cells(blob, node);
	if (na != 3 || pna < 1 || ns < 1)
		return;

	for (len /= sizeof(*p); len >= na + pna + ns; len -= na + pna + ns) {
		/* The parent address is in the address space of the parent */
		range_add(l, fdt_translate_address(blob, node, p + na),
			  fdtdec_get_number(p + na + pna, ns));
		p += na + pna + ns;
	}
}

static void add_mem_region(int *idx, u64 start, u64 end, u64 attrs)
{
	if (*idx >= MAX_MEM_MAP_REGIONS) {
		log_warning("virt-arm: too many memory regions\n");
		return;
	}

	my_mem_map[*idx].virt = start;
	my_mem_map[*idx].phys = start;
	my_mem_map[*idx].size = end - start;
	my_mem_map[*idx].attrs = attrs;
	(*idx)++;
}

/*
 * Build the memory map from the device tree: all memory nodes as normal
 * memory, and everything that devices have in their reg properties, or PCI
 * host bridges in their ranges, as device memory.
 */
static void setup_mem_map(const void *blob)
{
	/* On the stack, as BSS is not available before relocation */
	struct range_list ram = { .cnt = 0 }, dev = { .cnt = 0 };
	struct fdt_resource res;
	int node, i, r, idx = 0;
	const char *type;

	for (node = fdt_next_node(blob, 0, NULL); node >= 0;
	     node = fdt_next_node(blob, node, NULL)) {
		bool is_mem;

		if (!fdtdec_get_is_enabled(blob, node))
			continue;

		type = fdt_getprop(blob, node, "device_type", NULL);
		is_mem = type && !strcmp(type, "memory");

		/* Skip reg properties without size, e.g., of CPUs */
		if (!fdt_size_cells(blob, fdt_parent_offset(blob, node)))
			continue;

		for (i = 0; !fdt_get_resource(blob, node, "reg", i, &res); i++)
			range_add(is_mem ? &ram : &dev, res.start,
				  fdt_resource_size(&res));

		if (type && !strcmp(type, "pci"))
			add_pci_windows(blob, node, &dev);
	}

	range_merge(&ram);
	for (i = 0; i < ram.cnt; i++)
		add_mem_region(&idx, ram.r[i].start, ram.r[i].end,
			       PTE_BLOCK_MEMTYPE(MT_NORMAL) |
			       PTE_BLOCK_INNER_SHARE);

	/*
	 * Map devices in 2 MiB granularity to use block mappings and merge
	 * neighbours, but never let them overlap RAM.
	 */
	for (i = 0; i < dev.cnt; i++) {
		dev.r[i].start = round_down(dev.r[i].start, SZ_2M);
		dev.r[i].end = round_up(dev.r[i].end, SZ_2M);
	}
	range_merge(&dev);

	for (i = 0; i < dev.cnt; i++) {
		u64 start = dev.r[i].start, end = dev.r[i].end;

		for (r = 0; r < ram.cnt && start < end; r++) {
			if (ram.r[r].end <= start || ram.r[r].start >= end)
				continue;
			if (ram.r[r].start > start)
				add_mem_region(&idx, start, ram.r[r].start,
					       PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
					       PTE_BLOCK_NON_SHARE |
					       PTE_BLOCK_PXN | PTE_BLOCK_UXN);
			start = ram.r[r].end;
		}

		if (start < end)
			add_mem_region(&idx, start, end,
				       PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
				       PTE_BLOCK_NON_SHARE |
				       PTE_BLOCK_PXN | PTE_BLOCK_UXN);
	}
}

static bool has_flash(void)
{
	int node;

	for (node = fdt_node_offset_by_compatible(gd->fdt_blob, -1, "cfi-flash");
	     node >= 0;
	     node = fdt_node_offset_by_compatible(gd->fdt_blob, node, "cfi-flash"))
		if (fdtdec_get_is_enabled(gd->fdt_blob, node))
			return true;

	return false;
}

#if defined(CONFIG_ENV_IS_IN_EXT4)
#define ENV_FILE_INTERFACE	CONFIG_ENV_EXT4_INTERFACE
#define ENV_FILE_DEVICE_AND_PART CONFIG_ENV_EXT4_DEVICE_AND_PART
#elif defined(CONFIG_ENV_IS_IN_FAT)
#define ENV_FILE_INTERFACE	CONFIG_ENV_FAT_INTERFACE
#define ENV_FILE_DEVICE_AND_PART CONFIG_ENV_FAT_DEVICE_AND_PART
#endif

/* The type of the file system that holds the environment file */
static int env_file_fs_type(void)
{
#ifdef ENV_FILE_INTERFACE
	struct disk_partition info;
	struct blk_desc *desc;
	int part, type = FS_TYPE_ANY;

	if (!strcmp(ENV_FILE_INTERFACE, "virtio"))
		virtio_init();

	part = blk_get_device_part_str(ENV_FILE_INTERFACE,
				       ENV_FILE_DEVICE_AND_PART, &desc, &info,
				       1);
	if (part >= 0 && !fs_set_blk_dev_with_part(desc, part)) {
		type = fs_get_type();
		fs_close();
	}

	return type;
#else
	return FS_TYPE_ANY;
#endif
}

/*
 * Store the environment in the VM's flash, if the device tree has one, and
 * in a file on the VM's disk otherwise, on FAT or ext4, whichever the
 * partition has.
 */
enum env_location env_get_location(enum env_operation op, int prio)
{
	enum env_location file_locs[2];
	int n = 0;

	if (IS_ENABLED(CONFIG_ENV_IS_IN_MTD) && has_flash())
		return prio ? ENVL_UNKNOWN : ENVL_MTD;

	if (IS_ENABLED(CONFIG_ENV_IS_IN_EXT4))
		file_locs[n++] = ENVL_EXT4;
	if (IS_ENABLED(CONFIG_ENV_IS_IN_FAT))
		file_locs[n++] = ENVL_FAT;

	if (!n)
		return prio ? ENVL_UNKNOWN : ENVL_NOWHERE;

	/*
	 * Initialize all candidates, as only initialized locations are used
	 * later on. Also, block devices are not available yet at this point.
	 */
	if (op == ENVOP_INIT)
		return prio < n ? file_locs[prio] : ENVL_UNKNOWN;

	if (prio)
		return ENVL_UNKNOWN;

	if (n > 1 && env_file_fs_type() == FS_TYPE_FAT)
		return ENVL_FAT;

	return file_locs[0];
}

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

	/* For compressed kernel images, e.g. Image.gz, for booti */
	env_set_hex("kernel_comp_addr_r", ramstart + 0x08000000);
	env_set_hex("kernel_comp_size", 0x04000000);

	return 0;
}

int dram_init(void)
{
	if (fdtdec_setup_mem_size_base() != 0)
		return -EINVAL;

	/*
	 * When LPAE is enabled (ARMv7),
	 * 1:1 mapping is created using 2 MB blocks.
	 *
	 * In case amount of memory provided to u-boot/VM
	 * is not multiple of 2 MB, round down the amount
	 * of available memory to avoid hang during MMU
	 * initialization.
	 */
	if (CONFIG_IS_ENABLED(ARMV7_LPAE))
		gd->ram_size -= (gd->ram_size % 0x200000);

	setup_mem_map(gd->fdt_blob);

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
