/*
 *  linux/fs/ext4/bitmap.c
 *
 * Copyright (C) 1992, 1993, 1994, 1995
 * Remy Card (card@masi.ibp.fr)
 * Laboratoire MASI - Institut Blaise Pascal
 * Universite Pierre et Marie Curie (Paris VI)
 */

#include <linux/buffer_head.h>
#include <linux/jbd2.h>
#include "ext4.h"

static const int nibblemap[] = {4, 3, 3, 2, 3, 2, 2, 1, 3, 2, 2, 1, 2, 1, 1, 0};

unsigned int ext4_count_free(char *bitmap, unsigned int numchars)
{
	unsigned int i, sum = 0;

	for (i = 0; i < numchars; i++)
		sum += nibblemap[bitmap[i] & 0xf] +
			nibblemap[(bitmap[i] >> 4) & 0xf];
	return sum;
}

int ext4_inode_bitmap_csum_verify(struct super_block *sb, ext4_group_t group,
				  struct ext4_group_desc *gdp,
				  struct buffer_head *bh, int sz)
{
	__u32 hi;
	__u32 provided, calculated;
	struct ext4_sb_info *sbi = EXT4_SB(sb);

	if (!EXT4_HAS_RO_COMPAT_FEATURE(sb,
					EXT4_FEATURE_RO_COMPAT_METADATA_CSUM))
		return 1;

	provided = le16_to_cpu(gdp->bg_inode_bitmap_csum_lo);
	calculated = ext4_chksum(sbi, sbi->s_csum_seed, (__u8 *)bh->b_data, sz);
	if (sbi->s_desc_size >= EXT4_BG_INODE_BITMAP_CSUM_HI_END) {
		hi = le16_to_cpu(gdp->bg_inode_bitmap_csum_hi);
		provided |= (hi << 16);
	} else
		calculated &= 0xFFFF;

	return provided == calculated;
}

void ext4_inode_bitmap_csum_set(struct super_block *sb, ext4_group_t group,
				struct ext4_group_desc *gdp,
				struct buffer_head *bh, int sz)
{
	__u32 csum;
	struct ext4_sb_info *sbi = EXT4_SB(sb);

	if (!EXT4_HAS_RO_COMPAT_FEATURE(sb,
					EXT4_FEATURE_RO_COMPAT_METADATA_CSUM))
		return;

	csum = ext4_chksum(sbi, sbi->s_csum_seed, (__u8 *)bh->b_data, sz);
	gdp->bg_inode_bitmap_csum_lo = cpu_to_le16(csum & 0xFFFF);
	if (sbi->s_desc_size >= EXT4_BG_INODE_BITMAP_CSUM_HI_END)
		gdp->bg_inode_bitmap_csum_hi = cpu_to_le16(csum >> 16);
}
