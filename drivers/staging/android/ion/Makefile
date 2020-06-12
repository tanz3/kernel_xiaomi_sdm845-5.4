# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_ION)	+= ion.o ion_buffer.o ion_dma_buf.o ion_heap.o
CFLAGS_ion_buffer.o 	= -I$(src)
obj-y			+= heaps/

ifdef CONFIG_COMPAT
obj-$(CONFIG_ION_LEGACY) += compat_ion.o
endif
