#
# Makefile for nfc devices
#

ifeq ($(CONFIG_MACH_FIND7),y)
obj-$(CONFIG_PN544_NFC)		+= pn544-pn65o.o
else ifeq ($(CONFIG_MACH_N3),y)
obj-$(CONFIG_PN544_NFC)		+= pn544-pn65t.o
obj-$(CONFIG_PN544_NFC)		+= p61.o
else
obj-$(CONFIG_PN544_NFC)		+= pn544.o
endif
obj-$(CONFIG_NFC_PN533)		+= pn533.o
obj-$(CONFIG_NFC_WILINK)	+= nfcwilink.o
obj-$(CONFIG_NFC_QNCI)		+= nfc-nci.o

ccflags-$(CONFIG_NFC_DEBUG) := -DDEBUG
