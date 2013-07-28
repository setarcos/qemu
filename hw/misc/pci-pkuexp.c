/*
 * QEMU PCI PKU-EXP device
 *
 * Copyright (c) 2012 Pluto Yang <yangyj.ee@gmail.com>
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */
#include "hw/hw.h"
#include "hw/pci/pci.h"
#include "qemu/osdep.h"

#define PCI_PKUEXP_DEV(obj) \
    OBJECT_CHECK(PCIPkuExpState, (obj), TYPE_PCI_PKUEXP_DEV)

typedef struct PCIPkuExpState {
    /*< private >*/
    PCIDevice dev;
    /*< public >*/

    MemoryRegion mmio;
    MemoryRegion portio;
    int current;
} PCIPkuExpState;

#define TYPE_PCI_PKUEXP_DEV "pci-pkuexp"
#define IOPKU_IOSIZE 128
#define IOPKU_MEMSIZE 128

static void
pci_pkuexp_reset(PCIPkuExpState *d)
{
    if (d->current == -1) {
        return;
    }
    d->current = -1;
}

static void
pci_pkuexp_write(void *opaque, hwaddr addr, uint64_t val,
                  unsigned size, int type)
{
    PCIPkuExpState *d = opaque;
    printf("Write\n");
    if (d->current < 0) {
        return;
    }
}

static uint64_t
pci_pkuexp_read(void *opaque, hwaddr addr, unsigned size)
{
    printf("Read\n");
    PCIPkuExpState *d = opaque;
    if (d->current < 0) {
        return 0;
    }
    return 1;
}

static void
pci_pkuexp_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                       unsigned size)
{
    pci_pkuexp_write(opaque, addr, val, size, 0);
}

static void
pci_pkuexp_pio_write(void *opaque, hwaddr addr, uint64_t val,
                       unsigned size)
{
    pci_pkuexp_write(opaque, addr, val, size, 1);
}

static const MemoryRegionOps pci_pkuexp_mmio_ops = {
    .read = pci_pkuexp_read,
    .write = pci_pkuexp_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

static const MemoryRegionOps pci_pkuexp_pio_ops = {
    .read = pci_pkuexp_read,
    .write = pci_pkuexp_pio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

static int pci_pkuexp_init(PCIDevice *pci_dev)
{
    PCIPkuExpState *d = PCI_PKUEXP_DEV(pci_dev);
    uint8_t *pci_conf;

    pci_conf = pci_dev->config;

    pci_conf[PCI_INTERRUPT_PIN] = 0x1;

    memory_region_init_io(&d->mmio, OBJECT(d), &pci_pkuexp_mmio_ops, d,
                          "pci-pkuexp-mmio", IOPKU_MEMSIZE * 2);
    memory_region_init_io(&d->portio, OBJECT(d), &pci_pkuexp_pio_ops, d,
                          "pci-pkuexp-portio", IOPKU_IOSIZE * 2);
    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &d->mmio);
    pci_register_bar(pci_dev, 1, PCI_BASE_ADDRESS_SPACE_IO, &d->portio);

    d->current = -1;
    return 0;
}

static void
pci_pkuexp_uninit(PCIDevice *dev)
{
    PCIPkuExpState *d = PCI_PKUEXP_DEV(dev);

    pci_pkuexp_reset(d);
    memory_region_destroy(&d->mmio);
    memory_region_destroy(&d->portio);
}

static void qdev_pci_pkuexp_reset(DeviceState *dev)
{
    PCIPkuExpState *d = PCI_PKUEXP_DEV(dev);
    pci_pkuexp_reset(d);
}

static void pci_pkuexp_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->init = pci_pkuexp_init;
    k->exit = pci_pkuexp_uninit;
    k->vendor_id = PCI_VENDOR_ID_QEMU;
    k->device_id = 0x1898;
    k->revision = 0x00;
    k->class_id = PCI_CLASS_OTHERS;
    dc->desc = "PCI Pku Exp Device";
    dc->reset = qdev_pci_pkuexp_reset;
}

static const TypeInfo pci_pkuexp_info = {
    .name          = TYPE_PCI_PKUEXP_DEV,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(PCIPkuExpState),
    .class_init    = pci_pkuexp_class_init,
};

static void pci_pkuexp_register_types(void)
{
    type_register_static(&pci_pkuexp_info);
}

type_init(pci_pkuexp_register_types)
