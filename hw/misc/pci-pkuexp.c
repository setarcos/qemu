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
#include "qemu/timer.h"
#include "qemu/main-loop.h"
#ifdef WIN32
#include <windows.h>
#include <conio.h>
#else
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
static struct termios options;
#endif

#define PCI_PKUEXP_DEV(obj) \
    OBJECT_CHECK(PCIPkuExpState, (obj), TYPE_PCI_PKUEXP_DEV)

typedef struct PCIPkuExpState {
    /*< private >*/
    PCIDevice dev;
    /*< public >*/
#ifdef WIN32
    HANDLE fd;
#else
    int fd;
#endif
    uint8_t data;
    MemoryRegion mmio;
    MemoryRegion portio;
    qemu_irq irq;
} PCIPkuExpState;

static volatile uint8_t iostate;

#define TYPE_PCI_PKUEXP_DEV "pci-pkuexp"
#define IOPKU_IOSIZE        128
#define IOPKU_MEMSIZE       128
#define CMD_INT             0x00
#define CMD_READ            0x40
#define CMD_WRITE           0x80
#define S_WAIT_READ         0x10
#define S_BEGIN_IO          0x20

extern const char *pkuexpport;

static void pci_pkuexp_reset(PCIPkuExpState *d)
{
}

static void pkuexp_read_ack(PCIPkuExpState *d)
{
    ssize_t ret;
    uint8_t buf;

    TFR(ret = read(d->fd, &buf, 1));
    if (ret < 0) return;
    if ((buf & 0xC0) == CMD_WRITE) {
        iostate = 0;
        return;
    }
    if ((buf & 0xC0) == CMD_INT) {
        pci_set_irq(&d->dev, 1);
        return;
    }
    if ((buf & 0xC0) == CMD_READ) {
        TFR(ret = read(d->fd, &buf, 1));
        if (ret < 0) return;
        d->data = buf;
        iostate = 0;
    }
}

static void pku_read(void *opaque)
{
    PCIPkuExpState *d = opaque;
    pkuexp_read_ack(d);
}


static void
pci_pkuexp_write(void *opaque, hwaddr addr, uint64_t val,
                  unsigned size, int type)
{
    PCIPkuExpState *d = opaque;
    uint8_t buf[2];
    ssize_t ret;
    //printf("Write %d\n", (int)addr);
    if ((addr & 0xFF) > 0x40) {
        pci_set_irq(&d->dev, 0);
        return;
    }
    buf[0] = CMD_WRITE + (addr & 0x3F);
    buf[1] = val & 0xFF;
    iostate = S_BEGIN_IO;
    qemu_set_fd_handler(d->fd, NULL, NULL, NULL);
    TFR(ret = write(d->fd, buf, 2));
    if (ret < 0) {
        perror("PkuExp(write)");
        return;
    }
    while (iostate) {
        pkuexp_read_ack(d);
    }
    qemu_set_fd_handler(d->fd, pku_read, NULL, d);
}

static uint64_t
pci_pkuexp_read(void *opaque, hwaddr addr, unsigned size)
{
    PCIPkuExpState *d = opaque;
    uint8_t buf;
    ssize_t ret;
    //printf("Read %d\n", (int)addr);
    buf = CMD_READ + (addr & 0x3F);
    iostate = S_BEGIN_IO;
    qemu_set_fd_handler(d->fd, NULL, NULL, NULL);
    TFR(ret = write(d->fd, &buf, 1));
    if (ret < 0) {
        perror("PkuExp(read)");
        return ret;
    }
    while (iostate) {
        pkuexp_read_ack(d);
    }
    qemu_set_fd_handler(d->fd, pku_read, NULL, d);
    return d->data;
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

static void pkuexp_serial_init(PCIPkuExpState *d)
{
#ifdef WIN32
    d->fd = CreateFile(pkuexpport, GENERIC_READ | GENERIC_WRITE,
                       0, NULL, OPEN_EXISTING, 0, NULL);

    if (d->fd == INVALID_HANDLE_VALUE) {
        perror("PKUExp: Could not connect to target TTY");
        exit(1);
    }

    DCB dcb;
    if (!GetCommState(d->fd, &dcb)) {
        perror("PKUExp: Could not load config for target TTY");
        exit(1);
    }

    dcb.BaudRate = CBR_115200;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;

    if (!SetCommState(d->fd, &dcb)) {
        perror("PKUExp: Could not store config for target TTY");
        exit(1);
    }
#else
    d->fd = open(pkuexpport, O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (d->fd == -1) {
        perror("PKUExp: Could not connect to target TTY");
        exit(1);
    }

    if (ioctl(d->fd, TIOCEXCL) == -1) {
        perror("PKUExp: TTY not exclusively available");
        exit(1);
    }

    if (fcntl(d->fd, F_SETFL, 0) == -1) {
        perror("PKUExp: Could not switch to blocking I/O");
        exit(1);
    }

    if (tcgetattr(d->fd, &options) == -1) {
        perror("PKUExp: Could not get TTY attributes");
        exit(1);
    }

    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    /* set raw input, 1 second timeout */
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_iflag |= IGNCR;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 100;

    tcsetattr(d->fd, TCSANOW, &options);

    tcflush(d->fd, TCIOFLUSH);
#endif
}

static int pci_pkuexp_init(PCIDevice *pci_dev)
{
    PCIPkuExpState *d = PCI_PKUEXP_DEV(pci_dev);
    uint8_t *pci_conf;

    if (pkuexpport == NULL) {
        printf("You need to specify a serial device to use PkuExp\n");
        exit(1);
    }
    pkuexp_serial_init(d);
    qemu_set_fd_handler(d->fd, pku_read, NULL, d);

    pci_conf = pci_dev->config;

    pci_conf[PCI_INTERRUPT_PIN] = 0x1;

    //d->irq = pci_allocate_irq(&d->dev);

    memory_region_init_io(&d->mmio, OBJECT(d), &pci_pkuexp_mmio_ops, d,
                          "pci-pkuexp-mmio", IOPKU_MEMSIZE * 2);
    memory_region_init_io(&d->portio, OBJECT(d), &pci_pkuexp_pio_ops, d,
                          "pci-pkuexp-portio", IOPKU_IOSIZE * 2);
    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &d->mmio);
    pci_register_bar(pci_dev, 1, PCI_BASE_ADDRESS_SPACE_IO, &d->portio);

    return 0;
}

static void
pci_pkuexp_uninit(PCIDevice *dev)
{
    PCIPkuExpState *d = PCI_PKUEXP_DEV(dev);

    pci_pkuexp_reset(d);
    memory_region_destroy(&d->mmio);
    memory_region_destroy(&d->portio);
   // qemu_free_irq(d->irq);
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
