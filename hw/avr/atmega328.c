/*
 * QEMU AVR CPU
 *
 * Copyright (c) 2019 Michael Rolnik
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see
 * <http://www.gnu.org/licenses/lgpl-2.1.html>
 */

/*
 *  NOTE:
 *      This is not a real AVR board, this is an example!
 *      The CPU is an approximation of an ATmega2560, but is missing various
 *      built-in peripherals.
 *
 *      This example board loads provided binary file into flash memory and
 *      executes it from 0x00000000 address in the code memory space.
 *
 *      Currently used for AVR CPU validation
 *
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "cpu.h"
#include "hw/hw.h"
#include "sysemu/sysemu.h"
#include "sysemu/qtest.h"
#include "ui/console.h"
#include "hw/boards.h"
#include "hw/loader.h"
#include "qemu/error-report.h"
#include "exec/address-spaces.h"
#include "include/hw/sysbus.h"
#include "include/hw/char/avr_usart.h"
#include "include/hw/timer/avr_timer16.h"
#include "include/hw/misc/avr_mask.h"
#include "elf.h"
#include "hw/misc/unimp.h"

#define SIZE_FLASH 0x00008000
#define SIZE_SRAM 0x00000800
/*
 * Size of additional "external" memory, as if the AVR were configured to use
 * an external RAM chip.
 * Note that the configuration registers that normally enable this feature are
 * unimplemented.
 */
#define SIZE_EXMEM 0x00000000

/* Offsets of peripherals in emulated memory space (i.e. not host addresses)  */
#define PRR0_BASE 0x64
#define USART_BASE 0xc0
#define TIMER1_BASE 0x80
#define TIMER1_IMSK_BASE 0x6f
#define TIMER1_IFR_BASE 0x36

/* Interrupt numbers used by peripherals */
#define USART_RXC_IRQ 24
#define USART_DRE_IRQ 25
#define USART_TXC_IRQ 26

#define TIMER1_CAPT_IRQ 15
#define TIMER1_COMPA_IRQ 16
#define TIMER1_COMPB_IRQ 17
#define TIMER1_COMPC_IRQ 18

#define TIMER1_OVF_IRQ 19

/*  Power reduction     */
#define PRR0_BIT_PRTWI      0x07    /*  TWI */
#define PRR0_BIT_PRTIM2     0x06    /*  Timer/Counter2  */
#define PRR0_BIT_PRTIM0     0x05    /*  Timer/Counter0  */
#define PRR0_BIT_PRTIM1     0x03    /*  Timer/Counter1  */
#define PRR0_BIT_PRSPI      0x02    /*  Serial Peripheral Interface */
#define PRR0_BIT_PRUSART0   0x01    /*  USART0  */
#define PRR0_BIT_PRADC      0x00    /*  ADC */

typedef struct {
    MachineClass parent;
} ATMega328MachineClass;

typedef struct {
    MachineState parent;
    MemoryRegion *ram;
    MemoryRegion *flash;
    AVRUsartState *usart0;
    AVRTimer16State *timer1;
    AVRMaskState *prr[1];
} ATMega328MachineState;

#define TYPE_SAMPLE_MACHINE MACHINE_TYPE_NAME("atmega328")

#define SAMPLE_MACHINE(obj) \
    OBJECT_CHECK(ATMega328MachineState, obj, TYPE_SAMPLE_MACHINE)
#define SAMPLE_MACHINE_GET_CLASS(obj) \
    OBJECT_GET_CLASS(ATMega328MachineClass, obj, TYPE_SAMPLE_MACHINE)
#define SAMPLE_MACHINE_CLASS(klass) \
    OBJECT_CLASS_CHECK(ATMega328MachineClass, klass, TYPE_SAMPLE_MACHINE)

static void atmega328_init(MachineState *machine)
{
    ATMega328MachineState *sms = SAMPLE_MACHINE(machine);
    MemoryRegion *system_memory = get_system_memory();
    AVRCPU *cpu;
    const char *firmware = NULL;
    const char *filename;
    int bytes_loaded;
    SysBusDevice *busdev;
    DeviceState *cpudev;

    system_memory = get_system_memory();
    sms->ram = g_new(MemoryRegion, 1);
    sms->flash = g_new(MemoryRegion, 1);

    cpu = AVR_CPU(cpu_create(machine->cpu_type));
    cpudev = DEVICE(cpu);


    /* Flash memory */
    memory_region_init_rom(sms->flash, NULL, "avr.flash", SIZE_FLASH,
            &error_fatal);
    memory_region_add_subregion(system_memory, OFFSET_CODE, sms->flash);

    /* Devices not implemented */
    create_unimplemented_device("usart 0", OFFSET_DATA + 0x00c0, 0x0007);
    create_unimplemented_device("twi", OFFSET_DATA + 0x00b8, 0x0006);
    create_unimplemented_device("timer-counter-async-8bit 2", OFFSET_DATA + 0x00b0, 0x0007);
    create_unimplemented_device("timer-counter-16bit 1", OFFSET_DATA + 0x0080, 0x000c);
    create_unimplemented_device("ac / adc", OFFSET_DATA + 0x0078, 0x0008);
    create_unimplemented_device("int-controller", OFFSET_DATA + 0x0068, 0x0009);
    create_unimplemented_device("sys", OFFSET_DATA + 0x0060, 0x0007);
    create_unimplemented_device("spi", OFFSET_DATA + 0x004c, 0x0003);
    create_unimplemented_device("ext-mem-iface", OFFSET_DATA + 0x004a, 0x0002);
    create_unimplemented_device("timer-counter-pwm-8bit 0", OFFSET_DATA + 0x0043, 0x0006);
    create_unimplemented_device("ext-mem-iface", OFFSET_DATA + 0x003e, 0x0005);
    create_unimplemented_device("int-controller", OFFSET_DATA + 0x0035, 0x0009);
    create_unimplemented_device("gpio D", OFFSET_DATA + 0x0029, 0x0003);
    create_unimplemented_device("gpio C", OFFSET_DATA + 0x0026, 0x0003);
    create_unimplemented_device("gpio B", OFFSET_DATA + 0x0023, 0x0003);

    /* SRAM */
    memory_region_allocate_system_memory(
        sms->ram, NULL, "avr.ram", SIZE_SRAM + SIZE_EXMEM);
    memory_region_add_subregion(system_memory, OFFSET_DATA + 0x200, sms->ram);

    /* Power Reduction built-in peripheral */
    sms->prr[0] = AVR_MASK(sysbus_create_simple(TYPE_AVR_MASK,
                    OFFSET_DATA + PRR0_BASE, NULL));

    /* USART 0 built-in peripheral */
    sms->usart0 = AVR_USART(object_new(TYPE_AVR_USART));
    busdev = SYS_BUS_DEVICE(sms->usart0);
    qdev_prop_set_chr(DEVICE(sms->usart0), "chardev", serial_hd(0));
    object_property_set_bool(OBJECT(sms->usart0), true, "realized",
            &error_fatal);
    sysbus_mmio_map(busdev, 0, OFFSET_DATA + USART_BASE);

    /*
     * These IRQ numbers don't match the datasheet because we're counting from
     * zero and not including reset.
     */
    sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(cpudev, USART_RXC_IRQ));
    sysbus_connect_irq(busdev, 1, qdev_get_gpio_in(cpudev, USART_DRE_IRQ));
    sysbus_connect_irq(busdev, 2, qdev_get_gpio_in(cpudev, USART_TXC_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(sms->prr[1]), PRR0_BIT_PRUSART0, qdev_get_gpio_in(DEVICE(sms->usart0), 0));

    /* Timer 1 built-in periphal */
    sms->timer1 = AVR_TIMER16(object_new(TYPE_AVR_TIMER16));
    object_property_set_bool(OBJECT(sms->timer1), true, "realized",
            &error_fatal);
    busdev = SYS_BUS_DEVICE(sms->timer1);
    sysbus_mmio_map(busdev, 0, OFFSET_DATA + TIMER1_BASE);
    sysbus_mmio_map(busdev, 1, OFFSET_DATA + TIMER1_IMSK_BASE);
    sysbus_mmio_map(busdev, 2, OFFSET_DATA + TIMER1_IFR_BASE);
    sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(cpudev, TIMER1_CAPT_IRQ));
    sysbus_connect_irq(busdev, 1, qdev_get_gpio_in(cpudev, TIMER1_COMPA_IRQ));
    sysbus_connect_irq(busdev, 2, qdev_get_gpio_in(cpudev, TIMER1_COMPB_IRQ));
    sysbus_connect_irq(busdev, 4, qdev_get_gpio_in(cpudev, TIMER1_OVF_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(sms->prr[0]), PRR0_BIT_PRTIM1,
            qdev_get_gpio_in(DEVICE(sms->timer1), 0));

    /* Load firmware (contents of flash) trying to auto-detect format */
    firmware = machine->firmware;
    if (firmware != NULL) {
        filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, firmware);
        if (filename == NULL) {
            error_report("Unable to find %s", firmware);
            exit(1);
        }

        bytes_loaded = load_elf(
            filename, NULL, NULL, NULL, NULL, NULL, NULL, 0, EM_NONE, 0, 0);
        if (bytes_loaded < 0) {
            bytes_loaded = load_image_targphys(
                filename, OFFSET_CODE, SIZE_FLASH);
        }
        if (bytes_loaded < 0) {
            error_report(
                "Unable to load firmware image %s as ELF or raw binary",
                firmware);
            exit(1);
        }
    }
}

static void atmega328_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "ATmega328";
    mc->init = atmega328_init;
    mc->default_cpus = 1;
    mc->min_cpus = mc->default_cpus;
    mc->max_cpus = mc->default_cpus;
    mc->default_cpu_type = "avr5-avr-cpu";
    mc->is_default = 1;
}

static const TypeInfo atmega328_info = {
    .name = TYPE_SAMPLE_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(ATMega328MachineState),
    .class_size = sizeof(ATMega328MachineClass),
    .class_init = atmega328_class_init,
};

static void atmega328_machine_init(void)
{
    type_register_static(&atmega328_info);
}

type_init(atmega328_machine_init);
