/*
 * STM32 fa103c8 (Blue Pill) Development Board
 *
 * Copyright (C) 2018 Basel Alsayeh
 * Copyright (C) 2020 Luis CLaudio G Lopes
 *
 * Implementation based on
 * Olimex "STM-P103 Development Board Users Manual Rev. A, April 2008"
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "hw/arm/stm32.h"
#include "hw/sysbus.h"
#include "hw/arm/armv7m.h"
//#include "hw/devices.h"
#include "ui/console.h"
#include "sysemu/sysemu.h"
#include "hw/boards.h"
#include "hw/irq.h"
#include "hw/arm/boot.h"

typedef struct
{
 Stm32 *stm32;
 qemu_irq pin_irq[100];
 qemu_irq *pout_irq;
 DeviceState *gpio_a;
 DeviceState *gpio_b;
 DeviceState *gpio_c;
 DeviceState *gpio_d;
 DeviceState *uart1;
 DeviceState *uart2;
 DeviceState *uart3;
} Stm32_board;

Stm32_board * s;

extern unsigned short ADC_values[31];
//prototypes
void qemu_picsimlab_register(void (*picsimlab_write_pin_)(int pin,int value));
void qemu_picsimlab_set_apin(int chn,int value);
void qemu_picsimlab_set_pin(int pin,int value);

void (*picsimlab_write_pin)(int pin,int value) = NULL;

void qemu_picsimlab_register(void (*picsimlab_write_pin_)(int pin,int value))
{
  picsimlab_write_pin = picsimlab_write_pin_;
}

void qemu_picsimlab_set_pin(int pin,int value)
{
   qemu_mutex_lock_iothread ();
   if (value){
      qemu_irq_raise (s->pin_irq[pin]);
   }
   else{
      qemu_irq_lower (s->pin_irq[pin]);
   }
   qemu_mutex_unlock_iothread ();
}

void qemu_picsimlab_set_apin(int chn,int value)
{
   ADC_values[chn] = value;
}

static void
pout_irq_handler(void *opaque, int n, int level)
{
   (*picsimlab_write_pin)(n,level);
}


#define FLASH_SIZE 0x00020000
#define RAM_SIZE 0x00005000
/* Main SYSCLK frequency in Hz (24MHz) */
#define SYSCLK_FRQ 24000000ULL

static void
stm32_f103c8_picsimlab_init(MachineState *machine)
{

 Clock *sysclk;

 s = (Stm32_board *) g_malloc0 (sizeof (Stm32_board));

 sysclk = clock_new(OBJECT(machine), "SYSCLK");
 clock_set_hz(sysclk, SYSCLK_FRQ);
 stm32_init (FLASH_SIZE,
             RAM_SIZE,
             machine->kernel_filename,
             8000000,
             32768,
             sysclk);

 s->gpio_a = DEVICE (object_resolve_path ("/machine/stm32/gpio[a]", NULL));
 s->gpio_b = DEVICE (object_resolve_path ("/machine/stm32/gpio[b]", NULL));
 s->gpio_c = DEVICE (object_resolve_path ("/machine/stm32/gpio[c]", NULL));
 s->gpio_d = DEVICE (object_resolve_path ("/machine/stm32/gpio[d]", NULL));
 s->uart1 = DEVICE (object_resolve_path ("/machine/stm32/uart[1]", NULL));
 s->uart2 = DEVICE (object_resolve_path ("/machine/stm32/uart[2]", NULL));
 s->uart3 = DEVICE (object_resolve_path ("/machine/stm32/uart[3]", NULL));
 assert (s->gpio_a);
 assert (s->gpio_b);
 assert (s->gpio_c);
 assert (s->gpio_d);
 assert (s->uart2);
 assert (s->uart1);
 assert (s->uart3);

 s->pout_irq = qemu_allocate_irqs (pout_irq_handler, NULL, 49);

 //0
 //VBAT
 qdev_connect_gpio_out (s->gpio_c, 13, s->pout_irq[2]);
 s->pin_irq[2] = qdev_get_gpio_in (s->gpio_c, 13);
 qdev_connect_gpio_out (s->gpio_c, 14, s->pout_irq[3]);
 s->pin_irq[3] = qdev_get_gpio_in (s->gpio_c, 14);
 qdev_connect_gpio_out (s->gpio_c, 15, s->pout_irq[4]);
 s->pin_irq[4] = qdev_get_gpio_in (s->gpio_c, 15);
 qdev_connect_gpio_out (s->gpio_d, 0, s->pout_irq[5]);
 s->pin_irq[5] = qdev_get_gpio_in (s->gpio_d, 0);
 qdev_connect_gpio_out (s->gpio_d, 1, s->pout_irq[6]);
 s->pin_irq[6] = qdev_get_gpio_in (s->gpio_d, 1);
 //7 NRST
 //8 VSSA
 //9 VDDA
 qdev_connect_gpio_out (s->gpio_a, 0, s->pout_irq[10]);
 s->pin_irq[10] = qdev_get_gpio_in (s->gpio_a, 0);
 qdev_connect_gpio_out (s->gpio_a, 1, s->pout_irq[11]);
 s->pin_irq[11] = qdev_get_gpio_in (s->gpio_a, 1);
 qdev_connect_gpio_out (s->gpio_a, 2, s->pout_irq[12]);
 s->pin_irq[12] = qdev_get_gpio_in (s->gpio_a, 2);

 qdev_connect_gpio_out (s->gpio_a, 3, s->pout_irq[13]);
 s->pin_irq[13] = qdev_get_gpio_in (s->gpio_a, 3);
 qdev_connect_gpio_out (s->gpio_a, 4, s->pout_irq[14]);
 s->pin_irq[14] = qdev_get_gpio_in (s->gpio_a, 4);
 qdev_connect_gpio_out (s->gpio_a, 5, s->pout_irq[15]);
 s->pin_irq[15] = qdev_get_gpio_in (s->gpio_a, 5);
 qdev_connect_gpio_out (s->gpio_a, 6, s->pout_irq[16]);
 s->pin_irq[16] = qdev_get_gpio_in (s->gpio_a, 6);
 qdev_connect_gpio_out (s->gpio_a, 7, s->pout_irq[17]);
 s->pin_irq[17] = qdev_get_gpio_in (s->gpio_a, 7);
 qdev_connect_gpio_out (s->gpio_b, 0, s->pout_irq[18]);
 s->pin_irq[18] = qdev_get_gpio_in (s->gpio_b, 0);
 qdev_connect_gpio_out (s->gpio_b, 1, s->pout_irq[19]);
 s->pin_irq[19] = qdev_get_gpio_in (s->gpio_b, 1);
 qdev_connect_gpio_out (s->gpio_b, 2, s->pout_irq[20]);
 s->pin_irq[20] = qdev_get_gpio_in (s->gpio_b, 2);
 qdev_connect_gpio_out (s->gpio_b, 10, s->pout_irq[21]);
 s->pin_irq[21] = qdev_get_gpio_in (s->gpio_b, 10);
 qdev_connect_gpio_out (s->gpio_b, 11, s->pout_irq[22]);
 s->pin_irq[22] = qdev_get_gpio_in (s->gpio_b, 11);
 //23 VSS
 //24 VDD

 qdev_connect_gpio_out (s->gpio_b, 12, s->pout_irq[25]);
 s->pin_irq[25] = qdev_get_gpio_in (s->gpio_b, 12);
 qdev_connect_gpio_out (s->gpio_b, 13, s->pout_irq[26]);
 s->pin_irq[26] = qdev_get_gpio_in (s->gpio_b, 13);
 qdev_connect_gpio_out (s->gpio_b, 14, s->pout_irq[27]);
 s->pin_irq[27] = qdev_get_gpio_in (s->gpio_b, 14);
 qdev_connect_gpio_out (s->gpio_b, 15, s->pout_irq[28]);
 s->pin_irq[28] = qdev_get_gpio_in (s->gpio_b, 15);
 qdev_connect_gpio_out (s->gpio_a, 8, s->pout_irq[29]);
 s->pin_irq[29] = qdev_get_gpio_in (s->gpio_a, 8);
 qdev_connect_gpio_out (s->gpio_a, 9, s->pout_irq[30]);
 s->pin_irq[30] = qdev_get_gpio_in (s->gpio_a, 9);
 qdev_connect_gpio_out (s->gpio_a, 10, s->pout_irq[31]);
 s->pin_irq[31] = qdev_get_gpio_in (s->gpio_a, 10);
 qdev_connect_gpio_out (s->gpio_a, 11, s->pout_irq[32]);
 s->pin_irq[32] = qdev_get_gpio_in (s->gpio_a, 11);
 qdev_connect_gpio_out (s->gpio_a, 12, s->pout_irq[33]);
 s->pin_irq[33] = qdev_get_gpio_in (s->gpio_a, 12);
 qdev_connect_gpio_out (s->gpio_a, 13, s->pout_irq[34]);
 s->pin_irq[34] = qdev_get_gpio_in (s->gpio_a, 13);
 //35 VSS
 //36 VDD

 qdev_connect_gpio_out (s->gpio_a, 14, s->pout_irq[37]);
 s->pin_irq[37] = qdev_get_gpio_in (s->gpio_a, 14);
 qdev_connect_gpio_out (s->gpio_a, 15, s->pout_irq[38]);
 s->pin_irq[38] = qdev_get_gpio_in (s->gpio_a, 15);
 qdev_connect_gpio_out (s->gpio_b, 3, s->pout_irq[39]);
 s->pin_irq[39] = qdev_get_gpio_in (s->gpio_b, 3);
 qdev_connect_gpio_out (s->gpio_b, 4, s->pout_irq[40]);
 s->pin_irq[40] = qdev_get_gpio_in (s->gpio_b, 4);
 qdev_connect_gpio_out (s->gpio_b, 5, s->pout_irq[41]);
 s->pin_irq[41] = qdev_get_gpio_in (s->gpio_b, 5);
 qdev_connect_gpio_out (s->gpio_b, 6, s->pout_irq[42]);
 s->pin_irq[42] = qdev_get_gpio_in (s->gpio_b, 6);
 qdev_connect_gpio_out (s->gpio_b, 7, s->pout_irq[43]);
 s->pin_irq[43] = qdev_get_gpio_in (s->gpio_b, 7);
 //44 BOOT0 
 qdev_connect_gpio_out (s->gpio_b, 8, s->pout_irq[45]);
 s->pin_irq[45] = qdev_get_gpio_in (s->gpio_b, 8);
 qdev_connect_gpio_out (s->gpio_b, 9, s->pout_irq[46]);
 s->pin_irq[46] = qdev_get_gpio_in (s->gpio_b, 9);
 //47 VSS
 //48 VDD


 /* Connect RS232 to UART 1 */
 stm32_uart_connect (
                     (Stm32Uart *) s->uart1,
                     serial_hd(0),
                     STM32_USART1_NO_REMAP);

 /* These additional UARTs have not been tested yet... */
 stm32_uart_connect (
                     (Stm32Uart *) s->uart2,
                     serial_hd(1),
                     STM32_USART2_NO_REMAP);

 stm32_uart_connect (
                     (Stm32Uart *) s->uart3,
                     serial_hd(2),
                     STM32_USART3_NO_REMAP);

 armv7m_load_kernel(ARM_CPU(first_cpu),
                       machine->kernel_filename,
                       FLASH_SIZE);
}

static void stm32_f103c8_picsimlab_machine_init(MachineClass *mc)
{
    mc->desc = "STM32F103C8 (Blue Pill) Dev Board (PICSimLab)";
    mc->init = stm32_f103c8_picsimlab_init;
    mc->block_default_type = IF_IDE;
    mc->ignore_memory_transaction_failures = true;
}

DEFINE_MACHINE("stm32-f103c8-picsimlab-new", stm32_f103c8_picsimlab_machine_init)


