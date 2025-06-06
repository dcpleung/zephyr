.. zephyr:board:: xg24_ek2703a

Overview
********

The EFR32xG24 Explorer Kit (xG24-EK2703A) contains
a Wireless System-On-Chip from the EFR32MG24 family built on an
ARM Cortex®-M33 processor with excellent low power capabilities.

Hardware
********

- EFR32MG24B210F1536IM48-B Mighty Gecko SoC
- CPU core: ARM Cortex®-M33 with FPU
- Flash memory: 1536 kB
- RAM: 256 kB
- Transmit power: up to +10 dBm
- Operation frequency: 2.4 GHz
- Crystals for LFXO (32.768 kHz) and HFXO (39 MHz).

For more information about the EFR32MG24 SoC and BRD2703A board, refer to these
documents:

- `EFR32MG24 Website`_
- `EFR32MG24 Datasheet`_
- `EFR32xG24 Reference Manual`_
- `BRD2703A User Guide`_

Supported Features
==================

.. zephyr:board-supported-hw::

Connections and IOs
===================

In the following table, the column **Name** contains Pin names. For example, PA2
means Pin number 2 on PORTA, as used in the board's datasheets and manuals.

+-------+-------------+-------------------------------------+
| Name  | Function    | Usage                               |
+=======+=============+=====================================+
| PA4   | GPIO        | LED0                                |
+-------+-------------+-------------------------------------+
| PA7   | GPIO        | LED1                                |
+-------+-------------+-------------------------------------+
| PB2   | GPIO        | Push Button 0                       |
+-------+-------------+-------------------------------------+
| PB3   | GPIO        | Push Button 1                       |
+-------+-------------+-------------------------------------+
| PA5   | USART0_TX   | UART Console VCOM_TX US0_TX         |
+-------+-------------+-------------------------------------+
| PA6   | USART0_RX   | UART Console VCOM_RX US0_RX         |
+-------+-------------+-------------------------------------+

The default configuration can be found in
:zephyr_file:`boards/silabs/dev_kits/xg24_ek2703a/xg24_ek2703a_defconfig`

System Clock
============

The EFR32MG24 SoC is configured to use the 39 MHz external oscillator on the
board.

Serial Port
===========

The EFR32MG24 SoC has one USART and two EUSARTs.
USART0 is connected to the board controller and is used for the console.

Programming and Debugging
*************************

.. zephyr:board-supported-runners::

.. note::
   Before using the kit the first time, you should update the J-Link firmware
   in Simplicity Studio.

Flashing
========

The sample application :zephyr:code-sample:`hello_world` is used for this example.
Build the Zephyr kernel and application:

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: xg24_ek2703a
   :goals: build

Connect the xg24_ek2703a to your host computer using the USB port and you
should see a USB connection.

Open a serial terminal (minicom, putty, etc.) with the following settings:

- Speed: 115200
- Data: 8 bits
- Parity: None
- Stop bits: 1

Reset the board and you'll see the following message on the corresponding serial port
terminal session:

.. code-block:: console

   Hello World! xg24_ek2703a

Bluetooth
=========

To use the BLE function, run the command below to retrieve necessary binary
blobs from the SiLabs HAL repository.

.. code-block:: console

   west blobs fetch hal_silabs

Then build the Zephyr kernel and a Bluetooth sample with the following
command. The :zephyr:code-sample:`bluetooth_observer` sample application is used in
this example.

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/observer
   :board: xg24_ek2703a
   :goals: build

.. _EFR32MG24 Website:
   https://www.silabs.com/wireless/zigbee/efr32mg24-series-2-socs#

.. _EFR32MG24 Datasheet:
   https://www.silabs.com/documents/public/data-sheets/efr32mg24-datasheet.pdf

.. _EFR32xG24 Reference Manual:
   https://www.silabs.com/documents/public/reference-manuals/efr32xg24-rm.pdf

.. _BRD2703A User Guide:
   https://www.silabs.com/documents/public/user-guides/ug533-xg24-ek2703a.pdf
