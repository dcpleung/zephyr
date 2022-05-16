.. _i3c_api:

I3C
###

Overview
********

I3C (Improved Inter-Integrated Circuit) is a two-signal shared
peripheral interface bus.  Devices on the bus can operate in
two roles: as a "controller" that initiates transactions and
controls the clock, or as a "target" that responds to transaction
commands.

Currently, the API is based on `I3C Specification`_ version 1.1.1.

.. _i3c-controller-api:

I3C Controller API
==================

Zephyr's I3C controller API is used when an I3C controller controls
the bus, in particularly the start and stop conditions and the clock.
This is the most common mode, used to interact with I3C target
devices such as sensors.

Due to the nature of the I3C, there are devices on the bus where
they do not have addresses when powered on. Therefore, an additional
dynamic address assignment needs to be carried out by the I3C
controller. Here is a list of generic steps for initializing the I3C
controller by its device driver:

#. Initialize the data structure of the I3C controller device
   driver instance. This initialization function is provided to
   :c:macro:`I3C_DEVICE_DT_INST_DEFINE`.

   * Prepare the data struct to store device list when
     :c:func:`i3c_device_register` is called.

     * The :c:struct:`i3c_dev_list` is a generic structure to
       store the device list. If this is being used,
       this struct needs to be initialized by calling
       :c:func:`i3c_dev_list_init`.

#. The target devices on the same bus need to call
   :c:func:`i3c_device_register` so the controller device driver
   instance is aware of these target devices. This tells
   the controller device driver instance how to assign addresses
   (static vs. dynamic).

   * For I\ :sup:`2`\ C devices on the bus, they need to call
     :c:func:`i3c_i2c_device_register` to announce their presence.

   * Note that the controller should not assign addresses to
     devices if their device drivers have not registered their
     presence. This is due to the API requiring a device descriptor
     struct to be supplied, and not-yet-registered devices do not
     have associated descriptors.

   * An extra initialization function can be provided to
     :c:macro:`I3C_DEVICE_REGISTER_INIT` to aid in device
     registration. This function needs to call
     :c:func:`i3c_device_register` or
     :c:func:`i3c_i2c_device_register` according to device type.

#. The bus initialization function provided to
   :c:macro:`I3C_BUS_INIT` is called to initialize the bus.

   #. Initialize the hardware, including but not limited to:

      * Setup pin mux and directions.

      * Setup the clock for the controller.

      * Power on the hardware.

      * Configure the hardware (e.g. SCL clock frequency).

   #. Do ``RSTDAA`` to reset dynamic addresses of connected devices.
      If any connected devices have already been assigned an address,
      the bookkeeping data structures do not have records of these,
      for example, at power-on. So it is a good idea to reset and
      assign them new addresses.

   #. Do ``SETDASA`` to use static addresses as dynamic address
      if so desired.

      * ``SETAASA`` may not be supported for all connected devices
        to assign static addresses as dynamic addresses.

      * BCR and DCR need to be obtained separately to populate
        the relevant fields in the I3C target device descriptor
        struct.

   #. Do ``ENTDAA`` to start dynamic address assignment.

      * If there is a device waiting for address, it will send
        its Provisioned ID, BCR, and DCR back. Match the received
        Provisioned ID to the list of registered I3C devices.

        * If there is a match, assign an address (either from
          the stated static address if ``SETDASA`` has not been
          done, or use a free address).

          * Also, set the BCR and DCR fields in the device descriptor
            struct.

        * If there is no match, depending on policy, it can be
          assigned a free address, or the device driver can stop
          the assignment process and errors out.

          * Note that the I3C API requires device descriptor to
            function. A device without a device descriptor cannot be
            accessed through the API.

      * This step can be skipped if there is no connected devices
        requiring DAA.

   #. These are optional but highly recommended:

      * Do ``GETMRL`` and ``GETMWL`` to get maximum read/write
        length.

      * Do ``GETMXDS`` to get maximum read/write speed and maximum
        read turnaround time.

In-Band Interrupt (IBI)
-----------------------

If a target device can generate In-Band Interrupt (IBI),
the controller needs to be made aware of it.

* :c:func:`i3c_ibi_slot_request` to request/allocate an IBI slot
  in the controller.

  * This sets up the IBI slot so the controller can recognize
    incoming IBI from target devices.

  * This only programs the IBI slot, and the IBI is not active.
    Call :c:func:`i3c_ibi_enable` to activate the slot.

  * Note that there are usually limited IBI slots on
    the controller so this operation may fail.

* :c:func:`i3c_ibi_slot_free` to free a previosly allocated IBI
  slot. Use this when IBI is no longer needed to recognized by
  the controller.

* :c:func:`i3c_ibi_enable` enables the controller to raise event(s)
  when there is an incoming IBI.

* :c:func:`i3c_ibi_disable` tells controller not to raise any
  event(s) where there is incoming IBIs.

Device Tree
-----------

Here is an example for defining a I3C controller in device tree:

.. code-block:: devicetree

   i3c0: i3c@10000 {
           compatible = "vendor,i3c";

           #address-cells = < 0x3 >;
           #size-cells = < 0x0 >;

           reg = < 0x10000 0x1000 >;
           interrupts = < 0x1F 0x0 >;

           pinctrl-0 = < &pinmux-i3c >;
           pinctrl-names = "default";

           i2c-scl-hz = < 400000 >;

           i3c-scl-hz = < 12000000 >;

           status = "okay";

           i3c-dev0: i3c-dev0@420000ABCD12345678 {
                   compatible = "vendor,i3c-dev";

                   reg = < 0x42 0xABCD 0x12345678 >;

                   status = "okay";
           };

           i2c-dev0: i2c-dev0@380000000000000050 {
                   compatible = "vendor-i2c-dev";

                   reg = < 0x38 0x0 0x50 >;

                   status = "okay";
           };
   };

I3C Devices
^^^^^^^^^^^

For I3C devices, the ``reg`` property has 3 elements:

* The first one is the static address of the device.

  * Can be zero if static address is not used. Address will be
    assigned during DAA (Dynamic Address Assignment).

  * If non-zero and property ``assigned-address`` is not set,
    this will be the address of the device after SETDASA
    (Set Dynamic Address from Static Address) is issued.

* Second element is the upper 16-bit of the Provisioned ID (PID)
  which contains the manufacturer ID left-shifted by 1. This is
  the bits 33-47 (zero-based) of the 48-bit Provisioned ID.

* Third element contains the lower 32-bit of the Provisioned ID
  which is a combination of the part ID (left-shifted by 16,
  bits 16-31 of the PID) and the instance ID (left-shifted by 12,
  bits 12-15 of the PID).

Note that the unit-address (the part after ``@``) must match
the ``reg`` property fully where each element is treated as
32-bit integer, combining to form a 96-bit integer. This is
required for properly generating device tree macros.

I\ :sup:`2`\ C Devices
^^^^^^^^^^^^^^^^^^^^^^

For I\ :sup:`2`\ C devices where the device driver has support for
working under I3C bus, the device node can be described as
a child of the I3C controller. If the device driver is written to
only work with I\ :sup:`2`\ C controllers, define the node under
the I\ :sup:`2`\ C virtual controller as described below.
Otherwise, the ``reg`` property, similar to I3C devices,
has 3 elements:

* The first one is the static address of the device. This must be
  a valid address as I\ :sup:`2`\ C devices do not support
  dynamic address assignment.

* Second element is always zero.

* Third element is the LVR (Legacy Virtual Register):

  * bit[31:8] are unused.

  * bit[7:5] are the I\ :sup:`2`\ C device index:

    * Index ``0``

      * I3C device has a 50 ns spike filter where it is not
        affected by high frequency on SCL.

    * Index ``1``

      * I\ :sup:`2`\ C device does not have a 50 ns spike filter but
        can work with high frequency on SCL.

    * Index ``2``

      * I3C device does not have a 50 ns spike filter and
        cannot work with high frequency on SCL.

  * bit[4] is the I\ :sup:`2`\ C mode indicator:

    * ``0`` is FM+ mode.

    * ``1`` is FM mode.

Similar to I3C devices, the unit-address must match the ``reg``
property fully where each element is treated as 32-bit integer,
combining to form a 96-bit integer.

I\ :sup:`2`\ C Virtual Controller under I3C Bus
-----------------------------------------------

The I\ :sup:`2`\ C virtual controller device driver provides a way to
interface I\ :sup:`2`\ C devices on the I3C bus where the associated
device drivers can be used as-is without modifications. This requires
adding an intermediate node in the device tree:

.. code-block:: devicetree

   i3c0: i3c@10000 {
           <... I3C controller related properties ...>

           i3c0-i2c-ctrl: i3c-i2c-ctrl@ff0000000000000000 {
                   compatible = "virtual,i3c-i2c-controller";

                   reg = < 0xff 0x00 0x00 >;

                   lvr = < 0x50 >;

                   #address-cells = < 0x01 >;
                   #size-cells = < 0x00 >;

                   status = "okay";

                   i2c-dev0: i2c-dev0@42 {
                           compatible = "virtual,i2c-dev";

                           reg = < 0x42 > ;

                           status = "okay";
                   };
           };
   };

* The ``reg`` property has invalid I\ :sup:`2`\ C/I3C address (in the example
  above, ``0xff`` and two other values to satisfy the address cell requirement
  as a child of the I3C controller.

* The ``lvr`` property is a common LVR (Legacy Virtual Register) for all
  the I\ :sup:`2`\ C devices under the virtual bridge.

* Each node for I\ :sup:`2`\ C device can be defined the same as if it is under
  an I\ :sup:`2`\ C controller.

Configuration Options
*********************

Related configuration options:

* :kconfig:option:`CONFIG_I3C`
* :kconfig:option:`CONFIG_I3C_USE_GROUP_ADDR`
* :kconfig:option:`CONFIG_I3C_USE_IBI`
* :kconfig:option:`CONFIG_I3C_IBI_MAX_PAYLOAD_SIZE`

API Reference
*************

.. doxygengroup:: i3c_interface
.. doxygengroup:: i3c_ccc
.. doxygengroup:: i3c_addresses

Links
*****

.. _I3C Specification: https://www.mipi.org/specifications/i3c-sensor-specification
