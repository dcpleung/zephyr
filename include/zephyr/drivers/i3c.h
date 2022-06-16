/*
 * Copyright 2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_I3C_H_
#define ZEPHYR_INCLUDE_DRIVERS_I3C_H_

/**
 * @brief I3C Interface
 * @defgroup i3c_interface I3C Interface
 * @ingroup io_interfaces
 * @{
 */

#include <zephyr/types.h>
#include <zephyr/device.h>

#include <zephyr/drivers/i3c/addresses.h>
#include <zephyr/drivers/i3c/ccc.h>
#include <zephyr/drivers/i3c/ibi.h>
#include <zephyr/drivers/i2c.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Bus Characteristic Register (BCR)
 * - BCR[7:6]: Device Role
 *   - 0: I3C Target
 *   - 1: I3C Controller capable
 *   - 2: Reserved
 *   - 3: Reserved
 * - BCR[5]: Advanced Capabilities
 *   - 0: Does not support optional advanced capabilities.
 *   - 1: Supports optional advanced capabilities which
 *        can be viewed via GETCAPS CCC.
 * - BCR[4}: Virtual Target Support
 *   - 0: Is not a virtual target.
 *   - 1: Is a virtual target.
 * - BCR[3]: Offline Capable
 *   - 0: Will always response to I3C commands.
 *   - 1: Will not always response to I3C commands.
 * - BCR[2]: IBI Payload
 *   - 0: No data bytes following the accepted IBI.
 *   - 1: One data byte (MDB, Mandatory Data Byte) follows
 *        the accepted IBI. Additional data bytes may also
 *        follows.
 * - BCR[1]: IBI Request Capable
 *   - 0: Not capable
 *   - 1: Capable
 * - BCR[0]: Max Data Speed Limitation
 *   - 0: No Limitation
 *   - 1: Limitation obtained via GETMXDS CCC.
 */
#define I3C_BCR_MAX_DATA_SPEED_LIMIT			BIT(0)
#define I3C_BCR_IBI_REQUEST_CAPABLE			BIT(1)
#define I3C_BCR_IBI_PAYLOAD_HAS_DATA_BYTE		BIT(2)
#define I3C_BCR_OFFLINE_CAPABLE				BIT(3)
#define I3C_BCR_VIRTUAL_TARGET				BIT(4)
#define I3C_BCR_ADV_CAPABILITIES			BIT(5)

#define I3C_BCR_DEVICE_ROLE_I3C_TARGET			0U
#define I3C_BCR_DEVICE_ROLE_I3C_CONTROLLER_CAPABLE	1U

#define I3C_BCR_DEVICE_ROLE_SHIFT			6U
#define I3C_BCR_DEVICE_ROLE_MASK			(0x03U << I3C_BCR_DEVICE_ROLE_SHIFT)

#define I3C_BCR_DEVICE_ROLE(bcr)			\
	(((bcr) & I3C_BCR_DEVICE_ROLE_MASK) >> I3C_BCR_DEVICE_ROLE_SHIFT)

/*
 * Legacy Virtual Register (LVR)
 * - LVR[7:5]: I2C device index:
 *   - 0: I2C device has a 50 ns spike filter where
 *        it is not affected by high frequency on SCL.
 *   - 1: I2C device does not have a 50 ns spike filter
 *        but can work with high frequency on SCL.
 *   - 2: I2C device does not have a 50 ns spike filter
 *        and cannot work with high frequency on SCL.
 * - LVR[4]: I2C mode indicator:
 *   - 0: FM+ mode
 *   - 1: FM mode
 * - LVR[3:0]: Reserved.
 */
#define I3C_DCR_I2C_FM_PLUS_MODE			0
#define I3C_DCR_I2C_FM_MODE				1

#define I3C_DCR_I2C_MODE_SHIFT				4
#define I3C_DCR_I2C_MODE_MASK				BIT(4)

#define I3C_DCR_I2C_MODE(dcr)				\
	(((mode) & I3C_DCR_I2C_MODE_MASK) >> I3C_DCR_I2C_MODE_SHIFT)

#define I3C_DCR_I2C_DEV_IDX_0				0
#define I3C_DCR_I2C_DEV_IDX_1				1
#define I3C_DCR_I2C_DEV_IDX_2				2

#define I3C_DCR_I2C_DEV_IDX_SHIFT			5
#define I3C_DCR_I2C_DEV_IDX_MASK			(0x07U << I3C_DCR_I2C_DEV_IDX_SHIFT)

#define I3C_DCR_I2C_DEV_IDX(dcr)			\
	(((dcr) & I3C_DCR_I2C_DEV_IDX_MASK) >> I3C_DCR_I2C_DEV_IDX_SHIFT)

/**
 * @brief I3C bus mode
 */
enum i3c_bus_mode {
	/** Only I3C devices are on the bus. */
	I3C_BUS_MODE_PURE,

	/**
	 * Both I3C and legacy I2C devices are on the bus.
	 * The I2C devices have 50ns spike filter on SCL.
	 */
	I3C_BUS_MODE_MIXED_FAST,

	/**
	 * Both I3C and legacy I2C devices are on the bus.
	 * The I2C devices do not have 50ns spike filter on SCL
	 * and can tolerate maximum SDR SCL clock frequency.
	 */
	I3C_BUS_MODE_MIXED_LIMITED,

	/**
	 * Both I3C and legacy I2C devices are on the bus.
	 * The I2C devices do not have 50ns spike filter on SCL
	 * but cannot tolerate maximum SDR SCL clock frequency.
	 */
	I3C_BUS_MODE_MIXED_SLOW,

	I3C_BUS_MODE_MAX = I3C_BUS_MODE_MIXED_SLOW,
	I3C_BUS_MODE_INVALID,
};

/**
 * @brief I2C bus speed under I3C bus.
 *
 * Only FM and FM+ modes are supported for I2C devices under I3C bus.
 */
enum i3c_i2c_speed_type {
	/** I2C FM mode */
	I3C_I2C_SPEED_FM,

	/** I2C FM+ mode */
	I3C_I2C_SPEED_FMPLUS,

	I3C_I2C_SPEED_MAX = I3C_I2C_SPEED_FMPLUS,
	I3C_I2C_SPEED_INVALID,
};

/**
 * @brief I3C data rate
 *
 * I3C data transfer rate defined by the I3C specification.
 */
enum i3c_data_rate {
	/** Single Data Rate messaging */
	I3C_DATA_RATE_SDR,

	/** High Data Rate - Double Data Rate messaging */
	I3C_DATA_RATE_HDR_DDR,

	/** High Data Rate - Ternary Symbol Legacy-inclusive-Bus */
	I3C_DATA_RATE_HDR_TSL,

	/** High Data Rate - Ternary Symbol for Pure Bus */
	I3C_DATA_RATE_HDR_TSP,

	/** High Data Rate - Bulk Transport */
	I3C_DATA_RATE_HDR_BT,

	I3C_DATA_RATE_MAX = I3C_DATA_RATE_HDR_BT,
	I3C_DATA_RATE_INVALID,
};

/**
 * @brief I3C SDR Controller Error Codes
 *
 * These are error codes defined by the I3C specification.
 *
 * @c I3C_ERROR_CE_UNKNOWN and @c I3C_ERROR_CE_NONE are not
 * official error codes according to the specification.
 * These are there simply to aid in error handling during
 * interactions with the I3C drivers and subsystem.
 */
enum i3c_sdr_controller_error_codes {
	/** Transaction after sending CCC */
	I3C_ERROR_CE0,

	/** Monitoring Error */
	I3C_ERROR_CE1,

	/** No response to broadcast address (0x7E) */
	I3C_ERROR_CE2,

	/** Failed Controller Handoff */
	I3C_ERROR_CE3,

	/** Unknown error (not official error code) */
	I3C_ERROR_CE_UNKNOWN,

	/** No error (not official error code) */
	I3C_ERROR_CE_NONE,

	I3C_ERROR_CE_MAX = I3C_ERROR_CE_UNKNOWN,
	I3C_ERROR_CE_INVALID,
};

/**
 * @brief I3C SDR Target Error Codes
 *
 * These are error codes defined by the I3C specification.
 *
 * @c I3C_ERROR_TE_UNKNOWN and @c I3C_ERROR_TE_NONE are not
 * official error codes according to the specification.
 * These are there simply to aid in error handling during
 * interactions with the I3C drivers and subsystem.
 */
enum i3c_sdr_target_error_codes {
	/**
	 * Invalid Broadcast Address or
	 * Dynamic Address after DA assignment
	 */
	I3C_ERROR_TE0,

	/** CCC Code */
	I3C_ERROR_TE1,

	/** Write Data */
	I3C_ERROR_TE2,

	/** Assigned Address during Dynamic Address Arbitration */
	I3C_ERROR_TE3,

	/** 0x7E/R missing after RESTART during Dynamic Address Arbitration */
	I3C_ERROR_TE4,

	/** Transaction after detecting CCC */
	I3C_ERROR_TE5,

	/** Monitoring Error */
	I3C_ERROR_TE6,

	/** Dead Bus Recovery */
	I3C_ERROR_DBR,

	/** Unknown error (not official error code) */
	I3C_ERROR_TE_UNKNOWN,

	/** No error (not official error code) */
	I3C_ERROR_TE_NONE,

	I3C_ERROR_TE_MAX = I3C_ERROR_TE_UNKNOWN,
	I3C_ERROR_TE_INVALID,
};

/*
 * I3C_MSG_* are I3C Message flags.
 */

/** Write message to I3C bus. */
#define I3C_MSG_WRITE			(0U << 0U)

/** Read message from I2C bus. */
#define I3C_MSG_READ			BIT(0)

/** @cond INTERNAL_HIDDEN */
#define I3C_MSG_RW_MASK			BIT(0)
/** @endcond  */

/** Send STOP after this message. */
#define I3C_MSG_STOP			BIT(1)

/**
 * RESTART I3C transaction for this message.
 *
 * @note Not all I3C drivers have or require explicit support for this
 * feature. Some drivers require this be present on a read message
 * that follows a write, or vice-versa.  Some drivers will merge
 * adjacent fragments into a single transaction using this flag; some
 * will not.
 */
#define I3C_MSG_RESTART			BIT(2)

/** Transfer use HDR mode */
#define I3C_MSG_HDR			BIT(3)

/** I3C HDR Mode 0 */
#define I3C_MSG_HDR_MODE0		BIT(0)

/** I3C HDR Mode 1 */
#define I3C_MSG_HDR_MODE1		BIT(1)

/** I3C HDR Mode 2 */
#define I3C_MSG_HDR_MODE2		BIT(2)

/** I3C HDR Mode 3 */
#define I3C_MSG_HDR_MODE3		BIT(3)

/** I3C HDR Mode 4 */
#define I3C_MSG_HDR_MODE4		BIT(4)

/** I3C HDR Mode 5 */
#define I3C_MSG_HDR_MODE5		BIT(5)

/** I3C HDR Mode 6 */
#define I3C_MSG_HDR_MODE6		BIT(6)

/** I3C HDR Mode 7 */
#define I3C_MSG_HDR_MODE7		BIT(7)

/** I3C HDR-DDR (Double Data Rate) */
#define I3C_MSG_HDR_DDR			I3C_MSG_HDR_MODE0

/** I3C HDR-TSP (Ternary Symbol Pure-bus) */
#define I3C_MSG_HDR_TSP			I3C_MSG_HDR_MODE1

/** I3C HDR-TSL (Ternary Symbol Legacy-inclusive-bus) */
#define I3C_MSG_HDR_TSL			I3C_MSG_HDR_MODE2

/** I3C HDR-BT (Bulk Transport) */
#define I3C_MSG_HDR_BT			I3C_MSG_HDR_MODE3

/**
 * @brief One I3C Message.
 *
 * This defines one I3C message to transact on the I3C bus.
 *
 * @note Some of the configurations supported by this API may not be
 * supported by specific SoC I3C hardware implementations, in
 * particular features related to bus transactions intended to read or
 * write data from different buffers within a single transaction.
 * Invocations of i3c_transfer() may not indicate an error when an
 * unsupported configuration is encountered.  In some cases drivers
 * will generate separate transactions for each message fragment, with
 * or without presence of @ref I3C_MSG_RESTART in #flags.
 */
struct i3c_msg {
	/** Data buffer in bytes */
	uint8_t			*buf;

	/** Length of buffer in bytes */
	uint32_t		len;

	/** Flags for this message */
	uint8_t			flags;

	/**
	 * HDR mode (@c I3C_MSG_HDR_MODE*) for transfer
	 * if any @c I3C_MSG_HDR_* is set in @c flags.
	 *
	 * Use SDR mode if none is set.
	 */
	uint8_t			hdr_mode;
};

/**
 * @brief Type of configuration being passed to configure function.
 */
enum i3c_config_type {
	I3C_CONFIG_CONTROLLER,
	I3C_CONFIG_TARGET,
	I3C_CONFIG_CUSTOM,
};

/**
 * @brief Configuration parameters for I3C hardware to act as controller.
 */
struct i3c_config_controller {
	/**
	 * If the controller is to be the primary controller
	 * of the bus.
	 */
	bool is_primary;

	/**
	 * If the controller is to be the secondary controller
	 * of the bus.
	 */
	bool is_secondary;

	struct {
		/** SCL frequency (in Hz) for I3C transfers. */
		uint32_t i3c;

		/** SCL frequency (in Hz) for I2C transfers. */
		uint32_t i2c;
	} scl;

	/**
	 * Bit mask of supported HDR modes (0 - 7).
	 *
	 * This can be used to enable or disable HDR mode
	 * supported by the hardware at runtime.
	 */
	uint8_t supported_hdr;
};

/**
 * @brief Configuration parameters for I3C hardware to act as target device.
 *
 * This can also be used to configure the controller if it is to act as
 * a secondary controller on the bus.
 */
struct i3c_config_target {
	/**
	 * If the hardware is to act as a target device
	 * on the bus.
	 */
	bool enable;

	/**
	 * I3C target address.
	 *
	 * Used used when operates as secondary controller
	 * or as a target device.
	 */
	uint8_t static_addr;

	/** Provisioned ID. */
	uint64_t pid;

	/**
	 * True if lower 32-bit of Provisioned ID is random.
	 *
	 * This sets the bit 32 of Provisioned ID which means
	 * the lower 32-bit is random value.
	 */
	bool pid_random;

	/** Bus Characteristics Register (BCR). */
	uint8_t bcr;

	/** Device Characteristics Register (DCR). */
	uint8_t dcr;

	/** Maximum Read Length (MRL). */
	uint16_t max_read_len;

	/** Maximum Write Length (MWL). */
	uint16_t max_write_len;

	/**
	 * Bit mask of supported HDR modes (0 - 7).
	 *
	 * This can be used to enable or disable HDR mode
	 * supported by the hardware at runtime.
	 */
	uint8_t supported_hdr;
};

/**
 * @brief Custom I3C configuration parameters.
 *
 * This can be used to configure the I3C hardware on parameters
 * not covered by @see i3c_config_controller or @see i3c_config_target.
 * Mostly used to configure vendor specific parameters of the I3C
 * hardware.
 */
struct i3c_config_custom {
	/** ID of the configuration parameter. */
	uint32_t id;

	union {
		/** Value of configuration parameter. */
		uintptr_t val;

		/**
		 * Pointer to configuration parameter.
		 *
		 * Mainly used to pointer to a struct that
		 * the device driver understands.
		 */
		void *ptr;
	};
};

/**
 * @cond INTERNAL_HIDDEN
 *
 * These are for internal use only, so skip these in
 * public documentation.
 */
struct i3c_device_desc;
struct i3c_i2c_device_desc;
struct i3c_target_config;

__subsystem struct i3c_driver_api {
	/**
	 * Configure the I3C hardware.
	 *
	 * @see i3c_configure
	 *
	 * @param dev Pointer to controller device driver instance.
	 * @param type Type of configuration parameters being passed
	 *             in @p config.
	 * @param config Pointer to the configuration parameters.
	 *
	 * @return @see i3c_configure
	 */
	int (*configure)(const struct device *dev,
			 enum i3c_config_type type, void *config);

	/**
	 * Get configuration of the I3C hardware.
	 *
	 * @see i3c_config_get
	 *
	 * @param[in] dev Pointer to controller device driver instance.
	 * @param[in] type Type of configuration parameters being passed
	 *                 in @p config.
	 * @param[in, out] config Pointer to the configuration parameters.
	 *
	 * @return @see i3c_config_get
	 */
	int (*config_get)(const struct device *dev,
			  enum i3c_config_type type, void *config);

	/**
	 * Perform bus recovery
	 *
	 * Controller only API.
	 *
	 * @see i3c_recover_bus
	 *
	 * @param dev Pointer to controller device driver instance.
	 *
	 * @return @see i3c_recover_bus
	 */
	int (*recover_bus)(const struct device *dev);

	/**
	 * Perform Dynamic Address Assignment via ENTDAA.
	 *
	 * Controller only API.
	 *
	 * @see i3c_do_daa
	 *
	 * @param dev Pointer to controller device driver instance.
	 *
	 * @return @see i3c_do_daa
	 */
	int (*do_daa)(const struct device *dev);

	/**
	 * Send Common Command Code (CCC).
	 *
	 * Controller only API.
	 *
	 * @see i3c_do_ccc
	 *
	 * @param dev Pointer to controller device driver instance.
	 * @param payload Pointer to the CCC payload.
	 *
	 * @return @see i3c_do_ccc
	 */
	int (*do_ccc)(const struct device *dev,
		      struct i3c_ccc_payload *payload);

	/**
	 * Transfer messages in I3C mode.
	 *
	 * @see i3c_transfer
	 *
	 * @param dev Pointer to controller device driver instance.
	 * @param target Pointer to target device descriptor.
	 * @param msg Pointer to I3C messages.
	 * @param num_msgs Number of messages to transfer.
	 *
	 * @return @see i3c_transfer
	 */
	int (*i3c_xfers)(const struct device *dev,
			 struct i3c_device_desc *target,
			 struct i3c_msg *msgs,
			 uint8_t num_msgs);

	/**
	 * Register a I3C target device with controller.
	 *
	 * Controller only API.
	 *
	 * This registers a I3C target device with the controller
	 * so that the controller is aware of such device.
	 *
	 * @see i3c_device_register
	 *
	 * @param dev Pointer to controller device driver instance.
	 * @param desc Pointer to target device descriptor.
	 *
	 * @return @see i3c_device_register
	 */
	int (*i3c_device_register)(const struct device *dev,
				   struct i3c_device_desc *desc);

	/**
	 * Transfer messages in I2C mode.
	 *
	 * @see i3c_i2c_transfer
	 *
	 * @param dev Pointer to controller device driver instance.
	 * @param target Pointer to target device descriptor.
	 * @param msg Pointer to I2C messages.
	 * @param num_msgs Number of messages to transfer.
	 *
	 * @return @see i3c_i2c_transfer
	 */
	int (*i2c_xfers)(const struct device *dev,
			 struct i3c_i2c_device_desc *i2c_dev,
			 struct i2c_msg *msgs,
			 uint8_t num_msgs);

	/**
	 * Register a I2C target device with controller.
	 *
	 * Controller only API.
	 *
	 * This registers a I2C target device with the controller
	 * so that the controller is aware of such device.
	 *
	 * @see i3c_i2c_device_register
	 *
	 * @param dev Pointer to controller device driver instance.
	 * @param desc Pointer to target device descriptor.
	 *
	 * @return @see i3c_i2c_device_register
	 */
	int (*i2c_device_register)(const struct device *dev,
				   struct i3c_i2c_device_desc *desc);

	/**
	 * Raise In-Band Interrupt (IBI).
	 *
	 * Target device only API.
	 *
	 * @see i3c_ibi_request
	 *
	 * @param dev Pointer to controller device driver instance.
	 * @param request Pointer to IBI request struct.
	 *
	 * @return @see i3c_ibi_request
	 */
	int (*ibi_raise)(const struct device *dev,
			 struct i3c_ibi *request);

	/**
	 * Enable receiving IBI from a target.
	 *
	 * Controller only API.
	 *
	 * @see i3c_ibi_enable
	 *
	 * @param dev Pointer to controller device driver instance.
	 * @param target Pointer to target device descriptor.
	 *
	 * @return @see i3c_ibi_enable
	 */
	int (*ibi_enable)(const struct device *dev,
			  struct i3c_device_desc *target);

	/**
	 * Disable receiving IBI from a target.
	 *
	 * Controller only API.
	 *
	 * @see i3c_ibi_disable
	 *
	 * @param dev Pointer to controller device driver instance.
	 * @param target Pointer to target device descriptor.
	 *
	 * @return @see i3c_ibi_disable
	 */
	int (*ibi_disable)(const struct device *dev,
			   struct i3c_device_desc *target);

	/**
	 * Register config as target device of a controller.
	 *
	 * This tells the controller to act as a target device
	 * on the I3C bus.
	 *
	 * Target device only API.
	 *
	 * @see i3c_target_register
	 *
	 * @param dev Pointer to the controller device driver instance.
	 * @param cfg I3C target device configuration
	 *
	 * @return @see i3c_target_register
	 */
	int (*target_register)(const struct device *dev,
			       struct i3c_target_config *cfg);

	/**
	 * Unregister config as target device of a controller.
	 *
	 * This tells the controller to stop acting as a target device
	 * on the I3C bus.
	 *
	 * Target device only API.
	 *
	 * @see i3c_target_unregister
	 *
	 * @param dev Pointer to the controller device driver instance.
	 * @param cfg I3C target device configuration
	 *
	 * @return @see i3c_target_unregister
	 */
	int (*target_unregister)(const struct device *dev,
				 struct i3c_target_config *cfg);
};

/**
 * @endcond
 */

/**
 * @brief Structure describing a I3C target device.
 *
 * Instances of this are passed to the I3C controller device APIs,
 * for example:
 * - i3c_device_register() to tell the controller of a target device.
 * - i3c_transfers() to initiate data transfers between controller and
 *   target device.
 *
 * Fields @c bus, @c pid and @c static_addr must be initialized by
 * the module that implements the target device behavior prior to
 * passing the object reference to I3C controller device APIs.
 * @c static_addr can be zero if target device does not have static
 * address.
 *
 * Field @c node should not be initialized or modified manually.
 */
struct i3c_device_desc {
	/** Private, do not modify */
	sys_snode_t node;

	/** I3C bus to which this target device is attached */
	const struct device * const bus;

	/** Device driver instance of the I3C device */
	const struct device * const dev;

	/** Device Provisioned ID */
	const uint64_t pid;

	/**
	 * Static address for this target device.
	 *
	 * 0 if static address is not being used, and only dynamic
	 * address is used. This means that the target device must
	 * go through ENTDAA (Dynamic Address Assignment) to get
	 * a dynamic address before it can communicate with
	 * the controller. This means SETAASA and SETDASA CCC
	 * cannot be used to set dynamic address on the target
	 * device (as both are to tell target device to use static
	 * address as dynamic address).
	 */
	const uint8_t static_addr;

	/**
	 * Initial dynamic address.
	 *
	 * This is specified in the device tree property "assigned-address"
	 * to indicate the desired dynamic address during address
	 * assignment (SETDASA and ENTDAA).
	 *
	 * 0 if there is no preference.
	 */
	const uint8_t init_dynamic_addr;

	/**
	 * Dynamic Address for this target device used for communication.
	 *
	 * This is to be set by the controller driver in one of
	 * the following situations:
	 * - During Dynamic Address Assignment (during ENTDAA)
	 * - Reset Dynamic Address Assignment (RSTDAA)
	 * - Set All Addresses to Static Addresses (SETAASA)
	 * - Set New Dynamic Address (SETNEWDA)
	 * - Set Dynamic Address from Static Address (SETDASA)
	 *
	 * 0 if address has not been assigned.
	 */
	uint8_t dynamic_addr;

#if defined(CONFIG_I3C_USE_GROUP_ADDR) || defined(__DOXYGEN__)
	/**
	 * Group address for this target device. Set during:
	 * - Reset Group Address(es) (RSTGRPA)
	 * - Set Group Address (SETGRPA)
	 *
	 * 0 if group address has not been assigned.
	 */
	uint8_t group_addr;
#endif /* CONFIG_I3C_USE_GROUP_ADDR */

	/**
	 * Bus Characteristic Register (BCR)
	 * - BCR[7:6]: Device Role
	 *   - 0: I3C Target
	 *   - 1: I3C Controller capable
	 *   - 2: Reserved
	 *   - 3: Reserved
	 * - BCR[5]: Advanced Capabilities
	 *   - 0: Does not support optional advanced capabilities.
	 *   - 1: Supports optional advanced capabilities which
	 *        can be viewed via GETCAPS CCC.
	 * - BCR[4}: Virtual Target Support
	 *   - 0: Is not a virtual target.
	 *   - 1: Is a virtual target.
	 * - BCR[3]: Offline Capable
	 *   - 0: Will always response to I3C commands.
	 *   - 1: Will not always response to I3C commands.
	 * - BCR[2]: IBI Payload
	 *   - 0: No data bytes following the accepted IBI.
	 *   - 1: One data byte (MDB, Mandatory Data Byte) follows
	 *        the accepted IBI. Additional data bytes may also
	 *        follows.
	 * - BCR[1]: IBI Request Capable
	 *   - 0: Not capable
	 *   - 1: Capable
	 * - BCR[0]: Max Data Speed Limitation
	 *   - 0: No Limitation
	 *   - 1: Limitation obtained via GETMXDS CCC.
	 */
	uint8_t bcr;

	/**
	 * Device Characteristic Register (DCR)
	 *
	 * Describes the type of device. Refer to official documentation
	 * on what this number means.
	 */
	uint8_t dcr;

	struct {
		/** Maximum Read Speed */
		uint8_t maxrd;

		/** Maximum Write Speed */
		uint8_t maxwr;

		/** Maximum Read turnaround time in microseconds. */
		uint32_t max_read_turnaround;
	} data_speed;

	struct {
		/** Maximum Read Length */
		uint16_t mrl;

		/** Maximum Write Length */
		uint16_t mwl;

		/** Maximum IBI Payload Size. Valid only if BCR[2] is 1. */
		uint8_t max_ibi;
	} data_length;

	/** Private data by the controller to aid in transactions. Do not modify. */
	void *controller_priv;

#if defined(CONFIG_I3C_USE_IBI) || defined(__DOXYGEN__)
	/**
	 * In-Band Interrupt (IBI) callback.
	 */
	i3c_target_ibi_cb_t ibi_cb;
#endif /* CONFIG_I3C_USE_IBI */
};

/**
 * @brief Structure initializer for i3c_device_desc from devicetree
 *
 * This helper macro expands to a static initializer for a <tt>struct
 * i3c_device_desc</tt> by reading the relevant bus and device data
 * from the devicetree, and an associated IBI callback.
 *
 * @param node_id Devicetree node identifier for the I3C device whose
 *                struct i3c_device_desc to create an initializer for.
 * @param ibi_cb Pointer to IBI callback.
 */
#define I3C_DEVICE_DESC_DT_WITH_IBI_CB(node_id, ibi_cb)			\
	{								\
		.bus = DEVICE_DT_GET(DT_BUS(node_id)),			\
		.dev = DEVICE_DT_GET(node_id),				\
		.static_addr = DT_PROP_BY_IDX(node_id, reg, 0),		\
		.pid = ((uint64_t)DT_PROP_BY_IDX(node_id, reg, 1) << 32)\
		       | DT_PROP_BY_IDX(node_id, reg, 2),		\
		.ibi_cb = ibi_cb,					\
		.init_dynamic_addr =					\
			DT_PROP_OR(node_id, assigned_address, 0),	\
	}

/**
 * @brief Structure initializer for i3c_device_desc from devicetree
 *
 * This helper macro expands to a static initializer for a <tt>struct
 * i3c_device_desc</tt> by reading the relevant bus and device data
 * from the devicetree.
 *
 * @param node_id Devicetree node identifier for the I3C device whose
 *                struct i3c_device_desc to create an initializer for
 */
#define I3C_DEVICE_DESC_DT(node_id)					\
	{								\
		.bus = DEVICE_DT_GET(DT_BUS(node_id)),			\
		.dev = DEVICE_DT_GET(node_id),				\
		.static_addr = DT_PROP_BY_IDX(node_id, reg, 0),		\
		.pid = ((uint64_t)DT_PROP_BY_IDX(node_id, reg, 1) << 32)\
		       | DT_PROP_BY_IDX(node_id, reg, 2),		\
		.init_dynamic_addr =					\
			DT_PROP_OR(node_id, assigned_address, 0),	\
	}

/**
 * @brief Structure initializer for i3c_device_desc from devicetree instance
 *
 * This is equivalent to
 * <tt>I3C_DEVICE_DESC_DT_WITH_IBI_PAYLOAD(DT_DRV_INST(inst), ibi_cb)</tt>.
 *
 * @param inst Devicetree instance number
 * @param ibi_cb Pointer to IBI callback.
 */
#define I3C_DEVICE_DESC_DT_INST_WITH_IBI_CB(inst, ibi_cb)		\
	I3C_DEVICE_DESC_DT_WITH_IBI_CB(DT_DRV_INST(inst, ibi_cb))

/**
 * @brief Structure initializer for i3c_device_desc from devicetree instance
 *
 * This is equivalent to
 * <tt>I3C_DEVICE_DESC_DT(DT_DRV_INST(inst))</tt>.
 *
 * @param inst Devicetree instance number
 */
#define I3C_DEVICE_DESC_DT_INST(inst) \
	I3C_DEVICE_DESC_DT(DT_DRV_INST(inst))

/**
 * @brief Like DEVICE_DT_DEFINE() with I3C target device specifics.
 *
 * Defines a I3C target device which implements the I3C target device API.
 *
 * @param node_id The devicetree node identifier.
 *
 * @param init_fn Name of the init function of the driver.
 *
 * @param pm_device PM device resources reference (NULL if device does not use PM).
 *
 * @param data_ptr Pointer to the device's private data.
 *
 * @param cfg_ptr The address to the structure containing the
 *                configuration information for this instance of the driver.
 *
 * @param level The initialization level. See SYS_INIT() for
 *              details.
 *
 * @param prio Priority within the selected initialization level. See
 *             SYS_INIT() for details.
 *
 * @param api_ptr Provides an initial pointer to the API function struct
 *                used by the driver. Can be NULL.
 */
#define I3C_DEVICE_DT_DEFINE(node_id, init_fn, pm_device,		\
			     data_ptr, cfg_ptr, level, prio,		\
			     api_ptr, ...)				\
	DEVICE_DT_DEFINE(node_id, init_fn, pm_device,			\
			 data_ptr, cfg_ptr, level, prio,		\
			 api_ptr, __VA_ARGS__)

/**
 * @brief Like I3C_TARGET_DT_DEFINE() for an instance of a DT_DRV_COMPAT compatible
 *
 * @param inst instance number. This is replaced by
 * <tt>DT_DRV_COMPAT(inst)</tt> in the call to I3C_TARGET_DT_DEFINE().
 *
 * @param ... other parameters as expected by I3C_TARGET_DT_DEFINE().
 */
#define I3C_DEVICE_DT_INST_DEFINE(inst, ...)				\
	I3C_DEVICE_DT_DEFINE(DT_DRV_INST(inst), __VA_ARGS__)

/**
 * @brief Structure describing a I2C device on I3C bus.
 *
 * Instances of this are passed to the I3C controller device APIs,
 * for example:
 * () i3c_i2c_device_register() to tell the controller of an I2C device.
 * () i3c_i2c_transfers() to initiate data transfers between controller
 *    and I2C device.
 *
 * Fields other than @c node must be initialized by the module that
 * implements the device behavior prior to passing the object
 * reference to I3C controller device APIs.
 */
struct i3c_i2c_device_desc {
	/** Private, do not modify */
	sys_snode_t node;

	/** I3C bus to which this I2C device is attached */
	const struct device *bus;

	/** Static address for this I2C device. */
	const uint16_t addr;

	/**
	 * Legacy Virtual Register (LVR)
	 * - LVR[7:5]: I2C device index:
	 *   - 0: I2C device has a 50 ns spike filter where
	 *        it is not affected by high frequency on SCL.
	 *   - 1: I2C device does not have a 50 ns spike filter
	 *        but can work with high frequency on SCL.
	 *   - 2: I2C device does not have a 50 ns spike filter
	 *        and cannot work with high frequency on SCL.
	 * - LVR[4]: I2C mode indicator:
	 *   - 0: FM+ mode
	 *   - 1: FM mode
	 * - LVR[3:0]: Reserved.
	 */
	const uint8_t lvr;

	/** Private data by the controller to aid in transactions. Do not modify. */
	void *controller_priv;
};

/**
 * @brief Structure initializer for i3c_i2c_device_desc from devicetree
 *
 * This helper macro expands to a static initializer for a <tt>struct
 * i3c_i2c_device_desc</tt> by reading the relevant bus and device data
 * from the devicetree.
 *
 * @param node_id Devicetree node identifier for the I3C device whose
 *                struct i3c_i2c_device_desc to create an initializer for
 */
#define I3C_I2C_DEVICE_DESC_DT(node_id)					\
	{								\
		.bus = DEVICE_DT_GET(DT_BUS(node_id)),			\
		.addr = DT_PROP_BY_IDX(node_id, reg, 0),		\
		.lvr = DT_PROP_BY_IDX(node_id, reg, 2),			\
	}

/**
 * @brief Structure initializer for i3c_device_desc from devicetree instance
 *
 * This is equivalent to
 * <tt>I3C_I2C_DEVICE_DESC_DT(DT_DRV_INST(inst))</tt>.
 *
 * @param inst Devicetree instance number
 */
#define I3C_I2C_DEVICE_DESC_DT_INST(inst) \
	I3C_I2C_DEVICE_DESC_DT(DT_DRV_INST(inst))

/**
 * @brief Run an init function to perform I3C bus initialization
 *
 * This calls @p _init_fn to perform I3C bus initialization at init
 * level @c POST_KERNEL and priority @c CONFIG_I3C_BUS_INIT_PRIORITY.
 *
 * This needs to be called after the device driver instances of
 * attached I2C and I3C devices have registered themselves with
 * the controller device driver instance. Then, @p _init_fn performs
 * necessary steps to setup the controller hardware and to assign
 * addresses to attached devices (with @c SETAASA, @c SETDASA, and/or
 * @c ENTDAA). Only the devices that have called i3c_device_register()
 * will have dynamic addresses set or assigned. If there are I2C
 * devices on the bus, i3c_i2c_device_register() must be called to
 * tell the controller which addresses are reserved for I2C and
 * will not be assigned as dynamic addresses.
 *
 * @note Only the primary controller of the bus should initialize
 * the bus.
 *
 * @param _init_fn Pointer to the bus init function to run
 * @param _dev Pointer to the I3C controller device driver instance
 */
#define I3C_BUS_INIT(_init_fn, _dev) \
	Z_INIT_ENTRY_DEFINE(Z_SYS_NAME(_init_fn), _init_fn, _dev, \
			    POST_KERNEL, CONFIG_I3C_BUS_INIT_PRIORITY)

/**
 * @brief Run an init function to register devices to I3C controller
 *
 * This calls @p _init_fn to allow devices to register themselves
 * to the I3C controller at init level @c POST_KERNEL and
 * priority @c CONFIG_I3C_DEV_REGISTER_INIT_PRIORITY.
 *
 * This needs to be called after the I3C controller has initialized
 * its internal data structures to store device information. Also,
 * this must be called before bus initialization. The device
 * registration is to tell the controller the connected I2C and I3C
 * devices so the controller can assign dynamic addresses to I3C
 * devices, and reserve the addresses used by I2C (so they are not
 * being assigned to I3C devices).
 *
 * @param _init_fn Pointer to the bus init function to run
 * @param _dev Pointer to the I3C controller device driver instance
 */
#define I3C_DEVICE_REGISTER_INIT(_init_fn, _dev) \
	Z_INIT_ENTRY_DEFINE(Z_SYS_NAME(_init_fn), _init_fn, _dev, \
			    POST_KERNEL, CONFIG_I3C_DEV_REGISTER_INIT_PRIORITY)

/**
 * @brief Structure for describing attached devices for a controller.
 *
 * This contains linked lists describing attached I3C and I2C devices.
 * This also has an array of address slots which describes the status
 * of addresses (e.g. free or in use).
 *
 * This is a helper struct that can be used by controller device
 * driver to aid in device management.
 */
struct i3c_dev_list {
	/**
	 * Address slots:
	 * - Aid in dynamic address assignment.
	 * - Quick way to find out if a target address is
	 *   a I3C or I2C device.
	 */
	struct i3c_addr_slots addr_slots;

	struct {
		/**
		 * Linked list of attached I3C devices.
		 */
		sys_slist_t i3c;

		/**
		 * Linked list of attached I2C devices.
		 */
		sys_slist_t i2c;
	} devices;
};

/**
 * @brief Initialize the device list struct.
 *
 * This initializes the address slots and the linked lists of
 * attached devices.
 *
 * @param dev_list Pointer to the device list struct.
 */
void i3c_dev_list_init(struct i3c_dev_list *dev_list);

/**
 * @brief Add an I3C target device to the devce list.
 *
 * This adds an I3C target device to the device list, using
 * the I3C target descriptor.
 *
 * @param dev_list Pointer to the device list struct.
 * @param target Pointer to the I3C target device descriptor.
 *
 * @retval 0 if successful.
 * @retval -EINVAL if address is already taken (only when
 *                 static address is set).
 */
int i3c_dev_list_target_add(struct i3c_dev_list *dev_list,
			    struct i3c_device_desc *target);

/**
 * @brief Add an I2C device to the devce list.
 *
 * This adds an I2C device to the device list, using
 * the I2C device descriptor.
 *
 * @param dev_list Pointer to the device list struct.
 * @param i2c_dev Pointer to the I2C device descriptor.
 *
 * @retval 0 if successful.
 * @retval -EINVAL if address is already taken.
 */
int i3c_dev_list_i2c_devce_add(struct i3c_dev_list *dev_list,
			       struct i3c_i2c_device_desc *i2c_dev);

/**
 * @brief Find a I3C target device descriptor by Provisioned ID.
 *
 * This finds the I3C target device descriptor in the device list
 * matching the provided Provisioned ID (@p pid).
 *
 * @param dev_list Pointer to the device list struct.
 * @param pid Provisioned ID to be matched.
 *
 * @return Pointer the the I3C target device descriptor, or
 *         NULL if none is found.
 */
struct i3c_device_desc *i3c_dev_list_pid_find(struct i3c_dev_list *dev_list,
					      uint64_t pid);

/**
 * @brief Find a I3C target device descriptor by dynamic address.
 *
 * This finds the I3C target device descriptor in the device list
 * matching the dynamic address (@p addr)
 *
 * @param dev_list Pointer to the device list struct.
 * @param addr Dynamic address to be matched.
 *
 * @return Pointer the the I3C target device descriptor, or
 *         NULL if none is found.
 */
struct i3c_device_desc *i3c_dev_list_i3c_addr_find(struct i3c_dev_list *dev_list,
						   uint8_t addr);

/**
 * @brief Find a I2C target device descriptor by address.
 *
 * This finds the I2C target device descriptor in the device list
 * matching the address (@p addr)
 *
 * @param dev_list Pointer to the device list struct.
 * @param addr Address to be matched.
 *
 * @return Pointer the the I2C target device descriptor, or
 *         NULL if none is found.
 */
struct i3c_i2c_device_desc *i3c_dev_list_i2c_addr_find(struct i3c_dev_list *dev_list,
						       uint16_t addr);

/**
 * @brief Check if the address is free in device list.
 *
 * This checks the address against internal bookkeeping to see
 * if the address has not been taken. Works on both I3C and I2C
 * addresses (except with 10-bit addressing).
 *
 * @param dev_list Pointer to the device list struct.
 * @param addr Device address.
 *
 * @retval true if address is free.
 * @retval false if address is not free.
 */
static inline bool i3c_dev_list_addr_is_free(struct i3c_dev_list *dev_list,
					     uint8_t addr)
{
	return i3c_addr_slots_status_is_free(&dev_list->addr_slots, addr);
}

/**
 * @brief Mark the address as I3C device in device list.
 *
 * @param dev_list Pointer to the device list struct.
 * @param addr Device address.
 */
static inline void i3c_dev_list_addr_mark_i3c(struct i3c_dev_list *dev_list,
					      uint8_t addr)
{
	i3c_addr_slots_status_set(&dev_list->addr_slots, addr,
				  I3C_ADDR_SLOT_STATUS_I3C_DEV);
}

/**
 * @brief Mark the address as I2C device in device list.
 *
 * @param dev_list Pointer to the device list struct.
 * @param addr Device address.
 */
static inline void i3c_dev_list_addr_mark_i2c(struct i3c_dev_list *dev_list,
					      uint8_t addr)
{
	i3c_addr_slots_status_set(&dev_list->addr_slots, addr,
				  I3C_ADDR_SLOT_STATUS_I2C_DEV);
}

/**
 * @brief Mark the address as free (not used) in device list.
 *
 * @param dev_list Pointer to the device list struct.
 * @param addr Device address.
 */
static inline void i3c_dev_list_addr_mark_free(struct i3c_dev_list *dev_list,
					       uint8_t addr)
{
	i3c_addr_slots_status_set(&dev_list->addr_slots, addr,
				  I3C_ADDR_SLOT_STATUS_FREE);
}

/**
 * @brief Mark the address as reserved in device list.
 *
 * @param dev_list Pointer to the device list struct.
 * @param addr Device address.
 */
static inline void i3c_dev_list_addr_mark_rsvd(struct i3c_dev_list *dev_list,
					       uint8_t addr)
{
	i3c_addr_slots_status_set(&dev_list->addr_slots, addr,
				  I3C_ADDR_SLOT_STATUS_RSVD);
}

/**
 * @brief Helper function to find a usable address during ENTDAA.
 *
 * This is a helper function to find a usable address during
 * Dynamic Address Assignment. Given the PID (@p pid), it will
 * search through the device list for the matching device
 * descriptor. If the device descriptor indicates that there is
 * a preferred address (i.e. assigned-address in device tree,
 * @see i3c_device_desc::init_dynamic_addr), this preferred
 * address will be returned if this address is still available.
 * If it is not available, another free address will be returned.
 *
 * If @p must_match is true, the PID (@p pid) must match
 * one of the device in the device list.
 *
 * If @p must_match is false, this will return an arbitrary
 * address. This is useful when not all devices are described in
 * device tree. Or else, the DAA process cannot proceed since
 * there is no address to be assigned.
 *
 * If @p assigned_okay is true, it will return the same address
 * already assigned to the device
 * (@see i3c_device_desc::dynamic_addr). If no address has been
 * assigned, it behaves as if @p assigned_okay is false.
 * This is useful for assigning the same address to the same
 * device (for example, hot-join after device coming back from
 * suspend).
 *
 * If @p assigned_okay is false, the device cannot have an address
 * assigned already (that @see i3c_device_desc::dynamic_addr is not
 * zero). This is mainly used during the initial DAA.
 *
 * @param[in] dev_list Pointer to the device list struct.
 * @param[in] pid Provisioned ID of device to be assigned address.
 * @param[in] must_match True if PID must match devices in
 *			 the device list. False otherwise.
 * @param[in] assigned_okay True if it is okay to return the
 *                          address already assigned to the target
 *                          matching the PID (@p pid).
 * @param[out] target Store the pointer of the device descriptor
 *                    if it matches the incoming PID (@p pid).
 * @param[out] addr Address to be assigned to target device.
 *
 * @retval 0 if successful.
 * @retval -ENODEV if no device matches the PID (@p pid) in
 *                 the device list and @p must_match is true.
 * @retval -EINVAL if the device matching PID (@p pid) already
 *                 has an address assigned or invalid function
 *                 arguments.
 */
int i3c_dev_list_daa_addr_helper(struct i3c_dev_list *dev_list,
				 uint64_t pid, bool must_match,
				 bool assigned_okay,
				 struct i3c_device_desc **target,
				 uint8_t *addr);

/**
 * @brief Configure the I3C hardware.
 *
 * @param dev Pointer to controller device driver instance.
 * @param type Type of configuration parameters being passed
 *             in @p config.
 * @param config Pointer to the configuration parameters.
 *
 * @retval 0 If successful.
 * @retval -EINVAL If invalid configure parameters.
 * @retval -EIO General Input/Output errors.
 * @retval -ENOSYS If not implemented.
 */
static inline int i3c_configure(const struct device *dev,
				enum i3c_config_type type, void *config)
{
	const struct i3c_driver_api *api =
		(const struct i3c_driver_api *)dev->api;

	if (api->configure == NULL) {
		return -ENOSYS;
	}

	return api->configure(dev, type, config);
}

/**
 * @brief Get configuration of the I3C hardware.
 *
 * This provides a way to get the current configuration of the I3C hardware.
 *
 * This can return cached config or probed hardware parameters, but it has to
 * be up to date with current configuration.
 *
 * @param[in] dev Pointer to controller device driver instance.
 * @param[in] type Type of configuration parameters being passed
 *                 in @p config.
 * @param[in,out] config Pointer to the configuration parameters.
 *
 * Note that if @p type is @c I3C_CONFIG_CUSTOM, @p config must contain
 * the ID of the parameter to be retrieved.
 *
 * @retval 0 If successful.
 * @retval -EIO General Input/Output errors.
 * @retval -ENOSYS If not implemented.
 */
static inline int i3c_config_get(const struct device *dev,
				 enum i3c_config_type type, void *config)
{
	const struct i3c_driver_api *api =
		(const struct i3c_driver_api *)dev->api;

	if (api->config_get == NULL) {
		return -ENOSYS;
	}

	return api->config_get(dev, type, config);
}

/**
 * @brief Attempt bus recovery on the I3C bus.
 *
 * This routine asks the controller to attempt bus recovery.
 *
 * @retval 0 If successful.
 * @retval -EBUSY If bus recovery fails.
 * @retval -EIO General input / output error.
 * @retval -ENOSYS Bus recovery is not supported by the controller driver.
 */
static inline int i3c_recover_bus(const struct device *dev)
{
	const struct i3c_driver_api *api =
		(const struct i3c_driver_api *)dev->api;

	if (api->recover_bus == NULL) {
		return -ENOSYS;
	}

	return api->recover_bus(dev);
}

/**
 * @brief Perform Dynamic Address Assignment on the I3C bus.
 *
 * This routine asks the controller to perform dynamic address assignment
 * where the controller belongs. Only the active controller of the bus
 * should do this.
 *
 * @note For controller driver implementation, the controller should perform
 * SETDASA to allow static addresses to be the dynamic addresses before
 * actually doing ENTDAA.
 *
 * @param dev Pointer to the device structure for the controller driver
 *            instance.
 *
 * @retval 0 If successful.
 * @retval -EBUSY Bus is busy.
 * @retval -EIO General input / output error.
 * @retval -ENODEV If a provisioned ID does not match to any target devices
 *                 in the registered device list.
 * @retval -ENOSPC No more free addresses can be assigned to target.
 * @retval -ENOSYS Dynamic address assignment is not supported by
 *                 the controller driver.
 */
static inline int i3c_do_daa(const struct device *dev)
{
	const struct i3c_driver_api *api =
		(const struct i3c_driver_api *)dev->api;

	if (api->do_daa == NULL) {
		return -ENOSYS;
	}

	return api->do_daa(dev);
}

/**
 * @brief Send CCC to the bus.
 *
 * @param dev Pointer to the device structure for the controller driver
 *            instance.
 * @param payload Pointer to the structure describing the CCC payload.
 *
 * @retval 0 If successful.
 * @retval -EBUSY Bus is busy.
 * @retval -EIO General Input / output error.
 * @retval -EINVAL Invalid valid set in the payload structure.
 * @retval -ENOSYS Not implemented.
 */
__syscall int i3c_do_ccc(const struct device *dev,
			 struct i3c_ccc_payload *payload);

static inline int z_impl_i3c_do_ccc(const struct device *dev,
				    struct i3c_ccc_payload *payload)
{
	const struct i3c_driver_api *api =
		(const struct i3c_driver_api *)dev->api;

	if (api->do_ccc == NULL) {
		return -ENOSYS;
	}

	return api->do_ccc(dev, payload);
}

/**
 * @brief Perform data transfer from the controller to a I3C target device.
 *
 * This routine provides a generic interface to perform data transfer
 * to a target device synchronously. Use i3c_read()/i3c_write()
 * for simple read or write.
 *
 * The array of message @a msgs must not be NULL.  The number of
 * message @a num_msgs may be zero, in which case no transfer occurs.
 *
 * @note Not all scatter/gather transactions can be supported by all
 * drivers.  As an example, a gather write (multiple consecutive
 * `i3c_msg` buffers all configured for `I3C_MSG_WRITE`) may be packed
 * into a single transaction by some drivers, but others may emit each
 * fragment as a distinct write transaction, which will not produce
 * the same behavior.  See the documentation of `struct i3c_msg` for
 * limitations on support for multi-message bus transactions.
 *
 * @param target I3C target device descriptor.
 * @param msgs Array of messages to transfer.
 * @param num_msgs Number of messages to transfer.
 *
 * @retval 0 If successful.
 * @retval -EBUSY Bus is busy.
 * @retval -EIO General input / output error.
 */
__syscall int i3c_transfer(struct i3c_device_desc *target,
			   struct i3c_msg *msgs, uint8_t num_msgs);

static inline int z_impl_i3c_transfer(struct i3c_device_desc *target,
				      struct i3c_msg *msgs, uint8_t num_msgs)
{
	const struct i3c_driver_api *api =
		(const struct i3c_driver_api *)target->bus->api;

	return api->i3c_xfers(target->bus, target, msgs, num_msgs);
}

/**
 * @brief Register a target device with the controller driver instance.
 *
 * This makes the controller driver instance aware of a new target device,
 * and is inserted into the controller's connected device list.
 *
 * @note This only tells the controller about this target device.
 * Dynamic address of the device is not assigned yet.
 *
 * @param target I3C target device descriptor.
 *
 * @retval 0 if registration is successful.
 * @retval -EINVAL device static address is reserved or already assigned.
 */
static inline int i3c_device_register(struct i3c_device_desc *target)
{
	const struct i3c_driver_api *api =
		(const struct i3c_driver_api *)target->bus->api;

	if (api->i3c_device_register == NULL) {
		return -ENOSYS;
	}

	return api->i3c_device_register(target->bus, target);
}

/**
 * @brief Raise an In-Band Interrupt (IBI).
 *
 * This raises an In-Band Interrupt (IBI) to the active controller.
 *
 * @param dev Pointer to controller device driver instance.
 * @param request Pointer to the IBI request struct.
 *
 * @retval 0 if operation is successful.
 * @retval -EIO General input / output error.
 */
static inline int i3c_ibi_raise(const struct device *dev,
				struct i3c_ibi *request)
{
	const struct i3c_driver_api *api =
		(const struct i3c_driver_api *)dev->api;

	if (api->ibi_raise == NULL) {
		return -ENOSYS;
	}

	return api->ibi_raise(dev, request);
}

/**
 * @brief Enable IBI of a target device.
 *
 * This enables IBI of a target device where the IBI has already been
 * request.
 *
 * @param target I3C target device descriptor.
 *
 * @retval 0 If successful.
 * @retval -EIO General Input / output error.
 * @retval -ENOMEM If these is no more empty entries in
 *                 the controller's IBI table (if the controller
 *                 uses such table).
 */
static inline int i3c_ibi_enable(struct i3c_device_desc *target)
{
	const struct i3c_driver_api *api =
		(const struct i3c_driver_api *)target->bus->api;

	if (api->ibi_enable == NULL) {
		return -ENOSYS;
	}

	return api->ibi_enable(target->bus, target);
}

/**
 * @brief Disable IBI of a target device.
 *
 * This enables IBI of a target device where the IBI has already been
 * request.
 *
 * @param target I3C target device descriptor.
 *
 * @retval 0 If successful.
 * @retval -EIO General Input / output error.
 * @retval -ENODEV If IBI is not previously enabled for @p target.
 */
static inline int i3c_ibi_disable(struct i3c_device_desc *target)
{
	const struct i3c_driver_api *api =
		(const struct i3c_driver_api *)target->bus->api;

	if (api->ibi_disable == NULL) {
		return -ENOSYS;
	}

	return api->ibi_disable(target->bus, target);
}

/**
 * @brief Check if target's IBI has payload.
 *
 * This reads the BCR from the device descriptor struct to determine
 * whether IBI from device has payload.
 *
 * Note that BCR must have been obtained from device and
 * @see i3c_device_desc::bcr must be set.
 *
 * @return True if IBI has payload, false otherwise.
 */
static inline int i3c_ibi_has_payload(struct i3c_device_desc *target)
{
	return (target->bcr & I3C_BCR_IBI_PAYLOAD_HAS_DATA_BYTE)
		== I3C_BCR_IBI_PAYLOAD_HAS_DATA_BYTE;
}

/**
 * @brief Check if device is IBI capable.
 *
 * This reads the BCR from the device descriptor struct to determine
 * whether device is capable of IBI.
 *
 * Note that BCR must have been obtained from device and
 * @see i3c_device_desc::bcr must be set.
 *
 * @return True if IBI has payload, false otherwise.
 */
static inline int i3c_device_is_ibi_capable(struct i3c_device_desc *target)
{
	return (target->bcr & I3C_BCR_IBI_REQUEST_CAPABLE)
		== I3C_BCR_IBI_REQUEST_CAPABLE;
}

/**
 * @brief Write a set amount of data to an I3C target device.
 *
 * This routine writes a set amount of data synchronously.
 *
 * @param target I3C target device descriptor.
 * @param buf Memory pool from which the data is transferred.
 * @param num_bytes Number of bytes to write.
 *
 * @retval 0 If successful.
 * @retval -EBUSY Bus is busy.
 * @retval -EIO General input / output error.
 */
static inline int i3c_write(struct i3c_device_desc *target,
			    const uint8_t *buf, uint32_t num_bytes)
{
	struct i3c_msg msg;

	msg.buf = (uint8_t *)buf;
	msg.len = num_bytes;
	msg.flags = I3C_MSG_WRITE | I3C_MSG_STOP;

	return i3c_transfer(target, &msg, 1);
}

/**
 * @brief Read a set amount of data from an I3C target device.
 *
 * This routine reads a set amount of data synchronously.
 *
 * @param target I3C target device descriptor.
 * @param buf Memory pool that stores the retrieved data.
 * @param num_bytes Number of bytes to read.
 *
 * @retval 0 If successful.
 * @retval -EBUSY Bus is busy.
 * @retval -EIO General input / output error.
 */
static inline int i3c_read(struct i3c_device_desc *target,
			   uint8_t *buf, uint32_t num_bytes)
{
	struct i3c_msg msg;

	msg.buf = buf;
	msg.len = num_bytes;
	msg.flags = I3C_MSG_READ | I3C_MSG_STOP;

	return i3c_transfer(target, &msg, 1);
}

/**
 * @brief Write then read data from an I3C target device.
 *
 * This supports the common operation "this is what I want", "now give
 * it to me" transaction pair through a combined write-then-read bus
 * transaction.
 *
 * @param target I3C target device descriptor.
 * @param write_buf Pointer to the data to be written
 * @param num_write Number of bytes to write
 * @param read_buf Pointer to storage for read data
 * @param num_read Number of bytes to read
 *
 * @retval 0 if successful
 * @retval -EBUSY Bus is busy.
 * @retval -EIO General input / output error.
 */
static inline int i3c_write_read(struct i3c_device_desc *target,
				 const void *write_buf, size_t num_write,
				 void *read_buf, size_t num_read)
{
	struct i3c_msg msg[2];

	msg[0].buf = (uint8_t *)write_buf;
	msg[0].len = num_write;
	msg[0].flags = I3C_MSG_WRITE;

	msg[1].buf = (uint8_t *)read_buf;
	msg[1].len = num_read;
	msg[1].flags = I3C_MSG_RESTART | I3C_MSG_READ | I3C_MSG_STOP;

	return i3c_transfer(target, msg, 2);
}

/**
 * @brief Read multiple bytes from an internal address of an I3C target device.
 *
 * This routine reads multiple bytes from an internal address of an
 * I3C target device synchronously.
 *
 * Instances of this may be replaced by i3c_write_read().
 *
 * @param target I3C target device descriptor,
 * @param start_addr Internal address from which the data is being read.
 * @param buf Memory pool that stores the retrieved data.
 * @param num_bytes Number of bytes being read.
 *
 * @retval 0 If successful.
 * @retval -EBUSY Bus is busy.
 * @retval -EIO General input / output error.
 */
static inline int i3c_burst_read(struct i3c_device_desc *target,
				 uint8_t start_addr,
				 uint8_t *buf,
				 uint32_t num_bytes)
{
	return i3c_write_read(target,
			      &start_addr, sizeof(start_addr),
			      buf, num_bytes);
}

/**
 * @brief Write multiple bytes to an internal address of an I3C target device.
 *
 * This routine writes multiple bytes to an internal address of an
 * I3C target device synchronously.
 *
 * @warning The combined write synthesized by this API may not be
 * supported on all I3C devices.  Uses of this API may be made more
 * portable by replacing them with calls to i3c_write() passing a
 * buffer containing the combined address and data.
 *
 * @param target I3C target device descriptor.
 * @param start_addr Internal address to which the data is being written.
 * @param buf Memory pool from which the data is transferred.
 * @param num_bytes Number of bytes being written.
 *
 * @retval 0 If successful.
 * @retval -EBUSY Bus is busy.
 * @retval -EIO General input / output error.
 */
static inline int i3c_burst_write(struct i3c_device_desc *target,
				  uint8_t start_addr,
				  const uint8_t *buf,
				  uint32_t num_bytes)
{
	struct i3c_msg msg[2];

	msg[0].buf = &start_addr;
	msg[0].len = 1U;
	msg[0].flags = I3C_MSG_WRITE;

	msg[1].buf = (uint8_t *)buf;
	msg[1].len = num_bytes;
	msg[1].flags = I3C_MSG_WRITE | I3C_MSG_STOP;

	return i3c_transfer(target, msg, 2);
}

/**
 * @brief Read internal register of an I3C target device.
 *
 * This routine reads the value of an 8-bit internal register of an I3C target
 * device synchronously.
 *
 * @param target I3C target device descriptor.
 * @param reg_addr Address of the internal register being read.
 * @param value Memory pool that stores the retrieved register value.
 *
 * @retval 0 If successful.
 * @retval -EBUSY Bus is busy.
 * @retval -EIO General input / output error.
 */
static inline int i3c_reg_read_byte(struct i3c_device_desc *target,
				    uint8_t reg_addr, uint8_t *value)
{
	return i3c_write_read(target,
			      &reg_addr, sizeof(reg_addr),
			      value, sizeof(*value));
}

/**
 * @brief Write internal register of an I3C target device.
 *
 * This routine writes a value to an 8-bit internal register of an I3C target
 * device synchronously.
 *
 * @note This function internally combines the register and value into
 * a single bus transaction.
 *
 * @param target I3C target device descriptor.
 * @param reg_addr Address of the internal register being written.
 * @param value Value to be written to internal register.
 *
 * @retval 0 If successful.
 * @retval -EBUSY Bus is busy.
 * @retval -EIO General input / output error.
 */
static inline int i3c_reg_write_byte(struct i3c_device_desc *target,
				     uint8_t reg_addr, uint8_t value)
{
	uint8_t tx_buf[2] = {reg_addr, value};

	return i3c_write(target, tx_buf, 2);
}

/**
 * @brief Update internal register of an I3C target device.
 *
 * This routine updates the value of a set of bits from an 8-bit internal
 * register of an I3C target device synchronously.
 *
 * @note If the calculated new register value matches the value that
 * was read this function will not generate a write operation.
 *
 * @param target I3C target device descriptor.
 * @param reg_addr Address of the internal register being updated.
 * @param mask Bitmask for updating internal register.
 * @param value Value for updating internal register.
 *
 * @retval 0 If successful.
 * @retval -EBUSY Bus is busy.
 * @retval -EIO General input / output error.
 */
static inline int i3c_reg_update_byte(struct i3c_device_desc *target,
				      uint8_t reg_addr, uint8_t mask,
				      uint8_t value)
{
	uint8_t old_value, new_value;
	int rc;

	rc = i3c_reg_read_byte(target, reg_addr, &old_value);
	if (rc != 0) {
		return rc;
	}

	new_value = (old_value & ~mask) | (value & mask);
	if (new_value == old_value) {
		return 0;
	}

	return i3c_reg_write_byte(target, reg_addr, new_value);
}

/**
 * @brief Perform data transfer from the I3C controller to an I2C device.
 *
 * This routine provides a generic interface to perform data transfer
 * to a target device synchronously. Use i3c_i2c_read()/i3c_i2c_write()
 * for simple read or write.
 *
 * The array of message @a msgs must not be NULL.  The number of
 * message @a num_msgs may be zero, in which case no transfer occurs.
 *
 * @note Not all scatter/gather transactions can be supported by all
 * drivers.  As an example, a gather write (multiple consecutive
 * `i2c_msg` buffers all configured for `I2C_MSG_WRITE`) may be packed
 * into a single transaction by some drivers, but others may emit each
 * fragment as a distinct write transaction, which will not produce
 * the same behavior.  See the documentation of `struct i3c_msg` for
 * limitations on support for multi-message bus transactions.
 *
 * @param i2c_dev I2C device descriptor.
 * @param msgs Array of messages to transfer.
 * @param num_msgs Number of messages to transfer.
 *
 * @retval 0 If successful.
 * @retval -EBUSY Bus is busy.
 * @retval -EIO General input / output error.
 * @retval -ENOSYS If I2C transfers are not supported by the controller.
 */
__syscall int i3c_i2c_transfer(struct i3c_i2c_device_desc *i2c_dev,
			       struct i2c_msg *msgs, uint8_t num_msgs);

static inline int z_impl_i3c_i2c_transfer(struct i3c_i2c_device_desc *i2c_dev,
					  struct i2c_msg *msgs, uint8_t num_msgs)
{
	const struct i3c_driver_api *api =
		(const struct i3c_driver_api *)i2c_dev->bus->api;

	if (api->i2c_xfers == NULL) {
		return -ENOSYS;
	}

	return api->i2c_xfers(i2c_dev->bus, i2c_dev, msgs, num_msgs);
}

/**
 * @brief Register an I2C device with the I3C controller driver instance.
 *
 * This makes the controller driver instance aware of a new I2C device,
 * and is inserted into the controller's connected device list.
 *
 * @param i2c_dev I2C target device descriptor.
 *
 * @retval 0 If successful.
 * @retval -EINVAL Device address is reserved or already assigned.
 * @retval -EIO General input / output error.
 * @retval -ENOSYS If the controller does not allow I2C devices on bus.
 */
static inline int i3c_i2c_device_register(struct i3c_i2c_device_desc *i2c_dev)
{
	const struct i3c_driver_api *api =
		(const struct i3c_driver_api *)i2c_dev->bus->api;

	if (api->i2c_device_register == NULL) {
		return -ENOSYS;
	}

	return api->i2c_device_register(i2c_dev->bus, i2c_dev);
}

/**
 * @brief Write a set amount of data to an I2C device.
 *
 * This routine writes a set amount of data synchronously.
 *
 * @param i2c_dev I2C device descriptor.
 * @param buf Memory pool from which the data is transferred.
 * @param num_bytes Number of bytes to write.
 *
 * @retval 0 If successful.
 * @retval -EBUSY Bus is busy.
 * @retval -EIO General input / output error.
 */
static inline int i3c_i2c_write(struct i3c_i2c_device_desc *i2c_dev,
				const uint8_t *buf, uint32_t num_bytes)
{
	struct i2c_msg msg;

	msg.buf = (uint8_t *)buf;
	msg.len = num_bytes;
	msg.flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	return i3c_i2c_transfer(i2c_dev, &msg, 1);
}

/**
 * @brief Read a set amount of data from an I2C device.
 *
 * This routine reads a set amount of data synchronously.
 *
 * @param i2c_dev I2C device descriptor.
 * @param buf Memory pool that stores the retrieved data.
 * @param num_bytes Number of bytes to read.
 *
 * @retval 0 If successful.
 * @retval -EBUSY Bus is busy.
 * @retval -EIO General input / output error.
 */
static inline int i3c_i2c_read(struct i3c_i2c_device_desc *i2c_dev,
			       uint8_t *buf, uint32_t num_bytes)
{
	struct i2c_msg msg;

	msg.buf = buf;
	msg.len = num_bytes;
	msg.flags = I2C_MSG_READ | I2C_MSG_STOP;

	return i3c_i2c_transfer(i2c_dev, &msg, 1);
}

/**
 * @brief Write then read data from an I2C device.
 *
 * This supports the common operation "this is what I want", "now give
 * it to me" transaction pair through a combined write-then-read bus
 * transaction.
 *
 * @param i2c_dev I2C device descriptor.
 * @param write_buf Pointer to the data to be written
 * @param num_write Number of bytes to write
 * @param read_buf Pointer to storage for read data
 * @param num_read Number of bytes to read
 *
 * @retval 0 if successful
 * @retval -EBUSY Bus is busy.
 * @retval -EIO General input / output error.
 */
static inline int i3c_i2c_write_read(struct i3c_i2c_device_desc *i2c_dev,
				     const void *write_buf, size_t num_write,
				     void *read_buf, size_t num_read)
{
	struct i2c_msg msg[2];

	msg[0].buf = (uint8_t *)write_buf;
	msg[0].len = num_write;
	msg[0].flags = I2C_MSG_WRITE;

	msg[1].buf = (uint8_t *)read_buf;
	msg[1].len = num_read;
	msg[1].flags = I2C_MSG_RESTART | I2C_MSG_READ | I2C_MSG_STOP;

	return i3c_i2c_transfer(i2c_dev, msg, 2);
}

/**
 * @brief Read multiple bytes from an internal address of an I2C device.
 *
 * This routine reads multiple bytes from an internal address of an
 * I2C device synchronously.
 *
 * Instances of this may be replaced by i2c_write_read().
 *
 * @param i2c_dev I2C target device descriptor,
 * @param start_addr Internal address from which the data is being read.
 * @param buf Memory pool that stores the retrieved data.
 * @param num_bytes Number of bytes being read.
 *
 * @retval 0 If successful.
 * @retval -EBUSY Bus is busy.
 * @retval -EIO General input / output error.
 */
static inline int i3c_i2c_burst_read(struct i3c_i2c_device_desc *i2c_dev,
				     uint8_t start_addr,
				     uint8_t *buf,
				     uint32_t num_bytes)
{
	return i3c_i2c_write_read(i2c_dev,
				  &start_addr, sizeof(start_addr),
				  buf, num_bytes);
}

/**
 * @brief Write multiple bytes to an internal address of an I2C device.
 *
 * This routine writes multiple bytes to an internal address of an
 * I2C device synchronously.
 *
 * @warning The combined write synthesized by this API may not be
 * supported on all I2C devices.  Uses of this API may be made more
 * portable by replacing them with calls to i2c_write() passing a
 * buffer containing the combined address and data.
 *
 * @param i2c_dev I2C device descriptor.
 * @param start_addr Internal address to which the data is being written.
 * @param buf Memory pool from which the data is transferred.
 * @param num_bytes Number of bytes being written.
 *
 * @retval 0 If successful.
 * @retval -EBUSY Bus is busy.
 * @retval -EIO General input / output error.
 */
static inline int i3c_i2c_burst_write(struct i3c_i2c_device_desc *i2c_dev,
				      uint8_t start_addr,
				      const uint8_t *buf,
				      uint32_t num_bytes)
{
	struct i2c_msg msg[2];

	msg[0].buf = &start_addr;
	msg[0].len = 1U;
	msg[0].flags = I2C_MSG_WRITE;

	msg[1].buf = (uint8_t *)buf;
	msg[1].len = num_bytes;
	msg[1].flags = I2C_MSG_RESTART | I2C_MSG_WRITE | I2C_MSG_STOP;

	return i3c_i2c_transfer(i2c_dev, msg, 2);
}

/**
 * @brief Read internal register of an I2C device.
 *
 * This routine reads the value of an 8-bit internal register of an I2C
 * device synchronously.
 *
 * @param i2c_dev I2C device descriptor.
 * @param reg_addr Address of the internal register being read.
 * @param value Memory pool that stores the retrieved register value.
 *
 * @retval 0 If successful.
 * @retval -EBUSY Bus is busy.
 * @retval -EIO General input / output error.
 */
static inline int i3c_i2c_reg_read_byte(struct i3c_i2c_device_desc *i2c_dev,
					uint8_t reg_addr, uint8_t *value)
{
	return i3c_i2c_write_read(i2c_dev,
				  &reg_addr, sizeof(reg_addr),
				  value, sizeof(*value));
}

/**
 * @brief Write internal register of an I2C device.
 *
 * This routine writes a value to an 8-bit internal register of an I2C
 * device synchronously.
 *
 * @note This function internally combines the register and value into
 * a single bus transaction.
 *
 * @param i2c_dev I2C device descriptor.
 * @param reg_addr Address of the internal register being written.
 * @param value Value to be written to internal register.
 *
 * @retval 0 If successful.
 * @retval -EBUSY Bus is busy.
 * @retval -EIO General input / output error.
 */
static inline int i3c_i2c_reg_write_byte(struct i3c_i2c_device_desc *i2c_dev,
					 uint8_t reg_addr, uint8_t value)
{
	uint8_t tx_buf[2] = {reg_addr, value};

	return i3c_i2c_write(i2c_dev, tx_buf, 2);
}

/**
 * @brief Update internal register of an I2C device.
 *
 * This routine updates the value of a set of bits from an 8-bit internal
 * register of an I2C device synchronously.
 *
 * @note If the calculated new register value matches the value that
 * was read this function will not generate a write operation.
 *
 * @param i2c_dev I2C device descriptor.
 * @param reg_addr Address of the internal register being updated.
 * @param mask Bitmask for updating internal register.
 * @param value Value for updating internal register.
 *
 * @retval 0 If successful.
 * @retval -EBUSY Bus is busy.
 * @retval -EIO General input / output error.
 */
static inline int i3c_i2c_reg_update_byte(struct i3c_i2c_device_desc *i2c_dev,
					  uint8_t reg_addr, uint8_t mask,
					  uint8_t value)
{
	uint8_t old_value, new_value;
	int rc;

	rc = i3c_i2c_reg_read_byte(i2c_dev, reg_addr, &old_value);
	if (rc != 0) {
		return rc;
	}

	new_value = (old_value & ~mask) | (value & mask);
	if (new_value == old_value) {
		return 0;
	}

	return i3c_i2c_reg_write_byte(i2c_dev, reg_addr, new_value);
}


/**
 * @brief Dump out an I3C message
 *
 * Dumps out a list of I3C messages. For any that are writes (W), the data is
 * displayed in hex.
 *
 * It looks something like this (with name "testing"):
 *
 * D: I3C msg: testing, addr=56
 * D:    W len=01:
 * D: contents:
 * D: 06                      |.
 * D:    W len=0e:
 * D: contents:
 * D: 00 01 02 03 04 05 06 07 |........
 * D: 08 09 0a 0b 0c 0d       |......
 *
 * @param name Name of this dump, displayed at the top.
 * @param msgs Array of messages to dump.
 * @param num_msgs Number of messages to dump.
 * @param target I3C target device descriptor.
 */
void i3c_dump_msgs(const char *name, const struct i3c_msg *msgs,
		   uint8_t num_msgs, struct i3c_device_desc *target);

/**
 * @brief Dump out an I2C message
 *
 * Dumps out a list of I2C messages. For any that are writes (W), the data is
 * displayed in hex.
 *
 * It looks something like this (with name "testing"):
 *
 * D: I2C msg: testing, addr=56
 * D:    W len=01:
 * D: contents:
 * D: 06                      |.
 * D:    W len=0e:
 * D: contents:
 * D: 00 01 02 03 04 05 06 07 |........
 * D: 08 09 0a 0b 0c 0d       |......
 *
 * @param name Name of this dump, displayed at the top.
 * @param msgs Array of messages to dump.
 * @param num_msgs Number of messages to dump.
 * @param i2c_dev I2C device descriptor.
 */
static inline void i3c_i2c_dump_msgs(const char *name,
				     const struct i2c_msg *msgs,
				     uint8_t num_msgs,
				     struct i3c_i2c_device_desc *i2c_dev)
{
	i2c_dump_msgs(name, msgs, num_msgs, i2c_dev->addr);
}

/**
 * @brief Generic helper function to perform bus initialization.
 *
 * @param dev Pointer to controller device driver instance.
 * @param i3c_dev_list Pointer to I3C device list.
 *
 * @retval 0 If successful.
 * @retval -EBUSY Bus is busy.
 * @retval -EIO General input / output error.
 * @retval -ENODEV If a provisioned ID does not match to any target devices
 *                 in the registered device list.
 * @retval -ENOSPC No more free addresses can be assigned to target.
 * @retval -ENOSYS Dynamic address assignment is not supported by
 *                 the controller driver.
 */
int i3c_bus_init(const struct device *dev, struct i3c_dev_list *i3c_dev_list);

/**
 * @brief Get basic information from device and update device descriptor.
 *
 * This retrieves some basic information:
 *   * Bus Characteristics Register (GETBCR)
 *   * Device Characteristics Register (GETDCR)
 *   * Max Read Length (GETMRL)
 *   * Max Write Length (GETMWL)
 * from the device and update the corresponding fields of the device
 * descriptor.
 *
 * This only updates the field(s) in device descriptor
 * only if CCC operations succeed.
 *
 * @param[in,out] target I3C target device descriptor.
 *
 * @retval 0 if successful.
 * @retval -EIO General Input/Output error.
 */
int i3c_device_basic_info_get(struct i3c_device_desc *target);

struct i3c_target_callbacks {
	/**
	 * @brief Function called when a write to the device is initiated.
	 *
	 * This function is invoked by the controller when the bus completes
	 * a start condition for a write operation to the address associated
	 * with a particular device.
	 *
	 * A success return shall cause the controller to ACK the next byte
	 * received. An error return shall cause the controller to NACK the
	 * next byte received.
	 *
	 * @param config Configuration structure associated with the
	 *               device to which the operation is addressed.
	 *
	 * @return 0 if the write is accepted, or a negative error code.
	 */
	int (*write_requested_cb)(struct i3c_target_config *config);

	/**
	 * @brief Function called when a write to the device is continued.
	 *
	 * This function is invoked by the controller when it completes
	 * reception of a byte of data in an ongoing write operation to the
	 * device.
	 *
	 * A success return shall cause the controller to ACK the next byte
	 * received. An error return shall cause the controller to NACK the
	 * next byte received.
	 *
	 * @param config Configuration structure associated with the
	 *               device to which the operation is addressed.
	 *
	 * @param val the byte received by the controller.
	 *
	 * @return 0 if more data can be accepted, or a negative error
	 *         code.
	 */
	int (*write_received_cb)(struct i3c_target_config *config,
				 uint8_t val);

	/**
	 * @brief Function called when a read from the device is initiated.
	 *
	 * This function is invoked by the controller when the bus completes a
	 * start condition for a read operation from the address associated
	 * with a particular device.
	 *
	 * The value returned in @p val will be transmitted. A success
	 * return shall cause the controller to react to additional read
	 * operations. An error return shall cause the controller to ignore
	 * bus operations until a new start condition is received.
	 *
	 * @param config Configuration structure associated with the
	 *               device to which the operation is addressed.
	 *
	 * @param val Pointer to storage for the first byte of data to return
	 *            for the read request.
	 *
	 * @return 0 if more data can be requested, or a negative error code.
	 */
	int (*read_requested_cb)(struct i3c_target_config *config,
				 uint8_t *val);

	/**
	 * @brief Function called when a read from the device is continued.
	 *
	 * This function is invoked by the controller when the bus is ready to
	 * provide additional data for a read operation from the address
	 * associated with the device device.
	 *
	 * The value returned in @p val will be transmitted. A success
	 * return shall cause the controller to react to additional read
	 * operations. An error return shall cause the controller to ignore
	 * bus operations until a new start condition is received.
	 *
	 * @param config Configuration structure associated with the
	 *               device to which the operation is addressed.
	 *
	 * @param val Pointer to storage for the next byte of data to return
	 *            for the read request.
	 *
	 * @return 0 if data has been provided, or a negative error code.
	 */
	int (*read_processed_cb)(struct i3c_target_config *config,
				 uint8_t *val);

	/**
	 * @brief Function called when a stop condition is observed after a
	 * start condition addressed to a particular device.
	 *
	 * This function is invoked by the controller when the bus is ready to
	 * provide additional data for a read operation from the address
	 * associated with the device device. After the function returns the
	 * controller shall enter a state where it is ready to react to new
	 * start conditions.
	 *
	 * @param config Configuration structure associated with the
	 *               device to which the operation is addressed.
	 *
	 * @return Ignored.
	 */
	int (*stop_cb)(struct i3c_target_config *config);
};

/**
 * @brief Structure describing a device that supports the I3C target API.
 *
 * Instances of this are passed to the i3c_target_register() and
 * i3c_target_unregister() functions to indicate addition and removal
 * of a target device, respective.
 *
 * Fields other than @c node must be initialized by the module that
 * implements the device behavior prior to passing the object
 * reference to i3c_target_register().
 */
struct i3c_target_config {
	/** Private, do not modify */
	sys_snode_t node;

	/**
	 * Flags for the target device defined by I3C_TARGET_FLAGS_*
	 * constants.
	 */
	uint8_t flags;

	/** Address for this target device */
	uint8_t address;

	/** Callback functions */
	const struct i3c_target_callbacks *callbacks;
};

struct i3c_target_driver_api {
	int (*driver_register)(const struct device *dev);
	int (*driver_unregister)(const struct device *dev);
};

/**
 * @brief Registers the provided config as target device of a controller.
 *
 * Enable I3C target mode for the @p dev I3C bus driver using the provided
 * config struct (@p cfg) containing the functions and parameters to send bus
 * events. The I3C target will be registered at the address provided as
 * @ref i3c_target_config.address struct member. Any I3C bus events related
 * to the target mode will be passed onto I3C target device driver via a set of
 * callback functions provided in the 'callbacks' struct member.
 *
 * Most of the existing hardware allows simultaneous support for master
 * and target mode. This is however not guaranteed.
 *
 * @param dev Pointer to the device structure for an I3C controller
 *            driver configured in target mode.
 * @param cfg Config struct with functions and parameters used by
 *            the I3C target driver to send bus events
 *
 * @retval 0 Is successful
 * @retval -EINVAL If parameters are invalid
 * @retval -EIO General input / output error.
 * @retval -ENOSYS If target mode is not implemented
 */
static inline int i3c_target_register(const struct device *dev,
				      struct i3c_target_config *cfg)
{
	const struct i3c_driver_api *api =
		(const struct i3c_driver_api *)dev->api;

	if (api->target_register == NULL) {
		return -ENOSYS;
	}

	return api->target_register(dev, cfg);
}

/**
 * @brief Unregisters the provided config as target device
 *
 * This routine disables I3C target mode for the @p dev I3C bus driver using
 * the provided config struct (@p cfg) containing the functions and parameters
 * to send bus events.
 *
 * @param dev Pointer to the device structure for an I3C controller
 *            driver configured in target mode.
 * @param cfg Config struct with functions and parameters used by
 *            the I3C target driver to send bus events
 *
 * @retval 0 Is successful
 * @retval -EINVAL If parameters are invalid
 * @retval -ENOSYS If target mode is not implemented
 */
static inline int i3c_target_unregister(const struct device *dev,
					struct i3c_target_config *cfg)
{
	const struct i3c_driver_api *api =
		(const struct i3c_driver_api *)dev->api;

	if (api->target_unregister == NULL) {
		return -ENOSYS;
	}

	return api->target_unregister(dev, cfg);
}

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#include <syscalls/i3c.h>

#endif /* ZEPHYR_INCLUDE_DRIVERS_I3C_H_ */
