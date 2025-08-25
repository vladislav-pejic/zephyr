/*
 * Copyright (c) 2025 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_IIO_IIO_H_
#define ZEPHYR_INCLUDE_IIO_IIO_H_

#include <zephyr/sys/iterable_sections.h>
#include <zephyr/device.h>
#include <errno.h>

struct iio_chan_spec;

enum iio_chan_info_enum {
	IIO_CHAN_INFO_RAW = 0,
	IIO_CHAN_INFO_PROCESSED,
	IIO_CHAN_INFO_SCALE,
	IIO_CHAN_INFO_OFFSET,
	IIO_CHAN_INFO_CALIBSCALE,
	IIO_CHAN_INFO_CALIBBIAS,
	IIO_CHAN_INFO_CALIBPHASE,
	IIO_CHAN_INFO_PEAK,
	IIO_CHAN_INFO_PEAK_SCALE,
	IIO_CHAN_INFO_QUADRATURE_CORRECTION_RAW,
	IIO_CHAN_INFO_AVERAGE_RAW,
	IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY,
	IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY,
	IIO_CHAN_INFO_SAMP_FREQ,
	IIO_CHAN_INFO_FREQUENCY,
	IIO_CHAN_INFO_PHASE,
	IIO_CHAN_INFO_HARDWAREGAIN,
	IIO_CHAN_INFO_HYSTERESIS,
	IIO_CHAN_INFO_HYSTERESIS_RELATIVE,
	IIO_CHAN_INFO_INT_TIME,
	IIO_CHAN_INFO_ENABLE,
	IIO_CHAN_INFO_CALIBHEIGHT,
	IIO_CHAN_INFO_CALIBWEIGHT,
	IIO_CHAN_INFO_DEBOUNCE_COUNT,
	IIO_CHAN_INFO_DEBOUNCE_TIME,
	IIO_CHAN_INFO_CALIBEMISSIVITY,
	IIO_CHAN_INFO_OVERSAMPLING_RATIO,
	IIO_CHAN_INFO_THERMOCOUPLE_TYPE,
	IIO_CHAN_INFO_CALIBAMBIENT,
	IIO_CHAN_INFO_ZEROPOINT,
};

enum iio_chan_type {
	IIO_VOLTAGE = 0,
	IIO_CURRENT,
	IIO_POWER,
	IIO_ACCEL,
	IIO_ANGL_VEL,
	IIO_MAGN,
	IIO_LIGHT,
	IIO_INTENSITY,
	IIO_PROXIMITY,
	IIO_TEMP,
	IIO_INCLI,
	IIO_ROT,
	IIO_ANGL,
	IIO_TIMESTAMP,
	IIO_CAPACITANCE,
	IIO_ALTVOLTAGE,
	IIO_CCT,
	IIO_PRESSURE,
	IIO_HUMIDITYRELATIVE,
	IIO_ACTIVITY,
	IIO_STEPS,
	IIO_ENERGY,
	IIO_DISTANCE,
	IIO_VELOCITY,
	IIO_CONCENTRATION,
	IIO_RESISTANCE,
	IIO_PH,
	IIO_UVINDEX,
	IIO_ELECTRICALCONDUCTIVITY,
	IIO_COUNT,
	IIO_INDEX,
	IIO_GRAVITY,
	IIO_POSITIONRELATIVE,
	IIO_PHASE,
	IIO_MASSCONCENTRATION,
	IIO_DELTA_ANGL,
	IIO_DELTA_VELOCITY,
	IIO_GENERIC_DATA,
	IIO_FLAGS,
};


enum iio_event_type {
	IIO_EV_TYPE_THRESH,
	IIO_EV_TYPE_MAG,
	IIO_EV_TYPE_ROC,
	IIO_EV_TYPE_THRESH_ADAPTIVE,
	IIO_EV_TYPE_MAG_ADAPTIVE,
	IIO_EV_TYPE_CHANGE,
	IIO_EV_TYPE_MAG_REFERENCED,
	IIO_EV_TYPE_GESTURE,
	IIO_EV_TYPE_FAULT,
};

enum iio_event_direction {
	IIO_EV_DIR_EITHER,
	IIO_EV_DIR_RISING,
	IIO_EV_DIR_FALLING,
	IIO_EV_DIR_NONE,
	IIO_EV_DIR_SINGLETAP,
	IIO_EV_DIR_DOUBLETAP,
	IIO_EV_DIR_FAULT_OPENWIRE,
};

enum iio_event_info {
	IIO_EV_INFO_ENABLE,
	IIO_EV_INFO_VALUE,
	IIO_EV_INFO_HYSTERESIS,
	IIO_EV_INFO_PERIOD,
	IIO_EV_INFO_HIGH_PASS_FILTER_3DB,
	IIO_EV_INFO_LOW_PASS_FILTER_3DB,
	IIO_EV_INFO_TIMEOUT,
	IIO_EV_INFO_RESET_TIMEOUT,
	IIO_EV_INFO_TAP2_MIN_DELAY,
	IIO_EV_INFO_RUNNING_PERIOD,
	IIO_EV_INFO_RUNNING_COUNT,
};

enum iio_endian {
	IIO_CPU = 0,
	IIO_BE,
	IIO_LE,
};

struct iio_attr_list {
	struct iio_attr_list *next;
	struct iio_attr_list *prev;
};

/**
 * struct iio_dev - industrial I/O device
 * @modes:		[DRIVER] bitmask listing all the operating modes
 *			supported by the IIO device. This list should be
 *			initialized before registering the IIO device. It can
 *			also be filed up by the IIO core, as a result of
 *			enabling particular features in the driver
 *			(see iio_triggered_event_setup()).
 * @dev:		[DRIVER] device structure, should be assigned a parent
 *			and owner
 * @buffer:		[DRIVER] any buffer present
 * @scan_bytes:		[INTERN] num bytes captured to be fed to buffer demux
 * @available_scan_masks: [DRIVER] optional array of allowed bitmasks
 * @masklength:		[INTERN] the length of the mask established from
 *			channels
 * @active_scan_mask:	[INTERN] union of all scan masks requested by buffers
 * @scan_timestamp:	[INTERN] set if any buffers have requested timestamp
 * @trig:		[INTERN] current device trigger (buffer modes)
 * @pollfunc:		[DRIVER] function run on trigger being received
 * @pollfunc_event:	[DRIVER] function run on events trigger being received
 * @channels:		[DRIVER] channel specification structure table
 * @num_channels:	[DRIVER] number of channels specified in @channels.
 * @name:		[DRIVER] name of the device.
 * @label:              [DRIVER] unique name to identify which device this is
 * @info:		[DRIVER] callbacks and constant info from driver
 * @setup_ops:		[DRIVER] callbacks to call before and after buffer
 *			enable/disable
 * @priv:		[DRIVER] reference to driver's private information
 *			**MUST** be accessed **ONLY** via iio_priv() helper
 */
struct iio_dev {
	int				modes;
	const struct device 		*dev;

	// struct iio_buffer		*buffer;
	// int				scan_bytes;

	// const unsigned long		*available_scan_masks;
	// unsigned			masklength;
	// const unsigned long		*active_scan_mask;
	// bool				scan_timestamp;
	// struct iio_trigger		*trig;
	// struct iio_poll_func		*pollfunc;
	// struct iio_poll_func		*pollfunc_event;

	struct iio_chan_spec const	*channels;
	int				num_channels;

	const char			*name;
	// const char			*label;
	const struct iio_info		*info;
	// const struct iio_buffer_setup_ops	*setup_ops;
	struct iio_attr_list		channel_attr_list;
	void				*priv;
};

/**
 * struct iio_scan_type - specification for channel data format in buffer
 * @sign:		's' or 'u' to specify signed or unsigned
 * @realbits:		Number of valid bits of data
 * @storagebits:	Realbits + padding
 * @shift:		Shift right by this before masking out realbits.
 * @repeat:		Number of times real/storage bits repeats. When the
 *			repeat element is more than 1, then the type element
 			will show a repeat value. Otherwise, the number
 *			of repetitions is omitted.
 * @endianness:		little or big endian
 */
struct iio_scan_type {
	char	sign;
	uint8_t	realbits;
	uint8_t	storagebits;
	uint8_t	shift;
	uint8_t	repeat;
	enum iio_endian endianness;
};

/**
 * struct iio_event_spec - specification for a channel event
 * @type:		    Type of the event
 * @dir:		    Direction of the event
 * @mask_separate:	    Bit mask of enum iio_event_info values. Attributes
 *			    set in this mask will be registered per channel.
 * @mask_shared_by_type:    Bit mask of enum iio_event_info values. Attributes
 *			    set in this mask will be shared by channel type.
 * @mask_shared_by_dir:	    Bit mask of enum iio_event_info values. Attributes
 *			    set in this mask will be shared by channel type and
 *			    direction.
 * @mask_shared_by_all:	    Bit mask of enum iio_event_info values. Attributes
 *			    set in this mask will be shared by all channels.
 */
struct iio_event_spec {
	enum iio_event_type type;
	enum iio_event_direction dir;
	unsigned long mask_separate;
	unsigned long mask_shared_by_type;
	unsigned long mask_shared_by_dir;
	unsigned long mask_shared_by_all;
};

/**
 * struct iio_chan_spec - specification of a single channel
 * @type:		What type of measurement is the channel making.
 * @channel:		What number do we wish to assign the channel.
 * @channel2:		If there is a second number for a differential
 *			channel then this is it. If modified is set then the
 *			value here specifies the modifier.
 * @address:		Driver specific identifier.
 * @scan_index:		Monotonic index to give ordering in scans when read
 *			from a buffer.
 * @scan_type:		struct describing the scan type - mutually exclusive
 *			with ext_scan_type.
 * @ext_scan_type:	Used in rare cases where there is more than one scan
 *			format for a channel. When this is used, the flag
 *			has_ext_scan_type must be set and the driver must
 *			implement get_current_scan_type in struct iio_info.
 * @num_ext_scan_type:	Number of elements in ext_scan_type.
 * @info_mask_separate: What information is to be exported that is specific to
 *			this channel.
 * @info_mask_separate_available: What availability information is to be
 *			exported that is specific to this channel.
 * @info_mask_shared_by_type: What information is to be exported that is shared
 *			by all channels of the same type.
 * @info_mask_shared_by_type_available: What availability information is to be
 *			exported that is shared by all channels of the same
 *			type.
 * @info_mask_shared_by_dir: What information is to be exported that is shared
 *			by all channels of the same direction.
 * @info_mask_shared_by_dir_available: What availability information is to be
 *			exported that is shared by all channels of the same
 *			direction.
 * @info_mask_shared_by_all: What information is to be exported that is shared
 *			by all channels.
 * @info_mask_shared_by_all_available: What availability information is to be
 *			exported that is shared by all channels.
 * @event_spec:		Array of events which should be registered for this
 *			channel.
 * @num_event_specs:	Size of the event_spec array.
 * @ext_info:		Array of extended info attributes for this channel.
 *			The array is NULL terminated, the last element should
 *			have its name field set to NULL.
 * @extend_name:	Allows labeling of channel attributes with an
 *			informative name. Note this has no effect codes etc,
 *			unlike modifiers.
 *			This field is deprecated in favour of providing
 *			iio_info->read_label() to override the label, which
 *			unlike @extend_name does not affect sysfs filenames.
 * @datasheet_name:	A name used in in-kernel mapping of channels. It should
 *			correspond to the first name that the channel is referred
 *			to by in the datasheet (e.g. IND), or the nearest
 *			possible compound name (e.g. IND-INC).
 * @modified:		Does a modifier apply to this channel. What these are
 *			depends on the channel type.  Modifier is set in
 *			channel2. Examples are IIO_MOD_X for axial sensors about
 *			the 'x' axis.
 * @indexed:		Specify the channel has a numerical index. If not,
 *			the channel index number will be suppressed for sysfs
 *			attributes but not for event codes.
 * @output:		Channel is output.
 * @differential:	Channel is differential.
 * @has_ext_scan_type:	True if ext_scan_type is used instead of scan_type.
 */
struct iio_chan_spec {
	enum iio_chan_type	type;
	int			channel;
	int			channel2;
	// unsigned long		address;
	int			scan_index;
	union {
		struct iio_scan_type scan_type;
		struct {
			const struct iio_scan_type *ext_scan_type;
			unsigned int num_ext_scan_type;
		};
	};
	long			info_mask_separate;
	long			info_mask_separate_available;
	long			info_mask_shared_by_type;
	long			info_mask_shared_by_type_available;
	long			info_mask_shared_by_dir;
	long			info_mask_shared_by_dir_available;
	long			info_mask_shared_by_all;
	long			info_mask_shared_by_all_available;
	const struct iio_event_spec *event_spec;
	unsigned int		num_event_specs;
	const struct iio_chan_spec_ext_info *ext_info;
	const char		*extend_name;
	const char		*datasheet_name;
	unsigned		modified:1;
	unsigned		indexed:1;
	unsigned		output:1;
	unsigned		differential:1;
	unsigned		has_ext_scan_type:1;
};

/**
 * struct iio_dev_attr - iio specific device attribute
 * @name: Name of the attribute.
 * @l:		list head for maintaining list of dynamically created attrs
 * @c:		specification for the underlying channel
 * @buffer:	the IIO buffer to which this attribute belongs to (if any)
 */
struct iio_dev_attr {
	const char		*name;
	struct iio_attr_list l;
	struct iio_chan_spec const *c;
	// struct iio_buffer *buffer;
};

/**
 * struct iio_info - constant information about device
 * @event_attrs:	event control attributes
 * @attrs:		general purpose device attributes
 * @read_raw:		function to request a value from the device.
 *			mask specifies which value. Note 0 means a reading of
 *			the channel in question.  Return value will specify the
 *			type of value returned by the device. val and val2 will
 *			contain the elements making up the returned value.
 * @read_raw_multi:	function to return values from the device.
 *			mask specifies which value. Note 0 means a reading of
 *			the channel in question.  Return value will specify the
 *			type of value returned by the device. vals pointer
 *			contain the elements making up the returned value.
 *			max_len specifies maximum number of elements
 *			vals pointer can contain. val_len is used to return
 *			length of valid elements in vals.
 * @read_avail:		function to return the available values from the device.
 *			mask specifies which value. Note 0 means the available
 *			values for the channel in question.  Return value
 *			specifies if a IIO_AVAIL_LIST or a IIO_AVAIL_RANGE is
 *			returned in vals. The type of the vals are returned in
 *			type and the number of vals is returned in length. For
 *			ranges, there are always three vals returned; min, step
 *			and max. For lists, all possible values are enumerated.
 * @write_raw:		function to write a value to the device.
 *			Parameters are the same as for read_raw.
 * @read_label:		function to request label name for a specified label,
 *			for better channel identification.
 * @write_raw_get_fmt:	callback function to query the expected
 *			format/precision. If not set by the driver, write_raw
 *			returns IIO_VAL_INT_PLUS_MICRO.
 * @read_event_config:	find out if the event is enabled.
 * @write_event_config:	set if the event is enabled.
 * @read_event_value:	read a configuration value associated with the event.
 * @write_event_value:	write a configuration value for the event.
 * @read_event_label:	function to request label name for a specified label,
 *			for better event identification.
 * @validate_trigger:	function to validate the trigger when the
 *			current trigger gets changed.
 * @get_current_scan_type: must be implemented by drivers that use ext_scan_type
 *			in the channel spec to return the index of the currently
 *			active ext_scan type for a channel.
 * @update_scan_mode:	function to configure device and scan buffer when
 *			channels have changed
 * @debugfs_reg_access:	function to read or write register value of device
 * @of_xlate:		function pointer to obtain channel specifier index.
 *			When #iio-cells is greater than '0', the driver could
 *			provide a custom of_xlate function that reads the
 *			*args* and returns the appropriate index in registered
 *			IIO channels array.
 * @fwnode_xlate:	fwnode based function pointer to obtain channel specifier index.
 *			Functionally the same as @of_xlate.
 * @hwfifo_set_watermark: function pointer to set the current hardware
 *			fifo watermark level; see hwfifo_* entries in
 *			Documentation/ABI/testing/sysfs-bus-iio for details on
 *			how the hardware fifo operates
 * @hwfifo_flush_to_buffer: function pointer to flush the samples stored
 *			in the hardware fifo to the device buffer. The driver
 *			should not flush more than count samples. The function
 *			must return the number of samples flushed, 0 if no
 *			samples were flushed or a negative integer if no samples
 *			were flushed and there was an error.
 **/
struct iio_info {
	// const struct attribute_group	*event_attrs;
	// const struct attribute_group	*attrs;

	int (*read_raw)(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int *val,
			int *val2,
			long mask);

	int (*read_raw_multi)(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int max_len,
			int *vals,
			int *val_len,
			long mask);

	int (*read_avail)(struct iio_dev *indio_dev,
			  struct iio_chan_spec const *chan,
			  const int **vals,
			  int *type,
			  int *length,
			  long mask);

	int (*write_raw)(struct iio_dev *indio_dev,
			 struct iio_chan_spec const *chan,
			 int val,
			 int val2,
			 long mask);

	int (*read_label)(struct iio_dev *indio_dev,
			 struct iio_chan_spec const *chan,
			 char *label);

	int (*write_raw_get_fmt)(struct iio_dev *indio_dev,
			 struct iio_chan_spec const *chan,
			 long mask);

	int (*read_event_config)(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan,
				 enum iio_event_type type,
				 enum iio_event_direction dir);

	int (*write_event_config)(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan,
				  enum iio_event_type type,
				  enum iio_event_direction dir,
				  int state);

	int (*read_event_value)(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan,
				enum iio_event_type type,
				enum iio_event_direction dir,
				enum iio_event_info info, int *val, int *val2);

	int (*write_event_value)(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan,
				 enum iio_event_type type,
				 enum iio_event_direction dir,
				 enum iio_event_info info, int val, int val2);

	int (*read_event_label)(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				enum iio_event_type type,
				enum iio_event_direction dir,
				char *label);

	// int (*validate_trigger)(struct iio_dev *indio_dev,
	// 			struct iio_trigger *trig);
	int (*get_current_scan_type)(const struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan);
	int (*update_scan_mode)(struct iio_dev *indio_dev,
				const unsigned long *scan_mask);
	int (*debugfs_reg_access)(struct iio_dev *indio_dev,
				  unsigned reg, unsigned writeval,
				  unsigned *readval);
	// int (*fwnode_xlate)(struct iio_dev *indio_dev,
	// 		    const struct fwnode_reference_args *iiospec);
	int (*hwfifo_set_watermark)(struct iio_dev *indio_dev, unsigned val);
	int (*hwfifo_flush_to_buffer)(struct iio_dev *indio_dev,
				      unsigned count);
};

#define IIO_DEVICE_DT_NAME(node_id)						\
	_CONCAT(__iio_dev, DEVICE_DT_NAME_GET(node_id))

#define IIO_CHANNEL_INIT(child)							\
{										\
	.type = DT_PROP(child, chan_type),					\
	.channel = DT_PROP(child, channel),					\
	.scan_index = DT_PROP(child, scan_index),				\
	.indexed = DT_PROP_OR(child, indexed, 0),				\
	.differential = DT_PROP_OR(child, differential, 0),			\
	.scan_type = {								\
		.sign = DT_PROP(child, scan_type_sign)[0],			\
		.realbits = DT_PROP(child, scan_type_realbits),			\
		.endianness = DT_PROP(child, scan_type_endianness),		\
	},									\
}

/**
 * @brief Statically define and initialize an IIO device
 *
 * @param name Name of the IIO device
 * @param dev_ptr Pointer to the parent device structure
 */
#define IIO_DEVICE_DEFINE(node_id)								\
	static const struct iio_chan_spec							\
	iio_channels_##node_id[] = {								\
		DT_FOREACH_CHILD_SEP(DT_CHILD(node_id, iio_channels), IIO_CHANNEL_INIT, (,))	\
	};											\
	STRUCT_SECTION_ITERABLE(iio_dev, IIO_DEVICE_DT_NAME(node_id)) = {			\
		.dev = DEVICE_DT_GET(node_id),							\
		.channels = iio_channels_##node_id,						\
		.num_channels = ARRAY_SIZE(iio_channels_##node_id),				\
	};											\
	const struct iio_dev *iio_dev_ptr = &IIO_DEVICE_DT_NAME(node_id);

static inline void __iio_list_add(struct iio_attr_list *new,
			      struct iio_attr_list *prev,
			      struct iio_attr_list *next)
{
	next->prev = new;
	new->next = next;
	new->prev = prev;
	prev->next = new;
}

static inline void iio_list_add(struct iio_attr_list *new, struct iio_attr_list *head)
{
	__iio_list_add(new, head, head->next);
}

static inline void
iio_list_init(struct iio_attr_list *list)
{
	list->next = list;
	list->prev = list;
}

static inline int
iio_list_empty(struct iio_attr_list *list)
{
	return list->next == list;
}

static inline void
iio_list_insert(struct iio_attr_list *link, struct iio_attr_list *new_link)
{
	new_link->prev		= link->prev;
	new_link->next		= link;
	new_link->prev->next	= new_link;
	new_link->next->prev	= new_link;
}

static inline void
iio_list_append(struct iio_attr_list *list, struct iio_attr_list *new_link)
{
	iio_list_insert((struct iio_attr_list *)list, new_link);
}

static inline void
iio_list_prepend(struct iio_attr_list *list, struct iio_attr_list *new_link)
{
	iio_list_insert(list->next, new_link);
}

static inline void
iio_list_remove(struct iio_attr_list *link)
{
	link->prev->next = link->next;
	link->next->prev = link->prev;
}

#define iio_list_entry(link, type, member) \
	((type *)((char *)(link)-(unsigned long)(&((type *)0)->member)))

#define iio_list_head(list, type, member)		\
	iio_list_entry((list)->next, type, member)

#define iio_list_tail(list, type, member)		\
	iio_list_entry((list)->prev, type, member)

#define iio_list_next(elm, type, member)					\
	iio_list_entry((elm)->member.next, type, member)

#define iio_list_for_each_entry(pos, list, type, member)	\
	for (pos = iio_list_head(list, type, member);		\
	     &pos->member != (list);				\
	     pos = iio_list_next(pos, member))

int iio_device_register(struct iio_dev *indio_dev);

#endif /* ZEPHYR_INCLUDE_IIO_IIO_H_ */