/*
 * Copyright (c) 2025 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/iio/iio.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>
#include <stdio.h>

LOG_MODULE_REGISTER(IIO_HALL, CONFIG_IIO_LOG_LEVEL);

/* relies on pairs of these shared then separate */
static const char * const iio_chan_info_postfix[] = {
	[IIO_CHAN_INFO_RAW] = "raw",
	[IIO_CHAN_INFO_PROCESSED] = "input",
	[IIO_CHAN_INFO_SCALE] = "scale",
	[IIO_CHAN_INFO_OFFSET] = "offset",
	[IIO_CHAN_INFO_CALIBSCALE] = "calibscale",
	[IIO_CHAN_INFO_CALIBBIAS] = "calibbias",
	[IIO_CHAN_INFO_CALIBPHASE] = "calibphase",
	[IIO_CHAN_INFO_PEAK] = "peak_raw",
	[IIO_CHAN_INFO_PEAK_SCALE] = "peak_scale",
	[IIO_CHAN_INFO_QUADRATURE_CORRECTION_RAW] = "quadrature_correction_raw",
	[IIO_CHAN_INFO_AVERAGE_RAW] = "mean_raw",
	[IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY]
	= "filter_low_pass_3db_frequency",
	[IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY]
	= "filter_high_pass_3db_frequency",
	[IIO_CHAN_INFO_SAMP_FREQ] = "sampling_frequency",
	[IIO_CHAN_INFO_FREQUENCY] = "frequency",
	[IIO_CHAN_INFO_PHASE] = "phase",
	[IIO_CHAN_INFO_HARDWAREGAIN] = "hardwaregain",
	[IIO_CHAN_INFO_HYSTERESIS] = "hysteresis",
	[IIO_CHAN_INFO_HYSTERESIS_RELATIVE] = "hysteresis_relative",
	[IIO_CHAN_INFO_INT_TIME] = "integration_time",
	[IIO_CHAN_INFO_ENABLE] = "en",
	[IIO_CHAN_INFO_CALIBHEIGHT] = "calibheight",
	[IIO_CHAN_INFO_CALIBWEIGHT] = "calibweight",
	[IIO_CHAN_INFO_DEBOUNCE_COUNT] = "debounce_count",
	[IIO_CHAN_INFO_DEBOUNCE_TIME] = "debounce_time",
	[IIO_CHAN_INFO_CALIBEMISSIVITY] = "calibemissivity",
	[IIO_CHAN_INFO_OVERSAMPLING_RATIO] = "oversampling_ratio",
	[IIO_CHAN_INFO_THERMOCOUPLE_TYPE] = "thermocouple_type",
	[IIO_CHAN_INFO_CALIBAMBIENT] = "calibambient",
	[IIO_CHAN_INFO_ZEROPOINT] = "zeropoint",
};

static const struct iio_info iio_hal_info = {
	// .read_raw = iio_hal_read_raw,
	// .write_raw = iio_hal_write_raw,
	// .write_raw_get_fmt = iio_hal_write_raw_get_fmt,
	// .read_avail = iio_hal_read_avail,
	// .read_event_config = iio_hal_read_event_config,
	// .write_event_config = iio_hal_write_event_config,
	// .read_event_value = iio_hal_read_event_value,
	// .write_event_value = iio_hal_write_event_value,
	// .debugfs_reg_access = iio_hal_reg_access,
	// .hwfifo_set_watermark = iio_hal_set_watermark,
	// .update_scan_mode = iio_hal_update_scan_mode,
};

static int iio_check_unique_scan_index(struct iio_dev *indio_dev)
{
	int i, j;
	const struct iio_chan_spec *channels = indio_dev->channels;

	// if (!(indio_dev->modes & INDIO_ALL_BUFFER_MODES))
	// 	return 0;

	for (i = 0; i < indio_dev->num_channels - 1; i++) {
		if (channels[i].scan_index < 0) {
			continue;
		}
		for (j = i + 1; j < indio_dev->num_channels; j++) {
			if (channels[i].scan_index != channels[j].scan_index) {
				continue;
			}
			// if (iio_chan_same_size(&channels[i], &channels[j]))
			// 	continue;
			LOG_ERR("Duplicate scan index %d\n",
				channels[i].scan_index);
			return -EINVAL;
		}
	}

	return 0;
}

static
int __iio_device_attr_init(struct iio_dev_attr *dev_attr,
			   const char *postfix,
			   struct iio_chan_spec const *chan)//,
			//    enum iio_shared_by shared_by)
{
	int ret = 0;
	// char *name = NULL;
	// char *full_postfix;

	// sysfs_attr_init(&dev_attr->attr);

	/* Build up postfix of <extend_name>_<modifier>_postfix */
	// if (chan->modified && (shared_by == IIO_SEPARATE)) {
	// 	if (chan->extend_name)
	// 		full_postfix = kasprintf(GFP_KERNEL, "%s_%s_%s",
	// 					 iio_modifier_names[chan->channel2],
	// 					 chan->extend_name,
	// 					 postfix);
	// 	else
	// 		full_postfix = kasprintf(GFP_KERNEL, "%s_%s",
	// 					 iio_modifier_names[chan->channel2],
	// 					 postfix);
	// } else {
	// 	if (chan->extend_name == NULL || shared_by != IIO_SEPARATE)
	// 		full_postfix = kstrdup(postfix, GFP_KERNEL);
	// 	else
	// 		full_postfix = kasprintf(GFP_KERNEL,
	// 					 "%s_%s",
	// 					 chan->extend_name,
	// 					 postfix);
	// }
	// if (full_postfix == NULL)
	// 	return -ENOMEM;

	// if (chan->differential) { /* Differential can not have modifier */
	// 	switch (shared_by) {
	// 	case IIO_SHARED_BY_ALL:
	// 		name = kasprintf(GFP_KERNEL, "%s", full_postfix);
	// 		break;
	// 	case IIO_SHARED_BY_DIR:
	// 		name = kasprintf(GFP_KERNEL, "%s_%s",
	// 					iio_direction[chan->output],
	// 					full_postfix);
	// 		break;
	// 	case IIO_SHARED_BY_TYPE:
	// 		name = kasprintf(GFP_KERNEL, "%s_%s-%s_%s",
	// 				    iio_direction[chan->output],
	// 				    iio_chan_type_name_spec[chan->type],
	// 				    iio_chan_type_name_spec[chan->type],
	// 				    full_postfix);
	// 		break;
	// 	case IIO_SEPARATE:
	// 		if (!chan->indexed) {
	// 			WARN(1, "Differential channels must be indexed\n");
	// 			ret = -EINVAL;
	// 			goto error_free_full_postfix;
	// 		}
	// 		name = kasprintf(GFP_KERNEL,
	// 				    "%s_%s%d-%s%d_%s",
	// 				    iio_direction[chan->output],
	// 				    iio_chan_type_name_spec[chan->type],
	// 				    chan->channel,
	// 				    iio_chan_type_name_spec[chan->type],
	// 				    chan->channel2,
	// 				    full_postfix);
	// 		break;
	// 	}
	// } else { /* Single ended */
	// 	switch (shared_by) {
	// 	case IIO_SHARED_BY_ALL:
	// 		name = kasprintf(GFP_KERNEL, "%s", full_postfix);
	// 		break;
	// 	case IIO_SHARED_BY_DIR:
	// 		name = kasprintf(GFP_KERNEL, "%s_%s",
	// 					iio_direction[chan->output],
	// 					full_postfix);
	// 		break;
	// 	case IIO_SHARED_BY_TYPE:
	// 		name = kasprintf(GFP_KERNEL, "%s_%s_%s",
	// 				    iio_direction[chan->output],
	// 				    iio_chan_type_name_spec[chan->type],
	// 				    full_postfix);
	// 		break;

	// 	case IIO_SEPARATE:
	// 		if (chan->indexed)
	// 			name = kasprintf(GFP_KERNEL, "%s_%s%d_%s",
	// 					    iio_direction[chan->output],
	// 					    iio_chan_type_name_spec[chan->type],
	// 					    chan->channel,
	// 					    full_postfix);
	// 		else
	// 			name = kasprintf(GFP_KERNEL, "%s_%s_%s",
	// 					    iio_direction[chan->output],
	// 					    iio_chan_type_name_spec[chan->type],
	// 					    full_postfix);
	// 		break;
	// 	}
	// }
	// if (name == NULL) {
	// 	ret = -ENOMEM;
	// 	goto error_free_full_postfix;
	// }
	// dev_attr->attr.name = name;

// error_free_full_postfix:
// 	kfree(full_postfix);

	return ret;
}

int __iio_add_chan_dev_attr(const char *postfix,
			   struct iio_chan_spec const *chan,
			//    ssize_t (*readfunc)(struct device *dev,
			// 		       struct iio_dev_attr *attr,
			// 		       char *buf),
			//    ssize_t (*writefunc)(struct device *dev,
			// 			struct iio_dev_attr *attr,
			// 			const char *buf, size_t count),
			   uint64_t mask,
			//    enum iio_shared_by shared_by,
			//    struct device *dev,
			//    struct iio_buffer *buffer,
			   struct iio_attr_list *attr_list)
{
	int ret;
	struct iio_dev_attr *iio_attr;//, *t;

	iio_attr = malloc(sizeof(*iio_attr));
	if (iio_attr == NULL)
	{
		return -ENOMEM;
	}

	ret = __iio_device_attr_init(iio_attr,
				     postfix, chan);//,
				    // readfunc, writefunc);//, shared_by);
	if (ret){
		goto error_iio_dev_attr_free;
	}
	iio_attr->c = chan;
	// iio_attr->address = mask;
	// iio_attr->buffer = buffer;
	// iio_list_for_each_entry(t, attr_list, struct iio_dev_attr, l)
	// 	if (strcmp(t->dev_attr.name, iio_attr->dev_attr.name) == 0) {
	// 		if (shared_by == IIO_SEPARATE)
	// 			dev_err(dev, "tried to double register : %s\n",
	// 				t->dev_attr.attr.name);
	// 		ret = -EBUSY;
	// 		goto error_device_attr_deinit;
	// 	}
	iio_list_add(&iio_attr->l, attr_list);

	return 0;

// error_device_attr_deinit:
// 	__iio_device_attr_deinit(&iio_attr->dev_attr);
error_iio_dev_attr_free:
	free(iio_attr);
	return ret;
}

static int iio_device_add_info_mask_type(struct iio_dev *indio_dev,
					 struct iio_chan_spec const *chan,
					//  enum iio_shared_by shared_by,
					 const long *infomask)
{
	// struct iio_dev_opaque *iio_dev_opaque = to_iio_dev_opaque(indio_dev);
	int ret, attrcount = 0;

	// for_each_set_bit(i, infomask, sizeof(*infomask)*8) {
	for(int i = 0; i < sizeof(*infomask) * 8; i++) {
		if (*infomask & (1L << i)) {
			if (i >= ARRAY_SIZE(iio_chan_info_postfix)) {
				return -EINVAL;
			}

			ret = __iio_add_chan_dev_attr(iio_chan_info_postfix[i],
						chan,
						// &iio_read_channel_info,
						// &iio_write_channel_info,
						i,
						// shared_by,
						// &indio_dev->dev,
						// NULL,
						&indio_dev->channel_attr_list);
			// if ((ret == -EBUSY) && (shared_by != IIO_SEPARATE))
			// 	continue;
			if (ret < 0)
				return ret;
			attrcount++;
		}
	}

	return attrcount;
}

static int iio_device_add_info_mask_type_avail(struct iio_dev *indio_dev,
					       struct iio_chan_spec const *chan,
					//        enum iio_shared_by shared_by,
					       const long *infomask)
{
	// struct iio_dev_opaque *iio_dev_opaque = to_iio_dev_opaque(indio_dev);
	int ret, attrcount = 0;
	char avail_postfix[64];

	// for_each_set_bit(i, infomask, sizeof(*infomask) * 8) {
	for(int i = 0; i < sizeof(*infomask) * 8; i++) {
		if (*infomask & (1L << i)) {
			if (i >= ARRAY_SIZE(iio_chan_info_postfix)) {
				return -EINVAL;
			}

			snprintf(avail_postfix, sizeof(avail_postfix),
						"%s_available",
						iio_chan_info_postfix[i]);

			ret = __iio_add_chan_dev_attr(avail_postfix,
						chan,
						// &iio_read_channel_info_avail,
						// NULL,
						i,
						// shared_by,
						// &indio_dev->dev,
						// NULL,
						&indio_dev->channel_attr_list);
			// kfree(avail_postfix);
			// if ((ret == -EBUSY) && (shared_by != IIO_SEPARATE))
			// 	continue;
			if (ret < 0)
				return ret;
			attrcount++;
		}
	}

	return attrcount;
}

static int iio_device_add_channel(struct iio_dev *indio_dev,
					struct iio_chan_spec const *chan)
{
	// struct iio_dev_opaque *iio_dev_opaque = to_iio_dev_opaque(indio_dev);
	int ret, attrcount = 0;
	// const struct iio_chan_spec_ext_info *ext_info;

	if (chan->channel < 0) {
		return 0;
	}
	ret = iio_device_add_info_mask_type(indio_dev, chan,
					//     IIO_SEPARATE,
					    &chan->info_mask_separate);
	if (ret < 0)
		return ret;
	attrcount += ret;

	// ret = iio_device_add_info_mask_type_avail(indio_dev, chan,
	// 					  IIO_SEPARATE,
	// 					  &chan->info_mask_separate_available);
	// if (ret < 0)
	// 	return ret;
	// attrcount += ret;

	// ret = iio_device_add_info_mask_type(indio_dev, chan,
	// 				    IIO_SHARED_BY_TYPE,
	// 				    &chan->info_mask_shared_by_type);
	// if (ret < 0)
	// 	return ret;
	// attrcount += ret;

	// ret = iio_device_add_info_mask_type_avail(indio_dev, chan,
	// 					  IIO_SHARED_BY_TYPE,
	// 					  &chan->info_mask_shared_by_type_available);
	// if (ret < 0)
	// 	return ret;
	// attrcount += ret;

	// ret = iio_device_add_info_mask_type(indio_dev, chan,
	// 				    IIO_SHARED_BY_DIR,
	// 				    &chan->info_mask_shared_by_dir);
	// if (ret < 0)
	// 	return ret;
	// attrcount += ret;

	// ret = iio_device_add_info_mask_type_avail(indio_dev, chan,
	// 					  IIO_SHARED_BY_DIR,
	// 					  &chan->info_mask_shared_by_dir_available);
	// if (ret < 0)
	// 	return ret;
	// attrcount += ret;

	// ret = iio_device_add_info_mask_type(indio_dev, chan,
	// 				    IIO_SHARED_BY_ALL,
	// 				    &chan->info_mask_shared_by_all);
	// if (ret < 0)
	// 	return ret;
	// attrcount += ret;

	// ret = iio_device_add_info_mask_type_avail(indio_dev, chan,
	// 					  IIO_SHARED_BY_ALL,
	// 					  &chan->info_mask_shared_by_all_available);
	// if (ret < 0)
	// 	return ret;
	// attrcount += ret;

	// ret = iio_device_add_channel_label(indio_dev, chan);
	// if (ret < 0)
	// 	return ret;
	// attrcount += ret;

	// if (chan->ext_info) {
	// 	unsigned int i = 0;

	// 	for (ext_info = chan->ext_info; ext_info->name; ext_info++) {
	// 		ret = __iio_add_chan_devattr(ext_info->name,
	// 				chan,
	// 				ext_info->read ?
	// 				    &iio_read_channel_ext_info : NULL,
	// 				ext_info->write ?
	// 				    &iio_write_channel_ext_info : NULL,
	// 				i,
	// 				ext_info->shared,
	// 				&indio_dev->dev,
	// 				NULL,
	// 				&iio_dev_opaque->channel_attr_list);
	// 		i++;
	// 		if (ret == -EBUSY && ext_info->shared)
	// 			continue;

	// 		if (ret)
	// 			return ret;

	// 		attrcount++;
	// 	}
	// }

	return attrcount;
}

static int __iio_device_register(struct iio_dev *indio_dev)
{
	int i, ret = 0, attrcount, attrn, attrcount_orig = 0;
	struct iio_dev_attr *p;
	// struct attribute **attr, *clk = NULL;

	/* First count elements in any existing group */
	// if (indio_dev->info->attrs) {
	// 	attr = indio_dev->info->attrs->attrs;
	// 	while (*attr++ != NULL)
	// 		attrcount_orig++;
	// }
	// attrcount = attrcount_orig;
	/*
	 * New channel registration method - relies on the fact a group does
	 * not need to be initialized if its name is NULL.
	 */
	if (indio_dev->channels)
	{
		for (i = 0; i < indio_dev->num_channels; i++) {
			// const struct iio_chan_spec *chan =
			// 	&indio_dev->channels[i];

			// if (chan->type == IIO_TIMESTAMP)
			// 	clk = &dev_attr_current_timestamp_clock.attr;

// 			ret = iio_device_add_channel_sysfs(indio_dev, chan);
// 			if (ret < 0)
// 				goto error_clear_attrs;
// 			attrcount += ret;
		}
	}

// 	if (iio_dev_opaque->event_interface)
// 		clk = &dev_attr_current_timestamp_clock.attr;

// 	if (indio_dev->name)
// 		attrcount++;
// 	if (indio_dev->label)
// 		attrcount++;
// 	if (clk)
// 		attrcount++;

// 	iio_dev_opaque->chan_attr_group.attrs =
// 		kcalloc(attrcount + 1,
// 			sizeof(iio_dev_opaque->chan_attr_group.attrs[0]),
// 			GFP_KERNEL);
// 	if (iio_dev_opaque->chan_attr_group.attrs == NULL) {
// 		ret = -ENOMEM;
// 		goto error_clear_attrs;
// 	}
// 	/* Copy across original attributes, and point to original binary attributes */
// 	if (indio_dev->info->attrs) {
// 		memcpy(iio_dev_opaque->chan_attr_group.attrs,
// 		       indio_dev->info->attrs->attrs,
// 		       sizeof(iio_dev_opaque->chan_attr_group.attrs[0])
// 		       *attrcount_orig);
// 		iio_dev_opaque->chan_attr_group.is_visible =
// 			indio_dev->info->attrs->is_visible;
// 		iio_dev_opaque->chan_attr_group.bin_attrs =
// 			indio_dev->info->attrs->bin_attrs;
// 	}
// 	attrn = attrcount_orig;
// 	/* Add all elements from the list. */
// 	list_for_each_entry(p, &iio_dev_opaque->channel_attr_list, l)
// 		iio_dev_opaque->chan_attr_group.attrs[attrn++] = &p->dev_attr.attr;
// 	if (indio_dev->name)
// 		iio_dev_opaque->chan_attr_group.attrs[attrn++] = &dev_attr_name.attr;
// 	if (indio_dev->label)
// 		iio_dev_opaque->chan_attr_group.attrs[attrn++] = &dev_attr_label.attr;
// 	if (clk)
// 		iio_dev_opaque->chan_attr_group.attrs[attrn++] = clk;

// 	ret = iio_device_register_sysfs_group(indio_dev,
// 					      &iio_dev_opaque->chan_attr_group);
// 	if (ret)
// 		goto error_clear_attrs;

// 	return 0;

// error_clear_attrs:
// 	iio_free_chan_devattr_list(&iio_dev_opaque->channel_attr_list);

	return ret;
}

int iio_device_register(struct iio_dev *indio_dev)
{
	int ret;

	if (!indio_dev->info)
	{
		indio_dev->info = &iio_hal_info;
	}

	// iio_dev_opaque->driver_module = this_mod;

	// /* If the calling driver did not initialize firmware node, do it here */
	// if (dev_fwnode(&indio_dev->dev))
	// 	fwnode = dev_fwnode(&indio_dev->dev);
	// /* The default dummy IIO device has no parent */
	// else if (indio_dev->dev.parent)
	// 	fwnode = dev_fwnode(indio_dev->dev.parent);
	// device_set_node(&indio_dev->dev, fwnode);

	// fwnode_property_read_string(fwnode, "label", &indio_dev->label);

	ret = iio_check_unique_scan_index(indio_dev);
	if (ret < 0)
		return ret;

	// ret = iio_check_extended_name(indio_dev);
	// if (ret < 0)
	// 	return ret;

	// iio_device_register_debugfs(indio_dev);

	// ret = iio_buffers_alloc_sysfs_and_mask(indio_dev);
	// if (ret) {
	// 	dev_err(indio_dev->dev.parent,
	// 		"Failed to create buffer sysfs interfaces\n");
	// 	goto error_unreg_debugfs;
	// }

	// if (indio_dev->available_scan_masks)
	// 	iio_sanity_check_avail_scan_masks(indio_dev);

	ret = __iio_device_register(indio_dev);
	// if (ret) {
	// 	dev_err(indio_dev->dev.parent,
	// 		"Failed to register sysfs interfaces\n");
	// 	goto error_buffer_free_sysfs;
	// }
	// ret = iio_device_register_eventset(indio_dev);
	// if (ret) {
	// 	dev_err(indio_dev->dev.parent,
	// 		"Failed to register event set\n");
	// 	goto error_free_sysfs;
	// }
	// if (indio_dev->modes & INDIO_ALL_TRIGGERED_MODES)
	// 	iio_device_register_trigger_consumer(indio_dev);

	// if ((indio_dev->modes & INDIO_ALL_BUFFER_MODES) &&
	// 	indio_dev->setup_ops == NULL)
	// 	indio_dev->setup_ops = &noop_ring_setup_ops;

	// if (iio_dev_opaque->attached_buffers_cnt)
	// 	cdev_init(&iio_dev_opaque->chrdev, &iio_buffer_fileops);
	// else if (iio_dev_opaque->event_interface)
	// 	cdev_init(&iio_dev_opaque->chrdev, &iio_event_fileops);

	// if (iio_dev_opaque->attached_buffers_cnt || iio_dev_opaque->event_interface) {
	// 	indio_dev->dev.devt = MKDEV(MAJOR(iio_devt), iio_dev_opaque->id);
	// 	iio_dev_opaque->chrdev.owner = this_mod;
	// }

	// /* assign device groups now; they should be all registered now */
	// indio_dev->dev.groups = iio_dev_opaque->groups;

	// ret = cdev_device_add(&iio_dev_opaque->chrdev, &indio_dev->dev);
	// if (ret < 0)
	// 	goto error_unreg_eventset;

	return 0;

// error_unreg_eventset:
// 	iio_device_unregister_eventset(indio_dev);
// error_free_sysfs:
// 	iio_device_unregister_sysfs(indio_dev);
// error_buffer_free_sysfs:
// 	iio_buffers_free_sysfs_and_mask(indio_dev);
// error_unreg_debugfs:
// 	iio_device_unregister_debugfs(indio_dev);
	// return ret;
}