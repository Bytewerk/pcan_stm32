//****************************************************************************
// Copyright (C) 2003-2011  PEAK System-Technik GmbH
//
// linux@peak-system.com
// www.peak-system.com
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// Maintainer(s): Stephane Grosjean (s.grosjean@peak-system.com)
//
// Major contributions by:
//                Klaus Hitschler (klaus.hitschler@gmx.de)
//                Oliver Hartkopp (oliver.hartkopp@volkswagen.de) socketCAN
//
//****************************************************************************
#include "pcan_usbpro_sizeof_rec.h"
#include "pcan_usbpro_fw.h"

int pcan_usbpro_sizeof_rec(uint8_t data_type)
{
	switch (data_type)
	{
	case DATA_TYPE_USB2CAN_STRUCT_CANMSG_RX_8:
		return sizeof(struct pcan_usbpro_canmsg_rx_t);
	case DATA_TYPE_USB2CAN_STRUCT_CANMSG_RX_4:
		return sizeof(struct pcan_usbpro_canmsg_rx_t) - 4;
	case DATA_TYPE_USB2CAN_STRUCT_CANMSG_RX_0:
	case DATA_TYPE_USB2CAN_STRUCT_CANMSG_RTR_RX:
		return sizeof(struct pcan_usbpro_canmsg_rx_t) - 8;

	case DATA_TYPE_USB2CAN_STRUCT_CANMSG_STATUS_ERROR_RX:
		return sizeof(struct pcan_usbpro_canmsg_status_error_rx_t);

	case DATA_TYPE_USB2CAN_STRUCT_CALIBRATION_TIMESTAMP_RX:
		return sizeof(struct pcan_usbpro_calibration_ts_rx_t);

	case DATA_TYPE_USB2CAN_STRUCT_BUSLAST_RX:
		return sizeof(struct pcan_usbpro_buslast_rx_t);

	case DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_8:
		return sizeof(struct pcan_usbpro_canmsg_tx_t);
	case DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_4:
		return sizeof(struct pcan_usbpro_canmsg_tx_t) - 4;
	case DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_0:
		return sizeof(struct pcan_usbpro_canmsg_tx_t) - 8;

	case DATA_TYPE_USB2CAN_STRUCT_FKT_GETBAUDRATE:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETBAUDRATE:
		return sizeof(struct pcan_usbpro_baudrate_t);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_GETCANBUSACTIVATE:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETCANBUSACTIVATE:
		return sizeof(struct pcan_usbpro_bus_activity_t);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETSILENTMODE:
		return sizeof(struct pcan_usbpro_silent_mode_t);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETDEVICENR:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_GETDEVICENR:
		return sizeof(struct pcan_usbpro_dev_nr_t);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETWARNINGLIMIT:
		return sizeof(struct pcan_usbpro_warning_limit_t);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETLOOKUP_EXPLICIT:
		return sizeof(struct pcan_usbpro_lookup_explicit_t);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETLOOKUP_GROUP:
		return sizeof(struct pcan_usbpro_lookup_group_t);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETFILTERMODE:
		return sizeof(struct pcan_usbpro_filter_mode_t);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETRESET_MODE:
		return sizeof(struct pcan_usbpro_reset_mode_t);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETERRORFRAME:
		return sizeof(struct pcan_usbpro_error_frame_t);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_GETCANBUS_ERROR_STATUS:
		return sizeof(struct pcan_usbpro_error_status_t);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETREGISTER:
		return sizeof(struct pcan_usbpro_set_register_t);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_GETREGISTER:
		return sizeof(struct pcan_usbpro_get_register_t);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETGET_CALIBRATION_MSG:
		return sizeof(struct pcan_usbpro_calibration_t);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETGET_BUSLAST_MSG:
		return sizeof(struct pcan_usbpro_buslast_t);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETSTRING:
		return sizeof(struct pcan_usbpro_set_string_t);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_GETSTRING:
		return sizeof(struct pcan_usbpro_get_string_t);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_STRING:
		return sizeof(struct pcan_usbpro_string_t);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SAVEEEPROM:
		return sizeof(struct pcan_usbpro_save_eeprom_t);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_USB_IN_PACKET_DELAY:
		return sizeof(struct pcan_usbpro_packet_delay_t);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_TIMESTAMP_PARAM:
		return sizeof(struct pcan_usbpro_timestamp_param_t);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_ERROR_GEN_ID:
		return sizeof(struct pcan_usbpro_error_gen_id_t);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_ERROR_GEN_NOW:
		return sizeof(struct pcan_usbpro_error_gen_now_t);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SET_SOFTFILER:
		return sizeof(struct pcan_usbpro_softfiler_t);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SET_CANLED:
		return sizeof(struct pcan_usbpro_set_can_led_t);

	}

	return -1;
}

