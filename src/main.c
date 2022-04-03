/* main.c - Application main entry point */ 

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <sys/byteorder.h>

static void start_scan(void);

static struct bt_conn *default_conn;

static struct bt_uuid_16 uuid = BT_UUID_INIT_16(0);
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_subscribe_params subscribe_params;

void * ReceivedPayload; //points to received payload so far
uint16_t nbrOfBytesPayloadReceived; //nbr of bytes received in ALL received chunks so far
									//do NOT initialise with 0, otherwise this counter gets reset with each chunk reeived
									//must be global var, so value is not lost at exit of this function


static uint8_t notify_func(struct bt_conn *conn,
			   struct bt_gatt_subscribe_params *params,
			   const void *data, uint16_t length)
{
	printk("\n\nENTER notify_func\n");
	printk("UUID= %X \n", BT_UUID_HRS);
	if (!data) {
		printk("[UNSUBSCRIBED]\n");
		params->value_handle = 0U;
		return BT_GATT_ITER_STOP;
	}

	printk("[NOTIFICATION] data %p length %u\n", data, length);
	void * lp = data;
	for (unsigned char i=0; i<length; i++) {
 		printk("%#X ", *(unsigned char *)(lp+i));
	}
	printk("\n");
	//reassemble data
	unsigned char ChunkNbr=0; //Sequence number, starts with zero
	unsigned char MoreDataFollows=0; //indicates whether more data chunks follow;
									 //equals zero for last data chunk, or if data consists of only one chunk
	unsigned char ProtocolOverhead = sizeof(ChunkNbr) + sizeof(MoreDataFollows); //overhead in each chunk

	memcpy(&ChunkNbr, data, sizeof(ChunkNbr)); //extact ChunkNbr from data
	memcpy(&MoreDataFollows, data+sizeof(ChunkNbr), sizeof(MoreDataFollows)); //extact MoreDataFollows from data

	printk("ChunkNbr= %d\n", ChunkNbr);
	printk("MoreDataFollows= %d\n", MoreDataFollows);

	//append payload to already received payload
	if (ChunkNbr == 0) {
		//first Chunk
		nbrOfBytesPayloadReceived = length - ProtocolOverhead;
		ReceivedPayload = k_malloc(nbrOfBytesPayloadReceived);
		memcpy(ReceivedPayload, data + ProtocolOverhead, nbrOfBytesPayloadReceived);
	}
	else {
		//it is one of the following chunks, possibly the latest chunk
		//Zephyr does NOT support k_realloc like realloc in standard C
		//So allocate new memory, copy old values, and free old memory
		int16_t temp = nbrOfBytesPayloadReceived;
		nbrOfBytesPayloadReceived += (length - ProtocolOverhead);
		void * tempReceivedPayload = k_malloc(nbrOfBytesPayloadReceived); //allocate space for total payload received
		if (tempReceivedPayload == NULL) {printk("ERROR*** could not allocate memory\n");}
		memcpy(tempReceivedPayload, ReceivedPayload, temp); //copy already received payload until now
		memcpy(tempReceivedPayload + temp, data + ProtocolOverhead, (length - ProtocolOverhead)); //append new received payload

		k_free(ReceivedPayload); //free previous allocated space
		ReceivedPayload = tempReceivedPayload; //ReceivedPayload always point to complete set of received payloads
	}
	if (MoreDataFollows == 0) {
		//return complete payload to application
		printk("TOTAL PAYLOAD RECEIVED\n");
		for (int i=0; i<nbrOfBytesPayloadReceived; i++) {
			printk("%#X ", *(unsigned char *)(ReceivedPayload+i));
		}
		k_free(ReceivedPayload); 
	}

	//if not MoreDataFollows then finish received data (send to higher layer)
	//else wait for next chunk

	//structure of first chunk: <TAG>, <+sizeof(>LENGTH>, <MoreDataFollows> <payload>
	//structure of next chunks: <MoreDataFollows> <payload> , so NO TAG and NO LENGTH in these chunks
	//<TAG> is not yet used and <LENGTH> is extra check whether all data is received

	return BT_GATT_ITER_CONTINUE;
}

static uint8_t discover_func(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params)
{
	printk("***baswi*** ENTER discover_func\n");
	int err;

	if (!attr) {
		printk("Discover complete\n");
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	printk("[ATTRIBUTE] handle %u\n", attr->handle);

	if (!bt_uuid_cmp(discover_params.uuid, BT_UUID_HRS)) {			//returns 0(false) if uuid==HRS
		memcpy(&uuid, BT_UUID_HRS_MEASUREMENT, sizeof(uuid));
		discover_params.uuid = &uuid.uuid;
		discover_params.start_handle = attr->handle + 1;
		discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else if (!bt_uuid_cmp(discover_params.uuid,
				BT_UUID_HRS_MEASUREMENT)) {							//returns 0(false) if uuid==HRS_MEASUREMENT
		memcpy(&uuid, BT_UUID_GATT_CCC, sizeof(uuid));
		discover_params.uuid = &uuid.uuid;
		discover_params.start_handle = attr->handle + 2;
		discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
		subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else {
		subscribe_params.notify = notify_func;					//notification call back function
		subscribe_params.value = BT_GATT_CCC_NOTIFY;
		subscribe_params.ccc_handle = attr->handle;

		err = bt_gatt_subscribe(conn, &subscribe_params);		//else subscribe, Subscribe Attribute Value Notification.
																//subscribe to value notification using the Client Characteristic Configuration handle.
																//If notification received subscribe value callback is called to return notified value. One may then decide whether to unsubscribe directly from this callback. 
																//Notification callback with NULL data will not be called if subscription was removed by this method.
		if (err && err != -EALREADY) {
			printk("Subscribe failed (err %d)\n", err);
		} else {
			printk("[SUBSCRIBED]\n");
		}
		return BT_GATT_ITER_STOP; //unsubscribe from value notifications.
	}
	return BT_GATT_ITER_STOP; //unsubscribe from value notifications.
}

static bool eir_found(struct bt_data *data, void *user_data)
{
	printk("***baswi*** ENTER eir_found\n");
	bt_addr_le_t *addr = user_data;
	int i;

	printk("[AD]: %u data_len %u\n", data->type, data->data_len);

	switch (data->type) {
	case BT_DATA_UUID16_SOME:
	case BT_DATA_UUID16_ALL:
		if (data->data_len % sizeof(uint16_t) != 0U) {
			printk("AD malformed\n");
			return true; //baswi - continue parsing elements
		}

		for (i = 0; i < data->data_len; i += sizeof(uint16_t)) {
			struct bt_le_conn_param *param;
			struct bt_uuid *uuid;
			uint16_t u16;
			int err;

			memcpy(&u16, &data->data[i], sizeof(u16));
			uuid = BT_UUID_DECLARE_16(sys_le16_to_cpu(u16));
			if (bt_uuid_cmp(uuid, BT_UUID_HRS)) {					//baswi - function returns 0 (false) if uuids are equal
				continue;											//IF not HRS found, THEN execute next loop
			}
			printk("***baswi*** HRS found; continue parsing data\n");	//baswi HRS UUID found

			err = bt_le_scan_stop();
			if (err) {
				printk("Stop LE scan failed (err %d)\n", err);
				continue;
			}

			param = BT_LE_CONN_PARAM_DEFAULT;
			err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, //baswi - Automatically connect to remote devices in the 
																  //filter accept list.  The procedure will continue until 
																  //a single connection is established or the procedure is stopped
																  //through bt_conn_create_auto_stop. 
																  //To establish connections to all devices in the the filter 
																  //accept list the procedure should be started again
						param, &default_conn);
			if (err) {
				printk("Create conn failed (err %d)\n", err);
				start_scan();
			}

			return false; //baswi - stop parsing elements
		}
	}

	return true;
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type, //baswi - this call back function is called when a device is found
			 struct net_buf_simple *ad)
{
	printk("***baswi*** ENTER device_found\n");
	char dev[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(addr, dev, sizeof(dev));
	printk("[DEVICE]: %s, AD evt type %u, AD data len %u, RSSI %i\n",
	       dev, type, ad->len, rssi);

	/* We're only interested in connectable events */
	if (type == BT_GAP_ADV_TYPE_ADV_IND ||
	    type == BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		bt_data_parse(ad, eir_found, (void *)addr); //Helper for parsing the basic data types used for 
													//Extended Inquiry Response (EIR), Advertising Data (AD), and OOB data blocks.
													//The most common scenario is to call this helper on 
													//the advertising data received in the callback that was given to bt_le_scan_start().
													//eir_found: Callback function called for each element that’s found in the data. The callback should return true to continue parsing, or false to stop parsing.
	}
}

static void start_scan(void)
{
	printk("***baswi*** ENTER start_scan\n");
	int err;

	/* Use active scanning and disable duplicate filtering to handle any
	 * devices that might update their advertising data at runtime. */
	struct bt_le_scan_param scan_param = {
		.type       = BT_LE_SCAN_TYPE_ACTIVE,
		.options    = BT_LE_SCAN_OPT_NONE,
		.interval   = BT_GAP_SCAN_FAST_INTERVAL,
		.window     = BT_GAP_SCAN_FAST_WINDOW,
	};

	err = bt_le_scan_start(&scan_param, device_found);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
		return;
	}

	printk("Scanning successfully started\n");
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	printk("***baswi*** ENTER connected\n");
	char addr[BT_ADDR_LE_STR_LEN];
	int err;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (conn_err) {
		printk("Failed to connect to %s (%u)\n", addr, conn_err);

		bt_conn_unref(default_conn);
		default_conn = NULL;

		start_scan();
		return;
	}

	printk("Connected: %s\n", addr);

	if (conn == default_conn) {
		memcpy(&uuid, BT_UUID_HRS, sizeof(uuid));
		discover_params.uuid = &uuid.uuid;
		discover_params.func = discover_func;
		discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
		discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
		discover_params.type = BT_GATT_DISCOVER_PRIMARY;

		err = bt_gatt_discover(default_conn, &discover_params); //baswi - initiate Discover procedures
		if (err) {
			printk("Discover failed(err %d)\n", err);
			return;
		}
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("***baswi*** ENTER disconnected\n");
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected: %s (reason 0x%02x)\n", addr, reason);

	if (default_conn != conn) {
		return;
	}

	bt_conn_unref(default_conn);
	default_conn = NULL;

	start_scan();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

//call back called when MTU is updated
void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	printk("Updated MTU: TX: %d RX: %d bytes\n", tx, rx);
//@@@	CurrentMTUrx = rx;
	//@@@CurrentMTUtx = tx;
}

//baswi added
static struct bt_gatt_cb gatt_callbacks = {
	.att_mtu_updated = mtu_updated
};


void main(void)
{
	int err;
	err = bt_enable(NULL);

	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	bt_gatt_cb_register(&gatt_callbacks); 	//needed for mtu_updated call back function

	start_scan();
}
