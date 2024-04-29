/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
#include <zephyr/init.h>
#include <zephyr/sys/util.h>

#include <canopennode.h>

#define LOG_LEVEL CONFIG_CANOPEN_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(canopen_driver);

K_KERNEL_STACK_DEFINE(canopen_tx_workq_stack, CONFIG_CANOPENNODE_TX_WORKQUEUE_STACK_SIZE);

struct k_work_q canopen_tx_workq;

struct canopen_tx_work_container {
	struct k_work work;
	CO_CANmodule_t *CANmodule;
};

struct canopen_tx_work_container canopen_tx_queue;

static void canopen_detach_all_rx_filters(CO_CANmodule_t *CANmodule)
{
	uint16_t i;

	if (CANmodule == NULL || CANmodule->rxArray == NULL) {
		return;
	}

	if (CANmodule->useCANrxFilters) {
		for (i = 0U; i < CANmodule->rxSize; i++) {
			if (CANmodule->rxArray[i].filter_id != -ENOSPC) {
				can_remove_rx_filter(CANmodule->CANptr,
						     CANmodule->rxArray[i].filter_id);
				CANmodule->rxArray[i].filter_id = -ENOSPC;
			}
		}
	} else {
		if (CANmodule->global_filter_id != -ENOSPC) {
			can_remove_rx_filter(CANmodule->CANptr, CANmodule->global_filter_id);
			CANmodule->global_filter_id = -ENOSPC;
		}
	}
}

static void canopen_rx_callback(const struct device *dev, struct can_frame *frame, void *user_data)
{
	CO_CANmodule_t *CANmodule = (CO_CANmodule_t *)user_data;
	CO_CANrxMsg_t rxMsg;
	CO_CANrx_t *buffer;
	uint16_t i;

	ARG_UNUSED(dev);

	if (CANmodule == NULL) {
		LOG_ERR("failed to process CAN tx callback");
		return;
	}

	/* Loop through registered rx buffers in priority order */
	for (i = 0; i < CANmodule->rxSize; i++) {
		buffer = &CANmodule->rxArray[i];
		if (CANmodule->useCANrxFilters &&
		    (buffer->filter_id < 0 || buffer->CANrx_callback == NULL)) {
			continue;
		}

		if (((frame->id ^ buffer->ident) & buffer->mask) == 0U) {
			rxMsg.ident = frame->id;
			rxMsg.DLC = frame->dlc;
			memcpy(rxMsg.data, frame->data, frame->dlc);
			buffer->CANrx_callback(buffer->object, &rxMsg);
			break;
		}
	}
}

static void canopen_tx_callback(const struct device *dev, int error, void *user_data)
{
	CO_CANmodule_t *CANmodule = (CO_CANmodule_t *)user_data;

	ARG_UNUSED(dev);

	if (CANmodule == NULL) {
		LOG_ERR("failed to process CAN tx callback");
		return;
	}

	if (error == 0) {
		CANmodule->firstCANtxMessage = false;
		CANmodule->bufferInhibitFlag = false;
	}

	k_work_submit_to_queue(&canopen_tx_workq, &canopen_tx_queue.work);
}

static void canopen_tx_retry(struct k_work *item)
{
	struct canopen_tx_work_container *container =
		CONTAINER_OF(item, struct canopen_tx_work_container, work);
	CO_CANmodule_t *CANmodule = container->CANmodule;
	CO_CANtx_t *buffer;
	int err;
	uint16_t i;

	CO_LOCK_CAN_SEND(CANmodule);

	if (CANmodule->CANtxCount > 0U) {
		buffer = &CANmodule->txArray[0];

		/*  Try to send more buffers, process all empty ones */
		for (i = CANmodule->txSize; i > 0U; i--) {
			/* Try to send message */
			if (buffer->bufferFull) {
				struct can_frame frame;

				memset(&frame, 0, sizeof(frame));
				frame.id = buffer->ident;
				frame.dlc = buffer->DLC;
				frame.flags = 0;
				memcpy(frame.data, buffer->data, buffer->DLC);

				err = can_send(CANmodule->CANptr, &frame, K_NO_WAIT,
					       canopen_tx_callback, CANmodule);
				if (err == -EAGAIN) {
					break;
				} else if (err != 0) {
					LOG_ERR("failed to send CAN frame (ret %d)", err);
				}
				buffer->bufferFull = false;
				CANmodule->CANtxCount--;
				CANmodule->bufferInhibitFlag = buffer->syncFlag;
			}
			buffer++;
		}

		/* Clear counter if no more messages */
		if (i == 0U) {
			CANmodule->CANtxCount = 0U;
		}
	}

	CO_UNLOCK_CAN_SEND(CANmodule);
}

void CO_CANsetConfigurationMode(void *CANptr)
{
	int err;

	if (CANptr == NULL) {
		return;
	}

	err = can_stop(CANptr);
	if (err != 0 && err != -EALREADY) {
		LOG_ERR("failed to stop CAN interface (err %d)", err);
	}
}

void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule)
{
	int err;

	if (CANmodule == NULL) {
		return;
	}

	err = can_start(CANmodule->CANptr);
	if (err != 0 && err != -EALREADY) {
		LOG_ERR("failed to start CAN interface (err %d)", err);
		return;
	}
	CANmodule->CANnormal = true;
}

CO_ReturnError_t CO_CANmodule_init(CO_CANmodule_t *CANmodule, void *CANptr, CO_CANrx_t rxArray[],
				   uint16_t rxSize, CO_CANtx_t txArray[], uint16_t txSize,
				   uint16_t CANbitRate)
{
	uint16_t i;
	bool_t use_can_rx_filters = false;
	int err;
	int max_filters;

	/* Verify arguments */
	if (CANmodule == NULL || rxArray == NULL || txArray == NULL) {
		return CO_ERROR_ILLEGAL_ARGUMENT;
	}


	max_filters = can_get_max_filters(CANptr, false);
	if (max_filters != -ENOSYS) {
		if (max_filters < 0) {
			LOG_ERR("unable to determine number of CAN RX filters");
			return CO_ERROR_SYSCALL;
		}

		if (rxSize > max_filters) {
			use_can_rx_filters = false;
			LOG_DBG("insufficient number of concurrent CAN RX filters"
				" (needs %d, %d available)",
				rxSize, max_filters);
		} else if (rxSize < max_filters) {
			use_can_rx_filters = true;
			LOG_DBG("excessive number of concurrent CAN RX filters enabled"
				" (needs %d, %d available)",
				rxSize, max_filters);
		}
	}

	canopen_detach_all_rx_filters(CANmodule);

	/* Configure object variables */
	CANmodule->CANptr = CANptr;
	CANmodule->rxArray = rxArray;
	CANmodule->rxSize = rxSize;
	CANmodule->txArray = txArray;
	CANmodule->txSize = txSize;
	CANmodule->CANerrorStatus = 0;
	CANmodule->CANnormal = false;
	CANmodule->useCANrxFilters = use_can_rx_filters; /* microcontroller dependent */
	CANmodule->bufferInhibitFlag = false;
	CANmodule->firstCANtxMessage = true;
	CANmodule->CANtxCount = 0U;
	CANmodule->errOld = 0U;
	CANmodule->global_filter_id = -ENOSPC;

	for (i = 0U; i < rxSize; i++) {
		rxArray[i].ident = 0U;
		rxArray[i].mask = 0xFFFFU;
		rxArray[i].object = NULL;
		rxArray[i].CANrx_callback = NULL;
		rxArray[i].filter_id = -ENOSPC;
	}
	for (i = 0U; i < txSize; i++) {
		txArray[i].bufferFull = false;
	}

	/* Configure CAN module registers */
	err = can_set_mode(CANmodule->CANptr, CAN_MODE_NORMAL);
	if (err) {
		LOG_ERR("failed to configure CAN interface (err %d)", err);
		return CO_ERROR_ILLEGAL_ARGUMENT;
	}

	/* Configure CAN timing */
	err = can_set_bitrate(CANmodule->CANptr, KHZ(CANbitRate));
	if (err) {
		LOG_ERR("failed to configure CAN bitrate (err %d)", err);
		return CO_ERROR_ILLEGAL_ARGUMENT;
	}

	/* Configure CAN module hardware filters */
	if (CANmodule->useCANrxFilters) {
		/* CAN module filters are used, they will be configured with */
		/* CO_CANrxBufferInit() functions, called by separate CANopen */
		/* init functions. */
		/* Configure all masks so, that received message must match filter */
	} else {
		/* CAN module filters are not used, all messages with standard 11-bit */
		/* identifier will be received */
		/* Configure mask 0 so, that all messages with standard identifier are accepted */
		struct can_filter filter;
		filter.flags = CAN_FILTER_DATA;
		filter.id = 0x7FF;
		filter.mask = 0x000;

		CANmodule->global_filter_id =
			can_add_rx_filter(CANptr, canopen_rx_callback, CANmodule, &filter);
		if (CANmodule->global_filter_id < 0) {
			LOG_ERR("failed to add CAN rx callback, no free filter %d",
				CANmodule->global_filter_id);
			return CO_ERROR_OUT_OF_MEMORY;
		}
	}

	/* Configure CAN interrupt registers */

	/* Configure operating system */
	canopen_tx_queue.CANmodule = CANmodule;
	k_work_queue_start(&canopen_tx_workq, canopen_tx_workq_stack,
			   K_KERNEL_STACK_SIZEOF(canopen_tx_workq_stack),
			   CONFIG_CANOPENNODE_TX_WORKQUEUE_PRIORITY, NULL);
	k_thread_name_set(&canopen_tx_workq.thread, "canopen_tx_workq");
	k_work_init(&canopen_tx_queue.work, canopen_tx_retry);

	k_mutex_init(&CANmodule->send_mutex);
	k_mutex_init(&CANmodule->emcy_mutex);
	k_mutex_init(&CANmodule->od_mutex);

	return CO_ERROR_NO;
}

void CO_CANmodule_disable(CO_CANmodule_t *CANmodule)
{
	int err;

	if (CANmodule == NULL && CANmodule->CANptr == NULL) {
		return;
	}

	canopen_detach_all_rx_filters(CANmodule);

	err = can_stop(CANmodule->CANptr);
	if (err != 0 && err != -EALREADY) {
		LOG_ERR("failed to disable CAN interface (err %d)", err);
	}
}

CO_ReturnError_t CO_CANrxBufferInit(CO_CANmodule_t *CANmodule, uint16_t index, uint16_t ident,
				    uint16_t mask, bool_t rtr, void *object,
				    void (*CANrx_callback)(void *object, void *message))
{
	CO_ReturnError_t err = CO_ERROR_NO;

	if ((CANmodule != NULL) && (object != NULL) && (CANrx_callback != NULL) &&
	    (index < CANmodule->rxSize)) {
		CO_CANrx_t *buffer = &CANmodule->rxArray[index];

		/* Configure object variables */
		buffer->object = object;
		buffer->CANrx_callback = CANrx_callback;

		/* CAN identifier and CAN mask, bit aligned with CAN module FIFO buffers (RTR is
		 * extra) */
		buffer->ident = ident & 0x07FFU;
		buffer->mask = mask & 0x07FFU;

		/* Set CAN hardware module filter and mask. */
		if (CANmodule->useCANrxFilters) {
			struct can_filter filter;
			filter.flags = (rtr ? CAN_FILTER_RTR : CAN_FILTER_DATA);
			filter.id = ident;
			filter.mask = mask;

			if (buffer->filter_id >= 0) {
				can_remove_rx_filter(CANmodule->CANptr, buffer->filter_id);
			}

			buffer->filter_id = can_add_rx_filter(
				CANmodule->CANptr, canopen_rx_callback, CANmodule, &filter);
			if (buffer->filter_id < 0) {
				LOG_ERR("failed to add CAN rx callback, no free filter %d",
					buffer->filter_id);
				err = CO_ERROR_OUT_OF_MEMORY;
			}
		}
	} else {
		err = CO_ERROR_ILLEGAL_ARGUMENT;
	}

	return err;
}

CO_CANtx_t *CO_CANtxBufferInit(CO_CANmodule_t *CANmodule, uint16_t index, uint16_t ident,
			       bool_t rtr, uint8_t noOfBytes, bool_t syncFlag)
{
	CO_CANtx_t *buffer = NULL;

	if ((CANmodule != NULL) && (index < CANmodule->txSize)) {
		buffer = &CANmodule->txArray[index];

		/* CAN identifier, DLC and rtr, bit aligned with CAN module transmit buffer.
		 * Microcontroller specific. */
		buffer->ident = ((uint32_t)ident & 0x07FFU);
		buffer->rtr = rtr;
		buffer->DLC = noOfBytes;
		buffer->bufferFull = false;
		buffer->syncFlag = syncFlag;
	}

	return buffer;
}

CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer)
{
	CO_ReturnError_t err = CO_ERROR_NO;
	struct can_frame frame;

	if (CANmodule == NULL || CANmodule->CANptr == NULL || buffer == NULL) {
		return CO_ERROR_ILLEGAL_ARGUMENT;
	}

	if (buffer->bufferFull) {
		if (!CANmodule->firstCANtxMessage) {
			/* don't set error, if bootup message is still on buffers */
			CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
		}
		err = CO_ERROR_TX_OVERFLOW;
	}

	memset(&frame, 0, sizeof(frame));

	CO_LOCK_CAN_SEND(CANmodule);

	frame.id = buffer->ident;
	frame.dlc = buffer->DLC;
	frame.flags = buffer->rtr ? CAN_FRAME_RTR : 0;
	memcpy(frame.data, buffer->data, buffer->DLC);

	if (CANmodule->CANtxCount == 0) {
		CANmodule->bufferInhibitFlag = buffer->syncFlag;
		err = can_send(CANmodule->CANptr, &frame, K_NO_WAIT, canopen_tx_callback,
			       CANmodule);
		if (err == -EAGAIN) {
			buffer->bufferFull = true;
			CANmodule->CANtxCount++;
		} else if (err != 0) {
			LOG_ERR("failed to send CAN frame (err %d)", err);
		}
	} else {

		buffer->bufferFull = true;
		CANmodule->CANtxCount++;
	}
	CO_UNLOCK_CAN_SEND(CANmodule);

	return err;
}

void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule)
{
	uint32_t tpdoDeleted = 0U;

	CO_LOCK_CAN_SEND(CANmodule);
	/* Abort message from CAN module, if there is synchronous TPDO.
	 * Take special care with this functionality. */
	if (CANmodule->bufferInhibitFlag) {
		CANmodule->bufferInhibitFlag = false;
		tpdoDeleted = 1U;
	}
	/* delete also pending synchronous TPDOs in TX buffers */
	if (CANmodule->CANtxCount != 0U) {
		uint16_t i;
		CO_CANtx_t *buffer = &CANmodule->txArray[0];
		for (i = CANmodule->txSize; i > 0U; i--) {
			if (buffer->bufferFull) {
				if (buffer->syncFlag) {
					buffer->bufferFull = false;
					CANmodule->CANtxCount--;
					tpdoDeleted = 2U;
				}
			}
			buffer++;
		}
	}
	CO_UNLOCK_CAN_SEND(CANmodule);

	if (tpdoDeleted != 0U) {
		CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
	}
}

void CO_CANmodule_process(CO_CANmodule_t *CANmodule)
{
	struct can_bus_err_cnt err_cnt;
	enum can_state state;
	uint8_t rx_overflows;
	uint32_t err;
	int ret;

	/*
	 * TODO: Zephyr lacks an API for reading the rx mailbox
	 * overflow counter.
	 */
	rx_overflows = 0;

	ret = can_get_state(CANmodule->CANptr, &state, &err_cnt);
	if (ret != 0) {
		LOG_ERR("failed to get CAN controller state (ret %d)", ret);
		return;
	}

	err = ((uint32_t)err_cnt.tx_err_cnt << 16) | ((uint32_t)err_cnt.rx_err_cnt << 8) |
	      rx_overflows;

	if (err != CANmodule->errOld) {
		uint16_t status = CANmodule->CANerrorStatus;

		CANmodule->errOld = err;

		if (state == CAN_STATE_BUS_OFF) {
			/* Bus off */
			status |= CO_CAN_ERRTX_BUS_OFF;
		} else {
			/* recalculate CANerrorStatus, first clear some flags */
			status &= 0xFFFF ^ (CO_CAN_ERRTX_BUS_OFF | CO_CAN_ERRRX_WARNING |
					    CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_WARNING |
					    CO_CAN_ERRTX_PASSIVE);

			/* rx bus passive */
			if (err_cnt.rx_err_cnt > 128) {
				status |= CO_CAN_ERRRX_PASSIVE;
			}
			/* rx bus WARNING */
			if (err_cnt.rx_err_cnt > 96) {
				status |= CO_CAN_ERRRX_WARNING;
			}

			/* tx bus passive */
			if (err_cnt.tx_err_cnt > 128) {
				status |= CO_CAN_ERRTX_PASSIVE;
			}
			/* tx bus WARNING */
			if (err_cnt.tx_err_cnt > 96) {
				status |= CO_CAN_ERRTX_WARNING;
			}
		}

		/* This code can be activated if we can read the overflows*/
		if (false && rx_overflows != 0U) {
			status |= CO_CAN_ERRRX_OVERFLOW;
		}
		CANmodule->CANerrorStatus = status;
	}
}
