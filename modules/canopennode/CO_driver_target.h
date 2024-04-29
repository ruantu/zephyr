/*
 * Device and application specific definitions for CANopenNode.
 *
 * @file        CO_driver_target.h
 * @author      Janez Paternoster
 * @copyright   2021 Janez Paternoster
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef CO_DRIVER_TARGET_H
#define CO_DRIVER_TARGET_H

/* This file contains device and application specific definitions.
 * It is included from CO_driver.h, which contains documentation
 * for common definitions below. */

#include <zephyr/kernel.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef CO_DRIVER_CUSTOM
#include "CO_driver_custom.h"
#endif

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef CONFIG_CANOPENNODE_MULTIPLE_OD
#define CO_MULTIPLE_OD
#else
#define CO_USE_GLOBALS
#endif

/* Stack configuration override default values.
 * For more information see file CO_config.h. */
#define CO_CONFIG_GLOBAL_FLAG_TIMERNEXT CO_CONFIG_FLAG_TIMERNEXT

#ifndef CO_CONFIG_NMT
#define CO_CONFIG_NMT (CO_CONFIG_NMT_CALLBACK_CHANGE |      \
                       CO_CONFIG_NMT_MASTER |               \
                       CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE | \
                       CO_CONFIG_GLOBAL_FLAG_TIMERNEXT)
#endif

#ifndef CO_CONFIG_HB_CONS
#define CO_CONFIG_HB_CONS (CO_CONFIG_HB_CONS_ENABLE |           \
                           CO_CONFIG_HB_CONS_CALLBACK_CHANGE |  \
                           CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE | \
                           CO_CONFIG_GLOBAL_FLAG_TIMERNEXT |    \
                           CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC)
#endif

#ifndef CO_CONFIG_EM
#define CO_CONFIG_EM (CO_CONFIG_EM_PRODUCER |              \
                      CO_CONFIG_EM_PROD_CONFIGURABLE |     \
                      CO_CONFIG_EM_PROD_INHIBIT |          \
                      CO_CONFIG_EM_HISTORY |               \
                      CO_CONFIG_EM_STATUS_BITS |           \
                      CO_CONFIG_EM_CONSUMER |              \
                      CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE | \
                      CO_CONFIG_GLOBAL_FLAG_TIMERNEXT)
#endif

#ifndef CO_CONFIG_SDO_SRV
#define CO_CONFIG_SDO_SRV (CO_CONFIG_SDO_SRV_SEGMENTED |        \
                           CO_CONFIG_SDO_SRV_BLOCK |            \
                           CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE | \
                           CO_CONFIG_GLOBAL_FLAG_TIMERNEXT |    \
                           CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC)
#endif

#ifndef CO_CONFIG_SDO_SRV_BUFFER_SIZE
#define CO_CONFIG_SDO_SRV_BUFFER_SIZE 900
#endif

#ifndef CO_CONFIG_SDO_CLI
#define CO_CONFIG_SDO_CLI (CO_CONFIG_SDO_CLI_ENABLE |           \
                           CO_CONFIG_SDO_CLI_SEGMENTED |        \
                           CO_CONFIG_SDO_CLI_BLOCK |            \
                           CO_CONFIG_SDO_CLI_LOCAL |            \
                           CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE | \
                           CO_CONFIG_GLOBAL_FLAG_TIMERNEXT |    \
                           CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC)
#endif

#ifndef CO_CONFIG_TIME
#define CO_CONFIG_TIME (CO_CONFIG_TIME_ENABLE |              \
                        CO_CONFIG_TIME_PRODUCER |            \
                        CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE | \
                        CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC)
#endif

#ifndef CO_CONFIG_SYNC
#define CO_CONFIG_SYNC (CO_CONFIG_SYNC_ENABLE |           \
                        CO_CONFIG_SYNC_PRODUCER |         \
                        CO_CONFIG_FLAG_CALLBACK_PRE |     \
                        CO_CONFIG_GLOBAL_FLAG_TIMERNEXT | \
                        CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC)
#endif

#ifndef CO_CONFIG_PDO
#define CO_CONFIG_PDO (CO_CONFIG_RPDO_ENABLE |              \
                       CO_CONFIG_TPDO_ENABLE |              \
                       CO_CONFIG_RPDO_TIMERS_ENABLE |       \
                       CO_CONFIG_TPDO_TIMERS_ENABLE |       \
                       CO_CONFIG_PDO_SYNC_ENABLE |          \
                       CO_CONFIG_PDO_OD_IO_ACCESS |         \
                       CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE | \
                       CO_CONFIG_GLOBAL_FLAG_TIMERNEXT |    \
                       CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC)
#endif

#ifndef CO_CONFIG_LSS
#define CO_CONFIG_LSS (CO_CONFIG_LSS_MASTER | \
                       CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE)
#endif

#ifndef CO_CONFIG_CRC16
#define CO_CONFIG_CRC16 (CO_CONFIG_CRC16_ENABLE | \
                         CO_CONFIG_CRC16_EXTERNAL)
#endif

#ifndef CO_CONFIG_FIFO
#define CO_CONFIG_FIFO (CO_CONFIG_FIFO_ENABLE |         \
                        CO_CONFIG_FIFO_ALT_READ |       \
                        CO_CONFIG_FIFO_CRC16_CCITT |    \
                        CO_CONFIG_FIFO_ASCII_COMMANDS | \
                        CO_CONFIG_FIFO_ASCII_DATATYPES)
#endif

#ifdef CONFIG_CANOPENNODE_STORAGE
#ifndef CO_CONFIG_STORAGE
#define CO_CONFIG_STORAGE (CO_CONFIG_STORAGE_ENABLE)
#endif
#else
#ifndef CO_CONFIG_STORAGE
#define CO_CONFIG_STORAGE (0)
#endif
#endif

#ifdef CONFIG_CANOPENNODE_LEDS
#ifndef CO_CONFIG_LEDS
#define CO_CONFIG_LEDS (CO_CONFIG_LEDS_ENABLE | \
                        CO_CONFIG_GLOBAL_FLAG_TIMERNEXT)
#endif
#else
#ifndef CO_CONFIG_LEDS
#define CO_CONFIG_LEDS (0)
#endif
#endif

/* Basic definitions. If big endian, CO_SWAP_xx macros must swap bytes. */
#define CO_LITTLE_ENDIAN
#define CO_SWAP_16(x) x
#define CO_SWAP_32(x) x
#define CO_SWAP_64(x) x
        /* NULL is defined in stddef.h */
        /* true and false are defined in stdbool.h */
        /* int8_t to uint64_t are defined in stdint.h */
        typedef uint_fast8_t bool_t;
        typedef float float32_t;
        typedef double float64_t;

        typedef struct canopen_rx_msg
        {
                uint8_t data[8];
                uint16_t ident;
                uint8_t DLC;
        } CO_CANrxMsg_t;

/* Access to received CAN message */
#define CO_CANrxMsg_readIdent(msg) ((((uint16_t)(((CO_CANrxMsg_t *)(msg))->ident)) >> 2) & 0x7FF)
#define CO_CANrxMsg_readDLC(msg) ((uint8_t)(((CO_CANrxMsg_t *)(msg))->DLC))
#define CO_CANrxMsg_readData(msg) ((uint8_t *)(((CO_CANrxMsg_t *)(msg))->data))

        /* Received message object */
        typedef struct
        {
                uint16_t ident;
                uint16_t mask;
                int filter_id;
                void *object;
                void (*CANrx_callback)(void *object, void *message);
        } CO_CANrx_t;

        /* Transmit message object */
        typedef struct
        {
                uint32_t ident;
                uint8_t DLC;
                bool_t rtr;
                uint8_t data[8];
                volatile bool_t bufferFull;
                volatile bool_t syncFlag;
        } CO_CANtx_t;

        /* CAN module object */
        typedef struct
        {
                void *CANptr;
                CO_CANrx_t *rxArray;
                uint16_t rxSize;
                CO_CANtx_t *txArray;
                uint16_t txSize;
                uint16_t CANerrorStatus;
                volatile bool_t CANnormal;
                volatile bool_t useCANrxFilters;
                volatile bool_t bufferInhibitFlag;
                volatile bool_t firstCANtxMessage;
                volatile uint16_t CANtxCount;
                uint32_t errOld;

                /* Zephyr filter related */
                int global_filter_id;

                /* Zephyr mutex related */
                struct k_mutex send_mutex;
                struct k_mutex emcy_mutex;
                struct k_mutex od_mutex;
        } CO_CANmodule_t;

        /* Data storage object for one entry. */
        typedef struct
        {
                void *addr;
                size_t len;
                uint8_t subIndexOD;
                uint8_t attr;
                void *storageModule;
                uint16_t crc;
                size_t eepromAddrSignature;
                size_t eepromAddr;
                size_t offset;
                void *additionalParameters;
        } CO_storage_entry_t;

/* (un)lock critical section in CO_CANsend() */
#define CO_LOCK_CAN_SEND(CAN_MODULE)                          \
        do                                                        \
        {                                                         \
                k_mutex_lock(&((CAN_MODULE)->send_mutex), K_FOREVER); \
        } while (0);
#define CO_UNLOCK_CAN_SEND(CAN_MODULE)               \
        do                                               \
        {                                                \
                k_mutex_unlock(&((CAN_MODULE)->send_mutex)); \
        } while (0);

/* (un)lock critical section in CO_errorReport() or CO_errorReset() */
#define CO_LOCK_EMCY(CAN_MODULE)                              \
        do                                                        \
        {                                                         \
                k_mutex_lock(&((CAN_MODULE)->emcy_mutex), K_FOREVER); \
        } while (0);
#define CO_UNLOCK_EMCY(CAN_MODULE)                   \
        do                                               \
        {                                                \
                k_mutex_unlock(&((CAN_MODULE)->emcy_mutex)); \
        } while (0);

/* (un)lock critical section when accessing Object Dictionary */
#define CO_LOCK_OD(CAN_MODULE)                              \
        do                                                      \
        {                                                       \
                k_mutex_lock(&((CAN_MODULE)->od_mutex), K_FOREVER); \
        } while (0);
#define CO_UNLOCK_OD(CAN_MODULE)                   \
        do                                             \
        {                                              \
                k_mutex_unlock(&((CAN_MODULE)->od_mutex)); \
        } while (0);

/* Synchronization between CAN receive and message processing threads. */
#define CO_MemoryBarrier()
#define CO_FLAG_READ(rxNew) ((rxNew) != NULL)
#define CO_FLAG_SET(rxNew)  \
        {                       \
                CO_MemoryBarrier(); \
                rxNew = (void *)1L; \
        }
#define CO_FLAG_CLEAR(rxNew) \
        {                        \
                CO_MemoryBarrier();  \
                rxNew = NULL;        \
        }

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CO_DRIVER_TARGET_H */
