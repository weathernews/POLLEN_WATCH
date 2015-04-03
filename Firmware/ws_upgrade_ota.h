/*
 * Copyright 2014, Broadcom Corporation
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Broadcom Corporation.
 */

/** @file
*
* WICED Smart UART Upgrade 
*
* This file provides definitions and function prototypes for the
* WICED Smart Upgrade over the Peripheral UART
*
*/
#ifndef WS_OTA_FU_H
#define WS_OTA_FU_H

// Please note that all UUIDs need to be reversed when publishing in the database

// {9E5D1E47-5C13-43A0-8635-82AD38A1386F}
// static const GUID WS_UPGRADE_SERVICE =
// { 0x9e5d1e47, 0x5c13, 0x43a0, { 0x86, 0x35, 0x82, 0xad, 0x38, 0xa1, 0x38, 0x6f } };
#define UUID_WS_UPGRADE_SERVICE                      0x6f, 0x38, 0xa1, 0x38, 0xad, 0x82, 0x35, 0x86, 0xa0, 0x43, 0x13, 0x5c, 0x47, 0x1e, 0x5d, 0x9e

// {E3DD50BF-F7A7-4E99-838E-570A086C666B}
// static const GUID WS_UPGRADE_CHARACTERISTIC_CONTROL_POINT =
// { 0xe3dd50bf, 0xf7a7, 0x4e99, { 0x83, 0x8e, 0x57, 0xa, 0x8, 0x6c, 0x66, 0x6b } };
#define UUID_WS_UPGRADE_CHARACTERISTIC_CONTROL_POINT 0x6b, 0x66, 0x6c, 0x08, 0x0a, 0x57, 0x8e, 0x83, 0x99, 0x4e, 0xa7, 0xf7, 0xbf, 0x50, 0xdd, 0xe3

// {92E86C7A-D961-4091-B74F-2409E72EFE36}
// static const GUID WS_UPGRADE_CHARACTERISTIC_DATA =
// { 0x92e86c7a, 0xd961, 0x4091, { 0xb7, 0x4f, 0x24, 0x9, 0xe7, 0x2e, 0xfe, 0x36 } };
#define UUID_WS_UPGRADE_CHARACTERISTIC_DATA          0x36, 0xfe, 0x2e, 0xe7, 0x09, 0x24, 0x4f, 0xb7, 0x91, 0x40, 0x61, 0xd9, 0x7a, 0x6c, 0xe8, 0x92

// {347F7608-2E2D-47EB-91E9-75D4EDC4DE3B}
// static const GUID WS_UPGRADE_CHARACTERISTIC_APP_INFO =
// { 0x347f7608, 0x2e2d, 0x47eb, { 0x91, 0xe9, 0x75, 0xd4, 0xed, 0xc4, 0xde, 0x3b } };

#define UUID_WS_UPGRADE_CHARACTERISTIC_APP_INFO      0x3b, 0xde, 0xc4, 0xed, 0xd4, 0x75, 0x3b, 0x91, 0xeb, 0x47, 0x2d, 0x2e, 0x08, 0x76, 0x7f, 0x34

// command definitions for the OTA FW upgrade
#define WS_UPGRADE_COMMAND_PREPARE_DOWNLOAD                 1
#define WS_UPGRADE_COMMAND_DOWNLOAD                         2
#define WS_UPGRADE_COMMAND_VERIFY                           3
#define WS_UPGRADE_COMMAND_FINISH                           4 // not currently used
#define WS_UPGRADE_COMMAND_GET_STATUS                       5 // not currently used
#define WS_UPGRADE_COMMAND_CLEAR_STATUS                     6 // not currently used
#define WS_UPGRADE_COMMAND_ABORT                            7

// event definitions for the OTA FW upgrade
#define WS_UPGRADE_STATUS_OK                                0
#define WS_UPGRADE_STATUS_UNSUPPORTED_COMMAND               1
#define WS_UPGRADE_STATUS_ILLEGAL_STATE                     2
#define WS_UPGRADE_STATUS_VERIFICATION_FAILED               3
#define WS_UPGRADE_STATUS_INVALID_IMAGE                     4
#define WS_UPGRADE_STATUS_INVALID_IMAGE_SIZE                5
#define WS_UPGRADE_STATUS_MORE_DATA                         6
#define WS_UPGRADE_STATUS_INVALID_APPID                     7
#define WS_UPGRADE_STATUS_INVALID_VERSION                   8

// following definitions can be shared between client and sensor
// to avoid unnecessary GATT Discovery
//
#define HANDLE_WS_UPGRADE_SERVICE                           0xff00
#define HANDLE_WS_UPGRADE_CHARACTERISTIC_CONTROL_POINT      0xff01
#define HANDLE_WS_UPGRADE_CONTROL_POINT                     0xff02
#define HANDLE_WS_UPGRADE_CLIENT_CONFIGURATION_DESCRIPTOR   0xff03
#define HANDLE_WS_UPGRADE_CHARACTERISTIC_DATA               0xff04
#define HANDLE_WS_UPGRADE_DATA                              0xff05
#define HANDLE_WS_UPGRADE_CHARACTERISTIC_APP_INFO           0xff06
#define HANDLE_WS_UPGRADE_APP_INFO                          0xff07

// Maximum data packet length we can process
#define WS_UPGRADE_MAX_DATA_LEN                             23

// Application ID 2 bytes plus 1 bytes major and minor versions
#define WS_UPGRADE_SUFFIX_LEN                               4

#pragma pack(1)

// structure to pass application information to upgrade application
typedef struct
{
    UINT16 ID;
    UINT8  Version_Major;
    UINT8  Version_Minor;
} WS_UPGRADE_APP_INFO;
#pragma pack()


// define entry points to upgrade functionality
int ws_upgrade_ota_init(void);
int ws_upgrade_ota_handle_command (UINT8 *data, int len);
int ws_upgrade_ota_handle_configuration (UINT8 *data, int len);
int ws_upgrade_ota_handle_data (UINT8 *data, int len);

#endif
