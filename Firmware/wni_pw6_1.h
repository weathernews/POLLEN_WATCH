// (C) 1996-2015 Weathernews Inc. All Rights Reserved.

#define WNI_PW6_SERVICE_HANDLE	0x1001
#define WNI_PW6_COUNT_HANDLE	0x1021
#define WNI_PW6_COUNT_HANDLE_VALUE	0x1022
#define WNI_PW6_DATA_HANDLE	0x1023
#define WNI_PW6_DATA_HANDLE_VALUE	0x1024
#define WNI_PW6_COMMAND_HANDLE	0x1025
#define WNI_PW6_COMMAND_HANDLE_VALUE	0x1026

// base uuid = 020ca2xx-bc9c-11e4-bb52-0800200c9a66
#define WNI_PW6_UUID_DEFINE(x)	0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08, 0x52, 0xbb, 0xe4, 0x11, 0x9c, 0xbc, (UINT8)(x & 0xff), 0xa2, 0x0c, 0x02
#define WNI_PW6_SERVICE_UUID	WNI_PW6_UUID_DEFINE(0x60)
#define WNI_PW6_COUNT_UUID	WNI_PW6_UUID_DEFINE(0x61)
#define WNI_PW6_DATA_UUID	WNI_PW6_UUID_DEFINE(0x62)
#define WNI_PW6_COMMAND_UUID	WNI_PW6_UUID_DEFINE(0x63)

#define WNI_PW6_COMMAND_REQUEST	0x00
#define WNI_PW6_COMMAND_ERROR	0x01
#define	WNI_PW6_COMMAND_RESET	0x02
