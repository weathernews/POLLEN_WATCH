#
# Copyright 2014, Broadcom Corporation
# All Rights Reserved.
#
# This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
# the contents of this file may not be disclosed to third parties, copied
# or duplicated in any form, in whole or in part, without the prior
# written permission of Broadcom Corporation.
#

########################################################################
# Add Application sources here.
########################################################################
APP_SRC = wni_pw6_1.c ws_upgrade_ota.c ws_upgrade.c

########################################################################
# Add requires libraries here.
# This application requires a special patch to change stack size and 
# memory usage to support RSA functionality
########################################################################
APP_PATCHES_AND_LIBS += thread_and_mem_mgmt.a bd_addr_control.a rtc_api.a

########################################################################
################ DO NOT MODIFY FILE BELOW THIS LINE ####################
########################################################################
