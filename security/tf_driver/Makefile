#
# Copyright (c) 2006-2010 Trusted Logic S.A.
# All Rights Reserved.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston,
# MA 02111-1307 USA
#

# debug options
#EXTRA_CFLAGS += -O0 -DDEBUG -D_DEBUG -DCONFIG_TF_DRIVER_DEBUG_SUPPORT
EXTRA_CFLAGS += -DNDEBUG

EXTRA_CFLAGS += -DLINUX -DCONFIG_TF_TRUSTZONE -DCONFIG_TFN

ifdef S_VERSION_BUILD
EXTRA_CFLAGS += -DS_VERSION_BUILD=$(S_VERSION_BUILD)
endif

tf_driver-objs += scxlnx_util.o
tf_driver-objs += scxlnx_conn.o
tf_driver-objs += scxlnx_device.o
tf_driver-objs += scxlnx_comm.o
tf_driver-objs += scxlnx_comm_tz.o

obj-$(CONFIG_TRUSTED_FOUNDATIONS) += tf_driver.o
