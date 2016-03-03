/* 
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __CAPPXS_H__
#define __CAPPXS_H__

#include <linux/ioctl.h>

struct capPXS_callback{
    int (*capPXS_operate)(void* self, uint32_t command, void* buff_in, int size_in,
        void* buff_out, int size_out, int* actualout);
    long (*capPXS_ioctl) (struct file *, unsigned int, unsigned long);
    ssize_t (*capPXS_attr_show)(struct device_driver *driver, unsigned int cmd, char *buf);
    ssize_t (*capPXS_attr_store)(struct device_driver *driver, unsigned int cmd, const char *buf, size_t count);
    long (*capPXS_eint_registration) (void (EINT_FUNC_PTR) (void));
};

void capPXS_CB_registration(struct capPXS_callback *CB);

#endif

