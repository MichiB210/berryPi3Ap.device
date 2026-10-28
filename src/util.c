/*
 * berrypi3ap.device - minimal libc pieces (the device links without libc)
 * Copyright (C) 2026 MichiB210
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include <stddef.h>

void *memcpy(void *d, const void *s, size_t n)
{
    unsigned char *dp = d;
    const unsigned char *sp = s;
    while (n--)
        *dp++ = *sp++;
    return d;
}

void *memset(void *d, int c, size_t n)
{
    unsigned char *dp = d;
    while (n--)
        *dp++ = (unsigned char)c;
    return d;
}

