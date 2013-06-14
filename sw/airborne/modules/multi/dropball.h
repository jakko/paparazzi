/*
 * Copyright (C) 2010 ENAC
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** \file dropball.h
 *  \brief ////
 *
 */

#ifndef DROPBALL_H
#define DROPBALL_H

#include "std.h"

#define NB_DROPBALL_WP		4
struct dropball_waypoint {
  uint8_t wp;
  bool_t detected;
  bool_t dropped;
};

bool_t dropball_WpFound(void);
bool_t dropball_goto_block(void);
bool_t dropball_WpNew(uint8_t wp1, uint8_t wp2, uint8_t wp3, uint8_t wp4);

void on_dropball(void);
void parse_on_dropball_found(uint8_t wp_id,  uint8_t ac_id, uint32_t enu_x, uint32_t enu_y, uint32_t enu_z);

#endif // DROPBALL
