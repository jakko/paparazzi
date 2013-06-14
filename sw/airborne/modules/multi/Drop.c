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

/** \file Drop.c
 *  \brief
 *
 */

#include <stdio.h>

#include "modules/multi/Drop.h"

#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "generated/modules.h"

#include "state.h"
#include "messages.h"

#include "math/pprz_geodetic_int.h"

#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE

#include "subsystems/datalink/downlink.h"
#include "subsystems/datalink/datalink.h"
#include "subsystems/ins.h"
#include "mcu_periph/uart.h"
#include "firmwares/rotorcraft/navigation.h"

void DropBall_init(){}
uint8_t DropBall_doedrop(){
	printf("Ball Dropped\n");
	return FALSE;
}
