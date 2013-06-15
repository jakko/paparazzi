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

/** \file tcas.c
 *  \brief
 *
 */

#include <stdio.h>
#include <math.h> 
#include <stdlib.h>

#include "multi/dropball.h"

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

uint8_t move_dropball_wp_id;
uint8_t move_dropball_ac_id;

uint8_t block_id;
uint8_t ac_id;

/* Make a vector containing only the dropball waypoints and their status */
static struct dropball_waypoint dropball_waypoints[] = {
  {.wp = WP_DZ1, .detected = TRUE, .dropped = FALSE},
  {.wp = WP_DZ2, .detected = FALSE, .dropped = FALSE},
  {.wp = WP_DZ3, .detected = TRUE, .dropped = FALSE},
  {.wp = WP_DZ4, .detected = TRUE, .dropped = FALSE},
};

/* swap function */
void swap(uint8_t a, uint8_t b) { 
  uint8_t temp = a;
  a = b;
  b = temp;
}

/* find factorial of a number */
uint32_t factorial(uint32_t n) {
    if (n<=1)
        return(1);
    else
        n=n*factorial(n-1);
    return(n);
}

/* Send message */
bool_t dropball_WpFound(void){
  move_dropball_ac_id = 10;
  move_dropball_wp_id = WP_DZ1;
  struct LlaCoor_i new_cord;
  new_cord.lat = 434505560;
  new_cord.lon = 12640240;
  new_cord.alt = 500;

  DOWNLINK_SEND_FOUND_DROPBALL(DefaultChannel, DefaultDevice, &move_dropball_wp_id, &move_dropball_ac_id, &new_cord.lat, &new_cord.lon, &new_cord.alt);

  return FALSE;
}

/* Receive message */

/* parse for the real aircraft */
void on_dropball(void) {
  uint8_t ac_id = DL_DROPBALL_FOUND_ac_id(dl_buffer);
  /*if (ac_id != AC_ID) 
      return;*/
  uint8_t wp_id = DL_DROPBALL_FOUND_wp_id(dl_buffer);
  struct LlaCoor_i lla;
  struct EnuCoor_i enu;
  lla.lat = INT32_RAD_OF_DEG(DL_DROPBALL_FOUND_lat(dl_buffer));
  lla.lon = INT32_RAD_OF_DEG(DL_DROPBALL_FOUND_lon(dl_buffer));
  /* WP_alt is in cm, lla.alt in mm */
  lla.alt = DL_DROPBALL_FOUND_alt(dl_buffer)*10 - ins_ltp_def.hmsl + ins_ltp_def.lla.alt;
  enu_of_lla_point_i(&enu,&ins_ltp_def,&lla);
  enu.x = POS_BFP_OF_REAL(enu.x)/100;
  enu.y = POS_BFP_OF_REAL(enu.y)/100;
  enu.z = POS_BFP_OF_REAL(enu.z)/100;
  VECT3_ASSIGN(waypoints[wp_id], enu.x, enu.y, enu.z);
  DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id, &enu.x, &enu.y, &enu.z);
}

/* parse for the simulation */
void parse_on_dropball_found(uint8_t wp_id) {
  /* fill struct */
  uint8_t i = 0;
  for (i = 0; i < NB_DROPBALL_WP; i++) {
    if (dropball_waypoints[i].wp == wp_id)
      dropball_waypoints[i].detected = TRUE;
  }
}


/* Send an urgent go to block */
bool_t dropball_goto_block(void){
  block_id = 13;
  ac_id = 10;

  DOWNLINK_SEND_GOTOBLOCK(DefaultChannel, DefaultDevice, &block_id, &ac_id);

  return FALSE;
}

/* Find shortest route */
bool_t dropball_WpNew(uint8_t wp1, uint8_t wp2, uint8_t wp3, uint8_t wp4){

  struct dropball_new_waypoint {
  uint8_t wp;
  };
  struct dropball_new_waypoint dropball_new_waypoints[] = {
  {.wp = wp1},
  {.wp = wp2},
  {.wp = wp3},
  {.wp = wp4},
  };

  /* determine the length of the waypoints and set the initial route */
  uint8_t i, j;
  uint8_t wp_routes[24][6]; // With FIRST and LAST
  uint8_t wp_skip_count = 0; // Amount of waypoints that aren't found or already dropped
  uint8_t length = 0; // Amount of waypoints - wp_skip_count
  uint8_t swap_temp[4]; // Without FIRST and LAST

  wp_routes[0][0] = 0;
  printf(" wp_routes[0] = %i ", wp_routes[0][0]);
  for (i = 0; i < NB_DROPBALL_WP; i++) {
    // Check if we know where a dropball point is
    if ( dropball_waypoints[i].detected == FALSE ||  dropball_waypoints[i].dropped == TRUE){
      wp_skip_count++;
    }
    else {
      // Set the initial first route and swap temp
      wp_routes[0][i-wp_skip_count+1] = dropball_waypoints[i].wp;
      swap_temp[i-wp_skip_count] = dropball_waypoints[i].wp;
      length++;

      printf("%i " , wp_routes[0][i-wp_skip_count+1]);
    }
  }
  wp_routes[0][length+1] = WP_Start;
  /* determine length of the vector */
  printf(" %i\r\n length: %i skipped: %i\r\n", wp_routes[0][length+1], length, wp_skip_count);

  /* determine the distances between the route elements */
  uint32_t wp_distances[NB_WAYPOINT][NB_WAYPOINT];
  for(i = 0; i < length+1; i++) {
    for(j = i+1; j < length+2; j++) {
      struct Int32Vect3 pos_diff;
      
      // Check if it is the first waypoint
      if(wp_routes[0][i] == 0) {
        VECT3_DIFF(pos_diff, *stateGetPositionEnu_i(), waypoints[wp_routes[0][j]]);
      } else if(wp_routes[0][j] == 0) {
        VECT3_DIFF(pos_diff, waypoints[wp_routes[0][i]], *stateGetPositionEnu_i());
      } else {
        VECT3_DIFF(pos_diff, waypoints[wp_routes[0][i]], waypoints[wp_routes[0][j]]);
      }

      // Calculate the distance
      VECT3_ABS(pos_diff, pos_diff);  
      wp_distances[wp_routes[0][i]][wp_routes[0][j]] = sqrt((pos_diff.x)^2 + (pos_diff.y)^2 + (pos_diff.z)^2);
      wp_distances[wp_routes[0][j]][wp_routes[0][i]] = wp_distances[i][j];
     
      printf("Calculated distance %i <-> %i: %i\r\n", wp_routes[0][i], wp_routes[0][j], wp_distances[wp_routes[0][i]][wp_routes[0][j]]);
    }
  }
  
  /* initialize matrix with possible routes */
  uint8_t N;
  uint8_t z;
  uint32_t wp_route_dist[24];
  uint32_t min_distance = 4294967295;
  uint8_t min_route_idx;

  /* find all possible routes */
  for(z=1; z < factorial(length); z++) {
     for (i=0; i < length; i++) {
          N = length; 
          i = length - 1;  
      
          while(swap_temp[i-1] >= swap_temp[i]) {
            i = i - 1;
          }
          j = N;  
      
          while (swap_temp[j-1] <= swap_temp[i-1]) {
            j = j - 1;
          }
          swap(swap_temp[i-1], swap_temp[j-1]);
      
          i++;  
          j = N;
      
          while (i < j){
             swap(swap_temp[i-1], swap_temp[j-1]); 
             i++; 
             j--;  
          }
      }

      // Set the route and calculate the total distance
      wp_routes[z][0] = 0;
      printf(" Route %i: ", z);
      for (i=0; i < length; i++) {
        wp_routes[z][i+1] = swap_temp[i];
        wp_route_dist[z] += wp_distances[wp_routes[z][i]][wp_routes[z][i+1]]; 
        printf("%i ", swap_temp[i]);
      }
      wp_routes[z][length+1] = WP_Start;
      wp_route_dist[z] += wp_distances[wp_routes[z][length]][wp_routes[z][length+1]];
      printf("%i (distance: %i)\r\n", wp_routes[z][length+1], wp_route_dist[z]);

      // When the distance is less then the minimum route
      if(wp_routes[z][6] < min_distance) {
        min_route_idx = z;
        min_distance = wp_route_dist[z];
      }
  }
  
  /* move waypoints NEXT1, NEXT2, NEXT3 and NEXT4 to the new locations */
  printf("shorest route:");
  for (i=1; i < length; i++) {
    nav_move_waypoint(dropball_new_waypoints[i-1].wp, &waypoints[wp_routes[min_route_idx][i]]);
    printf(" %d ", wp_routes[min_route_idx][i]);
  }
  printf("\n shortest path: 	%i\n", min_distance);
  
  return FALSE;
}


