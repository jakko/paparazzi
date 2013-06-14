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

  /* determine waypoints for the route */
  uint8_t i;
  uint8_t vector[4];
  uint8_t idx = 0;
  uint8_t counter = 0;
  for (i = 0; i < NB_DROPBALL_WP; i++) {
    uint8_t idx_b = 0;
    if ( dropball_waypoints[i].detected == FALSE ||  dropball_waypoints[i].dropped == TRUE){
      idx++;
      idx_b = 1;
    }
    if(idx_b == 0){
      vector[i-idx] = dropball_waypoints[i].wp;
      printf(" vector: %i\n" , vector[i-idx]);
      counter++;
    }
  }
  /* determine length of the vector */
  uint8_t length = counter;
  printf(" length: %i\n" , length);
  
  /* initialize matrix with possible routes */
  uint8_t route[factorial(length)][length];
  uint8_t temp[length];
  uint8_t N;
  uint8_t z;

  for (i=0; i < length; i++) {
          route[0][i] = vector[i];
          temp[i] = vector[i];
          printf(" temp: %i\n" , temp[i]);
     }

  /* find all possible routes */
  for(z=1; z < factorial(length); z++) {
     for (i=0; i < length; i++) {
          N = length; 
          i = length - 1;  
      
          while( temp[i-1] >= temp[i] ){ 
            i = i - 1; 
          }
          uint8_t j = N;  
      
          while (temp[j-1] <= temp[i-1]){
            j = j - 1;  
          }
          swap(temp[i-1], temp[j-1]);
      
          i++;  
          j = N;
      
          while (i < j){
             swap(temp[i-1], temp[j-1]); 
             i++; 
             j--;  
          }
      }
      for (i=0; i < length; i++) {
          route[z][i] = temp[i];
      }
  }

  /* distance bewteen points and minimum distance */
  uint32_t calc_distance, min_distance = 4294967295;
  /* minimum distance idx */
  uint8_t min_distance_idx;
  /* initialize parameters for travelling salesman problem */
  uint8_t k;
  uint8_t m;
  uint8_t a0;
  uint8_t a;
  uint8_t b;
  uint8_t b0;
  uint32_t sum;
  /* x, y and z coordinates difference between two waypoints */
  struct Int32Vect3 pos_diff[3];
  struct Int32Vect3 pos_diff_i[1];
  struct Int32Vect3 pos_diff_f[1];
  /* distance bewteen current location and begin point */
  uint32_t calc_distance_i;
  /* distance bewteen last point and end point */
  uint32_t calc_distance_f;
  /* total route lenght */
  uint32_t total[factorial(length)];  

  /* calculate the length of all routes */
  for(k = 0; k < factorial(length) - 1; k++){
    sum = 0;
    for(m = 0; m < length - 1; m++){
      a0 = route[k][0];
      a = route[k][m];
      b = route[k][m+1];
      b0 = route[k][length-1];
      VECT3_DIFF(pos_diff[m], waypoints[a] , waypoints[b]);
      VECT3_ABS(pos_diff[m], pos_diff[m]);
      calc_distance = sqrt((pos_diff[m].x)^2 + (pos_diff[m].y)^2 + (pos_diff[m].z)^2);
      VECT3_DIFF(pos_diff_i[0], *stateGetPositionEnu_i() , waypoints[a0]);
      VECT3_ABS(pos_diff_i[0], pos_diff_i[0]);
      calc_distance_i = sqrt((pos_diff_i[0].x)^2 + (pos_diff_i[0].y)^2 + (pos_diff_i[0].z)^2);
      VECT3_DIFF(pos_diff_f[0], waypoints[b0] , waypoints[13]);
      VECT3_ABS(pos_diff_f[0], pos_diff_f[0]);
      calc_distance_f = sqrt((pos_diff_f[0].x)^2 + (pos_diff_f[0].y)^2 + (pos_diff_f[0].z)^2);
      sum = sum + calc_distance;
    }
   total[k] = sum + calc_distance_i + calc_distance_f;
   /* find the shortest path */
   if( total[k] < min_distance) {
      min_distance = total[k];
      min_distance_idx = k;
    }
  }
  
  /* move waypoints NEXT1, NEXT2, NEXT3 and NEXT4 to the new locations */
  printf("shorest route:");
  for (i=0; i < length; i++) {
    nav_move_waypoint(dropball_new_waypoints[i].wp, &waypoints[route[min_distance_idx][i]]);
    printf(" %d ", route[min_distance_idx][i]);
  }
  printf("\n route length and shortest path: %d %i\n", total[min_distance_idx], min_distance);
  
  return FALSE;
}


