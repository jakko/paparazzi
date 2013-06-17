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

/** \file modules/multi/dropball.c
 *  Dropball implementation for IMAV2013 (mavlab TU Delft DSE)
 *
 */

#include "multi/dropball.h"

#include "state.h"
#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "subsystems/ins.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/datalink/datalink.h"
#include "firmwares/rotorcraft/navigation.h"
#include "math/pprz_geodetic_int.h"

#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE

/* Make a vector containing only the dropball waypoints and their status */
static struct dropball_waypoint dropball_waypoints[] = {
  {WP_DZ1, TRUE, FALSE},
  {WP_DZ2, FALSE, FALSE},
  {WP_DZ3, TRUE, FALSE},
  {WP_DZ4, TRUE, FALSE},
};

/* find factorial of a number */
uint32_t factorial(uint32_t n) {
  if (n<=1)
    return(1);
  else
    n=n*factorial(n-1);
  return(n);
}

/* find the next lexicographical permutation */
bool_t next_lex_perm(uint8_t *a, int n) {
#define swap(i, j) {t = a[i]; a[i] = a[j]; a[j] = t;}
  uint8_t k, l, t;

  /* 1. Find the largest index k such that a[k] < a[k + 1]. If no such
	  index exists, the permutation is the last permutation. */
  for (k = n - 1; k && a[k - 1] >= a[k]; k--);
  if (!k--) return FALSE;

  /* 2. Find the largest index l such that a[k] < a[l]. Since k + 1 is
   such an index, l is well defined */
  for (l = n - 1; a[l] <= a[k]; l--);

  /* 3. Swap a[k] with a[l] */
  swap(k, l);

  /* 4. Reverse the sequence from a[k + 1] to the end */
  for (k++, l = n - 1; l > k; l--, k++)
  	swap(k, l);
  return TRUE;
#undef swap
}

/* Send message */
bool_t dropball_WpFound(void){
  uint8_t move_dropball_ac_id = 10;
  uint8_t move_dropball_wp_id = WP_DZ1;
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
  /*uint8_t ac_id = DL_DROPBALL_FOUND_ac_id(dl_buffer);
  if (ac_id != AC_ID)
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
  uint8_t block_id = 13;
  uint8_t ac_id = 10;

  DOWNLINK_SEND_GOTOBLOCK(DefaultChannel, DefaultDevice, &block_id, &ac_id);
  return FALSE;
}

/* Find shortest route */
bool_t dropball_WpNew(uint8_t wp1, uint8_t wp2, uint8_t wp3, uint8_t wp4){
  /* just copy the waypoints to a vector for easy use */
  uint8_t dropball_new_waypoints[4] = { wp1, wp2, wp3, wp4 };

  /* determine the length of the waypoints and set the initial route */
  uint8_t i, j; // Used for several loops
  uint8_t wp_skip_count = 0; // Amount of waypoints that aren't found or already dropped
  uint8_t length = 0; // Amount of waypoints - wp_skip_count
  uint8_t swap_temp[4]; // Without current position and end position

  /* Find all dropball points where I could go to */
  printf("Initial waypoints: ");
  for (i = 0; i < NB_DROPBALL_WP; i++) {
    // Check if we know where a dropball point is
    if ( dropball_waypoints[i].detected == FALSE ||  dropball_waypoints[i].dropped == TRUE){
      wp_skip_count++;
    }
    else {
      // Set the initial swap temp
      swap_temp[i-wp_skip_count] = dropball_waypoints[i].wp;
      length++;

      printf("%i " , swap_temp[i-wp_skip_count]);
    }
  }
  /* determine length of the vector */
  printf("(length: %i, skipped: %i)\r\n", length, wp_skip_count);

  /* determine the distances between the route elements */
  uint32_t wp_distances[NB_WAYPOINT][NB_WAYPOINT];
  for (i = 0; i < length-1; i++) {
    for (j = i+1; j < length; j++) {
      // Calculate the distance
      struct Int32Vect3 pos_diff;
      VECT3_DIFF(pos_diff, waypoints[swap_temp[i]], waypoints[swap_temp[j]]);
      VECT3_ABS(pos_diff, pos_diff);
      wp_distances[swap_temp[i]][swap_temp[j]] = sqrt(pow(pos_diff.x, 2) + pow(pos_diff.y,2) + pow(pos_diff.z,2));
      wp_distances[swap_temp[j]][swap_temp[i]] = wp_distances[swap_temp[i]][swap_temp[j]];
     
      printf("Calculated distance %i <-> %i: %i\r\n", swap_temp[i], swap_temp[j], wp_distances[swap_temp[i]][swap_temp[j]]);
    }
  }
  
  /* determine the distances between the current position and the end position */
  uint32_t first_distances[NB_WAYPOINT]; // Distances from current position to waypoints from route
  uint32_t last_distances[NB_WAYPOINT]; // Distances from waypoints from route to end position
  printf(" debug ");
  for (i = 0; i < length; i++) {
	struct Int32Vect3 pos_diff;
        printf(" debug ");
	// Distance from current position
	VECT3_DIFF(pos_diff, *stateGetPositionEnu_i(), waypoints[swap_temp[i]]);
	VECT3_ABS(pos_diff, pos_diff);
	first_distances[swap_temp[i]] = sqrt(pow(pos_diff.x, 2) + pow(pos_diff.y,2) + pow(pos_diff.z,2));
	printf("Calculated distance START -> %i: %i\r\n", swap_temp[i], first_distances[swap_temp[i]]);

	// Distance to end position
	VECT3_DIFF(pos_diff, waypoints[swap_temp[i]], waypoints[WP_Start]);
	VECT3_ABS(pos_diff, pos_diff);
	last_distances[swap_temp[i]] = sqrt(pow(pos_diff.x, 2) + pow(pos_diff.y,2) + pow(pos_diff.z,2));
	printf("Calculated distance %i -> END(WP_Start): %i\r\n", swap_temp[i], last_distances[swap_temp[i]]);
  }

  /* initialize matrix with possible routes */
  uint16_t wp_routs_fact = factorial(length); // Factorial of the length
  uint32_t wp_route_dist[wp_routs_fact]; // The distances of the routes
  uint8_t wp_routes[wp_routs_fact][length]; // All the possible routes TODO: Only save min_route and not min_route_idx
  uint32_t min_distance = 4294967295; // The minimum distance found (set to maximum uint32 at start)
  uint8_t min_route_idx = 0; // The index of the minimum distance route

  /* find all possible routes */
  for (i = 0; i < wp_routs_fact; i++) {
    /* Save the route and calculate the distance of the route */
    printf("Route %i: ", i);

    // Add distance from current position to first of route
    wp_route_dist[i] = first_distances[swap_temp[0]];

    // Add the route and update the distance from the route
    for (j = 0; j < length; j++) {
      wp_routes[i][j] = swap_temp[j];
      // Because distances are calculated between two points
      if (j != length-1)
        wp_route_dist[i] += wp_distances[swap_temp[j]][swap_temp[j+1]];
      printf("%i ", swap_temp[j]);
    }

    // Add distance from the last of the route to the end point
    wp_route_dist[i] += last_distances[swap_temp[length-1]];
    printf("(distance: %i)\r\n", wp_route_dist[i]);

    // When the distance is less then the minimum route
    if (wp_route_dist[i] < min_distance) {
      min_route_idx = i;
      min_distance = wp_route_dist[i];
    }

    /* Calculate the next route */
    next_lex_perm(swap_temp, length);
  }
  
  /* move waypoints NEXT1, NEXT2, NEXT3 and NEXT4 to the new locations */
  printf("Shortest route: ");
  for (i=0; i < length; i++) {
    nav_move_waypoint(dropball_new_waypoints[i], &waypoints[wp_routes[min_route_idx][i]]);
    printf("%d ", wp_routes[min_route_idx][i]);
  }
  printf("(distance: %i)\r\n", min_distance);
  
  return FALSE;
}


