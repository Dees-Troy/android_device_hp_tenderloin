/*
 * This is a userspace touchscreen driver for cypress ctma395 as used
 * in HP Touchpad configured for WebOS.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 * The code was written from scratch, the hard math and understanding the
 * device output by jonpry @ gmail
 * uinput bits and the rest by Oleg Drokin green@linuxhacker.ru
 * Multitouch detection by Rafael Brune mail@rbrune.de
 *
 * Copyright (c) 2011 CyanogenMod Touchpad Project.
 *
 *
 */

#include <linux/input.h>
#include <linux/uinput.h>
#include <linux/hsuart.h>
#include <sched.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/select.h>

#if 1
// This is for Android
#define UINPUT_LOCATION "/dev/uinput"
#else
// This is for webos and possibly other Linuxes
#define UINPUT_LOCATION "/dev/input/uinput"
#endif

/* Set to 1 to print coordinates to stdout. */
#define DEBUG 0

/* Set to 1 to see raw data from the driver */
#define RAW_DATA_DEBUG 0
// Removes values below threshold for easy reading, set to 0 to see everything.
// A value of 2 should remove most unwanted output
#define RAW_DATA_THRESHOLD 0

// set to 1 to see event logging
#define EVENT_DEBUG 0
// set to 1 to enable tracking ID logging
#define TRACK_ID_DEBUG 0

#define AVG_FILTER 1

#define USERSPACE_270_ROTATE 0

#define RECV_BUF_SIZE 1540
#define LIFTOFF_TIMEOUT 25000

#define MAX_TOUCH 5 // max touches that will be reported

// this value determines when a large distance change between one touch
// and another will be reported as 2 separate touches instead of a swipe
// this distance is in pixels
#define MAX_DELTA 130
#define MAX_PREV_DELTA 40
#define MAX_DELTA_ANGLE 0.25
#define MAX_DELTA_DEBUG 0 // set to 1 to see debug logging for max delta

#define TOUCH_THRESHOLD 28 // Threshold for what is considered a valid touch
#define LARGE_AREA_UNPRESS TOUCH_THRESHOLD - 4 // Threshold for end of a large area
#define LARGE_AREA_FRINGE 15 // Threshold for large area fringe
// used to decide if an adjacent point is part of a separate touch
#define PINCH_THRESHOLD 16

// Enables filtering of a single touch to make it easier to long press.
// Keeps the initial touch point the same so long as it stays within
// the radius (note it's not really a radius and is actually a square)
#define DEBOUNCE_FILTER 1
#define DEBOUNCE_RADIUS 10 // Radius for debounce in pixels
#define DEBOUNCE_DEBUG 0 // set to 1 to enable debounce logging

// this is roughly the value of 1024 / 40 or 768 / 30
#define PIXELS_PER_POINT 25

/** ------- end of user modifiable parameters ---- */
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))
#define isBetween(A, B, C) ( ((A-B) > 0) && ((A-C) < 0) )
// we square MAX_DELTA to prevent the need to use sqrt
#define MAX_DELTA_SQ (MAX_DELTA * MAX_DELTA)
#define MAX_PREV_DELTA_SQ (MAX_PREV_DELTA * MAX_PREV_DELTA)

#define X_AXIS_POINTS  30
#define Y_AXIS_POINTS  40
#define X_RESOLUTION  768
#define Y_RESOLUTION 1024

#define X_AXIS_MINUS1 X_AXIS_POINTS - 1 // 29
#define Y_AXIS_MINUS1 Y_AXIS_POINTS - 1 // 39

struct candidate {
	int pw;
	float touch_major;
	int i;
	int j;
};

struct touchpoint {
	int pw;
	float i;
	float j;
	int slot;
	int tracking_id;
	int prev_loc;
	float direction;
	int distance;
	int touch_major;
	int x;
	int y;
	int raw_x;
	int raw_y;
};

struct touchpoint tpoint[MAX_TOUCH];
struct touchpoint prevtpoint[MAX_TOUCH];
struct touchpoint prev2tpoint[MAX_TOUCH];

unsigned char cline[64];
unsigned int cidx = 0;
unsigned char matrix[X_AXIS_POINTS][Y_AXIS_POINTS];
int invalid_matrix[X_AXIS_POINTS][Y_AXIS_POINTS];
int uinput_fd;
int slot_in_use[MAX_TOUCH];

int send_uevent(int fd, __u16 type, __u16 code, __s32 value)
{
	struct input_event event;

#if EVENT_DEBUG
	char ctype[20], ccode[20];
	switch (type) {
		case EV_ABS:
			strcpy(ctype, "EV_ABS");
			break;
		case EV_KEY:
			strcpy(ctype, "EV_KEY");
			break;
		case EV_SYN:
			strcpy(ctype, "EV_SYN");
			break;
	}
	switch (code) {
		case ABS_MT_SLOT:
			strcpy(ccode, "ABS_MT_SLOT");
			break;
		case ABS_MT_TRACKING_ID:
			strcpy(ccode, "ABS_MT_TRACKING_ID");
			break;
		case ABS_MT_TOUCH_MAJOR:
			strcpy(ccode, "ABS_MT_TOUCH_MAJOR");
			break;
		case ABS_MT_POSITION_X:
			strcpy(ccode, "ABS_MT_POSITION_X");
			break;
		case ABS_MT_POSITION_Y:
			strcpy(ccode, "ABS_MT_POSITION_Y");
			break;
		case SYN_MT_REPORT:
			strcpy(ccode, "SYN_MT_REPORT");
			break;
		case SYN_REPORT:
			strcpy(ccode, "SYN_REPORT");
			break;
		case BTN_TOUCH:
			strcpy(ccode, "BTN_TOUCH");
			break;
	}
	printf("event type: '%s' code: '%s' value: %i \n", ctype, ccode, value);
#endif

	memset(&event, 0, sizeof(event));
	event.type = type;
	event.code = code;
	event.value = value;

	if (write(fd, &event, sizeof(event)) != sizeof(event)) {
		fprintf(stderr, "Error on send_event %d", sizeof(event));
		return -1;
	}

	return 0;
}

#if AVG_FILTER
void avg_filter(struct touchpoint *t) {
#if DEBUG
	printf("before: x=%d, y=%d", t->x, t->y);
#endif 
	float total_div = 6.0;
	int xsum = 4*t->raw_x + 2*prevtpoint[t->prev_loc].raw_x;
	int ysum = 4*t->raw_y + 2*prevtpoint[t->prev_loc].raw_y;
	if(prevtpoint[t->prev_loc].prev_loc > -1) {
		xsum += prev2tpoint[prevtpoint[t->prev_loc].prev_loc].raw_x;
		ysum += prev2tpoint[prevtpoint[t->prev_loc].prev_loc].raw_y;
		total_div += 1.0;
	}
	t->x = xsum / total_div;
	t->y = ysum / total_div;
#if DEBUG
	printf("|||| after: x=%d, y=%d\n", t->x, t->y);
#endif
}
#endif // AVG_FILTER

void liftoff_slot(int slot) {
	// sends a liftoff indicator for a specific slot
#if EVENT_DEBUG
	printf("liftoff slot function, lifting off slot: %i\n", slot);
#endif
	send_uevent(uinput_fd, EV_ABS, ABS_MT_SLOT, slot);
	send_uevent(uinput_fd, EV_ABS, ABS_MT_TRACKING_ID, -1);
}

void liftoff(void)
{
	// send liftoffs for any slots that haven't been lifted off
	int i;
	for (i=0; i<MAX_TOUCH; i++) {
		if (slot_in_use[i]) {
			slot_in_use[i] = 0;
			liftoff_slot(i);
		}
	}
	// sends liftoff events - nothing is touching the screen
#if EVENT_DEBUG
	printf("liftoff function\n");
#endif
	send_uevent(uinput_fd, EV_SYN, SYN_MT_REPORT, 0);
	send_uevent(uinput_fd, EV_SYN, SYN_REPORT, 0);
	send_uevent(uinput_fd, EV_KEY, BTN_TOUCH, 0);
}

void determine_area_loc_fringe(float *isum, float *jsum, int *tweight, int i, int j, int cur_touch_id){
	// Set fringe point to used for this touch point
	invalid_matrix[i][j] = cur_touch_id;

	float powered = pow(matrix[i][j], 1.5);
	*tweight += powered;
	*isum += powered * i;
	*jsum += powered * j;

	if (i > 0  && invalid_matrix[i-1][j] != -1 && invalid_matrix[i-1][j] != cur_touch_id) {
		if(matrix[i-1][j] >= LARGE_AREA_FRINGE && matrix[i-1][j] < matrix[i][j]) 
			determine_area_loc_fringe(isum, jsum, tweight, i - 1, j, cur_touch_id);
	}
	if(i < 29 && invalid_matrix[i+1][j] != -1 && invalid_matrix[i+1][j] != cur_touch_id) {
		if (matrix[i+1][j] >= LARGE_AREA_FRINGE && matrix[i+1][j] < matrix[i][j]) 
			determine_area_loc_fringe(isum, jsum, tweight, i + 1, j, cur_touch_id);
	}
	if(j > 0 && invalid_matrix[i][j-1] != -1 && invalid_matrix[i][j-1] != cur_touch_id) {
		if (matrix[i][j-1] >= LARGE_AREA_FRINGE && matrix[i][j-1] < matrix[i][j]) 
			determine_area_loc_fringe(isum, jsum, tweight, i, j-1, cur_touch_id);
	}
	if(j < 39 && invalid_matrix[i][j+1] != -1 && invalid_matrix[i][j+1] != cur_touch_id) {
		if (matrix[i][j+1] >= LARGE_AREA_FRINGE && matrix[i][j+1] < matrix[i][j])
			determine_area_loc_fringe(isum, jsum, tweight, i, j+1, cur_touch_id);
	}
}

void determine_area_loc(float *isum, float *jsum, int *tweight, int i, int j, int *mini, int *maxi, int *minj, int *maxj, int cur_touch_id){
	// Invalidate this touch point so that we don't process it later
	invalid_matrix[i][j] = -1;
	if (i < *mini)
		*mini = i;
	if (i > *maxi)
		*maxi = i;
	if (j < *minj)
		*minj = j;
	if (j > *maxj)
		*maxj = j;

	float powered = pow(matrix[i][j], 1.5);
	*tweight += powered;
	*isum += powered * i;
	*jsum += powered * j;

	// Check nearby points to see if they are above LARGE_AREA_UNPRESS
	if (i > 0  && invalid_matrix[i-1][j] != -1 && invalid_matrix[i-1][j] != cur_touch_id && matrix[i-1][j]) {
		// In touch area or in fringe and decreasing
		if (matrix[i-1][j] >= LARGE_AREA_UNPRESS && matrix[i-1][j] < matrix[i][j] + PINCH_THRESHOLD)
			determine_area_loc(isum, jsum, tweight, i - 1, j, mini, maxi, minj, maxj, cur_touch_id);
		else if(matrix[i-1][j] >= LARGE_AREA_FRINGE && matrix[i-1][j] < matrix[i][j])
			determine_area_loc_fringe(isum, jsum, tweight, i - 1, j, cur_touch_id);
	}
	if(i < 29 && invalid_matrix[i+1][j] != -1 && invalid_matrix[i+1][j] != cur_touch_id && matrix[i+1][j]) {
	// In touch area or in fringe and decreasing
		if (matrix[i+1][j] >= LARGE_AREA_UNPRESS && matrix[i+1][j] < matrix[i][j] + PINCH_THRESHOLD)
			determine_area_loc(isum, jsum, tweight, i + 1, j, mini, maxi, minj, maxj, cur_touch_id);
		else if (matrix[i+1][j] >= LARGE_AREA_FRINGE && matrix[i+1][j] < matrix[i][j])
			determine_area_loc_fringe(isum, jsum, tweight, i + 1, j, cur_touch_id);
	}
	if(j > 0 && invalid_matrix[i][j-1] != -1 && invalid_matrix[i][j-1] != cur_touch_id && matrix[i][j-1]) {
		// In touch area or in fringe and decreasing
		if (matrix[i][j-1] >= LARGE_AREA_UNPRESS && matrix[i][j-1] < matrix[i][j] + PINCH_THRESHOLD)
			determine_area_loc(isum, jsum, tweight, i, j-1, mini, maxi, minj, maxj, cur_touch_id);
		else if (matrix[i][j-1] >= LARGE_AREA_FRINGE && matrix[i][j-1] < matrix[i][j])
			determine_area_loc_fringe(isum, jsum, tweight, i, j-1, cur_touch_id);
	}
	if(j < 39 && invalid_matrix[i][j+1] != -1 && invalid_matrix[i][j+1] != cur_touch_id && matrix[i][j+1]) {
		// In touch area or in fringe and decreasing
		if (matrix[i][j+1] >= LARGE_AREA_UNPRESS && matrix[i][j+1] < matrix[i][j] + PINCH_THRESHOLD)
			determine_area_loc(isum, jsum, tweight, i, j+1, mini, maxi, minj, maxj, cur_touch_id);
		else if (matrix[i][j+1] >= LARGE_AREA_FRINGE && matrix[i][j+1] < matrix[i][j])
			determine_area_loc_fringe(isum, jsum, tweight, i, j+1, cur_touch_id);
	}
}

int calc_point(void)
{
	int i, j, k;
	int tweight = 0;
	int tpc = 0;
	float isum = 0, jsum = 0;
	float avgi, avgj;
	static int previoustpc, tracking_id = 0;
#if DEBOUNCE_FILTER
	int new_debounce_touch = 0;
	static int initialx, initialy;
#endif

	if (tpoint[0].x < -20) {
		previoustpc = 0;
#if DEBOUNCE_FILTER
		new_debounce_touch = 1;
#endif
	}

	// Record values for processing later
	for(i=0; i < previoustpc; i++) {
		prev2tpoint[i].i = prevtpoint[i].i;
		prev2tpoint[i].j = prevtpoint[i].j;
		prev2tpoint[i].pw = prevtpoint[i].pw;
		prev2tpoint[i].slot = prevtpoint[i].slot;
		prev2tpoint[i].tracking_id = prevtpoint[i].tracking_id;
		prev2tpoint[i].prev_loc = prevtpoint[i].prev_loc;
		prev2tpoint[i].direction = prevtpoint[i].direction;
		prev2tpoint[i].distance = prevtpoint[i].distance;
		prev2tpoint[i].touch_major = prevtpoint[i].touch_major;
		prev2tpoint[i].x = prevtpoint[i].x;
		prev2tpoint[i].y = prevtpoint[i].y;
		prev2tpoint[i].raw_x = prevtpoint[i].raw_x;
		prev2tpoint[i].raw_y = prevtpoint[i].raw_y;

		prevtpoint[i].i = tpoint[i].i;
		prevtpoint[i].j = tpoint[i].j;
		prevtpoint[i].pw = tpoint[i].pw;
		prevtpoint[i].slot = tpoint[i].slot;
		prevtpoint[i].tracking_id = tpoint[i].tracking_id;
		prevtpoint[i].prev_loc = tpoint[i].prev_loc;
		prevtpoint[i].direction = tpoint[i].direction;
		prevtpoint[i].distance = tpoint[i].distance;
		prevtpoint[i].touch_major = tpoint[i].touch_major;
		prevtpoint[i].x = tpoint[i].x;
		prevtpoint[i].y = tpoint[i].y;
		prevtpoint[i].raw_x = tpoint[i].raw_x;
		prevtpoint[i].raw_y = tpoint[i].raw_y;
	}

	// generate list of high values
	memset(&invalid_matrix, 0, sizeof(invalid_matrix));
	for(i=0; i < X_AXIS_POINTS; i++) {
		for(j=0; j < Y_AXIS_POINTS; j++) {
#if RAW_DATA_DEBUG
			if (matrix[i][j] < RAW_DATA_THRESHOLD)
				printf("   ");
			else
				printf("%2.2X ", matrix[i][j]);
#endif
			if (tpc < MAX_TOUCH) {
				if(matrix[i][j] > TOUCH_THRESHOLD && !invalid_matrix[i][j]) {
					// This is a large area press (e.g. the side of your thumb)
					// so we will scan all the points nearby and if they are
					// above the LARGE_AREA_UNPRESS we mark them invalid and
					// track the highest i,j location so that we can
					// calculate a "center" for the large area press.

					// Use local max for touch area
					int do_continue = 0;
					if(i > 0  && matrix[i-1][j] > matrix[i][j])
						do_continue = 1;
					if(i < X_AXIS_MINUS1 && matrix[i+1][j] > matrix[i][j])
						do_continue = 1;
					if(j > 0  && matrix[i][j-1] > matrix[i][j])
						do_continue = 1;
					if(j < Y_AXIS_MINUS1 && matrix[i][j+1] > matrix[i][j])
						do_continue = 1;
					if(i > 0 && j > 0 && matrix[i-1][j-1] > matrix[i][j])
						do_continue = 1;
					if(i < X_AXIS_MINUS1 && j > 0 && matrix[i+1][j-1] > matrix[i][j])
						do_continue = 1;
					if(i > 0  && j < Y_AXIS_MINUS1 && matrix[i-1][j+1] > matrix[i][j])
						do_continue = 1;
					if(i < X_AXIS_MINUS1 && j < Y_AXIS_MINUS1 && matrix[i+1][j+1] > matrix[i][j])
						do_continue = 1;
					if(do_continue)
						continue;

					isum = 0;
					jsum = 0;
					tweight = 0;
					int mini = i, maxi = i, minj = j, maxj = j;
					determine_area_loc(&isum, &jsum, &tweight, i, j, &mini, &maxi, &minj, &maxj, tpc + 1);

					avgi = isum / (float)tweight;
					avgj = jsum / (float)tweight;
					maxi = maxi - mini;
					maxj = maxj - minj;

					tpoint[tpc].pw = tweight;
					tpoint[tpc].i = avgi;
					tpoint[tpc].j = avgj;
					tpoint[tpc].touch_major = MAX(maxi, maxj) * PIXELS_PER_POINT;
					tpoint[tpc].tracking_id = -1;
					tpoint[tpc].slot = -1;
					tpoint[tpc].prev_loc = -1;
#if USERSPACE_270_ROTATE
					tpoint[tpc].x = tpoint[tpc].i * 768 / 29;
					tpoint[tpc].y = 1024 - tpoint[tpc].j * 1024 / 39;
#else
					tpoint[tpc].x = 1024 - tpoint[tpc].j * 1024 / 39;
					tpoint[tpc].y = 768 - tpoint[tpc].i * 768 / 29;
#endif // USERSPACE_270_ROTATE
					tpoint[tpc].raw_x = tpoint[tpc].x;
					tpoint[tpc].raw_y = tpoint[tpc].y;
					tpc++;
				}
			}
		}
#if RAW_DATA_DEBUG
		printf(" |\n"); // end of row
#endif
	}
#if RAW_DATA_DEBUG
	printf("end of raw data\n"); // helps separate one frame from the next
#endif

	// match up tracking IDs
	memset(slot_in_use, 0, sizeof slot_in_use);
	{
		int smallest_distance[MAX_TOUCH], cur_distance;
		int deltax,deltay;
		int smallest_distance_loc[MAX_TOUCH];
		// Find closest points for each touch
		for (i=0; i<tpc; i++) {
			smallest_distance[i] = 1000000;
			smallest_distance_loc[i] = -1;
			for (j=0; j<previoustpc; j++) {
				deltax = tpoint[i].raw_x - prevtpoint[j].raw_x;
				deltay = tpoint[i].raw_y - prevtpoint[j].raw_y;
				cur_distance = (deltax * deltax) + (deltay * deltay);
				if(cur_distance < smallest_distance[i]) {
					smallest_distance[i] = cur_distance;
					smallest_distance_loc[i] = j;
				}
			}
		}

		// Remove mapping for touches which aren't closest
		for (i=0; i<tpc; i++) {
			for (j=i + 1; j<tpc; j++) {
				if (smallest_distance_loc[i] > -1 &&
				   smallest_distance_loc[i] == smallest_distance_loc[j]) {
					if (smallest_distance[i] < smallest_distance[j])
						smallest_distance_loc[j] = -1;
					else
						smallest_distance_loc[i] = -1;
				}
			}
		}

		// Assign ids to closest touches
		for (i=0; i<tpc; i++) {
			if (smallest_distance_loc[i] > -1) {
				// filter for impossibly large changes in touches
				if (smallest_distance[i] > MAX_DELTA_SQ) {
					int need_lift = 1;
					if (prevtpoint[smallest_distance_loc[i]].distance > MAX_PREV_DELTA_SQ) {
						tpoint[i].direction = atan2(tpoint[i].x - prevtpoint[smallest_distance_loc[i]].x, tpoint[i].y - prevtpoint[smallest_distance_loc[i]].y);
						if (fabsf(tpoint[i].direction - prevtpoint[smallest_distance_loc[i]].direction) < MAX_DELTA_ANGLE) {
#if MAX_DELTA_DEBUG
							printf("previous direction is close enough, no liftoff\n");
#endif
							need_lift = 0;
						}
#if MAX_DELTA_DEBUG
						else
							printf("angle change too great, going to lift\n");
#endif
					}
#if MAX_DELTA_DEBUG
					else
						printf("previous distance too low, going to lift\n");
#endif
					if (need_lift) {
#if TRACK_ID_DEBUG
						printf("Over Delta - Closest Mapping %d - %d,%d - %d,%d -> %d,%d\n", prevtpoint[smallest_distance_loc[i]].tracking_id, smallest_distance_loc[i], i, tpoint[i].x, tpoint[i].y, prevtpoint[smallest_distance_loc[i]].x,prevtpoint[smallest_distance_loc[i]].y);
#endif
#if EVENT_DEBUG
						printf("sending max delta liftoff for slot: %i\n", prevtpoint[smallest_distance_loc[i]].slot);
#endif
#if MAX_DELTA_DEBUG
						printf("sending max delta liftoff for slot: %i\n", prevtpoint[smallest_distance_loc[i]].slot);
#endif
						liftoff_slot(prevtpoint[smallest_distance_loc[i]].slot);
						tpoint[i].tracking_id = tracking_id;
						tracking_id++;
					}
				} else {
#if TRACK_ID_DEBUG
					printf("Continued Mapping %d - %d,%d - %lf,%lf -> %lf,%lf\n", prevtpoint[smallest_distance_loc[i]].tracking_id, smallest_distance_loc[i], i, tpoint[i].i, tpoint[i].j, prevtpoint[smallest_distance_loc[i]].i,prevtpoint[smallest_distance_loc[i]].j);
#endif
					tpoint[i].tracking_id = prevtpoint[smallest_distance_loc[i]].tracking_id;
					tpoint[i].prev_loc = smallest_distance_loc[i];
					tpoint[i].distance = smallest_distance[i];
					tpoint[i].direction = atan2(tpoint[i].x - prevtpoint[smallest_distance_loc[i]].x, tpoint[i].y - prevtpoint[smallest_distance_loc[i]].y);
#if AVG_FILTER
					avg_filter(&tpoint[i]);
#endif // AVG_FILTER
				}
				tpoint[i].slot = prevtpoint[smallest_distance_loc[i]].slot;
				slot_in_use[prevtpoint[smallest_distance_loc[i]].slot] = 1;
			} else {
				tpoint[i].tracking_id = tracking_id;
				tracking_id++;
#if TRACK_ID_DEBUG
				printf("New Mapping - %lf,%lf - tracking ID: %i\n", tpoint[i].i, tpoint[i].j, tpoint[i].tracking_id);
#endif
			}
		}
	}

	// assign unused slots to touches that don't have a slot yet
	for (i=0; i<tpc; i++) {
		if (tpoint[i].slot < 0) {
			for (j=0; j<MAX_TOUCH; j++) {
				if (slot_in_use[j] == 0) {
					tpoint[i].slot = j;
					slot_in_use[j] = 1;
#if TRACK_ID_DEBUG
					printf("assign new slot to tpoint[%i], tracking ID: %i, slot: %i | %lf , %lf\n", i, tpoint[i].tracking_id, tpoint[i].slot, tpoint[i].i, tpoint[i].j);
#endif
					j = MAX_TOUCH;
				}
			}
		}
	}

	// report touches
	for (k = 0; k < tpc; k++) {
#if DEBOUNCE_FILTER
		// The debounce filter only works on a single touch.
		// We record the initial touchdown point, calculate a radius in
		// pixels and re-center the point if we're still within the
		// radius.  Once we leave the radius, we invalidate so that we
		// don't debounce again even if we come back to the radius.
		if (tpc == 1) {
			if (new_debounce_touch) {
				// We record the initial location of a new touch
				initialx = tpoint[k].x;
				initialy = tpoint[k].y;
#if DEBOUNCE_DEBUG
				printf("new touch recorded at %i, %i\n", initialx, initialy);
#endif
			} else if (initialx > -20) {
				// See if the current touch is still inside the debounce
				// radius
				if (abs(initialx - tpoint[k].x) <= DEBOUNCE_RADIUS
					&& abs(initialy - tpoint[k].y) <= DEBOUNCE_RADIUS) {
					// Set the point to the original point - debounce!
					tpoint[k].x = initialx;
					tpoint[k].y = initialy;
#if DEBOUNCE_DEBUG
					printf("debouncing!!!\n");
#endif
				} else {
					initialx = -100; // Invalidate
#if DEBOUNCE_DEBUG
					printf("done debouncing\n");
#endif
				}
			}
		}
#endif
#if EVENT_DEBUG
		printf("sending event for tracking ID: %i\n", tpoint[k].tracking_id);
#endif
		send_uevent(uinput_fd, EV_ABS, ABS_MT_SLOT, tpoint[k].slot);
		send_uevent(uinput_fd, EV_ABS, ABS_MT_TRACKING_ID, tpoint[k].tracking_id);
		send_uevent(uinput_fd, EV_ABS, ABS_MT_TOUCH_MAJOR, tpoint[k].touch_major);
		send_uevent(uinput_fd, EV_ABS, ABS_MT_POSITION_X, tpoint[k].x);
		send_uevent(uinput_fd, EV_ABS, ABS_MT_POSITION_Y, tpoint[k].y);
		send_uevent(uinput_fd, EV_SYN, SYN_MT_REPORT, 0);
	}
	if (tpc > 0) {
		send_uevent(uinput_fd, EV_SYN, SYN_REPORT, 0);
		send_uevent(uinput_fd, EV_KEY, BTN_TOUCH, 1);
	}
	previoustpc = tpc; // store the touch count for the next run
	if (tracking_id > 1000)
		tracking_id = 0; // reset tracking ID counter if it gets too big
	return tpc; // return the touch count
}


int cline_valid(unsigned int extras)
{
	if(cline[0] == 0xff && cline[1] == 0x43 && cidx == 44-extras) {
		//printf("cidx %d\n", cline[cidx-1]);
		return 1;
	}
	if(cline[0] == 0xff && cline[1] == 0x47 && cidx > 4 && cidx == (cline[2]+4-extras)) {
		//printf("cidx %d\n", cline[cidx-1]);
		return 1;
	}
	return 0;
}

void put_byte(unsigned char byte)
{
	//printf("Putc %d %d\n", cidx, byte);
	if(cidx==0 && byte != 0xFF)
		return;

	// Sometimes a send is aborted by the touch screen. all we get is an out of place 0xFF
	if(byte == 0xFF && !cline_valid(1))
		cidx = 0;

	cline[cidx++] = byte;
}

int consume_line(void)
{
	int i,j,ret=0;

	if(cline[1] == 0x47) {
		// calculate the data points. all transfers complete
		ret = calc_point();
	}

	if(cline[1] == 0x43) {
		// This is a start event. clear the matrix
		if(cline[2] & 0x80) {
			for(i=0; i < X_AXIS_POINTS; i++)
				for(j=0; j < Y_AXIS_POINTS; j++)
					matrix[i][j] = 0;
		}

		// Write the line into the matrix
		for(i=0; i < Y_AXIS_POINTS; i++)
			matrix[cline[2] & 0x1F][i] = cline[i+3];
	}

	cidx = 0;

	return ret;
}

int snarf2(unsigned char* bytes, int size)
{
	int i,ret=0;

	for(i=0; i < size; i++) {
		put_byte(bytes[i]);
		if(cline_valid(0))
			ret += consume_line();
	}

	return ret;
}

void open_uinput(void)
{
	struct uinput_user_dev device;

	memset(&device, 0, sizeof device);

	uinput_fd=open(UINPUT_LOCATION,O_WRONLY);
	strcpy(device.name,"HPTouchpad");

	device.id.bustype=BUS_VIRTUAL;
	device.id.vendor = 1;
	device.id.product = 1;
	device.id.version = 1;

#if USERSPACE_270_ROTATE
	device.absmax[ABS_MT_POSITION_X] = X_RESOLUTION;
	device.absmax[ABS_MT_POSITION_Y] = Y_RESOLUTION;
#else
	device.absmax[ABS_MT_POSITION_X] = Y_RESOLUTION;
	device.absmax[ABS_MT_POSITION_Y] = X_RESOLUTION;
#endif // USERSPACE_270_ROTATE
	device.absmin[ABS_MT_POSITION_X] = 0;
	device.absmin[ABS_MT_POSITION_Y] = 0;
	device.absfuzz[ABS_MT_POSITION_X] = 2;
	device.absflat[ABS_MT_POSITION_X] = 0;
	device.absfuzz[ABS_MT_POSITION_Y] = 1;
	device.absflat[ABS_MT_POSITION_Y] = 0;

	if (write(uinput_fd,&device,sizeof(device)) != sizeof(device))
		fprintf(stderr, "error setup\n");

	if (ioctl(uinput_fd,UI_SET_EVBIT,EV_KEY) < 0)
		fprintf(stderr, "error evbit key\n");

	if (ioctl(uinput_fd,UI_SET_EVBIT, EV_SYN) < 0)
		fprintf(stderr, "error evbit key\n");

	if (ioctl(uinput_fd,UI_SET_EVBIT,EV_ABS) < 0)
		fprintf(stderr, "error evbit rel\n");

	if (ioctl(uinput_fd,UI_SET_ABSBIT,ABS_MT_TRACKING_ID) < 0)
		fprintf(stderr, "error trkid rel\n");

	if (ioctl(uinput_fd,UI_SET_ABSBIT,ABS_MT_TOUCH_MAJOR) < 0)
		fprintf(stderr, "error tool rel\n");

	//if (ioctl(uinput_fd,UI_SET_ABSBIT,ABS_MT_WIDTH_MAJOR) < 0)
	//	fprintf(stderr, "error tool rel\n");

	if (ioctl(uinput_fd,UI_SET_ABSBIT,ABS_MT_POSITION_X) < 0)
		fprintf(stderr, "error tool rel\n");

	if (ioctl(uinput_fd,UI_SET_ABSBIT,ABS_MT_POSITION_Y) < 0)
		fprintf(stderr, "error tool rel\n");

	if (ioctl(uinput_fd,UI_SET_KEYBIT,BTN_TOUCH) < 0)
		fprintf(stderr, "error evbit rel\n");

	if (ioctl(uinput_fd,UI_DEV_CREATE) < 0)
		fprintf(stderr, "error create\n");
}

void clear_arrays(void)
{
	// clears arrays (for after a total liftoff occurs
	int i;
	for(i=0; i<MAX_TOUCH; i++) {
		tpoint[i].pw = -1000;
		tpoint[i].i = -1000;
		tpoint[i].j = -1000;
		tpoint[i].slot = -1;
		tpoint[i].tracking_id = -1;
		tpoint[i].prev_loc = -1;
		tpoint[i].direction = 0;
		tpoint[i].distance = 0;
		tpoint[i].touch_major = 0;
		tpoint[i].x = -1000;
		tpoint[i].y = -1000;

		prevtpoint[i].pw = -1000;
		prevtpoint[i].i = -1000;
		prevtpoint[i].j = -1000;
		prevtpoint[i].slot = -1;
		prevtpoint[i].tracking_id = -1;
		prevtpoint[i].prev_loc = -1;
		prevtpoint[i].direction = 0;
		prevtpoint[i].distance = 0;
		prevtpoint[i].touch_major = 0;
		prevtpoint[i].x = -1000;
		prevtpoint[i].y = -1000;

		prev2tpoint[i].pw = -1000;
		prev2tpoint[i].i = -1000;
		prev2tpoint[i].j = -1000;
		prev2tpoint[i].slot = -1;
		prev2tpoint[i].tracking_id = -1;
		prev2tpoint[i].prev_loc = -1;
		prev2tpoint[i].direction = 0;
		prev2tpoint[i].distance = 0;
		prev2tpoint[i].touch_major = 0;
		prev2tpoint[i].x = -1000;
		prev2tpoint[i].y = -1000;
	}
}

int main(int argc, char** argv)
{
	struct hsuart_mode uart_mode;
	int uart_fd, nbytes, need_liftoff = 1;
	unsigned char recv_buf[RECV_BUF_SIZE];
	fd_set fdset;
	struct timeval seltmout;
	struct sched_param sparam = { .sched_priority = 99 /* linux maximum, nonportable */};

	/* We set ts server priority to RT so that there is no delay in
	 * in obtaining input and we are NEVER bumped from CPU until we
	 * give it up ourselves. */
	if (sched_setscheduler(0 /* that's us */, SCHED_FIFO, &sparam))
		perror("Cannot set RT priority, ignoring: ");

	uart_fd = open("/dev/ctp_uart", O_RDONLY|O_NONBLOCK);
	if(uart_fd<=0) {
		printf("Could not open uart\n");
		return 0;
	}

	open_uinput();

	ioctl(uart_fd,HSUART_IOCTL_GET_UARTMODE,&uart_mode);
	uart_mode.speed = 0x3D0900;
	ioctl(uart_fd, HSUART_IOCTL_SET_UARTMODE,&uart_mode);

	ioctl(uart_fd, HSUART_IOCTL_FLUSH, 0x9);

	while(1) {
		FD_ZERO(&fdset);
		FD_SET(uart_fd, &fdset);
		seltmout.tv_sec = 0;
		/* 2x tmout */
		seltmout.tv_usec = LIFTOFF_TIMEOUT;

		if (0 == select(uart_fd + 1, &fdset, NULL, NULL, &seltmout)) {
			/* Timeout means liftoff, send event */
#if DEBUG
			printf("timeout! sending liftoff\n");
#endif

			if (need_liftoff) {
#if EVENT_DEBUG
				printf("timeout called liftoff\n");
#endif
				liftoff();
				need_liftoff = 0;
			}
			clear_arrays();

			FD_ZERO(&fdset);
			FD_SET(uart_fd, &fdset);
			/* Now enter indefinite sleep iuntil input appears */
			select(uart_fd + 1, &fdset, NULL, NULL, NULL);
			/* In case we were wrongly woken up check the event
			 * count again */
			continue;
		}
			
		nbytes = read(uart_fd, recv_buf, RECV_BUF_SIZE);
		
		if(nbytes <= 0)
			continue;
#if DEBUG
		printf("Received %d bytes\n", nbytes);
		int i;
		for(i=0; i < nbytes; i++)
			printf("%2.2X ",recv_buf[i]);
		printf("\n");
#endif
		if (!snarf2(recv_buf,nbytes)) {
			// sometimes there is data, but no valid touches due to threshold
			if (need_liftoff) {
#if EVENT_DEBUG
				printf("snarf2 called liftoff\n");
#endif
				liftoff();
				need_liftoff = 0;
			}
			clear_arrays();
		} else
			need_liftoff = 1;
	}

	return 0;
}