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

#define MAX_TOUCH 5
#define MAX_CLIST 75
#define MAX_DELTA 25 // This value is squared to prevent the need to use sqrt
#define TOUCH_THRESHOLD 36 // Threshold for what is considered a valid touch

// Enables filtering of larger touch areas like the side of your thumb into
// a single touch
#define LARGE_AREA_FILTER 1
#define LARGE_AREA_THRESHOLD 36 // Threshold to invoke the large area filter
#define LARGE_AREA_UNPRESS 32 // Threshold for end of the large area

// Enables filtering of a single touch to make it easier to long press.
// Keeps the initial touch point the same so long as it stays within
// the radius (note it's not really a radius and is actually a square)
#define DEBOUNCE_FILTER 1
#define DEBOUNCE_RADIUS 2 // Radius for debounce in pixels

/** ------- end of user modifiable parameters ---- */
#if DEBOUNCE_FILTER
#if USERSPACE_270_ROTATE
#define DEBOUNCE_RADIUS_RAW_X ((float)DEBOUNCE_RADIUS / 1024 * 39)
#define DEBOUNCE_RADIUS_RAW_Y ((float)DEBOUNCE_RADIUS /  768 * 29)
#else
#define DEBOUNCE_RADIUS_RAW_X ((float)DEBOUNCE_RADIUS /  768 * 29)
#define DEBOUNCE_RADIUS_RAW_Y ((float)DEBOUNCE_RADIUS / 1024 * 39)
#endif
#endif

#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))
#define isBetween(A, B, C) ( ((A-B) > 0) && ((A-C) < 0) )

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
	int tracking_id;
	float direction;
	float distance;
	int touch_major;
	int x;
	int y;
	unsigned short isValid;
};

struct touchpoint tpoint[MAX_TOUCH];
struct touchpoint prevtpoint[MAX_TOUCH];
struct touchpoint prev2tpoint[MAX_TOUCH];

unsigned char cline[64];
unsigned int cidx=0;
unsigned char matrix[30][40];
int invalid_matrix[30][40];
int uinput_fd;

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


int tpcmp(const void *v1, const void *v2)
{
	return ((*(struct candidate *)v2).pw - (*(struct candidate *)v1).pw);
}

int dist(int x1, int y1, int x2, int y2)  {
	return pow(x1 - x2, 2)+pow(y1 - y2, 2);
}


#if AVG_FILTER
int isClose(struct touchpoint a, struct touchpoint b)
{
	if (isBetween(b.i, a.i+2.5, a.i-2.5) && isBetween(b.j, a.j+2.5, a.j-2.5))
		return 1;
	return 0;
}

// return 1 if b is closer
// return 2 if c is closer
int find_closest(struct touchpoint a, struct touchpoint b, struct touchpoint c)
{
	int diffB = fabs(a.i - b.i) + fabs(a.j - b.j);
	int diffC = fabs(a.i - c.i) + fabs(a.j - c.j);

	if (diffB < diffC)
		return 1;
	else
		return 2;
}

int avg_filter(struct touchpoint *t) {
	int tp1_found, tp2_found, i;
	tp1_found = tp2_found = -1;

	for(i=0; i<MAX_TOUCH; i++) {
		if(isClose(*t, prevtpoint[i])) {
			if(tp1_found < 0) {
				tp1_found = i;
			} else {
				if (find_closest(*t, prevtpoint[tp1_found], prevtpoint[i]) == 2)
					tp1_found = i;
			}
		}
		if(isClose(*t, prev2tpoint[i])) {
			if(tp2_found < 0) {
				tp2_found = i;
			} else {
				if (find_closest(*t, prev2tpoint[tp2_found], prev2tpoint[i]) == 2)
					tp2_found = i;
			}
		}
	}
#if DEBUG
	printf("before: i=%f, j=%f", t->i, t->j);
#endif 
	if (tp1_found >= 0 && tp2_found >= 0) {
		t->i = (t->i + prevtpoint[tp1_found].i + prev2tpoint[tp2_found].i) / 3.0;
		t->j = (t->j + prevtpoint[tp1_found].j + prev2tpoint[tp2_found].j) / 3.0;
	}
#if DEBUG
	printf("|||| after: i=%f, j=%f\n", t->i, t->j);
#endif
	return 0;
}
#endif // AVG_FILTER

void liftoff(void)
{
	// sends liftoff events - nothing is touching the screen
#if EVENT_DEBUG
	printf("liftoff function\n");
#endif
	send_uevent(uinput_fd, EV_ABS, ABS_MT_SLOT, tpoint[0].tracking_id);
	send_uevent(uinput_fd, EV_ABS, ABS_MT_TRACKING_ID, tpoint[0].tracking_id);
	send_uevent(uinput_fd, EV_ABS, ABS_MT_TOUCH_MAJOR, 0);
	send_uevent(uinput_fd, EV_ABS, ABS_MT_POSITION_X, tpoint[0].x);
	send_uevent(uinput_fd, EV_ABS, ABS_MT_POSITION_Y, tpoint[0].y);
	send_uevent(uinput_fd, EV_SYN, SYN_MT_REPORT, 0);
	send_uevent(uinput_fd, EV_SYN, SYN_REPORT, 0);
	send_uevent(uinput_fd, EV_KEY, BTN_TOUCH, 0);
}

#if LARGE_AREA_FILTER
void check_large_area_points(int *highi, int *highj, int i, int j, int *mini, int *maxi, int *minj, int *maxj){
	// Invalidate this touch point so that we don't process it later
	invalid_matrix[i][j] = 1;
	// Track the highest value for the touch
	if (matrix[i][j] > matrix[*highi][*highj]) {
		*highi = i;
		*highj = j;
	}
	// track the min and max range of the touch for ABS_MT_TOUCH_MAJOR
	if (i < *mini)
		*mini = i;
	if (i > *maxi)
		*maxi = i;
	if (j < *minj)
		*minj = j;
	if (j > *maxj)
		*maxj = j;
	// Check nearby points to see if they are above LARGE_AREA_UNPRESS
	if(i > 0  && !invalid_matrix[i - 1][j] && matrix[i - 1][j] > LARGE_AREA_UNPRESS)
		check_large_area_points(highi, highj, i - 1, j, mini, maxi, minj, maxj);
	if(i < 29 && !invalid_matrix[i + 1][j] && matrix[i + 1][j] > LARGE_AREA_UNPRESS)
		check_large_area_points(highi, highj, i + 1, j, mini, maxi, minj, maxj);
	if(j > 0  && !invalid_matrix[i][j - 1] && matrix[i][j - 1] > LARGE_AREA_UNPRESS)
		check_large_area_points(highi, highj, i, j - 1, mini, maxi, minj, maxj);
	if(j < 39 && !invalid_matrix[i][j + 1] && matrix[i][j + 1] > LARGE_AREA_UNPRESS)
		check_large_area_points(highi, highj, i, j + 1, mini, maxi, minj, maxj);
}
#endif

void swap_touchpoints(struct touchpoint *tpoint1, struct touchpoint *tpoint2) {
	struct touchpoint copytemp;

	copytemp.pw = tpoint2->pw;
	tpoint2->pw = tpoint1->pw;
	tpoint1->pw = copytemp.pw;
	copytemp.i = tpoint2->i;
	tpoint2->i = tpoint1->i;
	tpoint1->i = copytemp.i;
	copytemp.j = tpoint2->j;
	tpoint2->j = tpoint1->j;
	tpoint1->j = copytemp.j;
	copytemp.tracking_id = tpoint2->tracking_id;
	tpoint2->tracking_id = tpoint1->tracking_id;
	tpoint1->tracking_id = copytemp.tracking_id;
	copytemp.direction = tpoint2->direction;
	tpoint2->direction = tpoint1->direction;
	tpoint1->direction = copytemp.direction;
	copytemp.distance = tpoint2->distance;
	tpoint2->distance = tpoint1->distance;
	tpoint1->distance = copytemp.distance;
	copytemp.touch_major = tpoint2->touch_major;
	tpoint2->touch_major = tpoint1->touch_major;
	tpoint1->touch_major = copytemp.touch_major;
	copytemp.x = tpoint2->x;
	tpoint2->x = tpoint1->x;
	tpoint1->x = copytemp.x;
	copytemp.y = tpoint2->y;
	tpoint2->y = tpoint1->y;
	tpoint1->y = copytemp.y;
	copytemp.isValid = tpoint2->isValid;
	tpoint2->isValid = tpoint1->isValid;
	tpoint1->isValid = copytemp.isValid;
}

int calc_point(void)
{
	int i,j,k,l;
	int tweight=0;
	int tpc=0;
	float isum=0, jsum=0;
	float avgi, avgj;
	float powered;
	static int previoustpc;
	int clc=0;
	struct candidate clist[MAX_CLIST];
#if DEBOUNCE_FILTER
	int new_debounce_touch = 0;
	static float initiali, initialj;

	if (tpoint[0].i < 0)
		new_debounce_touch = 1;
#endif

	if (tpoint[0].i < 0)
		previoustpc = 0;

	// Record values for processing later
	for(i=0; i < previoustpc; i++) {
		prev2tpoint[i].i = prevtpoint[i].i;
		prev2tpoint[i].j = prevtpoint[i].j;
		prev2tpoint[i].pw = prevtpoint[i].pw;
		prev2tpoint[i].tracking_id = prevtpoint[i].tracking_id;
		prev2tpoint[i].direction = prevtpoint[i].direction;
		prev2tpoint[i].distance = prevtpoint[i].distance;
		prev2tpoint[i].touch_major = prevtpoint[i].touch_major;
		prev2tpoint[i].x = prevtpoint[i].x;
		prev2tpoint[i].y = prevtpoint[i].y;
		prev2tpoint[i].isValid = prevtpoint[i].isValid;
		prevtpoint[i].i = tpoint[i].i;
		prevtpoint[i].j = tpoint[i].j;
		prevtpoint[i].pw = tpoint[i].pw;
		prevtpoint[i].tracking_id = tpoint[i].tracking_id;
		prevtpoint[i].direction = tpoint[i].direction;
		prevtpoint[i].distance = tpoint[i].distance;
		prevtpoint[i].touch_major = tpoint[i].touch_major;
		prevtpoint[i].x = tpoint[i].x;
		prevtpoint[i].y = tpoint[i].y;
		prevtpoint[i].isValid = tpoint[i].isValid;
	}

	// generate list of high values
#if LARGE_AREA_FILTER
	memset(&invalid_matrix, 0, sizeof(invalid_matrix));
#endif
	for(i=0; i < 30; i++) {
		for(j=0; j < 40; j++) {
#if RAW_DATA_DEBUG
			if (matrix[i][j] < RAW_DATA_THRESHOLD)
				printf("   ");
			else
				printf("%2.2X ", matrix[i][j]);
#endif
			if (clc < MAX_CLIST) {
				if(matrix[i][j] > LARGE_AREA_THRESHOLD && !invalid_matrix[i][j]) {
					// This is a large area press (e.g. the side of your thumb)
					// so we will scan all the points nearby and if they are
					// above the LARGE_AREA_UNPRESS we mark them invalid and
					// track the highest i,j location so that we can
					// calculate a "center" for the large area press.
					int highi = i, highj = j, mini = i, maxi = i, minj = j, maxj = j;
					check_large_area_points(&highi, &highj, i, j, &mini, &maxi, &minj, &maxj);
					// Set the point to the center
					clist[clc].i = highi;
					clist[clc].j = highj;
					maxi = maxi - mini;
					maxj = maxj - minj;
					clist[clc].touch_major = (maxi * maxi) + (maxj * maxj);
					clist[clc].pw = matrix[highi][highj];
					clc++;
				}
			}
		}
	}
#if RAW_DATA_DEBUG
	printf("end of raw data\n"); // helps separate one frame from the next
#endif

#if DEBUG
	printf("%d %d %d \n", clist[0].pw, clist[1].pw, clist[2].pw);
#endif

	for(k=0; k < MIN(clc, 20); k++) {
		int newtp=1;
		int rad=3; // radius around candidate to use for calculation
		int mini = clist[k].i - rad+1;
		int maxi = clist[k].i + rad;
		int minj = clist[k].j - rad+1;
		int maxj = clist[k].j + rad;
		
		// discard points close to already detected touches
		for(l=0; l<tpc; l++) {
			if(tpoint[l].i >= mini+1 && tpoint[l].i < maxi-1 && tpoint[l].j >= minj+1 && tpoint[l].j < maxj-1)
				newtp=0;
		}
		
		// calculate new touch near the found candidate
		if(newtp && tpc < MAX_TOUCH) {
			tweight=0;
			isum=0;
			jsum=0;
			for(i=MAX(0, mini); i < MIN(30, maxi); i++) {
				for(j=MAX(0, minj); j < MIN(40, maxj); j++) {
					int dd = dist(i,j,clist[k].i,clist[k].j);
					powered = matrix[i][j];
					if(dd == 2 && 0.65f * matrix[clist[k].i][clist[k].j] < matrix[i][j] ) 
						powered = 0.65f * matrix[clist[k].i][clist[k].j];
					if(dd == 4 && 0.15f * matrix[clist[k].i][clist[k].j] < matrix[i][j] ) 
						powered = 0.15f * matrix[clist[k].i][clist[k].j];
					if(dd == 5 && 0.10f * matrix[clist[k].i][clist[k].j] < matrix[i][j] ) 
						powered = 0.10f * matrix[clist[k].i][clist[k].j];
					if(dd == 8 && 0.05f * matrix[clist[k].i][clist[k].j] < matrix[i][j] ) 
						powered = 0.05f * matrix[clist[k].i][clist[k].j];
					
					powered = pow(powered, 1.5);
					tweight += powered;
					isum += powered * i;
					jsum += powered * j;
				}
			}
			avgi = isum / (float)tweight;
			avgj = jsum / (float)tweight;
			tpoint[tpc].pw = tweight;
			tpoint[tpc].i = avgi;
			tpoint[tpc].j = avgj;
			tpoint[tpc].touch_major = clist[k].touch_major;
			tpoint[tpc].tracking_id = -1;
			tpoint[tpc].isValid = 1;
			tpc++;
#if DEBUG
			printf("Coords %d %lf, %lf, %d\n", tpc, avgi, avgj, tweight);
#endif
		}
	}
	int tracking_id_in_use[MAX_TOUCH];
		for (i=0; i<MAX_TOUCH; i++)
			tracking_id_in_use[i] = 0;
	// match up tracking IDs
	{
		float smallest_distance;
		int smallest_distance_loc;
		
		int changes = 0;
#if TRACK_ID_DEBUG
		if (previoustpc != tpc) {
			printf("********> TOUCH COUNT CHANGE!, prevtpc: %i tpc %i\n", previoustpc, tpc);
			for (i=0; i<tpc; i++) 
				printf("      tpoint[%i] - %lf , %lf\n", i, tpoint[i].i, tpoint[i].j);
			for (i=0; i<previoustpc; i++) 
				printf("  prevtpoint[%i] - %lf , %lf   ID: %i\n", i, prevtpoint[i].i, prevtpoint[i].j, prevtpoint[i].tracking_id);
		}
#endif
		while (changes < MAX(tpc, previoustpc)) {
			for (i=changes; i<tpc; i++) {
				int needs_swap = 0;
				float absi = fabsf(tpoint[i].i - prevtpoint[i].i),
					  absj = fabsf(tpoint[i].j - prevtpoint[i].j);
				smallest_distance = (absi * absi) + (absj + absj);
				smallest_distance_loc = i;
#if TRACK_ID_DEBUG
				if (previoustpc != tpc) printf("--i=%i smallest_distance=%lf smallest_loc=%i absi=%lf absj=%lf\n", i, smallest_distance, smallest_distance_loc, absi, absj);
				if (previoustpc != tpc) printf("--tpoint.i=%lf , tpoint.j=%lf\n", tpoint[i].i, tpoint[i].j);
#endif
				for (j=i+1; j<previoustpc; j++) {
					absi = fabsf(tpoint[i].i - prevtpoint[j].i);
					absj = fabsf(tpoint[i].j - prevtpoint[j].j);
#if TRACK_ID_DEBUG
					if (previoustpc != tpc) printf("++j=%i absi=%lf absj=%lf new_distance=%lf\n", j, absi, absj, (absi * absi) + (absj + absj));
					if (previoustpc != tpc) printf("++prevtpoint.i=%lf , prevtpoint.j=%lf\n", prevtpoint[i].i, prevtpoint[i].j);
#endif
					if ((absi * absi) + (absj + absj) < smallest_distance) {
						smallest_distance = (absi * absi) + (absj + absj);
						smallest_distance_loc = j;
						needs_swap = 1;
#if TRACK_ID_DEBUG
						if (previoustpc != tpc) printf("*****new smaller distance found in j=%i smallest_distance=%lf\n", j, smallest_distance);
#endif
					}
				}
				if (needs_swap) {
					swap_touchpoints(&prevtpoint[smallest_distance_loc], &prevtpoint[i]);
#if TRACK_ID_DEBUG
					if (previoustpc != tpc) printf("XXXXXswapped prevtpoint %i and %i\n", i, smallest_distance_loc);
#endif
				}
			}
#if TRACK_ID_DEBUG
			if (previoustpc != tpc) printf("<<<<<<<<<<<switch from prevtpoint to tpoint scanning>>>>>>>>>>>>\n");
#endif
			for (i=changes; i<previoustpc; i++) {
				int needs_swap = 0;
				float absi = fabsf(prevtpoint[i].i - tpoint[i].i),
					  absj = fabsf(prevtpoint[i].j - tpoint[i].j);
				smallest_distance = (absi * absi) + (absj + absj);
				smallest_distance_loc = i;
#if TRACK_ID_DEBUG
				if (previoustpc != tpc) printf("--i=%i smallest_distance=%lf smallest_loc=%i absi=%lf absj=%lf\n", i, smallest_distance, smallest_distance_loc, absi, absj);
				if (previoustpc != tpc) printf("--tpoint.i=%lf , tpoint.j=%lf\n", tpoint[i].i, tpoint[i].j);
#endif
				for (j=i+1; j<tpc; j++) {
					absi = fabsf(prevtpoint[i].i - tpoint[j].i);
					absj = fabsf(prevtpoint[i].j - tpoint[j].j);
#if TRACK_ID_DEBUG
					if (previoustpc != tpc) printf("++j=%i absi=%lf absj=%lf new_distance=%lf\n", j, absi, absj, (absi * absi) + (absj + absj));
					if (previoustpc != tpc) printf("++tpoint.i=%lf , tpoint.j=%lf\n", tpoint[i].i, tpoint[i].j);
#endif
					if ((absi * absi) + (absj + absj) < smallest_distance) {
						smallest_distance = (absi * absi) + (absj + absj);
						smallest_distance_loc = j;
						needs_swap = 1;
#if TRACK_ID_DEBUG
						if (previoustpc != tpc) printf("*****new smaller distance found in j=%i smallest_distance=%lf\n", j, smallest_distance);
#endif
					}
				}
				if (needs_swap) {
					swap_touchpoints(&tpoint[smallest_distance_loc], &tpoint[i]);
#if TRACK_ID_DEBUG
					if (previoustpc != tpc) printf("XXXXXswapped tpoint %i and %i\n", i, smallest_distance_loc);
#endif
				}
			}
			tpoint[changes].tracking_id = prevtpoint[changes].tracking_id;
			tracking_id_in_use[tpoint[changes].tracking_id] = 1;
#if TRACK_ID_DEBUG
			if (previoustpc != tpc) printf("^^^^^^^^^^^^at tpoint[%i], tracking id: %i | tpoint %lf , %lf || prevtpoint %lf, %lf\n", i, tpoint[changes].tracking_id, tpoint[changes].i, tpoint[changes].j, prevtpoint[changes].i, prevtpoint[changes].j);
#endif
			changes++;
#if TRACK_ID_DEBUG
			if (previoustpc != tpc) {
				printf("       pass complete, changes is now %i\n", changes);
				for (i=0; i<tpc; i++) 
					printf("      tpoint[%i] - %lf , %lf   ID: %i\n", i, tpoint[i].i, tpoint[i].j, tpoint[i].tracking_id);
				for (i=0; i<previoustpc; i++) 
					printf("  prevtpoint[%i] - %lf , %lf   ID: %i\n", i, prevtpoint[i].i, prevtpoint[i].j, prevtpoint[i].tracking_id);
			}
#endif
		}
#if TRACK_ID_DEBUG
		if (previoustpc != tpc) printf("--------> TOUCH COUNT CHANGE END! ---------------------------\n");
#endif
	}
	// assign unused tracking IDs to touches that don't have an ID yet
	for (i=0; i<tpc; i++) {
		if (tpoint[i].tracking_id < 0 && tpoint[i].isValid) {
			j=0;
			while (j < MAX_TOUCH) {
				if (tracking_id_in_use[j] == 0) {
					tpoint[i].tracking_id = j;
					tracking_id_in_use[j] = 1;
#if TRACK_ID_DEBUG
					printf("assign new ID to tpoint[%i], tracking id: %i | %lf , %lf\n", i, tpoint[i].tracking_id, tpoint[i].i, tpoint[i].j);
#endif
					j = MAX_TOUCH;
				}
				j++;
			}
		}
	}
	// lift off any missing IDs
	if (previoustpc > tpc) {
		for (i=0; i<previoustpc; i++) {
			if (!tracking_id_in_use[prevtpoint[i].tracking_id]) {
#if EVENT_DEBUG
				printf("lifted off tracking ID %i\n", prevtpoint[i].tracking_id);
#endif
#if TRACK_ID_DEBUG
				printf("lifted off tracking ID %i\n", prevtpoint[i].tracking_id);
#endif
				send_uevent(uinput_fd, EV_ABS, ABS_MT_SLOT, prevtpoint[i].tracking_id);
				send_uevent(uinput_fd, EV_ABS, ABS_MT_TRACKING_ID, prevtpoint[i].tracking_id);
				send_uevent(uinput_fd, EV_ABS, ABS_MT_TOUCH_MAJOR, 0);
				send_uevent(uinput_fd, EV_ABS, ABS_MT_POSITION_X, prevtpoint[i].x);
				send_uevent(uinput_fd, EV_ABS, ABS_MT_POSITION_Y, prevtpoint[i].y);
				send_uevent(uinput_fd, EV_SYN, SYN_MT_REPORT, 0);
				//send_uevent(uinput_fd, EV_SYN, SYN_REPORT, 0);
				//send_uevent(uinput_fd, EV_KEY, BTN_TOUCH, 0);
			}
		}
	}

	/* Filter touches for impossibly large moves that indicate a liftoff and
	 * re-touch */
	if (previoustpc == 1 && tpc == 1) {
		float deltai, deltaj, total_delta;
		deltai = tpoint[0].i - prevtpoint[0].i;
		deltaj = tpoint[0].j - prevtpoint[0].j;
		// calculate squared hypotenuse
		total_delta = (deltai * deltai) + (deltaj * deltaj);
		if (total_delta > MAX_DELTA) {
#if EVENT_DEBUG
			printf("max delta liftoff for tracking ID: %i\n", tpoint[0].tracking_id);
#endif
			liftoff();
		}
	}

	// report touches
	for (k = 0; k < tpc; k++) {
		if (tpoint[k].isValid) {
#if AVG_FILTER
			avg_filter(&tpoint[k]);
#endif // AVG_FILTER
#if DEBOUNCE_FILTER
			// The debounce filter only works on a single touch.
			// We record the initial touchdown point, calculate a radius in
			// pixels and re-center the point if we're still within the
			// radius.  Once we leave the radius, we invalidate so that we
			// don't debounce again even if we come back to the radius.
			if (tpc == 1) {
				if (new_debounce_touch) {
					// We record the initial location of a new touch
					initiali = tpoint[k].i;
					initialj = tpoint[k].j;
				} else if (initiali > -20) {
					// See if the current touch is still inside the debounce
					// radius
					if (fabsf(initiali - tpoint[k].i) <= DEBOUNCE_RADIUS_RAW_X
					    && fabsf(initialj - tpoint[k].j) <= DEBOUNCE_RADIUS_RAW_Y) {
						// Set the point to the original point - debounce!
						tpoint[k].i = initiali;
						tpoint[k].j = initialj;
					} else {
						initiali = -100; // Invalidate
					}
				}
			}
#endif
#if EVENT_DEBUG
			printf("sending event for tracking ID: %i\n", tpoint[k].tracking_id);
#endif
			send_uevent(uinput_fd, EV_ABS, ABS_MT_SLOT, tpoint[k].tracking_id);
			send_uevent(uinput_fd, EV_ABS, ABS_MT_TRACKING_ID, tpoint[k].tracking_id);
			send_uevent(uinput_fd, EV_ABS, ABS_MT_TOUCH_MAJOR, tpoint[k].touch_major);
#if USERSPACE_270_ROTATE
			tpoint[k].x = tpoint[k].i*768/29;
			tpoint[k].y = 1024-tpoint[k].j*1024/39;
#else
			tpoint[k].x = 1024-tpoint[k].j*1024/39;
			tpoint[k].y = 768-tpoint[k].i*768/29;
#endif // USERSPACE_270_ROTATE
			send_uevent(uinput_fd, EV_ABS, ABS_MT_POSITION_X, tpoint[k].x);
			send_uevent(uinput_fd, EV_ABS, ABS_MT_POSITION_Y, tpoint[k].y);
			send_uevent(uinput_fd, EV_SYN, SYN_MT_REPORT, 0);
			tpoint[k].isValid = 0;
		}
	}

	send_uevent(uinput_fd, EV_SYN, SYN_REPORT, 0);
	send_uevent(uinput_fd, EV_KEY, BTN_TOUCH, 1);
	previoustpc = tpc; // store the touch count for the next run
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
			for(i=0; i < 30; i++)
				for(j=0; j < 40; j++)
					matrix[i][j] = 0;
		}

		// Write the line into the matrix
		for(i=0; i < 40; i++)
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
	device.id.vendor=1;
	device.id.product=1;
	device.id.version=1;

#if USERSPACE_270_ROTATE
	device.absmax[ABS_MT_POSITION_X]=768;
	device.absmax[ABS_MT_POSITION_Y]=1024;
#else
	device.absmax[ABS_MT_POSITION_X]=1024;
	device.absmax[ABS_MT_POSITION_Y]=768;
#endif // USERSPACE_270_ROTATE
	device.absmin[ABS_MT_POSITION_X]=0;
	device.absmin[ABS_MT_POSITION_Y]=0;
	device.absfuzz[ABS_MT_POSITION_X]=2;
	device.absflat[ABS_MT_POSITION_X]=0;
	device.absfuzz[ABS_MT_POSITION_Y]=1;
	device.absflat[ABS_MT_POSITION_Y]=0;

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
		tpoint[i].i = -1000;
		tpoint[i].j = -1000;
		tpoint[i].tracking_id = -1;
		tpoint[i].direction = 0;
		tpoint[i].distance = 0;
		tpoint[i].touch_major = 0;
		tpoint[i].isValid = 0;

		prevtpoint[i].i = -1000;
		prevtpoint[i].j = -1000;
		prevtpoint[i].tracking_id = -1;
		prevtpoint[i].direction = 0;
		prevtpoint[i].distance = 0;
		prevtpoint[i].touch_major = 0;
		prevtpoint[i].isValid = 0;

		prev2tpoint[i].i = -1000;
		prev2tpoint[i].j = -1000;
		prev2tpoint[i].tracking_id = -1;
		prev2tpoint[i].direction = 0;
		prev2tpoint[i].distance = 0;
		prev2tpoint[i].touch_major = 0;
		prev2tpoint[i].isValid = 0;
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

		if (0 == select(uart_fd+1, &fdset, NULL, NULL, &seltmout)) {
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
			select(uart_fd+1, &fdset, NULL, NULL, NULL);
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
