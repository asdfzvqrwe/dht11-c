#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/time.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <poll.h>
#include <signal.h>
#include <assert.h>

#include "libgp.h"

static int result;
static int Tmr1ms = 0;
static int cnt_ms = 0;
static int gpio_pin = 22;
static int segment[] = {6, 19, 25, 12 ,20, 26, 23 ,21};
static int digits[] = {16, 13, 5};
static int num[][7] = {
    
    {1, 1, 1, 1, 1, 1, 0},
    {0, 1, 1, 0, 0, 0, 0},
    {1, 1, 0, 1, 1, 0, 1},
    {1, 1, 1, 1, 0, 0, 1},
    {0, 1, 1, 0, 0, 1, 1},
    {1, 0, 1, 1, 0, 1, 1},
    {1, 0, 1, 1, 1, 1, 1},
    {1, 1, 1, 0, 0, 0, 0},
    {1, 1, 1, 1, 1, 1, 1},
    {1, 1, 1, 1, 0, 1, 1},
    {0, 0, 0, 0, 0, 0, 0},
    
};

static volatile bool timeout = false;

static inline void
set_timer(long usec) {
	static struct itimerval timer = {
		{ 0, 0 },	// Interval
		{ 0, 0 }	// it_value
	};
	int rc;

	timer.it_value.tv_sec = 0;
	timer.it_value.tv_usec = usec;
	
	timer.it_interval.tv_sec = 0;
	timer.it_interval.tv_usec = usec;

	rc = setitimer(ITIMER_REAL,&timer,NULL);
	assert(!rc);
	timeout = false;
}

static inline void
timeofday(struct timespec *t) {
	clock_gettime(CLOCK_MONOTONIC,t);
}

static int
ms_diff(struct timespec *t0,struct timespec *t1) {
	int dsec = (int)(t1->tv_sec - t0->tv_sec);
	int dms = (t1->tv_nsec - t0->tv_nsec) / 1000000L;

	assert(dsec >= 0);
	dms += dsec * 1000;
	return dms;
}

static inline long
ns_diff(struct timespec *t0,struct timespec *t1) {
	int dsec = (int)(t1->tv_sec - t0->tv_sec);
	long dns = t1->tv_nsec - t0->tv_nsec;

	assert(dsec >= 0);
	dns += dsec * 1000000000L;
	return dns;
}

static void
wait_ready(void) {
	static struct timespec t0 = {0L,0L};
	struct timespec t1;

	if ( !t0.tv_sec ) {
		timeofday(&t0);
		--t0.tv_sec;
	}

	for (;;) {
		timeofday(&t1);
		//if ( ms_diff(&t0,&t1) >= 1000 ) {
		if ( ms_diff(&t0,&t1) >= 1500 ) {		/*every 1.5s*//* 210110 SGsoft */
			t0 = t1;
			return;
		}
		usleep(100);
	}
}

static void
wait_ms(int ms) {
#if 1
	Tmr1ms = ms;

	while(Tmr1ms){
		usleep(100);
	}
#else
	struct pollfd p[1];
	int rc;

	rc = poll(&p[0],0,ms);
	assert(!rc);
#endif
}

static inline int
wait_change(long *nsec) {
	int b1;
	struct timespec t0, t1;
	int b0 = gpio_read(gpio_pin);

	timeofday(&t0);

	while ( (b1 = gpio_read(gpio_pin)) == b0 && !timeout )
		;
	timeofday(&t1);

	if ( !timeout ) {
		*nsec = ns_diff(&t0,&t1);
		return b1;
	}
	*nsec = 0;
	return 0;
}

static inline int
read_bit(void) {
	long nsec;

	wait_change(&nsec);
	wait_change(&nsec);
	//return nsec > 35000;
	return nsec > 45000;				/* 210110 SGsoft */
}

static unsigned
read_40bits(void) {
	short sx = 40;
	uint64_t acc = 0;

	while ( sx-- > 0 )
		acc |= (uint64_t)read_bit() << sx;

	#if 1	/* 220110 SGsoft */
	unsigned cksum = acc & 0xFF;
	unsigned rh = acc >> 24;
	unsigned temp = (acc >> 8) & 0xFFFFF;
	
	unsigned comp = 0;
	for(unsigned i = 1;i < 5;i++){
		comp += ((acc >> (8 * i)) & 0xFF) ;
	}
	
	//printf("[DEBUG] acc = 0x%llx\n",acc);
	//printf("[DEBUG] RH 0x%x%% Temperature 0x%x C : cksum = 0x%x, comp = 0x%x\n",rh,temp,cksum,comp);
	
	if ( comp != cksum )
		return 0;

	return (rh << 16) | temp;
	
	#else
	unsigned cksum = acc & 0xFF;
	unsigned rh = acc >> 32;
	unsigned temp = (acc >> 16) & 0xFF;
	unsigned comp = (rh + temp) & 0xFF;
	
	comp = 0;
	for(unsigned i = 1;i < 5;i++){
		comp += ((acc >> (8 * i)) & 0xFF) ;
	}
	
	//printf("[DEBUG] acc = 0x%llx\n",acc);
	//printf("[DEBUG] RH %d%% Temperature %d C : cksum = %d, comp = %d\n",rh,temp,cksum,comp);

	if ( comp != cksum )
		return 0;

	return (rh << 8) | temp;
	#endif	/*1*/
}

static void
sigalrm_handler(int signo) {
	timeout = true;
}
static int dig;
int digi [3] = {10, 10, 10};

static void
seg_handler(int signo) {
	gpio_write(digits[dig], 1);

	dig++;
	if(dig >= 3) {
		dig = 0;
	}
	
	for(int j=0; j<7; j++) {
	gpio_write(segment[j], num[digi[dig]][j]);
	}
	if(dig ==1) {
		gpio_write(segment[7], 1);
	} else {
		gpio_write(segment[7], 0);
	}
	
	gpio_write(digits[dig], 0);
	
	if(Tmr1ms) {
		Tmr1ms--;
	}
	
	cnt_ms++;
	if(cnt_ms >= 200) {
		timeout = true;
	}
}
static void
usage(const char *cmd) {
	
	printf(
		"Usage:\t%s [-g gpio] [-h]\n"
		"where:\n"
		"\t-g gpio\tSpecify GPIO pin (22 is default)\n"
		"\t-h\tThis help\n",
		cmd);	
}



int
main(int argc,char **argv) {
	static char options[] = "hg:";
	struct sigaction new_action;
	int reading = 0;
	long nsec;
	int oc, b;

	while ( (oc = getopt(argc,argv,options)) != -1 ) {
		switch ( oc ) {
		case 'g':
			gpio_pin = atoi(optarg);
			if ( gpio_pin <= 0 ) {
				fprintf(stderr,"Invalid gpio: -g %s\n",
					optarg);
				exit(1);
			}
			break;
		case 'h':
			usage(argv[0]);
			exit(0);
		default:
			usage(argv[0]);
			exit(1);
		}		
	}
	#if 1
	
	new_action.sa_handler = seg_handler;
	sigemptyset(&new_action.sa_mask);
	new_action.sa_flags = 0;
	sigaction(SIGALRM,&new_action,NULL);
	
	#else
	new_action.sa_handler = sigalrm_handler;
	sigemptyset(&new_action.sa_mask);
	new_action.sa_flags = 0;
	sigaction(SIGALRM,&new_action,NULL);
	#endif

	gpio_open();

	gpio_configure_io(gpio_pin,Output);
	gpio_write(gpio_pin,1);

	for(int i=0; i<8; i++) {
		gpio_configure_io(segment[i], Output);
		gpio_write(segment[i], 1);
	}

	for(int i=0; i<3; i++) {
		gpio_configure_io(digits[i], Output);
		gpio_write(digits[i], 1);
	}

	for(int j=0; j<7; j++) {
		gpio_write(segment[j], num[1][j]);
		gpio_write(segment[7], 0);
	}
		
	set_timer(1000);

	for (;; ++reading) {
		wait_ready();

		gpio_write(gpio_pin,1);
		gpio_configure_io(gpio_pin,Output);
		wait_ms(3);

		//set_timer(100000);	// 100 ms
		//set_timer(200000);	/*200 ms*//* 210110 SGsoft */
		

		cnt_ms = 0;
		timeout = false;

		gpio_write(gpio_pin,0);
		wait_ms(30);
		gpio_configure_io(gpio_pin,Input);

		b = wait_change(&nsec);

		/*
		 * If the returned value is 1, it is likely
		 * that we were fast enough to catch the
		 * pullup resistor action. When that happens
		 * look for the next transition (expecting
		 * b == 0).
		 */
		if ( b == 1 )
			b = wait_change(&nsec);
		if ( b || nsec > 20000 ) { // Expecting about 12 us
			printf("%04d: Fail, b0=%d, %ld nsec\n",reading,b,nsec);
			continue;
		}

		/*
		 * This is the 80 us transition from 0 to 1:
		 */
		b = wait_change(&nsec);
		if ( !b || nsec < 40000 || nsec > 90000 ) {
			printf("%04d: Fail, b1=%d, %ld nsec\n",reading,b,nsec);
			continue;
		}

		/*
		 * Wait for the 80 us transition from 1 to 0:
		 */
		b = wait_change(&nsec);

		if ( b != 0 || nsec < 40000 || nsec > 90000 ) {
			printf("%04d: Fail, b2=%d, %ld nsec\n",reading,b,nsec);
			continue;
		}

		/*
		 * Read the 40-bit value from the DHT11. The
		 * returned value is distilled into 16-bits:
		 */
		unsigned resp = read_40bits();

		if ( !resp ) {
			printf("%04d: Fail, Checksum error.\n",reading);
			continue;
		}

		#if 1		/* 220110 SGsoft */
		#if 1
		float rh = ((resp >> 24) & 0xFF) + ((resp >> 16) & 0xFF) * 0.1;
		float temp = ((resp >> 8) & 0xFF) + (resp & 0xFF) * 0.1;
		
		printf("%04d: RH %.1f%% Temperature %.1f C\n",reading,rh,temp);
		#else
		int rh = resp >> 24;
		int temp = (resp >> 8 & 0xFF);
		
		printf("%04d: RH %d%% Temperature %d C\n",reading,rh,temp);
		#endif	/*1*/
		#else
		int rh = resp >> 8;
		int temp = (resp & 0xFF);
		
		printf("%04d: RH %d%% Temperature %d C\n",reading,rh,temp);
		#endif	/*1*/

		result = temp * 10;
		
	digi[0] = result / 100;
	digi[1] = result / 10 % 10;
	digi[2] = result % 100 % 10;

		/*for(int i=0; i<10; i++) {
			for(int loop=0; loop<7; loop++) {
				gpio_write(segment[loop], num[i][loop]);
			}
			if(digits[i] == 1) {
				gpio_write(21, 1);
			} else {
				gpio_write(21, 0);
			}	
			gpio_write(digits[i], 0);
			wait_ms(3);
			gpio_write(digits[i], 1);
		}*/	

		/*for(int i=0; i<200; i++) {
			for(int digit=0; i<3; i++) {
				for(int loop=0; i<7; i++) {
					gpio_write(segment[loop], num[result][loop]);
				}
				if(digit == 1) {
					gpio_write(21, 1);
				}else {
				gpio_write(21, 0);
				}
				gpio_write(digits[digit], 0);
				wait_ms(10);
				gpio_write(digits[digit], 1);
				}	
			}*/
	}
	return 0;
}

// End dht11.c

