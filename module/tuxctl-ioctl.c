 /* tuxctl-ioctl.c
 *
 * Driver (skeleton) for the mp2 tuxcontrollers for ECE391 at UIUC.
 *
 * Mark Murphy 2006
 * Andrew Ofisher 2007
 * Steve Lumetta 12-13 Sep 2009
 * Puskar Naha 2013
 */

#include <asm/current.h>
#include <asm/uaccess.h>

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/file.h>
#include <linux/miscdevice.h>
#include <linux/kdev_t.h>
#include <linux/tty.h>
#include <linux/spinlock.h>

#include "tuxctl-ld.h"
#include "tuxctl-ioctl.h"
#include "mtcp.h"

#define debug(str, ...) printk(KERN_DEBUG "%s: " str, __FUNCTION__, ## __VA_ARGS__)

static char bp[3];
static unsigned long led_state;
static unsigned long ack = 0;

static void tux_init(struct tty_struct* tty);
static void set_led(struct tty_struct* tty, unsigned long arg);
static int tux_buttons(struct tty_struct* tty, unsigned long arg);
static char display_char(char hex, char dp_on);

static spinlock_t button_lock = SPIN_LOCK_UNLOCKED;

/************************ Protocol Implementation *************************/

/* tuxctl_handle_packet()
 * IMPORTANT : Read the header for tuxctl_ldisc_data_callback() in
 * tuxctl-ld.c. It calls this function, so all warnings there apply
 * here as well.
 */
void tuxctl_handle_packet (struct tty_struct* tty, unsigned char* packet) {
    unsigned a, b, c;
    char c1;
    char c2;

    a = packet[0]; /* Avoid printk() sign extending the 8-bit */
    b = packet[1]; /* values when printing them. */
    c = packet[2];

    switch (a) {
      case MTCP_ERROR:
          return;
      case MTCP_ACK:
          ack = 0;  //high if it should wait low if it can process 
          return;
      case MTCP_RESET:       
        c1 = MTCP_LED_USR; 
        c2 = MTCP_BIOC_ON; 
        tuxctl_ldisc_put(tty, &c1, 1); 
        tuxctl_ldisc_put(tty, &c2, 1);
        
        set_led(tty, led_state);   //restire last led state
        return;
      case MTCP_BIOC_EVENT:
          bp[1] = b;
          bp[2] = c;
          return;
      default:
          return;
    }
 /*printk("packet : %x %x %x\n", a, b, c); */
}

/* 
 * display_char
 *   DESCRIPTION: Gets bits of which segments to turn on in LED to display given char. If decimal is on then 
				  display decimal.

 *   INPUTS: char hex, char dp
 *   OUTPUTS: none
 *   RETURN VALUE: 8 bit for led
 */

char display_char(char hex, char dp){
 char led;
 switch (hex) {
	// each case is the 7 segment display that needs to be put up on the led
	case 0x0: 
		led = 0xE7; //7 segmenet display for 0
		break;
	case 0x1: 
		led = 0x06;  
		break;
	case 0x2: 
		led = 0xCB;  
 		break;
	case 0x3: 
		led = 0x8F; 
		break;
	case 0x4: 
		led = 0x2E; 
		break;
	case 0x5: 
		led = 0xAD; 
		break;
	case 0x6: 
		led = 0xED; 
		break;
	case 0x7: 
		led = 0x86; 
		break;
	case 0x8: 
		led = 0xEF; 
		break;
	case 0x9: 
		led = 0xAF; 
		break;
	case 0xA: 
		led = 0xEE; 
		break;
	case 0xB: 
		led = 0x6D; 
		break;
	case 0xC: 
		led = 0xE1; 
		break;
	case 0xD: 
		led = 0x4F; 
		break;
	case 0xE: 
		led = 0xE9; 
		break;
	case 0xF: 
		led = 0xE8; 
		break;
	default: 
		return 0;
 	}

	//set bit 4 to 1
	if(dp){
	led += 0x10; //00010000
	}
 return led;
}


/*The argument is a 32-bit integer of the following form: The low 16-bits specify a number whose
hexadecimal value is to be displayed on the 7-segment displays. The low 4 bits of the third byte
specifies which LED’s should be turned on. The low 4 bits of the highest byte (bits 27:24) specify
whether the corresponding decimal points should be turned on. This ioctl should return 0.
*/


void set_led(struct tty_struct* tty, unsigned long arg) {


    char check_on, bitmask, led_dec, cur_led, dec, cur_hex;

    uint8_t led_buf[6];   // LED buffer
    unsigned long hex, bytemask; 
    int buf_size;
    int i;

    hex = arg; //tmp for arg


	led_buf[0] = MTCP_LED_USR;
	tuxctl_ldisc_put(tty, led_buf, 1);
    led_buf[0] = MTCP_LED_SET;   //set user-set LED-display values
    check_on = (arg >> 16) & 0x0F;  //shift arg 16 bits to determine which led is on and use bytemask to check if on
    led_buf[1] = check_on;         //index 1 for on off
    led_dec = (arg >> 24) & 0x0F; //right shift 24 bits to determine with bytemask get which decimal point on

    bitmask = 0x01;    //bitmask
    bytemask = 0x000F;  //bytemask
    buf_size = 2;        //buff input size
    //led buffer for each led       
    for(i = 0; i < 4; i++){ 
        cur_led = check_on & bitmask;
        if(cur_led == bitmask){  // if led on
            dec = led_dec & bitmask;
            cur_hex =(char)(hex & bytemask);
            led_buf[buf_size] = display_char(cur_hex, dec);
            buf_size += 1; // go to next index in arg 
        }
        bitmask <<= 1;   // check next led on
        hex >>= 4;    // get next hex val
    }

	//check if have ack
    if(ack) {
        return;
    }
    ack = 1;
	led_state = arg;
    (tuxctl_ldisc_put(tty,led_buf,buf_size));


}


/* 
 * tux_init
 *   DESCRIPTION: Initializes tux and starts with the led to 0
 *   INPUTS: tty strict
 *   OUTPUTS: void
 *   RETURN VALUE: none
 */
void tux_init(struct tty_struct* tty){
 	char c1 = MTCP_LED_USR; //put led into user mode
 	char c2 = MTCP_BIOC_ON; //enable button interrupt on change
 	ack = 1; 	//set ack to 1

 	tuxctl_ldisc_put(tty, &c1, 1); 
	tuxctl_ldisc_put(tty, &c2, 1);
 	led_state = 0xFFFF0000; // set led to 0
 	set_led(tty, led_state);
 	return;
}

/* 
 * tux_buttons
 *   DESCRIPTION: Gets button from kernel space and copies to userspace
 *   INPUTS: tty strict and argument
 *   OUTPUTS: void
 *   RETURN VALUE: returns 0 if works and EINVAL if it doesnt
 */
int tux_buttons(struct tty_struct* tty, unsigned long arg) {
	char but;
	unsigned long *ptr;

	ptr = (unsigned long *)arg;
	if(ptr == NULL){
	return -EINVAL;
	}
	spin_lock_irq(&button_lock);
	but = ((bp[1] & 0x0F) | ((bp[2]<<4) & 0xF0)); //using button packet get
	copy_to_user(ptr, &but, 1); //get from kernel -> user
	spin_unlock_irq(&button_lock);
	return 0;
}

/******** IMPORTANT NOTE: READ THIS BEFORE IMPLEMENTING THE IOCTLS ************
 *                                                                            *
 * The ioctls should not spend any time waiting for responses to the commands *
 * they send to the controller. The data is sent over the serial line at      *
 * 9600 BAUD. At this rate, a byte takes approximately 1 millisecond to       *
 * transmit; this means that there will be about 9 milliseconds between       *
 * the time you request that the low-level serial driver send the             *
 * 6-byte SET_LEDS packet and the time the 3-byte ACK packet finishes         *
 * arriving. This is far too long a time for a system call to take. The       *
 * ioctls should return immediately with success if their parameters are      *
 * valid.                                                                     *
 *                                                                            *
 ******************************************************************************/

int tuxctl_ioctl (struct tty_struct* tty, struct file* file, 
	      unsigned cmd, unsigned long arg)
 {
	switch (cmd) {
		case TUX_INIT:
			tux_init(tty);
			return 0;
		case TUX_BUTTONS:
			return tux_buttons(tty, arg);
		case TUX_SET_LED:
			set_led(tty, arg);
			return 0;
		case TUX_LED_ACK:
		case TUX_LED_REQUEST:
		case TUX_READ_LED:

		default:
			return -EINVAL;
	}
}



