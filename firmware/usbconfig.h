/* Name: usbconfig.h
 * Project: V-USB, virtual USB port for Atmel's(r) AVR(r) microcontrollers
 * Author: Christian Starkjohann
 * Creation Date: 2005-04-01
 * Tabsize: 4
 * Copyright: (c) 2005 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
 * This Revision: $Id: usbconfig.h 799 2010-07-27 17:30:13Z cs $
 */

#ifndef __usbconfig_h_included__
#define __usbconfig_h_included__

/* YOU SHOULD NOT NEED TO MODIFY THIS FILE! All configurations are supposed
 * to go into bootloaderconfig.h!
 */

/* ---------------------------- Hardware Config ---------------------------- */

/* All the port and pin assignments, as well as the clock speed and CRC
   setting are now in bootloaderconfig.h: */

#include "bootloaderconfig.h"

#define USB_INTR_VECTOR         USB_handler // The name must be any valid function name

/* --------------------------- Functional Range ---------------------------- */

#ifndef USB_CFG_MAX_BUS_POWER   // allow bootloaderconfig.h to override
#define USB_CFG_MAX_BUS_POWER           100
#endif
/* Set this variable to the maximum USB bus power consumption of your device.
 * The value is in milliamperes. [It will be divided by two since USB
 * communicates power requirements in units of 2 mA.]
 */

#define USB_CFG_DRIVER_FLASH_PAGE       0
/* If the device has more than 64 kBytes of flash, define this to the 64 k page
 * where the driver's constants (descriptors) are located. Or in other words:
 * Define this to 1 for boot loaders on the ATMega128.
 */


#ifndef __ASSEMBLER__
	void calibrateOscillatorASM(void); // from osccalASM.S
/*
  #if FAST_EXIT_NO_USB_MS>0
    extern uint16_union_t idlePolls;
    #define USB_RESET_HOOK(resetStarts)  if(!resetStarts){ idlePolls.b[1]=0; calibrateOscillatorASM();}
  #else
    #define USB_RESET_HOOK(resetStarts)  if(!resetStarts){ calibrateOscillatorASM();}
  #endif
*/
  #define USB_CFG_HAVE_MEASURE_FRAME_LENGTH   0
#endif


/* define this macro to 1 if you want the function usbMeasureFrameLength()
 * compiled in. This function can be used to calibrate the AVR's RC oscillator.
 */


/* -------------------------- Device Description --------------------------- */

#define USB_CFG_VENDOR_ID 0xD0, 0x16 /* = 0x16d0 */
/* USB vendor ID for the device, low byte first. If you have registered your
 * own Vendor ID, define it here. Otherwise you may use one of obdev's free
 * shared VID/PID pairs. Be sure to read USB-IDs-for-free.txt for rules!
 */
#define  USB_CFG_DEVICE_ID 0x53, 0x07 /* = 0x0753 = Digistump */
/* This is the ID of the product, low byte first. It is interpreted in the
 * scope of the vendor ID. If you have registered your own VID with usb.org
 * or if you have licensed a PID from somebody else, define it here. Otherwise
 * you may use one of obdev's free shared VID/PID pairs. See the file
 * USB-IDs-for-free.txt for details!
 */
#define USB_CFG_DEVICE_VERSION MICRONUCLEUS_VERSION_MINOR, MICRONUCLEUS_VERSION_MAJOR
/* Version number of the device: Minor number first, then major number.
 */
 // electric arrow - not compliant with obdev's rules but we'll have our own vid-pid soon
//#define USB_CFG_VENDOR_NAME 0x2301
//#define USB_CFG_VENDOR_NAME_LEN 1
//#define USB_CFG_VENDOR_NAME 'd','i','g','i','s','t','u','m','p','.','c','o','m'
//#define USB_CFG_VENDOR_NAME_LEN 13
/* These two values define the vendor name returned by the USB device. The name
 * must be given as a list of characters under single quotes. The characters
 * are interpreted as Unicode (UTF-16) entities.
 * If you don't want a vendor name string, undefine these macros.
 * ALWAYS define a vendor name containing your Internet domain name if you use
 * obdev's free shared VID/PID pair. See the file USB-IDs-for-free.txt for
 * details.
 */
//#define USB_CFG_DEVICE_NAME 0x00B5,'B'
//#define USB_CFG_DEVICE_NAME_LEN 2
/* Same as above for the device name. If you don't want a device name, undefine
 * the macros. See the file USB-IDs-for-free.txt before you assign a name if
 * you use a shared VID/PID.
 */
/*#define USB_CFG_SERIAL_NUMBER   'N', 'o', 'n', 'e' */
/*#define USB_CFG_SERIAL_NUMBER_LEN   0 */
/* Same as above for the serial number. If you don't want a serial number,
 * undefine the macros.
 * It may be useful to provide the serial number through other means than at
 * compile time. See the section about descriptor properties below for how
 * to fine tune control over USB descriptors such as the string descriptor
 * for the serial number.
 */
#define USB_CFG_DEVICE_CLASS        0xff    /* set to 0 if deferred to interface */
#define USB_CFG_DEVICE_SUBCLASS     0
//#define USB_CFG_DEVICE_CLASS    0xFE /* application specific */
//#define USB_CFG_DEVICE_SUBCLASS 0x01 /* device firmware upgrade */
/* See USB specification if you want to conform to an existing device class.
 * Class 0xff is "vendor specific".
 */
#define USB_CFG_INTERFACE_CLASS     0   /* define class here if not at device level */
#define USB_CFG_INTERFACE_SUBCLASS  0
#define USB_CFG_INTERFACE_PROTOCOL  0
/* See USB specification if you want to conform to an existing device class or
 * protocol. The following classes must be set at interface level:
 * HID class is 3, no subclass and protocol required (but may be useful!)
 * CDC class is 2, use subclass 2 and protocol 1 for ACM
 */

#define USB_PUBLIC static
/* Use the define above if you #include usbdrv.c instead of linking against it.
 * This technique saves a couple of bytes in flash memory.
 */

/* ------------------- Fine Control over USB Descriptors ------------------- */
/* If you don't want to use the driver's default USB descriptors, you can
 * provide our own. These can be provided as (1) fixed length static data in
 * flash memory, (2) fixed length static data in RAM or (3) dynamically at
 * runtime in the function usbFunctionDescriptor(). See usbdrv.h for more
 * information about this function.
 * Descriptor handling is configured through the descriptor's properties. If
 * no properties are defined or if they are 0, the default descriptor is used.
 * List of static descriptor names (must be declared PROGMEM if in flash):
 *   char usbDescriptorDevice[];
 *   char usbDescriptorConfiguration[];
 *   char usbDescriptorHidReport[];
 *   char usbDescriptorString0[];
 *   int usbDescriptorStringVendor[];
 *   int usbDescriptorStringDevice[];
 *   int usbDescriptorStringSerialNumber[];
 * Other descriptors can't be provided statically, they must be provided
 * dynamically at runtime.
 *
 * Descriptor properties are or-ed or added together, e.g.:
 * #define USB_CFG_DESCR_PROPS_DEVICE   (USB_PROP_IS_RAM | USB_PROP_LENGTH(18))
 *
 * The following descriptors are defined:
 *   USB_CFG_DESCR_PROPS_DEVICE
 *   USB_CFG_DESCR_PROPS_CONFIGURATION
 *   USB_CFG_DESCR_PROPS_UNKNOWN (for all descriptors not handled by the driver)
 *
 */

#define USB_CFG_DESCR_PROPS_DEVICE                  0
#define USB_CFG_DESCR_PROPS_CONFIGURATION           0
#define USB_CFG_DESCR_PROPS_UNKNOWN                 0

#endif /* __usbconfig_h_included__ */
