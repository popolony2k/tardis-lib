/**
  * This software is developed and maintained by PlanetaMessenger.org/
  * RobotMessenger.org
  * Specs, New and updated versions can be found at
  * http://www.planetamessenger.org or http://www.robotmessenger.org.
  * If you want contact the Team please send a email to Project Manager
  * Leidson Campos Alves Ferreira at leidson@planetamessenger.org or
  * PopolonY2k at popolony2k@popolony2k.com.br.
  *
  * Copyleft (C) since 2015 by PlanetaMessenger.org/RobotMessenger.org.
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation; either version 3 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program; if not, write to the Free Software Foundation,
  * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
  */

/**
 *
 * $Id: $
 * $Author: $
 * $Name:  $
 * $Revision: $
 * $State: $
 *
 */


#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <pthread.h>
#include <limits.h>


/**
 * ASCII code used by serial communication applications.
 */
#define    ASCII_NUL               0x00
#define    ASCII_SOH               0x01
#define    ASCII_STX               0x02
#define    ASCII_ETX               0x03
#define    ASCII_EOT               0x04
#define    ASCII_ENQ               0x05
#define    ASCII_ACK               0x06
#define    ASCII_BEL               0x07
#define    ASCII_BS                0x08
#define    ASCII_HT                0x09
#define    ASCII_NL                0x0a
#define    ASCII_VT                0x0b
#define    ASCII_NP_FF             0x0c
#define    ASCII_CR                0x0d
#define    ASCII_SO                0x0e
#define    ASCII_SI                0x0f
#define    ASCII_DLE               0x10
#define    ASCII_XON_DC1           0x11
#define    ASCII_DC2               0x12
#define    ASCII_XOFF_DC3          0x13
#define    ASCII_DC4               0x14
#define    ASCII_NAK               0x15
#define    ASCII_SYN               0x16
#define    ASCII_ETB               0x17
#define    ASCII_CAN               0x18
#define    ASCII_EM                0x19
#define    ASCII_SUB               0x1a
#define    ASCII_ESC               0x1b
#define    ASCII_FS                0x1c
#define    ASCII_GS                0x1d
#define    ASCII_RS                0x1e
#define    ASCII_US                0x1f

/**
 * Serial speed definitions
 */
#define    BRATE_0                 B0
#define    BRATE_50                B50
#define    BRATE_75                B75
#define    BRATE_110               B110
#define    BRATE_134               B134
#define    BRATE_150               B150
#define    BRATE_200               B200
#define    BRATE_300               B300
#define    BRATE_600               B600
#define    BRATE_1200              B1200
#define    BRATE_1800              B1800
#define    BRATE_2400              B2400
#define    BRATE_4800              B4800
#define    BRATE_9600              B9600
#define    BRATE_19200             B19200
#define    BRATE_38400             B38400
#define    BRATE_57600             B57600
//#define    BRATE_76800             B76800
#define    BRATE_115200            B115200

/**
 * Char size.
 */
#define    CSIZE_5BIT              CS5
#define    CSIZE_6BIT              CS6
#define    CSIZE_7BIT              CS7
#define    CSIZE_8BIT              CS8

/**
 * Parity.
 */
#define    PARITY_NONE             0
#define    PARITY_EVEN             1
#define    PARITY_ODD              2
#define    PARITY_SPACE            PARITY_NONE

/**
 * Hardware and software flow control
 */
#define    OFF                     0
#define    ON                      1

/**
 * Stop bits
 */
#define    STOP_BITS_1             0
#define    STOP_BITS_2             1

/**
  * Serial events callback definition
  */
typedef void ( *SERIAL_IO_FN ) ( int );


/**
  * Serial options structure.
  */
struct stSerialOptions  {
  int                         nSpeed;             // Baud rate
  int                         nCharSize;          // Char size
  int                         nParity;            // Parity
  int                         nStopBits;          // Stop bits
  int                         nEcho;              // Echo ON/OFF
  int                         nHardwareFlowCtrl;  // Hardware Flow control
  int                         nSoftwareFlowCtrl;  // Software Flow control
};

/**
  * Serial device handler structure.
  */
struct stSerialDevice  {
  int                         nReadTimeout;
  int                         nDevFd;
  int                         nIsOpen;
  char                        szDeviceFileName[PATH_MAX];
  pthread_t                   nThreadId;
  struct stSerialOptions      serialOptions;
  SERIAL_IO_FN                pReceiveSerialFn;
};


/*
 * Serial communication functions.
 */

void ResetSerial( struct stSerialDevice *pDev );

int OpenSerial( struct stSerialDevice *pDev );
int CloseSerial( struct stSerialDevice *pDev );
int IsSerialOpen( struct stSerialDevice *pDev );

void ApplySerialOptions( struct stSerialDevice *pDev );
void ResetSerialOptions( struct stSerialDevice *pDev );

#endif  // __SERIAL_H__
