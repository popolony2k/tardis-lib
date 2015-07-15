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

#include "comm/tserial.h"
#include "comm/tbaseio.h"
#include <sys/stat.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>



/**
  * Open a serial device.
  * @param pDev Pointer to a @see stSerialDevice which communication will be
  * stablished;
  * @return 0 for failure, 1 for success;
  */
int OpenSerial( struct stSerialDevice *pDev )  {

  pDev -> device.nDevFd = open( pDev -> device.szDeviceFileName,
                                O_RDWR | O_NOCTTY );

  if( pDev -> device.nDevFd  > 0 )  {
    int nRetCode = StartIOEventsReceiver( &pDev -> device );

    if( !nRetCode )
      close( pDev -> device.nDevFd );
    else
      ApplySerialOptions( pDev );
  }

  return pDev -> device.nIsEvtRunning;
}

/**
  * Close a serial communication previously established by the @see OpenSerial
  * function.
  * @param pDev Pointer to a @see stSerialDevice which communication will be
  * closed;
  */
int CloseSerial( struct stSerialDevice *pDev )  {

  int    nRetCode = 0;

  if( pDev -> device.nDevFd > 0 )  {
    nRetCode = 1;
    StopIOEventsReceiver( &pDev -> device );
    WaitIOEvents( &pDev -> device );
  }

  return nRetCode;
}

/**
 * Read data from serial device.
 * @param nMode The function operation mode (SERIAL_IO_MODE_DIRECT or
 * SERIAL_IO_MODE_BUFFERED);
 * @param pDev Pointer to an opened serial device handler;
 * @param pBuffer Pointer to a buffer to receive the data read;
 * @param nBufferSize The Buffer size to read;
 */
ssize_t ReadSerial( int nMode,
                    struct stSerialDevice *pDev,
                    void *pBuffer,
                    int nBufferSize ) {

  if( nMode == SERIAL_IO_MODE_DIRECT )
    return read( pDev -> device.nDevFd, pBuffer, nBufferSize );
  else
    return ReadIO( &pDev -> device, pBuffer, nBufferSize );
}

/**
 * Write data to a serial device.
 * @param nMode The function operation mode (SERIAL_IO_MODE_DIRECT or
 * SERIAL_IO_MODE_BUFFERED);
 * @param pDev Pointer to an opened serial device handler;
 * @param pBuffer Pointer to a buffer containing the data to send;
 * @param nBufferSize The Buffer size;
 */
ssize_t WriteSerial( int nMode,
                     struct stSerialDevice *pDev,
                     void *pBuffer,
                     int nBufferSize ) {

  if( nMode == SERIAL_IO_MODE_DIRECT )
    return write( pDev -> device.nDevFd, pBuffer, nBufferSize );
  else
    return WriteIO( &pDev -> device, pBuffer, nBufferSize );
}

/**
  * Reset the serail communication structure passed by parameter, applying its
  * default values.
  * @param pDev Pointer to a @see stSerialDevice which will be
  * initialized;
  * WARNING: DON'T USE this function for valid and working @see stSerialDevice;
  */
void ResetSerial( struct stSerialDevice *pDev )  {

  ResetIODevice( &pDev -> device );
  memset( pDev -> device.szDeviceFileName, 0, PATH_MAX );
  ResetSerialOptions( pDev );
}

/**
  * Apply the serial communication chosen options for stablished connection;
  * @param pDev Pointer to the @see stSerialDevice structure containing the
  * @see stSerialOptions which will be applied;
  */
void ApplySerialOptions( struct stSerialDevice *pDev )  {

  struct termios  options;


  tcgetattr( pDev -> device.nDevFd, &options );

  /* Baud rate */
  cfsetispeed( &options, pDev -> serialOptions.nSpeed );
  cfsetospeed( &options, pDev -> serialOptions.nSpeed );
  options.c_cflag |= ( CLOCAL | CREAD );
  tcsetattr( pDev -> device.nDevFd , TCSANOW, &options );
  tcflush( pDev -> device.nDevFd , TCIFLUSH );

  /* Char size */
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= pDev -> serialOptions.nCharSize;
  tcsetattr( pDev -> device.nDevFd , TCSANOW, &options );
  tcflush( pDev -> device.nDevFd, TCIFLUSH );

  /* Stop bits */
  if( pDev -> serialOptions.nStopBits == STOP_BITS_2 )
    options.c_cflag |= CSTOPB;
  else
    options.c_cflag &= ~CSTOPB;

  /* Parity */
  switch( pDev -> serialOptions.nParity )  {

    case PARITY_NONE  : {
      options.c_cflag &= ~PARENB;
      options.c_cflag &= ~CSTOPB;
      options.c_cflag &= ~CSIZE;
      options.c_cflag |= CS8;
    }  break;

    case PARITY_EVEN  : {
      options.c_cflag |= PARENB;
      options.c_cflag &= ~PARODD;
      options.c_cflag &= ~CSTOPB;
      options.c_cflag &= ~CSIZE;
      options.c_cflag |= CS7;
    }  break;

    case PARITY_ODD  : {
      options.c_cflag |= PARENB;
      options.c_cflag |= ~PARODD;
      options.c_cflag &= ~CSTOPB;
      options.c_cflag &= ~CSIZE;
      options.c_cflag |= CS7;
    }  break;
  }

  tcsetattr( pDev -> device.nDevFd , TCSANOW, &options );
  tcflush( pDev -> device.nDevFd , TCIFLUSH );

  /* Hardware flow control */
  if( pDev -> serialOptions.nHardwareFlowCtrl == ON )
    options.c_cflag |= CRTSCTS;
  else
    options.c_cflag &= ~CRTSCTS;

  tcsetattr( pDev -> device.nDevFd , TCSANOW, &options );
  tcflush( pDev -> device.nDevFd , TCIFLUSH );

  /* Software flow control */
  if( pDev -> serialOptions.nSoftwareFlowCtrl == ON )
    options.c_iflag |= ( IXON | IXOFF | IXANY );
  else
    options.c_iflag &= ~( IXON | IXOFF | IXANY );

  tcsetattr( pDev -> device.nDevFd , TCSANOW, &options );
  tcflush( pDev -> device.nDevFd , TCIFLUSH );

  /* Character echo */
  options.c_lflag = pDev -> serialOptions.nEcho;

  tcsetattr( pDev -> device.nDevFd, TCSANOW, &options );
  tcflush( pDev -> device.nDevFd, TCIFLUSH );
}

/**
  * Reset the @see stSerialDevice @see stSerialOptions to its default
  * values;
  * @param pDev Pointer to the @see stSerialDevice structure containing the
  * @see stSerialOptions which will be reseted;
  */
void ResetSerialOptions( struct stSerialDevice *pDev )  {

  pDev -> serialOptions.nSpeed            = BRATE_9600;
  pDev -> serialOptions.nCharSize         = CSIZE_8BIT;
  pDev -> serialOptions.nParity           = PARITY_NONE;
  pDev -> serialOptions.nStopBits         = STOP_BITS_1;
  pDev -> serialOptions.nEcho             = OFF;
  pDev -> serialOptions.nHardwareFlowCtrl = OFF;
  pDev -> serialOptions.nSoftwareFlowCtrl = OFF;
}
