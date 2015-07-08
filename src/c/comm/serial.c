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

#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include "serial.h"



/**
  * Implement the thread which will manage the data receiving from device.
  * @param pArg Pointer to a @see stSerialDevice to observe;
  */
void* __SerialThreadLoop( void *pArg )  {

  //struct  timeval     readTimeout;
  struct stSerialDevice  *pDev = ( struct stSerialDevice * ) pArg;
  int                    nMaxFd = pDev -> nDevFd + 1;
  fd_set                 readfs;


  // Timeout settings
  //memset( &readTimeout, 0, sizeof( readTimeout ) );
  //readTimeout.tv_usec = nReadTimeout * 1000;           // Miliseconds
  //readTimeout.tv_sec  = READ_SEC_TIMEOUT;              // Seconds
  FD_ZERO( &readfs );

  while( pDev -> nIsOpen )  {
    FD_SET( pDev -> nDevFd, &readfs );
    select( nMaxFd, &readfs, NULL, NULL, NULL /*&readTimeout*/ );

    // Event handling
    if( FD_ISSET( pDev -> nDevFd, &readfs ) && pDev -> pReceiveSerialFn )
      pDev -> pReceiveSerialFn( pDev );
  }

  close( pDev -> nDevFd );
  pDev -> nDevFd = -1;
  pDev -> nThreadId = 0;

  return NULL;
}

/**
  * Wait for the serial thread to stop.
  * @param pDev Pointer to a previously opened @see stSerialDevice struct;
  */
void __WaitForSerialThreadStop( struct stSerialDevice *pDev )  {

  if( pDev -> nThreadId != 0 )
    pthread_join( pDev -> nThreadId, NULL );
}

/**
  * Reset the serail communication structure passed by parameter, applying its
  * default values.
  * @param pDev Pointer to a @see stSerialDevice which will be
  * initialized;
  * WARNING: DON'T USE this function for valid and working devices pointed
  * by a @see stSerialDevice struct;
  */
void ResetSerialDevice( struct stSerialDevice *pDev )  {

  pDev -> nIsOpen = 0;
  pDev -> nReadTimeout = 30000;
  pDev -> nDevFd = -1;
  pDev -> nThreadId = 0;
  pDev -> pReceiveSerialFn = NULL;
  pDev -> pReadIOFn  = NULL;
  pDev -> pWriteIOFn = NULL;
  memset( pDev -> szDeviceFileName, 0, PATH_MAX );

  ResetSerialOptions( pDev );
}

/**
  * Open a serial device.
  * @param pDev Pointer to a @see stSerialDevice which communication will be
  * stablished;
  * @return 0 for failure, 1 for success;
  */
int OpenSerial( struct stSerialDevice *pDev )  {

  pDev -> nIsOpen = 0;
  pDev -> nDevFd  = open( pDev -> szDeviceFileName, O_RDWR | O_NOCTTY );

  if( pDev -> nDevFd  > 0 )  {

    int nRetCode = 0;

    /*
     * If there's a receive callback assigned to respond receive events, the
     * receiver thread will be started.
     */
    if( pDev -> pReceiveSerialFn )
      nRetCode = pthread_create( &pDev -> nThreadId,
                                 NULL,
                                 __SerialThreadLoop,
                                 ( void * ) pDev );
    if( nRetCode )
      close( pDev -> nDevFd );
    else  {
      ApplySerialOptions( pDev );
      pDev -> nIsOpen = 1;
    }
  }

  return pDev -> nIsOpen;
}

/**
  * Close a serial communication previously stablished by the @see OpenSerial
  * function.
  * @param pDev Pointer to a @see stSerialDevice which communication will be
  * closed;
  */
int CloseSerial( struct stSerialDevice *pDev )  {

  int    nRetCode = 0;

  if( pDev -> nDevFd > 0 )  {
    nRetCode = 1;
    pDev -> nIsOpen = 0;   // Notify the thread to exit
    __WaitForSerialThreadStop( pDev );
  }

  return nRetCode;
}

/**
  * Chek if the serial communication is still opened;
  * @param pDev Pointer to a @see stSerialDevice which communication will be
  * checked;
  */
int IsSerialOpen( struct stSerialDevice *pDev ) {

  return pDev -> nIsOpen;
}

/**
  * Apply the serial communication chosen options for stablished connection;
  * @param pDev Pointer to the @see stSerialDevice structure containing the
  * @see stSerialOptions which will be applied;
  */
void ApplySerialOptions( struct stSerialDevice *pDev )  {

  struct termios  options;


  tcgetattr( pDev -> nDevFd, &options );

  // Baud rate
  cfsetispeed( &options, pDev -> serialOptions.nSpeed );
  cfsetospeed( &options, pDev -> serialOptions.nSpeed );
  options.c_cflag |= ( CLOCAL | CREAD );
  tcsetattr( pDev -> nDevFd , TCSANOW, &options );
  tcflush( pDev -> nDevFd , TCIFLUSH );

  // Char size
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= pDev -> serialOptions.nCharSize;
  tcsetattr( pDev -> nDevFd , TCSANOW, &options );
  tcflush( pDev -> nDevFd, TCIFLUSH );

  // Stop bits
  if( pDev -> serialOptions.nStopBits == STOP_BITS_2 )
    options.c_cflag |= CSTOPB;
  else
    options.c_cflag &= ~CSTOPB;

  // Parity
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

  tcsetattr( pDev -> nDevFd , TCSANOW, &options );
  tcflush( pDev -> nDevFd , TCIFLUSH );

  // Hardware flow control
  if( pDev -> serialOptions.nHardwareFlowCtrl == ON )
    options.c_cflag |= CRTSCTS;
  else
    options.c_cflag &= ~CRTSCTS;

  tcsetattr( pDev -> nDevFd , TCSANOW, &options );
  tcflush( pDev -> nDevFd , TCIFLUSH );

  // Software flow control
  if( pDev -> serialOptions.nSoftwareFlowCtrl == ON )
    options.c_iflag |= ( IXON | IXOFF | IXANY );
  else
    options.c_iflag &= ~( IXON | IXOFF | IXANY );

  tcsetattr( pDev -> nDevFd , TCSANOW, &options );
  tcflush( pDev -> nDevFd , TCIFLUSH );

  // Charactere echo
  options.c_lflag = pDev -> serialOptions.nEcho;

  tcsetattr( pDev -> nDevFd, TCSANOW, &options );
  tcflush( pDev -> nDevFd, TCIFLUSH );
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


//vector<char> Serial :: Read( int nDataLen, int nBufferLen ) throw( Exception ) {
//
//  if( bIsOpen )  {
//    char                *pData = new char[nDataLen];
//
//    if( pData )  {
//      try  {
//        int             nRead  = Read( pData, nDataLen, nBufferLen );
//
//        if( nRead == -1 )  {
//          delete [] pData;
//          throw Exception( "Error reading device " + strOpenDevice, READ_ERROR );
//        }
//        else  {
//          vector<char>     vData;
//
//          copy( pData, pData + nRead, back_inserter( vData ) );
//          delete [] pData;
//
//          return vData;
//        }
//      }
//      catch( Exception e )  {
//        throw e;
//      }
//    }
//    else
//      throw Exception( "Memory overflow", NO_MEMORY );
//  }
//  else
//    throw Exception( "No devices open", NO_DEVICES_OPEN );
//}
//
//ssize_t Serial :: Read( void *pData, int nDataLen, int nBufferLen ) throw( Exception )  {
//
//  if( nDataLen <= 0 )
//    throw Exception( "Invalid data lenght", INVALID_PARAMETER );
//  else
//    if( bIsOpen )  {
//      int                 nRead     = 0;
//      int                 nCount    = 0;
//      int                 nSizeOf   = ( nBufferLen == FULL_BUFFER ? nDataLen : nBufferLen );
//      clock_t             startTime = clock();
//      int                 nMaxFd    = fdDev + 1;
//      timeval             readTimeout;
//      fd_set              readfs;
//      int                 nRet;
//
//
//      memset( &readTimeout, 0, sizeof( readTimeout ) );
//      readTimeout.tv_usec = nReadTimeout * 1000;
//
//      do {
//        if( nReadTimeout > 0 )  {
//          FD_ZERO( &readfs );
//          FD_SET( fdDev, &readfs );
//          nRet = select( nMaxFd, &readfs, NULL, NULL, &readTimeout );
//        }
//        else
//          nRet = 1;
//
//        if( nRet )
//          nRead = read( fdDev, &( ( char * ) pData )[nCount], ( nBufferLen == FULL_BUFFER ? ( nSizeOf - nCount ) : nBufferLen ) );
//        else  {
//          throw Exception( "Data read timeout", READ_TIMEOUT );
//          return nCount;
//        }
//
//        if( nRead > 0 )
//          nCount+=nRead;
//        else  {
//          usleep( 10 );
//
//          if( ( clock() - startTime ) > ( nReadTimeout * 1000 ) )  {
//            throw Exception( "Data read timeout", READ_TIMEOUT );
//            return nCount;
//          }
//        }
//      } while( nCount < nDataLen );
//
//      return nCount;
//    }
//    else
//      throw Exception( "No devices open", NO_DEVICES_OPEN );
//
//  return -1;
//}
//
//ssize_t Serial :: Write( const void *pData, int nDataLen )  {
//
//  if( bIsOpen )
//    return write( fdDev, pData, nDataLen );
//  else
//    return -1;
//}
//
//ssize_t Serial :: Write( vector<char> vData ) {
//
//  char          *pData = new char[vData.size()];
//  ssize_t       nRes;
//
//
//  if( pData )  {
//    for( int nCount = 0; nCount < vData.size(); nCount++ )
//      pData[nCount] = vData[nCount];
//
//    nRes = Write( pData, vData.size() );
//    delete [] pData;
//
//    return nRes;
//  }
//  else
//    return -1;
//}
