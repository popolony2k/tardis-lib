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

#include "comm/tbaseio.h"
#include <string.h>
#include <unistd.h>

/*
 * Internal definitions
 */
#define __WRITE_PACKET_SIZE     16   /* Write packet size */



/**
 * Read data from specified device passed by parameter.
 * This function ensures the data buffer loading, controlling communication
 * timeout and  data fragmentation loading.
 * @param pDev Pointer to an opened serial device handler;
 * @param pBuffer Pointer to a buffer to receive the data read;
 * @param nBufferSize The Buffer size to read;
 */
int ReadIO( struct stDevice *pDev, void *pBuffer, int nBufferSize )  {

  if( pDev )  {

    if( nBufferSize <= 0 )
      return IO_INVALID_BUFFER_SIZE;
    else  {
      if( pDev -> nDevFd <= 0 )
        return IO_INVALID_FD_HANDLE;
      else  {
        struct timeval      readTimeout;
        fd_set              readfs;
        int                 nRead;
        int                 nCount = 0;
        int                 nMaxFd = pDev -> nDevFd + 1;
        int                 nRet;

        memset( &readTimeout, 0, sizeof( readTimeout ) );
        readTimeout.tv_usec = pDev -> nReadTimeout * 1000;

        do {
          FD_ZERO( &readfs );
          FD_SET( pDev -> nDevFd, &readfs );
          nRead = 0;
          nRet  = select( nMaxFd, &readfs, NULL, NULL,
                          ( pDev -> nReadTimeout > 0 ? &readTimeout : NULL ) );

          if( FD_ISSET( pDev -> nDevFd, &readfs ) && ( nRet != -1 ) )
            nRead = read( pDev -> nDevFd,
                          &( ( char * ) pBuffer )[nCount],
                          ( nBufferSize - nCount ) );
          else  {
            /* Timeout has occurred ?? */
            if( nCount == 0 )
              return IO_TIMEOUT;
            else
              return nCount;
          }

          if( nRead > 0 )
            nCount+=nRead;
          else  /* Connection lost ?? */
            return nCount;

        } while( nCount < nBufferSize );

        return nCount;
      }
    }
  }

  return IO_INVALID_DEVICE_HANDLE;
}

/**
 * Write data to the specified device passed by parameter.
 * This function ensures the data buffer sending, controlling communication
 * timeout and data fragmentation sending.
 * @param pDev Pointer to an opened serial device handler;
 * @param pBuffer Pointer to a buffer containing the data to be written;
 * @param nBufferSize The Buffer size to write;
 */
int WriteIO( struct stDevice *pDev, void *pBuffer, int nBufferSize )  {
  if( pDev )  {
    if( pDev -> nDevFd <= 0 )
      return IO_INVALID_FD_HANDLE;
    else  {
      int                 nWrite;
      int                 nCount         = 0;
      int                 nRet           = 0;
      int                 nPacketSize    = __WRITE_PACKET_SIZE;
      int                 nMaxFd         = pDev -> nDevFd  + 1;
      struct timeval      writeTimeout;
      fd_set              writefs;

      memset( &writeTimeout, 0, sizeof( writeTimeout ) );
      writeTimeout.tv_usec = pDev -> nWriteTimeout * 1000;

      do {
        FD_ZERO( &writefs );
        FD_SET( pDev -> nDevFd, &writefs );
        nWrite = 0;
        nRet = select( nMaxFd, NULL, &writefs, NULL,
                       ( pDev -> nWriteTimeout > 0 ? &writeTimeout : NULL ) );

        if( FD_ISSET( pDev -> nDevFd, &writefs ) && ( nRet != -1 ) )  {
          int nSize = ( nBufferSize < __WRITE_PACKET_SIZE ? nBufferSize :
                        nPacketSize );

          nWrite = write( pDev -> nDevFd,
                          &( ( char * ) pBuffer )[nCount], nSize );
        }
        else
          return IO_TIMEOUT;  /* Timeout has occurred ?? */

        if( nWrite > 0 )  {
          nCount+=nWrite;

          if( ( nBufferSize > __WRITE_PACKET_SIZE ) &&
              ( ( nBufferSize - nCount ) < __WRITE_PACKET_SIZE ) )
              nPacketSize = ( nBufferSize - nCount );
        }
        else  /* Connection lost ?? */
          return IO_ERROR;

      } while( ( nCount < nBufferSize ) && ( nPacketSize > 0 ) );

      return nCount;
    }
  }

  return IO_INVALID_DEVICE_HANDLE;
}
