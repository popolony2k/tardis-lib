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
#include <poll.h>
#include <unistd.h>

/*
 * Internal module definitions
 */
#define __WRITE_PACKET_SIZE     128   /* Write packet size */
#define __POLL_TIMEOUT          1000  /* Timeout for event polling */
#define __DEFAULT_IO_TIMEOUT    10000 /* Default I/O timeout */


/**
  * Implement the thread which will manage the event handling
  * for data receiving from device.
  * @param pArg Pointer to a @see stDevice to observe;
  */
void* __EvtHandler( void *pArg )  {

  struct stDevice  *pDev = ( struct stDevice * ) pArg;
  struct pollfd    pfd;


  memset( &pfd, 0, sizeof( struct pollfd ) );
  pfd.fd = pDev -> nDevFd;
  pfd.events = POLLIN;

  while( pDev -> nIsEvtRunning )  {
    pfd.revents = 0;

    if( ( poll( &pfd, 1, __POLL_TIMEOUT ) > 0 ) && ( pfd.revents == POLLIN ) )
      pDev -> pOnReceiveFn( pDev );
  }

  close( pDev -> nDevFd );
  pDev -> nDevFd    = -1;
  pDev -> nThreadId = 0;
  pDev -> nIsEvtRunning = 0;

  return NULL;
}

/**
 * Reset the @see stDevice passed as parameter;
 * @param pDev Pointer to the @see stDevice which will be reseted;
 * WARNING: DON'T USE this function for valid and working @see stDevices;
 */
void ResetIODevice( struct stDevice *pDev )  {

  pDev -> nDevFd    = -1;
  pDev -> nThreadId = 0;
  pDev -> nIsEvtRunning = 0;
  pDev -> pOnReceiveFn  = NULL;
  pDev -> nReadTimeout  = __DEFAULT_IO_TIMEOUT;
  pDev -> nWriteTimeout = __DEFAULT_IO_TIMEOUT;
}

/**
 * Start the IO event receiver thread manager.
 * @param pDev Pointer to @see stDevice which event will be listen;
 */
int StartIOEventsReceiver( struct stDevice *pDev )  {

  /*
   * If there's a receive callback assigned to respond receive events, the
   * receiver thread will be started.
   */
  if( pDev -> pOnReceiveFn )  {
    pDev -> nIsEvtRunning = 1;

    if( pthread_create( &pDev -> nThreadId, NULL, __EvtHandler,( void * ) pDev ) )
      pDev -> nIsEvtRunning = 0;
  }

  return pDev -> nIsEvtRunning;
}

/**
 * Stop an open IO event receiver thread manager.
 * @param pDev Pointer to @see stDevice which event will be stopped;
 */
void StopIOEventsReceiver( struct stDevice *pDev )  {

  pDev -> nIsEvtRunning = 0;
}

/**
 * Wait for I/O events from the @see stDevice passed as parameter.
 * @param pDev The @see stDevice whose events will be listened;
 */
void WaitIOEvents( struct stDevice *pDev )  {

  if( pDev -> nThreadId != 0 )
    pthread_join( pDev -> nThreadId, NULL );
}

/**
 * Read data from specified device passed by parameter.
 * This function ensures the data buffer loading, controlling communication
 * timeout and  data fragmentation loading.
 * @param pDev Pointer to an opened serial device handler;
 * @param pBuffer Pointer to a buffer to receive the data read;
 * @param nBufferSize The Buffer size to read;
 */
ssize_t ReadIO( struct stDevice *pDev, void *pBuffer, int nBufferSize )  {

  if( pDev )  {
    if( nBufferSize <= 0 )
      return IO_INVALID_BUFFER_SIZE;
    else  {
      if( pDev -> nDevFd <= 0 )
        return IO_INVALID_FD_HANDLE;
      else  {
        struct timeval      readTimeout;
        fd_set              readfs;
        int                 nRet;
        int                 nRead;
        int                 nMaxFd = pDev -> nDevFd + 1;
        ssize_t             nCount = 0;

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
ssize_t WriteIO( struct stDevice *pDev, void *pBuffer, int nBufferSize )  {
  if( pDev )  {
    if( pDev -> nDevFd <= 0 )
      return IO_INVALID_FD_HANDLE;
    else  {
      int                 nWrite;
      int                 nRet         = 0;
      int                 nPacketSize  = __WRITE_PACKET_SIZE;
      int                 nMaxFd       = pDev -> nDevFd  + 1;
      ssize_t             nCount       = 0;
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

/**
 * Performs a device reading until the specified delimiter to be reached.
 * @param pDev Pointer to an opened serial device handler;
 * @param pBuffer Pointer to a buffer to receive the data read;
 * @param nBufferSize The size of buffer pointed by *pBuffer;
 * @param szDelimiter The string containing the delimiter for end of
 * buffer reading;
 */
ssize_t ReadDelim( struct stDevice *pDev,
                   void *pBuffer,
                   int nBufferSize,
                   const char *szDelim )  {

  if( pDev )  {
    if( nBufferSize <= 0 )
      return IO_INVALID_BUFFER_SIZE;
    else  {
      if( pDev -> nDevFd <= 0 )
        return IO_INVALID_FD_HANDLE;
      else  {
        struct timeval      readTimeout;
        fd_set              readfs;
        int                 nEOD        = 0;  /* End of delimiter */
        int                 nDelimCount = 0;
        int                 nDelimSize  = strlen( szDelim );
        int                 nRet;
        int                 nRead;
        int                 nMaxFd = pDev -> nDevFd + 1;
        ssize_t             nCount = 0;


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
                          1 );
          else  {
            /* Timeout has occurred ?? */
            if( nCount == 0 )
              return IO_TIMEOUT;
            else
              return nCount;
          }

          if( nRead > 0 )  {
            /* Check for the delimiter. */
            if( ( ( char * ) pBuffer )[nCount] == szDelim[nDelimCount] )  {
              nDelimCount++;
              nEOD = ( nDelimCount == nDelimSize );
            }
            else
              nDelimCount = 0;

            nCount+=nRead;
          }
          else  /* Connection lost ?? */
            return nCount;

        } while( ( nCount < nBufferSize ) && !nEOD );

        return nCount;
      }
    }
  }

  return IO_INVALID_DEVICE_HANDLE;
}
