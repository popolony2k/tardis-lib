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


int ReadIO( struct stDevice *pDev, void *pBuffer, int nBufferSize )  {

  if( pDev )  {

    if( nBufferSize <= 0 )
      return INVALID_BUFFER_SIZE;
    else  {
      if( pDev -> nDevFd <= 0 )
        return INVALID_FD_HANDLE;
      else  {
        int                 nRead;
        int                 nCount    = 0;
        int                 nMaxFd    = pDev -> nDevFd + 1;
        int                 nRet;
        struct timeval      readTimeout;
        fd_set              readfs;


        memset( &readTimeout, 0, sizeof( readTimeout ) );
        readTimeout.tv_usec = pDev -> nReadTimeout * 1000;

        do {
          FD_ZERO( &readfs );
          FD_SET( pDev -> nDevFd, &readfs );
          nRead = 0;
          nRet  = select( nMaxFd, &readfs, NULL, NULL, ( pDev -> nReadTimeout > 0 ? &readTimeout : NULL ) );

          if( FD_ISSET( pDev -> nDevFd, &readfs ) && ( nRet != -1 ) )
            nRead = read( pDev -> nDevFd, &( ( char * ) pBuffer )[nCount], ( nBufferSize - nCount ) );
          else  {
            /* Timeout has occurred ?? */
            if( nCount == 0 )
              return IO_TIMEOUT;
            else
              return nCount;
          }

          if( nRead > 0 )
            nCount+=nRead;
          else  {
            /* Connection lost ?? */
            //pIO -> bConnectionBroken = TRUE;
            return nCount;
          }

        } while( nCount < nBufferSize );

        return nCount;
      }
    }
  }

  return INVALID_DEVICE_HANDLE;
}
