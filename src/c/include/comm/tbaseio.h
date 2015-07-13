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

#ifndef __TBASEIO_H__
#define __TBASEIO_H__

#include <comm/tcommtypes.h>
#include <limits.h>

#define    IO_SUCCESS             -1000
#define    IO_TIMEOUT             -1001
#define    IO_ERROR               -1002
#define    INVALID_BUFFER_SIZE    -1003
#define    INVALID_FD_HANDLE      -1004
#define    INVALID_DEVICE_HANDLE  -1005
#define    IO_LAST_ID             -1006


int ReadIO( struct stDevice *pDev, void *pBuffer, int nBufferSize );
int WriteIO( struct stDevice *pDev, void *pBuffer, int nBufferSize );


#endif /* __TBASEIO_H__ */
