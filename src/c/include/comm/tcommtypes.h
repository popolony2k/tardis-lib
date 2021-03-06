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

#ifndef __TCOMMTYPES_H__
#define __TCOMMTYPES_H__

#include <limits.h>
#include <pthread.h>
#include <sys/types.h>


/**
  * Module Callback definitions
  */
/* Event handling callback */
typedef void ( *IO_EVENT_FN ) ( void *pDev );

/**
  * General device handler structure.
  */
struct stDevice  {
  pthread_t                   nThreadId;
  int                         nReadTimeout;
  int                         nWriteTimeout;
  int                         nDevFd;
  int                         nIsEvtRunning;
  char                        szDeviceFileName[PATH_MAX];
  IO_EVENT_FN                 pOnReceiveFn;
};

#endif /* __TCOMMTYPES_H__ */
