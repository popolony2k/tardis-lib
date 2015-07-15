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

#include "main.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>


/**
 * Event handler for serial OnReceive callback.
 * @param pDev Pointer to a stSerialDevice data structure;
 */
void OnSerialRead( void *pDev )  {

  struct stSerialDevice  *pSerialDev = ( struct stSerialDevice * ) pDev;
  char pData[1024];

  memset( pData, 0, 1024 );
  int nRead = ReadSerial( SERIAL_IO_MODE_BUFFERED, pSerialDev, pData, 1023 );

  printf( "OnSerialRead() - Event received %d - [%s]\n", nRead, pData );
}

/**
 * Initialize the serial device passed by parameter.
 * @param pDev Pointer to the device which will be initialized;
 */
void InitSerialDevice( struct stSerialDevice *pDev )  {

  pDev -> pReceiveSerialFn = OnSerialRead;
  strcpy( pDev -> device.szDeviceFileName, "/dev/rfcomm4" );
}


/**
 * TARDIS-GPS main entry point.
 */
int main( int argc, char *argv[] )  {

  struct stSerialDevice     serialDev;

  InitSerialDevice( &serialDev );

  if( OpenSerial( &serialDev ) )  {
    WaitForEvents( &serialDev );

    CloseSerial( &serialDev );
  }

  return EXIT_SUCCESS;
}

