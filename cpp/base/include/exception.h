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
 

#ifndef __EXCEPTION_H__
#define __EXCEPTION_H__

using namespace std;

#include <string>

/**
  * Predefined exception types
  */
#define NO_MEMORY               0x00
#define NO_DEVICES_OPEN         0x01
#define READ_ERROR              0x02
#define INVALID_PARAMETER       0x03
#define READ_TIMEOUT            0x04
#define KEY_NOT_FOUND           0x05



class Exception  {

  string         strException;
  int            nType;
  
  
  public:
  
   Exception( string strException, int nType );
  ~Exception( void );
  
  string GetString( void );
  int GetType( void );
};

#endif  // __EXCEPTION_H__
