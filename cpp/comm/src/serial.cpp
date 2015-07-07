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
#include <fcntl.h>
#include "serial.h"



void *__SerialThreadFn( void *arg )  {

  Serial  *pSelf = ( Serial * ) arg;

  pSelf -> ThreadLoop();

  return NULL;
}

void Serial :: ThreadLoop( void )  {

  //struct  timeval     readTimeout;
  fd_set              readfs;
  int                 nMaxFd = fdDev + 1;


  // Seta timeout para leitura dos dados.
  //memset( &readTimeout, 0, sizeof( readTimeout ) );
  //readTimeout.tv_usec = nReadTimeout * 1000;           // Miliseconds
  //readTimeout.tv_sec  = READ_SEC_TIMEOUT;              // Seconds
  FD_ZERO( &readfs );
  
  while( bIsOpen )  {
    FD_SET( fdDev, &readfs );
    select( nMaxFd, &readfs, NULL, NULL, NULL /*&readTimeout*/ );

    // Event handling
    if( FD_ISSET( fdDev, &readfs ) )  {
      for( int nCount = 0; nCount < events.size(); nCount++ )
        events[nCount] -> OnReceive( fdDev );
    }
  }
}

Serial :: Serial( void )  {

  bIsOpen       = false;
  strOpenDevice = "";
  nReadTimeout  = 30000;
  events.clear();
  ResetOptions();
}

Serial :: ~Serial( void )  {

  Close();
  events.clear();
}

bool Serial :: Open( string strDevice )  {

  if( ( fdDev = open( strDevice.c_str(), O_RDWR | O_NOCTTY ) ) == -1 )  {
    bIsOpen = false;
    return false;
  }
  else  {
    ApplyOptions();
    bIsOpen = true;
    
    // Serial input thread
    if( pthread_create( &threadId, NULL, __SerialThreadFn, ( void * ) this ) )  {
      bIsOpen = false;
      return false;
    }
    else  {
      strOpenDevice = strDevice;
      return true;
    }
  }
}

bool Serial :: Close( void )  {

  bIsOpen = false;
  
  if( strOpenDevice == "" )
    return false;
  else  {
    close( fdDev );
    strOpenDevice = "";
    fdDev = -1;
    return true;
  }
}

bool Serial :: IsOpen( void )  {

  return bIsOpen;
}

vector<char> Serial :: Read( int nDataLen, int nBufferLen ) throw( Exception ) {

  if( bIsOpen )  {
    char                *pData = new char[nDataLen];

    if( pData )  {
      try  {
        int             nRead  = Read( pData, nDataLen, nBufferLen );

        if( nRead == -1 )  {
          delete [] pData;
          throw Exception( "Error reading device " + strOpenDevice, READ_ERROR );
        }
        else  {
          vector<char>     vData;

          copy( pData, pData + nRead, back_inserter( vData ) );
          delete [] pData;

          return vData;
        }
      }
      catch( Exception e )  {
        throw e;
      }
    }
    else
      throw Exception( "Memory overflow", NO_MEMORY );
  }
  else
    throw Exception( "No devices open", NO_DEVICES_OPEN );
}

ssize_t Serial :: Read( void *pData, int nDataLen, int nBufferLen ) throw( Exception )  {

  if( nDataLen <= 0 )
    throw Exception( "Invalid data lenght", INVALID_PARAMETER );
  else
    if( bIsOpen )  {
      int                 nRead     = 0;
      int                 nCount    = 0;
      int                 nSizeOf   = ( nBufferLen == FULL_BUFFER ? nDataLen : nBufferLen );
      clock_t             startTime = clock();
      int                 nMaxFd    = fdDev + 1;
      timeval             readTimeout;
      fd_set              readfs;
      int                 nRet;


      memset( &readTimeout, 0, sizeof( readTimeout ) );
      readTimeout.tv_usec = nReadTimeout * 1000;
      
      do {
        if( nReadTimeout > 0 )  {
          FD_ZERO( &readfs );
          FD_SET( fdDev, &readfs );
          nRet = select( nMaxFd, &readfs, NULL, NULL, &readTimeout );
        }
        else
          nRet = 1;

        if( nRet )
          nRead = read( fdDev, &( ( char * ) pData )[nCount], ( nBufferLen == FULL_BUFFER ? ( nSizeOf - nCount ) : nBufferLen ) );
        else  {
          throw Exception( "Data read timeout", READ_TIMEOUT );
          return nCount;
        }

        if( nRead > 0 )
          nCount+=nRead;
        else  {
          usleep( 10 );

          if( ( clock() - startTime ) > ( nReadTimeout * 1000 ) )  {
            throw Exception( "Data read timeout", READ_TIMEOUT );
            return nCount;
          }
        }
      } while( nCount < nDataLen );

      return nCount;
    }
    else
      throw Exception( "No devices open", NO_DEVICES_OPEN );

  return -1;
}

ssize_t Serial :: Write( const void *pData, int nDataLen )  {

  if( bIsOpen )
    return write( fdDev, pData, nDataLen );
  else
    return -1;
}

ssize_t Serial :: Write( vector<char> vData ) {

  char          *pData = new char[vData.size()];
  ssize_t       nRes;


  if( pData )  {
    for( int nCount = 0; nCount < vData.size(); nCount++ )
      pData[nCount] = vData[nCount];

    nRes = Write( pData, vData.size() );
    delete [] pData;

    return nRes;
  }
  else
    return -1;
}

void Serial :: SetOptions( SerialOptions options )  {

  serialOptions = options;
}

SerialOptions &Serial :: GetOptions( void )  {

  return serialOptions;
}

void Serial :: ApplyOptions( void )  {

  struct termios  options;
  
  
  tcgetattr( fdDev, &options );
  
  // Baud rate
  cfsetispeed( &options, serialOptions.nSpeed );
  cfsetospeed( &options, serialOptions.nSpeed );
  options.c_cflag |= ( CLOCAL | CREAD );
  tcsetattr( fdDev, TCSANOW, &options );
  tcflush( fdDev, TCIFLUSH );
  
  // Char size
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= serialOptions.nCharSize;
  tcsetattr( fdDev, TCSANOW, &options );
  tcflush( fdDev, TCIFLUSH );
  
  // Stop bits
  if( serialOptions.nStopBits == STOP_BITS_2 )
    options.c_cflag |= CSTOPB;
  else
    options.c_cflag &= ~CSTOPB;
  
  // Parity
  switch( serialOptions.nParity )  {
  
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

  tcsetattr( fdDev, TCSANOW, &options );
  tcflush( fdDev, TCIFLUSH );
  
  // Hardware flow control
  if( serialOptions.nHardwareFlowCtrl == ON )
    options.c_cflag |= CRTSCTS;
  else
    options.c_cflag &= ~CRTSCTS;
    
  tcsetattr( fdDev, TCSANOW, &options );
  tcflush( fdDev, TCIFLUSH );
  
  // Software flow control
  if( serialOptions.nSoftwareFlowCtrl == ON )
    options.c_iflag |= ( IXON | IXOFF | IXANY );
  else
    options.c_iflag &= ~( IXON | IXOFF | IXANY );

  tcsetattr( fdDev, TCSANOW, &options );
  tcflush( fdDev, TCIFLUSH );

  // Charactere echo
  options.c_lflag = serialOptions.nEcho;

  tcsetattr( fdDev, TCSANOW, &options );
  tcflush( fdDev, TCIFLUSH );
}

void Serial :: ResetOptions( void )  {

  serialOptions.nSpeed            = BRATE_9600;
  serialOptions.nCharSize         = CSIZE_8BIT;
  serialOptions.nParity           = PARITY_NONE;
  serialOptions.nStopBits         = STOP_BITS_1;
  serialOptions.nEcho             = OFF;
  serialOptions.nHardwareFlowCtrl = OFF;
  serialOptions.nSoftwareFlowCtrl = OFF;
}

void Serial :: SetReadTimeout( int nReadTimeout )  {

  this -> nReadTimeout = nReadTimeout;
}

int Serial :: GetReadTimeout( void )  {

  return nReadTimeout;
}

int Serial :: GetFileHandle( void )  {

  return fdDev;
}

void Serial :: Wait( void )  {

  pthread_join( threadId, NULL );
}  

void Serial :: AddSerialEventListener( SerialEventListener *pListener )  {

  events.push_back( pListener );
}

void Serial :: RemoveSerialEventListener( SerialEventListener *pListener )  {

  events.erase( find( events.begin(), events.end(), pListener ) );
}

void Serial :: ReleaseAllListeners( void )  {

  for( int nCount = 0; nCount < events.size(); nCount++ )
    delete events[nCount];
    
  events.clear();
}
