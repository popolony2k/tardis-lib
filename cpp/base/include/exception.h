
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
