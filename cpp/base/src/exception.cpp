
/**
 *
 * $Id: $
 * $Author: $
 * $Name:  $
 * $Revision: $
 * $State: $
 *
 */

 
#include "exception.h"
#include "util.h"



Exception :: Exception( string strException, int nType )  {

  Util :: PrintLn( strException, LOG_ERR );
  this -> strException = strException;
  this -> nType        = nType;
}

Exception :: ~Exception( void )  {

  strException.clear();
}

string Exception :: GetString( void )  {

  return strException;
}

int Exception :: GetType( void )  {

  return nType;
}
