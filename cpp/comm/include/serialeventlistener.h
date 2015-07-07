
/**
 *
 * $Id: $
 * $Author: $
 * $Name:  $
 * $Revision: $
 * $State: $
 *
 */

 
#ifndef __SERIALEVENTLISTENER_H__
#define __SERIALEVENTLISTENER_H__


class SerialEventListener  {

  public:

  virtual void OnReceive( int fdDev ) = 0;
};

#endif  // __SERIALEVENTLISTENER_H__
