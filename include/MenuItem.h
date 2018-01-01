#pragma once 
#include <Arduino.h>

class MenuItem {
	public: 
	  MenuItem (String menuName);
    MenuItem ();
	  ~MenuItem();
	  String          Name;
	  uint16_t        Value;
	  uint16_t*       EEPROMAddress;
	  static uint16_t MenuItemCount;
	  
	  void Save (); 
};

