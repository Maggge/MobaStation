/**
  MobaStation user configuration loader

  This file is part of the MobaStation Project, GPL v3 (see LICENSE)
*/

//Includes the optional MobaStationConfig.h of the user (a copy of
//MobaStationConfig.example.h in the library folder) exactly once.
//Every library file that has #ifndef defaults for config options includes
//this header first, also the ones that are compiled without MobaStation.h.
#ifndef __MOBASTATION_USER_CONFIG__
#define __MOBASTATION_USER_CONFIG__

#if defined(__has_include)
#  if __has_include("MobaStationConfig.h")
#    include "MobaStationConfig.h"
#  endif
#endif

#endif
