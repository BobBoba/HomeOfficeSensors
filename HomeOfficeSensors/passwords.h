// passwords.h

#ifndef _PASSWORDS_h
#define _PASSWORDS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

extern const char *ssid; // wifi ssid
extern const char *pass; // wifi password

extern char azureHost[]; // xxx.azure-devices.net
extern int azurePort; // usually 443
extern char authSAS[]; // SharedAccessSignature sr=....
extern char deviceName[];
//extern char azureUri[]; // for receiving messages


#endif

