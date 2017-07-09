/*
 * extensions.c:
 *	Originally part of the GPIO program to test, peek, poke and otherwise
 *	noodle with the GPIO hardware on the Raspberry Pi.
 *	Now used as a general purpose library to allow systems to dynamically
 *	add in new devices into wiringPi at program run-time.
 *	Copyright (c) 2012-2015 Gordon Henderson
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <ctype.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <fcntl.h>

#include <wiringPi.h>

#include "mcp23008.h"
#include "mcp23016.h"
#include "mcp23017.h"
#include "mcp23s08.h"
#include "mcp23s17.h"
#include "sr595.h"
#include "pcf8574.h"
#include "pcf8591.h"
#include "mcp3002.h"
#include "mcp3004.h"
#include "mcp4802.h"
#include "mcp3422.h"
#include "max31855.h"
#include "max5322.h"
#include "ads1115.h"
#include "sn3218.h"
#include "drcSerial.h"
#include "drcNet.h"
#include "../wiringPiD/drcNetCmd.h"
#include "pseudoPins.h"
#include "bmp180.h"
#include "htu21d.h"
#include "ds18b20.h"
#include "rht03.h"

#include "wpiExtensions.h"

// Do extension function abstraction

using do_extension_t = int (char*, int, char*) ;

// Trivial setup function abstraction

using trivial_setup_t = int (int) ;

// I2C setup function abstraction

using i2c_setup_t = int (int, int) ;

// SPI setup function abstraction

using spi_setup_t = int (int, int) ;

// SPI with port setup function abstraction

using spi_port_setup_t = int (int, int, int) ;

extern int wiringPiDebug ;

static int verbose ;
static char errorMessage [1024] ;


// Local structure to hold details

struct extensionFunctionStruct
{
  const char *name ;
  do_extension_t *function ;
} ;


/*
 * verbError:
 *	Convenient error handling
 *********************************************************************************
 */

static void verbError (const char *message, ...)
{
  va_list argp ;
  va_start (argp, message) ;
    vsnprintf (errorMessage, 1023, message, argp) ;
  va_end (argp) ;

  if (verbose)
    fprintf (stderr, "%s\n", errorMessage) ;
}


/*
 * extractInt:
 *	Check & return an integer at the given location (prefixed by a :)
 *********************************************************************************
 */

static char *extractInt (char *progName, char *p, int *num)
{
  if (*p != ':')
  {
    verbError ("%s: colon expected", progName) ;
    return NULL ;
  }

  ++p ;

  if (!isdigit (*p))
  {
    verbError ("%s: digit expected", progName) ;
    return NULL ;
  }

  *num = strtol (p, NULL, 0) ;

// Increment p, but we need to check for hex 0x

  if ((*p == '0') && (*(p + 1) == 'x'))
    p +=2 ;

  while (isxdigit (*p))
    ++p ;

  return p ;
}


/*
 * extractStr:
 *	Check & return a string at the given location (prefixed by a :)
 *	Note: The string can be enclosed in []'s to escape colons. This is
 *	so we can handle IPv6 addresses which contain colons and the []'s is
 *	a common way to prepresent them.
 *********************************************************************************
 */

static char *extractStr (char *progName, char *p, char **str)
{
  char *q, *r ;
  int quoted = FALSE ;

  if (*p != ':')
  {
    verbError ("%s: colon expected", progName) ;
    return NULL ;
  }

  ++p ;

  if (*p == '[')
  {
    quoted = TRUE ;
    ++p ;
  }

  if (!isprint (*p))	// Is this needed?
  {
    verbError ("%s: character expected", progName) ;
    return NULL ;
  }

  q = p ;
  if (quoted)
  {
    while ((*q != 0) && (*q != ']'))
      ++q ;
  }
  else
  {
    while ((*q != 0) && (*q != ':'))
      ++q ;
  }

  *str = r = (char*) calloc (q - p + 2, 1) ;	// Zeros it

  while (p != q)
    *r++ = *p++ ;

  if (quoted)				// Skip over the ] to the :
    ++p ;

  return p ;
}


/*
 * doExtensionTrivial<
 *   trivialSetup; Trivial extension setup function reference
 * >;
 *	Calls trivialSetup
 *********************************************************************************
 */

template <
  trivial_setup_t &trivialSetup
>
static int doExtensionTrivial (UNU char *progName, int pinBase, UNU char *params)
{
  trivialSetup (pinBase) ;

  return TRUE ;
}


/*
 * doExtensionI2C<
 *   i2cSetup; I2C extension setup function reference
 *   i2cMin; Minimum I2C address
 *   i2cMax; Maximum I2C address
 * >:
 *	Extracts an integer parameter, verifies i2c address then calls i2cSetup
 *********************************************************************************
 */

template <
  i2c_setup_t &i2cSetup,
  const int i2cMin,
  const int i2cMax
>
static int doExtensionI2C (char *progName, int pinBase, char *params)
{
  int i2c ;

  if ((params = extractInt (progName, params, &i2c)) == NULL)
    return FALSE ;

  if ((i2c < i2cMin) || (i2c > i2cMax))
  {
    verbError ("%s: i2c address (0x%X) out of range", progName, i2c) ;
    return FALSE ;
  }

  i2cSetup (pinBase, i2c) ;

  return TRUE ;
}


/*
 * doExtensionSPI<
 *   spiSetup; SPI extension setup function reference
 *   spiMin; Minimum SPI address
 *   spiMax; Maximum SPI address
 * >:
 *	Extracts an integer parameter, verifies spi channel then calls spiSetup
 *********************************************************************************
 */

template <
  spi_setup_t &spiSetup,
  const int spiMin,
  const int spiMax
>
static int doExtensionSPI (char *progName, int pinBase, char *params)
{
  int spi ;

  if ((params = extractInt (progName, params, &spi)) == NULL)
    return FALSE ;

  if ((spi < spiMin) || (spi > spiMax))
  {
    verbError ("%s: SPI channel (%d) out of range", progName, spi) ;
    return FALSE ;
  }

  spiSetup (pinBase, spi) ;

  return TRUE ;
}


/*
 * doExtensionSPIPort<
 *   spiPortSetup; SPI with port extension setup function reference
 *   spiMin; Minimum SPI address
 *   spiMax; Maximum SPI address
 *   portMin; Minimum port address
 *   portMax; Maximum port address
 * >:
 *	Extracts an integer parameter, verifies spi channel and port then calls
 *  spiPortSetup
 *********************************************************************************
 */

template <
  spi_port_setup_t &spiPortSetup,
  const int spiMin,
  const int spiMax,
  const int portMin,
  const int portMax
>
static int doExtensionSPIPort (char *progName, int pinBase, char *params)
{
  int spi, port ;

  if ((params = extractInt (progName, params, &spi)) == NULL)
    return FALSE ;

  if ((spi < spiMin) || (spi > spiMax))
  {
    verbError ("%s: SPI address (%d) out of range", progName, spi) ;
    return FALSE ;
  }

  if ((params = extractInt (progName, params, &port)) == NULL)
    return FALSE ;

  if ((port < portMin) || (port > portMax))
  {
    verbError ("%s: port address (%d) out of range", progName, port) ;
    return FALSE ;
  }

  spiPortSetup (pinBase, spi, port) ;

  return TRUE ;
}


/*
 * doExtensionSr595:
 *	Shift Register 74x595
 *	sr595:base:pins:data:clock:latch
 *********************************************************************************
 */

static int doExtensionSr595 (char *progName, int pinBase, char *params)
{
  int pins, data, clock, latch ;

// Extract pins

  if ((params = extractInt (progName, params, &pins)) == NULL)
    return FALSE ;

  if ((pins < 8) || (pins > 32))
  {
    verbError ("%s: pin count (%d) out of range - 8-32 expected.", progName, pins) ;
    return FALSE ;
  }

  if ((params = extractInt (progName, params, &data)) == NULL)
    return FALSE ;

  if ((params = extractInt (progName, params, &clock)) == NULL)
    return FALSE ;

  if ((params = extractInt (progName, params, &latch)) == NULL)
    return FALSE ;

  sr595Setup (pinBase, pins, data, clock, latch) ;

  return TRUE ;
}


/*
 * doExtensionDs18b20:
 *	1-Wire Temperature
 *	htu21d:base:serialNum
 *********************************************************************************
 */

static int doExtensionDs18b20 (char *progName, int pinBase, char *params)
{
  char *serialNum ;

  if ((params = extractStr (progName, params, &serialNum)) == NULL)
    return FALSE ;

  return ds18b20Setup (pinBase, serialNum) ;
}


/*
 * doExtensionRht03:
 *	Maxdetect 1-Wire Temperature & Humidity
 *	rht03:base:piPin
 *********************************************************************************
 */

static int doExtensionRht03 (char *progName, int pinBase, char *params)
{
  int piPin ;

  if ((params = extractInt (progName, params, &piPin)) == NULL)
    return FALSE ;

  return rht03Setup (pinBase, piPin) ;
}


/*
 * doExtensionMcp3422:
 *	Analog IO
 *	mcp3422:base:i2cAddr
 *********************************************************************************
 */

static int doExtensionMcp3422 (char *progName, int pinBase, char *params)
{
  int i2c, sampleRate, gain ;

  if ((params = extractInt (progName, params, &i2c)) == NULL)
    return FALSE ;

  if ((i2c < 0x03) || (i2c > 0x77))
  {
    verbError ("%s: i2c address (0x%X) out of range", progName, i2c) ;
    return FALSE ;
  }

  if ((params = extractInt (progName, params, &sampleRate)) == NULL)
    return FALSE ;

  if ((sampleRate < 0) || (sampleRate > 3))
  {
    verbError ("%s: sample rate (%d) out of range", progName, sampleRate) ;
    return FALSE ;
  }

  if ((params = extractInt (progName, params, &gain)) == NULL)
    return FALSE ;

  if ((gain < 0) || (gain > 3))
  {
    verbError ("%s: gain (%d) out of range", progName, gain) ;
    return FALSE ;
  }

  mcp3422Setup (pinBase, i2c, sampleRate, gain) ;

  return TRUE ;
}


/*
 * doExtensionDrcS:
 *	Interface to a DRC Serial system
 *	drcs:base:pins:serialPort:baud
 *********************************************************************************
 */

static int doExtensionDrcS (char *progName, int pinBase, char *params)
{
  char *port ;
  int pins, baud ;

  if ((params = extractInt (progName, params, &pins)) == NULL)
    return FALSE ;

  if ((pins < 1) || (pins > 1000))
  {
    verbError ("%s: pins (%d) out of range (2-1000)", progName, pins) ;
    return FALSE ;
  }

  if ((params = extractStr (progName, params, &port)) == NULL)
    return FALSE ;

  if (strlen (port) == 0)
  {
    verbError ("%s: serial port device name required", progName) ;
    return FALSE ;
  }

  if ((params = extractInt (progName, params, &baud)) == NULL)
    return FALSE ;

  if ((baud < 1) || (baud > 4000000))
  {
    verbError ("%s: baud rate (%d) out of range", progName, baud) ;
    return FALSE ;
  }

  drcSetupSerial (pinBase, pins, port, baud) ;

  return TRUE ;
}


/*
 * doExtensionDrcNet:
 *	Interface to a DRC Network system
 *	drcn:base:pins:ipAddress:port:password
 *********************************************************************************
 */

static int doExtensionDrcNet (char *progName, int pinBase, char *params)
{
  int pins ;
  char *ipAddress, *port, *password ;
  char pPort [1024] ;

  if ((params = extractInt (progName, params, &pins)) == NULL)
    return FALSE ;

  if ((pins < 1) || (pins > 1000))
  {
    verbError ("%s: pins (%d) out of range (2-1000)", progName, pins) ;
    return FALSE ;
  }

  if ((params = extractStr (progName, params, &ipAddress)) == NULL)
    return FALSE ;

  if (strlen (ipAddress) == 0)
  {
    verbError ("%s: ipAddress required", progName) ;
    return FALSE ;
  }

  if ((params = extractStr (progName, params, &port)) == NULL)
    return FALSE ;

  if (strlen (port) == 0)
  {
    sprintf (pPort, "%d", DEFAULT_SERVER_PORT) ;
    port = pPort ;
  }

  if ((params = extractStr (progName, params, &password)) == NULL)
    return FALSE ;

  if (strlen (password) == 0)
  {
    verbError ("%s: password required", progName) ;
    return FALSE ;
  }

  return drcSetupNet (pinBase, pins, ipAddress, port, password) ;
}



/*
 * Function list
 *********************************************************************************
 */

static struct extensionFunctionStruct extensionFunctions [] =
{
  { "mcp23008",		&doExtensionI2C < mcp23008Setup, 0x01, 0x77 > 		},
  { "mcp23016",		&doExtensionI2C < mcp23016Setup, 0x03, 0x77 > 		},
  { "mcp23017",		&doExtensionI2C < mcp23017Setup, 0x03, 0x77 > 		},
  { "mcp23s08",		&doExtensionSPIPort < mcp23s08Setup, 0, 1, 0, 7 > 	},
  { "mcp23s17",		&doExtensionSPIPort < mcp23s17Setup, 0, 1, 0, 7 > 	},
  { "sr595",		&doExtensionSr595					},
  { "pcf8574",		&doExtensionI2C < pcf8574Setup, 0x03, 0x77 >		},
  { "pcf8591",		&doExtensionI2C < pcf8591Setup, 0x03, 0x77 >		},
  { "bmp180",		&doExtensionTrivial < bmp180Setup >			},
  { "pseudoPins",	&doExtensionTrivial < pseudoPinsSetup >			},
  { "htu21d",		&doExtensionTrivial < htu21dSetup >			},
  { "ds18b20",		&doExtensionDs18b20					},
  { "rht03",		&doExtensionRht03					},
  { "mcp3002",		&doExtensionSPI < mcp3002Setup, 0, 1 >			},
  { "mcp3004",		&doExtensionSPI < mcp3004Setup, 0, 1 >			},
  { "mcp4802",		&doExtensionSPI < mcp4802Setup, 0, 1 >			},
  { "mcp3422",		&doExtensionMcp3422					},
  { "max31855",		&doExtensionSPI < max31855Setup, 0, 1 >			},
  { "ads1115",		&doExtensionI2C < ads1115Setup, 0x03, 0x03 >		},
  { "max5322",		&doExtensionSPI < max5322Setup, 0, 1 >			},
  { "sn3218",		&doExtensionTrivial < sn3218Setup >			},
  { "drcs",		&doExtensionDrcS					},
  { "drcn",		&doExtensionDrcNet					},
  { NULL,		NULL		 					},
} ;


/*
 * loadWPiExtension:
 *	Load in a wiringPi extension
 *	The extensionData always starts with the name, a colon then the pinBase
 *	number. Other parameters after that are decoded by the module in question.
 *********************************************************************************
 */

int loadWPiExtension (char *progName, char *extensionData, int printErrors)
{
  char *p ;
  char *extension = extensionData ;
  struct extensionFunctionStruct *extensionFn ;
  unsigned pinBase = 0 ;

  verbose = printErrors ;

// Get the extension name by finding the first colon

  p = extension ;
  while (*p != ':')
  {
    if (!*p)	// ran out of characters
    {
      verbError ("%s: extension name not terminated by a colon", progName) ;
      return FALSE ;
    }
    ++p ;
  }
  *p++ = 0 ;

// Simple ATOI code

  if (!isdigit (*p))
  {
    verbError ("%s: decimal pinBase number expected after extension name", progName) ;
    return FALSE ;
  }

  while (isdigit (*p))
  {
    if (pinBase > 2147483647) // 2^31-1 ... Lets be realistic here...
    {
      verbError ("%s: pinBase too large", progName) ;
      return FALSE ;
    }

    pinBase = pinBase * 10 + (*p - '0') ;
    ++p ;
  }

  if (pinBase < 64)
  {
    verbError ("%s: pinBase (%d) too small. Minimum is 64.", progName, pinBase) ;
    return FALSE ;
  }

// Search for extensions:

  for (extensionFn = extensionFunctions ; extensionFn->name != NULL ; ++extensionFn)
  {
    if (strcmp (extensionFn->name, extension) == 0)
      return extensionFn->function (progName, pinBase, p) ;
  }

  fprintf (stderr, "%s: extension %s not found", progName, extension) ;
  return FALSE ;
}
