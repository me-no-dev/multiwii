/********************************************************************/
/****             spektrum satellite RX config                   ****/
/********************************************************************/

/* The following lines apply only for Spektrum Satellite Receiver
   Spektrum Satellites are 3V devices.  DO NOT connect to 5V!
   For MEGA boards, attach sat grey wire to RX1, pin 19. Sat black wire to ground. Sat orange wire to Mega board's 3.3V (or any other 3V to 3.3V source).
   For PROMINI, attach sat grey to RX0.  Attach sat black to ground.  
     There is no 3.3V source on a pro mini; you can either use a different 3V source, or attach orange to 5V with a 3V regulator in-line (such as http://search.digikey.com/scripts/DkSearch/dksus.dll?Detail&name=MCP1700-3002E/TO-ND)
     If you use an inline-regulator, a standard 3-pin servo connector can connect to ground, +5V, and RX0; solder the correct wires (and the 3V regulator!) to a Spektrum baseRX-to-Sat cable that has been cut in half. 
     NOTE: Because there is only one serial port on the Pro Mini, using a Spektrum Satellite implies you CANNOT use the PC based configuration tool. Further, you cannot use on-aircraft serial LCD as the baud rates are incompatible. You can configure by one of two methods:
       1) Use an on-aircraft i2c LCD (such as Eagle Tree or LCD03) for setting gains, reading sensors, etc.
       2) Available now: Comment out the Spektrum definition, upload, plug in PC, configure; uncomment the Spektrum definition, upload, plug in RX, and fly.  Repeat as required to configure. */
//#define SPEKTRUM 1024
#define SPEKTRUM 2048



/*       end of Settings for spektrum satellite RX                  */
/*------------------------------------------------------------------*/



/********************************************************************/
/****           spektrum satellite defines                       ****/
/********************************************************************/
#define SPECIAL_RX
#define SERIAL_RX
#define READ_SPECIAL_RX_WO_INT
#define READ_SPECIAL_RX_W_INT
#define SPEK_MAX_CHANNEL 7
#define SPEK_FRAME_SIZE 16
#if (SPEKTRUM == 1024)
  #define SPEK_CHAN_SHIFT  2       // Assumes 10 bit frames, that is 1024 mode.
  #define SPEK_CHAN_MASK   0x03    // Assumes 10 bit frames, that is 1024 mode.
#endif
#if (SPEKTRUM == 2048)
  #define SPEK_CHAN_SHIFT  3       // Assumes 11 bit frames, that is 2048 mode.
  #define SPEK_CHAN_MASK   0x07    // Assumes 11 bit frames, that is 2048 mode.
#endif










