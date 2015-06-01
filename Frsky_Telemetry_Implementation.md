# Introduction #

Frsky provides two series of receivers supporting telemetry.
The first protocol is implemented with the D-series receivers (D8R-II, D8R-XP) and it has been published.

The second protocol is implemented with the X-series receivers (X4R, X6R, X8R) and it has not been published. It is also called S.PORT.

Since [r1754](https://code.google.com/p/multiwii/source/detail?r=1754) the first Frsky telemetry protocol (D-series) is being integrated into the multiwii trunk. Two additional files Telemetry.cpp/h involve the code leaving room for further telemetry protocol implementations (e.g. S.PORT, HOTT, etc).

# Details #

**Hardware**
The D-series receivers (D8R-II, D8R-XP) have a side port with the pins **Rx**, **A1** and **A2**. You have to connect **Rx** via a level converter (additional hardware converting the signal) to the **Tx** pin of a free multiwii serial port.

Since flight controllers based upon promini or promicro controllers provide only one or two serials, which are most problably already in use, a mega controller with 4 serials is recommended. Experiments with smaller controllers using software serial failed due to performance issues.

The pins **A1** and **A2** provide the measurement of external (A1 also internal if connected to internal 3.3V via jumper) voltages. The voltages may not exceed 3.3V, therefore a resistor devider has to be used.

**Display**
The recommended easy way for displaying Frsky telemetry data is to buy the Frsky display FLD-02 directly to your transmitter module (DJT, DHT or DFT).

Alternatively, the telemetry data can be displayed on Er9x or OpenTx transmitters. But this requires specific hardware and software patches.

**Software**
In order to enable Frsky telemetry you have to enable it by uncommenting _#define FRSKY\_TELEMETRY_ in the file config.h.
The related serial has to be set up via _TELEMETRY\_SERIAL_, the default serial port used for telemetry is number 3.
That should be enough to make Frsky telemetry work.

By defining _COORDFORMAT\_DECIMALMINUTES_ you change the coordinate format (longitude and latitude) from decimal degrees (DD.dddddd°) to degrees+decimal minutes (DD°MM.mmmm).

With _FRSKY\_FLD02_ you make sure that only those data are transmitted, which also are used for the Frsky display FLD-02. Leave this commented out if you use the internal display of the Er9x or OpenTx transmitters.

With _TELEMETRY\_ALT\_BARO_ the barometric altitude relative to ground is displayed.
With _TELEMETRY\_ALT\_GPS_ the GPS altitude (relative to sea level) is displayed.
Don't use both definitions for a FLD-02, since it only can display one altitude.

With _TELEMETRY\_COURSE\_MAG_ or _TELEMETRY\_COURSE\_GPS_ the course (from compass or GPS) is displayed. FLD-02 will not display this value.

**FLD-02 Display**
On the first page following values are displayed:
  * arming time
  * fuel = battery capacity left (powermeter required)
  * internal voltage
  * battery voltage
  * temperature (baro required)
  * numbers of satellites (GPS required, instead of temperature2)
  * rssi

Second page:
  * altitude (either baro or GPS required)
  * RPM=Return distance to home Position in Meter (=distance to home instead of round per minute, GPS required)
  * speed (GPS required)
  * amperage (powermeter required)
  * capacity (internal FLD-02 calculation)

Third page: (only for cell voltage measurement via VBAT\_CELLS definition)
  * total voltage
  * voltage of each cell
Due to the current implementation of cell voltage measurement there is only a limited resolution of one decimal (e.g. 3.9V).

Fourth page:
  * acceleration i x-, y-, and z-axis
  * latitude
  * longitude

# Remarks #
My performance measurement have not shown any negative impact neither to the loop time nor to the functionality. The telemetry routine uses an own scheduler and runs within the annexCode time frame of 650us, which is not exceeded.

Nevertheless, the code is still under development. That means it has not been fully tested. Every use is on your own risk. (But I have been using the previous software since more than one year without issues.)

You are invited to provide your remarks and proposals for the further development of Frsky telemetry. You are also invited to propose the implementations of other telemetry protocols which could be hosted in the new files Telemetry.cpp/h.