
      conf.pid[ROLL].P8     = 33;  conf.pid[ROLL].I8    = 30; conf.pid[ROLL].D8     = 23;
      conf.pid[PITCH].P8    = 33; conf.pid[PITCH].I8    = 30; conf.pid[PITCH].D8    = 23;
      conf.pid[PIDLEVEL].P8 = 90; conf.pid[PIDLEVEL].I8 = 10; conf.pid[PIDLEVEL].D8 = 100;

    conf.pid[YAW].P8      = 68;  conf.pid[YAW].I8     = 45;  conf.pid[YAW].D8     = 0;
    conf.pid[PIDALT].P8   = 64; conf.pid[PIDALT].I8   = 25; conf.pid[PIDALT].D8   = 24;

    conf.pid[PIDPOS].P8  = POSHOLD_P * 100;     conf.pid[PIDPOS].I8    = POSHOLD_I * 100;       conf.pid[PIDPOS].D8    = 0;
    conf.pid[PIDPOSR].P8 = POSHOLD_RATE_P * 10; conf.pid[PIDPOSR].I8   = POSHOLD_RATE_I * 100;  conf.pid[PIDPOSR].D8   = POSHOLD_RATE_D * 1000;
    conf.pid[PIDNAVR].P8 = NAV_P * 10;          conf.pid[PIDNAVR].I8   = NAV_I * 100;           conf.pid[PIDNAVR].D8   = NAV_D * 1000;
  
    conf.pid[PIDMAG].P8   = 40;

    conf.pid[PIDVEL].P8 = 0;      conf.pid[PIDVEL].I8 = 0;    conf.pid[PIDVEL].D8 = 0;

    conf.rcRate8 = 90; conf.rcExpo8 = 65;
    conf.rollPitchRate = 0;
    conf.yawRate = 0;
    conf.dynThrPID = 0;
    conf.thrMid8 = 50; conf.thrExpo8 = 0;
    for(i=0;i<CHECKBOXITEMS;i++) {conf.activate[i] = 0;}
    conf.angleTrim[0] = 0; conf.angleTrim[1] = 0;
    conf.powerTrigger1 = 0;
