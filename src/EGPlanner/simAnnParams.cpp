#include "EGPlanner/simAnnParams.h"


SimAnnParams SimAnnParams::ANNEAL_DEFAULT()
{
    SimAnnParams params;

    params.YC = 7.0;
    params.HC = 7.0;
    params.YDIMS = 8;
    params.HDIMS = 8;
    params.NBR_ADJ = 1.0;
    params.ERR_ADJ = 1.0e-6;
    params.DEF_T0 = 1.0e6;
    params.DEF_K0 = 30000;

    return params;
}

SimAnnParams SimAnnParams::ANNEAL_LOOP()
{
    SimAnnParams params;

    params.YC = 7.0;
    params.HC = 7.0;
    params.YDIMS = 8;
    params.HDIMS = 8;
    params.NBR_ADJ = 1.0;
    params.ERR_ADJ = 1.0e-6;
    params.DEF_T0 = 1.0e6;
    params.DEF_K0 = 35000;

    return params;
}

SimAnnParams SimAnnParams::ANNEAL_MODIFIED()
{
    SimAnnParams params;

    params.YC = 2.38;
    params.YDIMS = 8;
    params.HC = 2.38;
    params.HDIMS = 8;
    params.NBR_ADJ = 1.0e-3;
    params.ERR_ADJ = 1.0e-1;
    params.DEF_T0 = 1.0e3;
    params.DEF_K0 = 0;

    return params;
}

SimAnnParams SimAnnParams::ANNEAL_STRICT()
{
    SimAnnParams params;

    params.YC = 0.72;
    params.HC = 0.22;
    params.YDIMS = 2;
    params.HDIMS = 2;
    params.NBR_ADJ = 1.0;
    params.ERR_ADJ = 1.0; //meant to work with ENERGY_STRICT_AUTOGRASP which multiplies eps. gq by 1.0e2
    params.DEF_T0 = 10.0;
    params.DEF_K0 = 0;

    return params;
}

SimAnnParams SimAnnParams::ANNEAL_ONLINE()
{
    SimAnnParams params;

    params.YC = 0.24;
    params.HC = 0.54;
    params.YDIMS = 2;
    params.HDIMS = 3;
    params.NBR_ADJ = 1.0;
    params.ERR_ADJ = 1.0e-2;
    params.DEF_T0 = 1.0e1;
    params.DEF_K0 = 100;

    return params;
}



