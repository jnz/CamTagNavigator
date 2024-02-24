#pragma once

// Große Halbachse [m]:
#define ELLIP_a  ((double)( 6378137.0 ))

// Kleine Halbachse [m]:
#define ELLIP_b  ((double)( 6356752.3142 ))

// Polkrümmungshalbmesser c = a^2/b [m]:
// WGS84.c = 6399593.6258;
#define ELLIP_c  ( ELLIP_a*ELLIP_a/ELLIP_b )

// Erdschwere = Gravitationskonstante * Erdmasse [m^3 / s^2]:
// (Wellenhof 2008, S.432) (entspricht auch Takasu) (für GLONASS ect. ist Konstant anders)
#define ELLIP_GM  ((double)( 398600500000000.0 ))

// lineare Exzentrizität [m]:
// #define ELLIP_E  ((double)( 521854.0084 ))
#define ELLIP_E  ((double)sqrt(ELLIP_a*ELLIP_a - ELLIP_b*ELLIP_b))

// erste numerische Exzentrizität e2^2 = (E/a)^2:
// #define ELLIP_e2  ((double)( 0.00669437999014 ))
#define ELLIP_e2 ( (ELLIP_a*ELLIP_a - ELLIP_b*ELLIP_b)/(ELLIP_a*ELLIP_a) )

// zweite numerische Exzentrizität e_2^2 = (E/b)^2:
// #define ELLIP_e_2 ((double)( 0.00673949674228 ))
#define ELLIP_e_2 ( (ELLIP_a*ELLIP_a - ELLIP_b*ELLIP_b)/(ELLIP_b*ELLIP_b) )

// Rotationsgeschwindigkeit [rad / s]:
#define ELLIP_omega ((double)( 7.2921151467E-5 ))

// Normalbeschleunigung [m / s^2] bei B=00.0° und h=0:
#define ELLIP_gamma_a ((double)( 9.7803253359 ))

// Normalbeschleunigung [m / s^2] bei B=90.0° und h=0:
#define ELLIP_gamma_b ((double)( 9.8321849378 ))

// Abplattung
#define ELLIP_f  ((double)( 1.0/298.257223563 ))
// ELLIP_f = 1.0 - sqrt(1.0 - WGS84.e2);
