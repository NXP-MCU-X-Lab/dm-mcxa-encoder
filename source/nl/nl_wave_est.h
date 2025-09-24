// nl_wave_est.h
#ifndef NL_WAVE_EST_H
#define NL_WAVE_EST_H

#include "nl.h"
#include "nl_sin_frq_est.h"


/**
 * @brief Wave heave estimation structure
 * 
 * This structure contains state variables and intermediate results needed for wave heave estimation
 * including sway (x), surge (y), and heave (z) motion components
 */
typedef struct 
{
    nl_t dt;                     // Sample time interval (seconds)
    nl_t acc_n[3];              // Acceleration in navigation frame [x, y, z]
    nl_t acc_h[3];              // Acceleration in horizontal frame [x, y, z]

    // Three-axis wave motion estimation [0:Sway, 1:Surge, 2:Heave]
    nl_t acc_hp[3];             // High-pass filtered acceleration
    nl_t vel[3];                // Velocity (raw integration)
    nl_t vel_hp[3];             // High-pass filtered velocity
    nl_t displacement[3];       // Displacement estimate (raw integration)
    nl_t displacement_ap[3];    // Phase compensated displacement
    nl_t phase_to_correct[3];   // Phase correction for each axis
    nl_t envelope[3];           // Motion envelope for each axis
    nl_sin_frq_est_t sin_frq_est[3];
} nl_wave_est_t;

// Axis definitions for clarity
#define AXIS_SWAY   0   // X-axis (lateral motion)
#define AXIS_SURGE  1   // Y-axis (longitudinal motion)  
#define AXIS_HEAVE  2   // Z-axis (vertical motion)

#endif


