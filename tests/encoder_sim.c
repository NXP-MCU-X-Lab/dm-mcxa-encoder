/*
 * Host-side simulation harness for the dual-track inductive encoder algorithm.
 *
 * Drives the REAL source/app_encoder.c -- the shipping algorithm, not a
 * reimplementation -- with a synthetic track signal model. Every number printed
 * here therefore describes the firmware code path, not a model of it.
 *
 * Build:
 *   gcc -O2 -I../source -o encoder_sim encoder_sim.c \
 *       ../source/app_encoder.c ../source/app_encoder_defaults.c -lm
 *
 * Modes:
 *   slip  sweep harmonic distortion, report branch-slip rate and angle error
 *   sep   dump the 16/15 cross-residual per config, for reference-free INL analysis
 *   inl   dump angle error vs rotor angle for one config
 */

#include "app_encoder.h"
#include "app_adc.h"   /* ADC_SAMPLE_RATE_HZ -- same source the firmware uses */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PI_D 3.14159265358979323846
#define RAD (PI_D / 180.0)

/* ---------- reproducible gaussian noise ---------- */

typedef struct { unsigned long long s; } rng_t;

static double rng_uniform(rng_t *r)
{
    r->s ^= r->s << 13; r->s ^= r->s >> 7; r->s ^= r->s << 17;
    return (double)((r->s >> 11) & 0x1FFFFFFFFFFFFFULL) / 9007199254740992.0;
}

static double rng_gauss(rng_t *r)
{
    double u1 = rng_uniform(r), u2 = rng_uniform(r);
    if (u1 < 1e-300) u1 = 1e-300;
    return sqrt(-2.0 * log(u1)) * cos(2.0 * PI_D * u2);
}

/* ---------- signal model ---------- */

/* Defaults are the measured HW_V2 board values from app_encoder_defaults.c:
 * amp = 1/t, center as stored. A1 (16-cycle, internal OPAMP0/1) has a 2:1
 * sin/cos gain mismatch; A2 (15-cycle, external TLV9062) is matched to ~1%. */
typedef struct {
    double amp_sin, amp_cos;
    double center_sin, center_cos;
    double quad_err_deg;              /* cos channel deviation from exact 90 deg */
    double h2, h2_phase_deg;          /* harmonic amplitude relative to fundamental */
    double h3, h3_phase_deg;
} track_model_t;

typedef struct {
    track_model_t a1, a2;             /* a1 = 16 cycles/rev, a2 = 15 cycles/rev */
    double ecc_mech_deg;              /* eccentricity: mechanical angle error amplitude */
    double ecc_phase_deg;
    double noise_lsb;
    double skew_us;                   /* a2 is sampled this much after a1 */
    double speed_dps;                 /* rotation speed while running */
    /* Calibration rotation is a separate condition from running: it is what the
     * operator (or a production fixture) does during the capture window, and it
     * turns out to matter -- see mode_calbias. */
    double cal_speed_dps;
    double cal_jitter_frac;           /* per-sample speed variation, 0 = perfectly constant */
} sim_model_t;

static void model_defaults(sim_model_t *m)
{
    memset(m, 0, sizeof(*m));
    m->a1.amp_sin = 1.0 / 0.000183; m->a1.amp_cos = 1.0 / 0.000092;
    m->a1.center_sin = 11744.0;      m->a1.center_cos = 22415.5;
    m->a1.quad_err_deg = 0.5;
    m->a2.amp_sin = 1.0 / 0.000083;  m->a2.amp_cos = 1.0 / 0.000082;
    m->a2.center_sin = 21819.0;      m->a2.center_cos = 21027.0;
    m->a2.quad_err_deg = 0.1;
    m->noise_lsb = 3.0;
    m->skew_us = 3.9;
    m->speed_dps = 3600.0;            /* 600 rpm */
    /* About one slow manual revolution during the 8.192 s capture. */
    m->cal_speed_dps = 48.0;          /* 8 rpm */
    m->cal_jitter_frac = 0.05;
}

/* One shared periodic waveform per track: the cos channel is the same coil
 * shape shifted 90 deg, so harmonics appear coherently on both channels --
 * which is exactly why they survive ellipse correction as phase error. */
static double track_wave(const track_model_t *m, double phi_deg)
{
    return sin(phi_deg * RAD)
         + m->h2 * sin(2.0 * phi_deg * RAD + m->h2_phase_deg * RAD)
         + m->h3 * sin(3.0 * phi_deg * RAD + m->h3_phase_deg * RAD);
}

static unsigned short quantize(double v, double noise, rng_t *rng)
{
    double x = v + rng_gauss(rng) * noise;
    if (x < 0.0) x = 0.0;
    if (x > 65535.0) x = 65535.0;
    return (unsigned short)(x + 0.5);
}

static void track_sample(const track_model_t *m, double phi_deg, double noise,
                         rng_t *rng, unsigned short *sin_raw, unsigned short *cos_raw)
{
    *sin_raw = quantize(m->center_sin + m->amp_sin * track_wave(m, phi_deg), noise, rng);
    *cos_raw = quantize(m->center_cos + m->amp_cos * track_wave(m, phi_deg + 90.0 + m->quad_err_deg),
                        noise, rng);
}

/* Eccentricity is a mechanical angle error, so it reaches each track's
 * electrical phase multiplied by that track's cycles/rev. */
static double ecc_offset(const sim_model_t *m, double theta_deg)
{
    return m->ecc_mech_deg * sin((theta_deg + m->ecc_phase_deg) * RAD);
}

static void make_sample(const sim_model_t *m, double theta_deg, double speed_dps,
                        rng_t *rng, encoder_raw_sample_t *out)
{
    double theta_a2 = theta_deg + speed_dps * (m->skew_us * 1e-6);
    double phi16 = 16.0 * (theta_deg + ecc_offset(m, theta_deg));
    double phi15 = 15.0 * (theta_a2  + ecc_offset(m, theta_a2));

    track_sample(&m->a1, phi16, m->noise_lsb, rng, &out->a1_sin_raw, &out->a1_cos_raw);
    track_sample(&m->a2, phi15, m->noise_lsb, rng, &out->a2_sin_raw, &out->a2_cos_raw);
}

/* ---------- helpers mirroring the firmware angle conventions ---------- */

static double wrapd(double d) { d = fmod(d, 360.0); return d < 0.0 ? d + 360.0 : d; }

static double serrd(double a, double e)
{
    double d = wrapd(a - e);
    return d > 180.0 ? d - 360.0 : d;
}

/* ---------- run the real firmware calibration path ---------- */

/* Mirrors app_encoder_runtime.c: 1 kHz decimated capture of
 * ENCODER_CAL_SAMPLE_COUNT samples, then solve, then zero on the LAST sample. */
static int run_factory_cal(const sim_model_t *m, rng_t *rng, const encoder_inl_t *inl,
                           encoder_calibration_t *cal, double *theta_zero)
{
    encoder_cal_stats_t stats;
    encoder_raw_sample_t s, last;
    unsigned int i;
    uint32_t status = 0;
    /* firmware decimates 10 kHz -> 1 kHz, so each cal sample is 10 ADC periods */
    double dtheta_nom = m->cal_speed_dps * (10.0 / (double)ADC_SAMPLE_RATE_HZ);
    double theta = 0.0;
    double last_step = dtheta_nom;

    memset(&last, 0, sizeof(last));
    encoder_cal_stats_init(&stats);
    for (i = 0U; i < ENCODER_CAL_SAMPLE_COUNT; i++) {
        make_sample(m, theta, m->cal_speed_dps, rng, &s);
        encoder_cal_stats_accumulate(&stats, &s);
        last = s;
        last_step = dtheta_nom * (1.0 + m->cal_jitter_frac * rng_gauss(rng));
        theta += last_step;
    }
    *theta_zero = theta - last_step;

    if (!encoder_cal_stats_build(&stats, cal, &status)) {
        fprintf(stderr, "factory cal solve FAILED, status=0x%X\n", (unsigned)status);
        return 0;
    }
    if (!encoder_capture_zero(cal, inl, &last, &status)) {
        fprintf(stderr, "zero capture FAILED, status=0x%X\n", (unsigned)status);
        return 0;
    }
    return 1;
}

typedef struct {
    unsigned long samples;
    unsigned long slips;
    double max_abs_err;
    double rms_err;
    unsigned long status_flagged;   /* samples where the firmware raised ANY status bit */
    unsigned long mismatch_flagged; /* samples where TRACK_MISMATCH specifically fired */
    /* |fine_delta| is the real branch margin indicator: it reaches 180 deg exactly
     * when the coarse estimate sits on a branch boundary. Reporting its peak shows
     * how much margin an error source consumes, independent of whether it slipped. */
    double max_abs_fine_delta;
} run_stats_t;

/* Slip threshold: 40% of one fine branch (22.5 deg), same criterion used in the
 * review Monte Carlo. Well above any harmonic-induced smooth error. */
#define SLIP_THRESHOLD_DEG (0.4 * (360.0 / (double)ENCODER_TRACK16_CYCLES))

static void run_encoder(const sim_model_t *m, rng_t *rng,
                        const encoder_calibration_t *cal, const encoder_inl_t *inl,
                        double theta_zero,
                        unsigned long n_samples, run_stats_t *st,
                        FILE *csv, const char *csv_tag)
{
    encoder_state_t state;
    encoder_result_t res;
    encoder_raw_sample_t s;
    double theta = theta_zero;
    double dtheta = m->speed_dps / (double)ADC_SAMPLE_RATE_HZ;
    double sum_sq = 0.0;
    unsigned long i;

    memset(st, 0, sizeof(*st));
    encoder_state_init(&state);

    for (i = 0UL; i < n_samples; i++) {
        double truth, err, fine_delta;

        make_sample(m, theta, m->speed_dps, rng, &s);
        encoder_process(&state, cal, inl, &s, &res, NULL);

        truth = wrapd(theta - theta_zero);
        err = serrd(res.angle_deg_raw, truth);
        /* cross-residual, recomputed from published phases (no firmware change) */
        fine_delta = serrd(res.phase16_deg, wrapd(res.coarse_deg * 16.0));

        st->samples++;
        if (fabs(err) > SLIP_THRESHOLD_DEG) st->slips++;
        if (fabs(err) > st->max_abs_err) st->max_abs_err = fabs(err);
        sum_sq += err * err;
        if (res.status != ENCODER_STATUS_OK) st->status_flagged++;
        if ((res.status & ENCODER_STATUS_TRACK_MISMATCH) != 0UL) st->mismatch_flagged++;
        if (fabs(fine_delta) > st->max_abs_fine_delta) st->max_abs_fine_delta = fabs(fine_delta);

        if (csv != NULL) {
            fprintf(csv, "%s,%.6f,%.6f,%.6f\n", csv_tag, truth, fine_delta, err);
        }
        theta += dtheta;
    }
    st->rms_err = sqrt(sum_sq / (double)(st->samples ? st->samples : 1UL));
}

/* ---------- modes ---------- */

static void mode_slip(void)
{
    static const double h3_sweep[] = { 0.000, 0.005, 0.010, 0.015, 0.020, 0.030, 0.040, 0.060 };
    unsigned int k;

    printf("# branch-slip vs 3rd-harmonic distortion (both tracks), real encoder_process()\n");
    printf("# board-measured front-end asymmetry, 3 LSB ADC noise, 3.9 us A1->A2 skew, 600 rpm\n");
    printf("%-8s %-10s %-12s %-12s %-12s %-13s %s\n",
           "h3", "slips", "slips/s", "max_err", "rms_err", "peak|fdelta|", "margin_used");

    for (k = 0U; k < sizeof(h3_sweep) / sizeof(h3_sweep[0]); k++) {
        sim_model_t m;
        encoder_calibration_t cal;
        run_stats_t st;
        rng_t rng;
        double theta_zero;
        unsigned long n = 200000UL;   /* 20 s at 10 kHz */

        rng.s = 0x243F6A8885A308D3ULL;
        model_defaults(&m);
        m.a1.h3 = h3_sweep[k]; m.a1.h3_phase_deg = 20.0;
        m.a2.h3 = h3_sweep[k]; m.a2.h3_phase_deg = -35.0;
        m.a1.h2 = 0.5 * h3_sweep[k]; m.a2.h2 = 0.5 * h3_sweep[k];

        if (!run_factory_cal(&m, &rng, NULL, &cal, &theta_zero)) continue;
        run_encoder(&m, &rng, &cal, NULL, theta_zero, n, &st, NULL, NULL);

        printf("%-8.3f %-10lu %-12.1f %-12.3f %-12.4f %-13.1f %.0f%%\n",
               h3_sweep[k], st.slips,
               (double)st.slips * (double)ADC_SAMPLE_RATE_HZ / (double)n,
               st.max_abs_err, st.rms_err, st.max_abs_fine_delta,
               100.0 * st.max_abs_fine_delta / 180.0);
    }
}

/* Four configs isolating each error source, so the FFT of the cross-residual
 * shows unambiguously where each one lands in mechanical order. */
static void mode_sep(void)
{
    /* "clean" and "asym_only" are the noise floor references: clean has a
     * perfectly matched front end and nothing injected, so anything it shows is
     * an artifact of the pipeline itself rather than of an injected error. */
    static const struct { const char *tag; int matched, t1h, t2h, ecc; } cfg[] = {
        { "clean",       1, 0, 0, 0 },
        { "asym_only",   0, 0, 0, 0 },
        { "track1_only", 0, 1, 0, 0 },
        { "track2_only", 0, 0, 1, 0 },
        { "ecc_only",    0, 0, 0, 1 },
        { "all",         0, 1, 1, 1 },
    };
    unsigned int k;

    printf("config,theta_deg,fine_delta_deg,angle_err_deg\n");
    for (k = 0U; k < sizeof(cfg) / sizeof(cfg[0]); k++) {
        sim_model_t m;
        encoder_calibration_t cal;
        run_stats_t st;
        rng_t rng;
        double theta_zero;

        rng.s = 0x13198A2E03707344ULL;
        model_defaults(&m);
        m.noise_lsb = 0.0;      /* isolate deterministic content */
        m.skew_us = 0.0;
        m.speed_dps = 3600.0;   /* 600 rpm -> exactly 1000 samples per rev at 10 kHz */
        if (cfg[k].matched) {
            m.a1.amp_sin = m.a1.amp_cos = 16000.0;
            m.a1.center_sin = m.a1.center_cos = 32768.0;
            m.a1.quad_err_deg = 0.0;
            m.a2.amp_sin = m.a2.amp_cos = 16000.0;
            m.a2.center_sin = m.a2.center_cos = 32768.0;
            m.a2.quad_err_deg = 0.0;
        }
        if (cfg[k].t1h) { m.a1.h3 = 0.02; m.a1.h3_phase_deg = 20.0; }
        if (cfg[k].t2h) { m.a2.h3 = 0.02; m.a2.h3_phase_deg = -35.0; }
        if (cfg[k].ecc) { m.ecc_mech_deg = 0.05; m.ecc_phase_deg = 70.0; }

        if (!run_factory_cal(&m, &rng, NULL, &cal, &theta_zero)) continue;
        /* 8 full revolutions, uniformly sampled in theta */
        run_encoder(&m, &rng, &cal, NULL, theta_zero, 8000UL, &st, stdout, cfg[k].tag);
    }
}

/* For a perfect encoder fine_delta is identically zero, so with a clean model any
 * residual measures calibration error alone. build_track_calibration() takes the
 * ellipse centre from min/max of the captured samples; when the calibration
 * rotation speed is commensurate with the decimated sample rate the rotor only
 * ever visits a few distinct electrical phases, min/max never reach the true
 * extremes, and the centre is biased. The conic fit cannot see this because it
 * treats the supplied centre as exact. */
static void mode_calbias(void)
{
    static const struct { double rpm; double jitter; } cond[] = {
        { 300.0, 0.00 }, { 600.0, 0.00 }, { 900.0, 0.00 }, { 1200.0, 0.00 },
        { 601.0, 0.00 }, { 613.7, 0.00 }, { 1237.3, 0.00 },
        { 600.0, 0.01 }, { 600.0, 0.05 }, { 600.0, 0.20 },
    };
    unsigned int k;

    printf("# calibration centre bias: clean model, perfect front end, zero noise\n");
    printf("# fine_delta is identically 0 for an ideal encoder, so every value below is cal error\n");
    printf("%-10s %-14s %-16s %-14s %s\n",
           "cal_rpm", "distinct_ph", "peak|fdelta|", "margin_used", "max_angle_err");

    for (k = 0U; k < sizeof(cond) / sizeof(cond[0]); k++) {
        sim_model_t m;
        encoder_calibration_t cal;
        run_stats_t st;
        rng_t rng;
        double theta_zero, elec_step, frac;
        long distinct;

        rng.s = 0x13198A2E03707344ULL;
        model_defaults(&m);
        m.noise_lsb = 0.0;
        m.skew_us = 0.0;
        m.a1.amp_sin = m.a1.amp_cos = 16000.0;
        m.a1.center_sin = m.a1.center_cos = 32768.0;
        m.a1.quad_err_deg = 0.0;
        m.a2.amp_sin = m.a2.amp_cos = 16000.0;
        m.a2.center_sin = m.a2.center_cos = 32768.0;
        m.a2.quad_err_deg = 0.0;
        m.speed_dps = 3600.0;                    /* run condition held fixed */
        m.cal_speed_dps = cond[k].rpm * 6.0;      /* only the calibration condition varies */
        m.cal_jitter_frac = cond[k].jitter;

        /* electrical phase advance per decimated cal sample, track 1 */
        elec_step = 16.0 * m.cal_speed_dps * (10.0 / (double)ADC_SAMPLE_RATE_HZ);
        frac = fmod(elec_step, 360.0) / 360.0;
        /* crude count of distinct phases visited: denominator of frac as a fraction */
        distinct = 0;
        {
            long d;
            for (d = 1; d <= 20000; d++) {
                double v = frac * (double)d;
                if (fabs(v - floor(v + 0.5)) < 1e-9) { distinct = d; break; }
            }
            if (distinct == 0) distinct = -1;   /* effectively dense */
        }

        if (!run_factory_cal(&m, &rng, NULL, &cal, &theta_zero)) continue;
        run_encoder(&m, &rng, &cal, NULL, theta_zero, 8000UL, &st, NULL, NULL);

        if (cond[k].jitter > 0.0) distinct = -1;   /* jitter destroys commensurability */
        if (distinct > 0)
            printf("%-10.1f %-9.2f %-13ld %-15.3f %-14.0f%% %.4f\n",
                   cond[k].rpm, cond[k].jitter, distinct, st.max_abs_fine_delta,
                   100.0 * st.max_abs_fine_delta / 180.0, st.max_abs_err);
        else
            printf("%-10.1f %-9.2f %-13s %-15.3f %-14.0f%% %.4f\n",
                   cond[k].rpm, cond[k].jitter, "dense", st.max_abs_fine_delta,
                   100.0 * st.max_abs_fine_delta / 180.0, st.max_abs_err);
    }
}

/* What does the calibration solver actually recover, versus the model truth?
 * Separates "the fit is biased" from "the fit is fine but the signal is not an
 * ellipse", which the slip sweep alone cannot distinguish. */
static void mode_calcheck(void)
{
    static const double h3_sweep[] = { 0.000, 0.001, 0.002, 0.003, 0.005, 0.010, 0.020 };
    unsigned int k;

    printf("# recovered vs true track-A1 calibration, board front end, cal at 613.7 rpm + 5%% jitter\n");
    printf("%-8s %-14s %-14s %-14s %-14s\n",
           "h3", "d_center_sin", "d_center_cos", "amp_sin_err%", "amp_cos_err%");

    for (k = 0U; k < sizeof(h3_sweep) / sizeof(h3_sweep[0]); k++) {
        sim_model_t m;
        encoder_calibration_t cal;
        rng_t rng;
        double theta_zero;

        rng.s = 0x243F6A8885A308D3ULL;
        model_defaults(&m);
        m.a1.h3 = h3_sweep[k]; m.a1.h3_phase_deg = 20.0;
        m.a2.h3 = h3_sweep[k]; m.a2.h3_phase_deg = -35.0;
        m.a1.h2 = 0.5 * h3_sweep[k]; m.a2.h2 = 0.5 * h3_sweep[k];

        if (!run_factory_cal(&m, &rng, NULL, &cal, &theta_zero)) continue;

        /* t00 and t11 invert the per-axis amplitude when the ellipse is near
         * axis-aligned, which it is here (t10 << t00). */
        printf("%-8.3f %-14.1f %-14.1f %-14.2f %-14.2f\n",
               h3_sweep[k],
               (double)cal.a1.center_sin - m.a1.center_sin,
               (double)cal.a1.center_cos - m.a1.center_cos,
               100.0 * ((1.0 / (double)cal.a1.t00) - m.a1.amp_sin) / m.a1.amp_sin,
               100.0 * ((1.0 / (double)cal.a1.t11) - m.a1.amp_cos) / m.a1.amp_cos);
    }
}

/* Does the reference-free harmonic estimator converge, and how much branch margin
 * does it hand back? The correction is evaluated at the measured phase rather than
 * the true one, so each pass leaves a second-order residual -- iterating shows
 * whether that shrinks geometrically or oscillates. */
/* Refine an INL table by one pass: measure the residual THROUGH the table
 * currently in force, then add the newly measured correction on top. Measuring
 * with the table disabled would just recompute the same first-pass answer
 * forever. Two passes reach the floor; a third adds nothing. */
static int refine_inl_pass(const sim_model_t *m, rng_t *rng,
                           const encoder_calibration_t *cal, double theta_zero,
                           unsigned long n, encoder_inl_t *inl)
{
    static encoder_inl_t residual;
    encoder_state_t state;
    encoder_result_t res;
    encoder_diag_t dg;
    encoder_raw_sample_t s;
    double theta = theta_zero;
    const double dtheta = m->speed_dps / (double)ADC_SAMPLE_RATE_HZ;
    unsigned int bin;
    unsigned long i;

    encoder_inl_init(&residual);
    encoder_state_init(&state);
    for (i = 0UL; i < n; i++) {
        make_sample(m, theta, m->speed_dps, rng, &s);
        encoder_process(&state, cal, inl->valid ? inl : NULL, &s, &res, &dg);
        encoder_inl_accumulate(&residual, &dg, &res);
        theta += dtheta;
    }
    if (!encoder_inl_solve(&residual)) return 0;

    for (bin = 0U; bin < ENCODER_INL_BINS; bin++) {
        inl->t16.correction[bin] += residual.t16.correction[bin];
        inl->t15.correction[bin] += residual.t15.correction[bin];
    }
    inl->valid = true;
    return 1;
}

static void mode_inlfit(double h3)
{
    static encoder_inl_t inl;          /* ~1.5 KB, kept off the stack */
    sim_model_t m;
    encoder_calibration_t cal;
    run_stats_t st;
    rng_t rng;
    double theta_zero;
    unsigned int pass;

    rng.s = 0x243F6A8885A308D3ULL;
    model_defaults(&m);
    m.a1.h3 = h3; m.a1.h3_phase_deg = 20.0;
    m.a2.h3 = h3; m.a2.h3_phase_deg = -35.0;
    m.a1.h2 = 0.5 * h3; m.a2.h2 = 0.5 * h3;
    m.ecc_mech_deg = 0.05; m.ecc_phase_deg = 70.0;

    encoder_inl_init(&inl);
    if (!run_factory_cal(&m, &rng, NULL, &cal, &theta_zero)) return;

    printf("# reference-free INL convergence, h3=%.1f%% h2=%.1f%% both tracks, ecc 0.05 deg\n",
           100.0 * h3, 50.0 * h3);
    printf("%-6s %-13s %-13s %-14s %-12s %s\n",
           "pass", "max_err_deg", "rms_err_deg", "peak|fdelta|", "margin_used", "slips");

    for (pass = 0U; pass <= 3U; pass++)
    {
        const unsigned long n = 200000UL;   /* 20 s at 10 kHz, ~333 revolutions */

        /* Measure with whatever table we have so far. */
        run_encoder(&m, &rng, &cal, inl.valid ? &inl : NULL, theta_zero, n, &st, NULL, NULL);
        printf("%-6u %-13.4f %-13.4f %-14.1f %-12.0f%% %lu\n",
               pass, st.max_abs_err, st.rms_err, st.max_abs_fine_delta,
               100.0 * st.max_abs_fine_delta / 180.0, st.slips);

        if (pass == 3U) break;

        if (!refine_inl_pass(&m, &rng, &cal, theta_zero, n, &inl)) {
            printf("  solve FAILED (thin bin coverage)\n");
            return;
        }
    }
}

/* ENCODER_BRANCH_CONFIDENCE_LIMIT_DEG guards the branch decision by tripping
 * TRACK_MISMATCH when |fine_delta| approaches the branch boundary. But
 * |fine_delta| also grows with uncorrected harmonic distortion, so the threshold
 * and the board's harmonic content are coupled: past some h3 the guard starts
 * firing on healthy samples, and the encoder freezes intermittently instead of
 * slipping. This maps that boundary with and without the INL table. */
static void mode_faultmargin(void)
{
    static const double h3_sweep[] = { 0.000, 0.010, 0.020, 0.030, 0.040, 0.050, 0.060 };
    unsigned int k;

    printf("# TRACK_MISMATCH false-trip margin vs harmonic distortion\n");
    printf("# guard trips at |fine_delta| > %.0f deg\n",
           (double)ENCODER_BRANCH_CONFIDENCE_LIMIT_DEG);
    printf("%-7s %-13s %-11s %-13s %-11s %s\n",
           "h3", "raw_peak", "raw_trips", "inl_peak", "inl_trips", "verdict");

    for (k = 0U; k < sizeof(h3_sweep) / sizeof(h3_sweep[0]); k++) {
        static encoder_inl_t inl;
        sim_model_t m;
        encoder_calibration_t cal;
        run_stats_t raw_st, inl_st;
        rng_t rng;
        double theta_zero;
        const unsigned long n = 200000UL;
        const char *verdict;

        rng.s = 0x243F6A8885A308D3ULL;
        model_defaults(&m);
        m.a1.h3 = h3_sweep[k]; m.a1.h3_phase_deg = 20.0;
        m.a2.h3 = h3_sweep[k]; m.a2.h3_phase_deg = -35.0;
        m.a1.h2 = 0.5 * h3_sweep[k]; m.a2.h2 = 0.5 * h3_sweep[k];
        m.ecc_mech_deg = 0.05; m.ecc_phase_deg = 70.0;

        encoder_inl_init(&inl);
        if (!run_factory_cal(&m, &rng, NULL, &cal, &theta_zero)) continue;

        run_encoder(&m, &rng, &cal, NULL, theta_zero, n, &raw_st, NULL, NULL);
        if (!refine_inl_pass(&m, &rng, &cal, theta_zero, n, &inl) ||
            !refine_inl_pass(&m, &rng, &cal, theta_zero, n, &inl)) {
            printf("%-7.3f  INL solve failed\n", h3_sweep[k]);
            continue;
        }
        run_encoder(&m, &rng, &cal, &inl, theta_zero, n, &inl_st, NULL, NULL);

        if (raw_st.mismatch_flagged == 0UL)      verdict = "ok";
        else if (inl_st.mismatch_flagged == 0UL) verdict = "needs INL";
        else                                     verdict = "TRIPS EVEN WITH INL";

        printf("%-7.3f %-13.1f %-11lu %-13.1f %-11lu %s\n",
               h3_sweep[k], raw_st.max_abs_fine_delta, raw_st.mismatch_flagged,
               inl_st.max_abs_fine_delta, inl_st.mismatch_flagged, verdict);
    }
}

/* angle_counts now follows the unfiltered Vernier solve, so standstill dither is
 * whatever the raw solve produces. The hysteresis latches the first sample and
 * hides it, so measure the raw counts directly -- that is what sets how much
 * hysteresis the control interface actually needs. */
static void mode_standstill(void)
{
    static const double h3_sweep[] = { 0.000, 0.020, 0.040 };
    unsigned int k;

    printf("# standstill dither of the raw Vernier solve, BEFORE output hysteresis\n");
    printf("# ENCODER_ANGLE_COUNT_HYSTERESIS is currently %d counts\n",
           ENCODER_ANGLE_COUNT_HYSTERESIS);
    printf("%-7s %-12s %-12s %-14s %s\n",
           "h3", "span_counts", "rms_counts", "peak_dev_cnt", "hysteresis_needed");

    for (k = 0U; k < sizeof(h3_sweep) / sizeof(h3_sweep[0]); k++) {
        sim_model_t m;
        encoder_calibration_t cal;
        encoder_state_t state;
        encoder_result_t res;
        encoder_raw_sample_t s;
        rng_t rng;
        double theta_zero, sum = 0.0, sum_sq = 0.0, mean;
        double min_c = 1e30, max_c = -1e30, peak_dev = 0.0;
        const unsigned long n = 100000UL;
        unsigned long i;

        rng.s = 0x243F6A8885A308D3ULL;
        model_defaults(&m);
        m.a1.h3 = h3_sweep[k]; m.a2.h3 = h3_sweep[k];
        m.a1.h2 = 0.5 * h3_sweep[k]; m.a2.h2 = 0.5 * h3_sweep[k];
        m.speed_dps = 0.0;

        if (!run_factory_cal(&m, &rng, NULL, &cal, &theta_zero)) continue;

        encoder_state_init(&state);
        {
            /* Zero capture parks the rotor at angle 0, so raw dither straddles the
             * 0/360 wrap. Measure signed deviation from the first sample, never the
             * wrapped value itself. */
            double reference = 0.0;

            for (i = 0UL; i < n; i++) {
                double counts;

                make_sample(&m, theta_zero, 0.0, &rng, &s);
                encoder_process(&state, &cal, NULL, &s, &res, NULL);
                if (i == 0UL) reference = res.angle_deg_raw;
                /* raw solve in counts, bypassing the published hysteresis */
                counts = serrd(res.angle_deg_raw, reference) *
                         (double)ENCODER_COUNTS_PER_REV / 360.0;
                sum += counts;
                sum_sq += counts * counts;
                if (counts < min_c) min_c = counts;
                if (counts > max_c) max_c = counts;
            }
        }
        mean = sum / (double)n;
        peak_dev = (max_c - mean > mean - min_c) ? (max_c - mean) : (mean - min_c);

        printf("%-7.3f %-12.2f %-12.2f %-14.2f %.0f\n",
               h3_sweep[k], max_c - min_c,
               sqrt((sum_sq / (double)n) - (mean * mean)),
               peak_dev, ceil(peak_dev));
    }
}

/* Can a board that was never calibrated converge on its own from the compiled-in
 * defaults? That is the whole "zero factory steps" question. The defaults in
 * app_encoder_defaults.c are one specific board's measured values, so a second
 * board differs from them -- this sweeps how far it can differ and still
 * bootstrap. Mirrors the runtime loop exactly: apply trim, process, update trim. */
static int mode_bootstrap(void)
{
    static const double amp_ratio[] = { 0.50, 0.70, 0.85, 1.00, 1.20, 1.50, 1.90, 2.00 };
    static const double centre_shift[] = { 0.0, 2000.0 };
    unsigned int a, c;
    int passed = 1;

    printf("# bootstrap from board defaults, no factory calibration at all\n");
    printf("# 'this board' differs from app_encoder_defaults.c by the given factors\n");
    printf("%-10s %-11s %-11s %-9s %-13s %-13s %s\n",
           "amp_ratio", "ctr_shift", "lock_revs", "solves", "span_start", "span_end", "verdict");

    for (c = 0U; c < sizeof(centre_shift) / sizeof(centre_shift[0]); c++) {
    for (a = 0U; a < sizeof(amp_ratio) / sizeof(amp_ratio[0]); a++) {
        sim_model_t m;
        encoder_calibration_t defaults, effective;
        encoder_runtime_trim_t trim;
        encoder_state_t state;
        encoder_result_t result;
        encoder_raw_sample_t sample;
        rng_t rng = { 0x243F6A8885A308D3ULL };
        double theta = 0.0;
        const double step = 3600.0 / (double)ADC_SAMPLE_RATE_HZ;   /* 600 rpm */
        /* No zero capture here, so the angle carries an arbitrary constant offset.
         * Measure the error SPAN over each window -- that is the real accuracy,
         * independent of where zero happens to sit. */
        double lo_start = 1e30, hi_start = -1e30, lo_end = 1e30, hi_end = -1e30;
        double err_start, err_end;
        unsigned long lock_sample = 0UL;   /* 0 = never locked */
        unsigned long i;
        const unsigned long n = 400000UL;   /* 40 s, ~400 revolutions */

        model_defaults(&m);
        m.a1.amp_sin *= amp_ratio[a]; m.a1.amp_cos *= amp_ratio[a];
        m.a2.amp_sin *= amp_ratio[a]; m.a2.amp_cos *= amp_ratio[a];
        m.a1.center_sin += centre_shift[c]; m.a1.center_cos -= centre_shift[c];
        m.a2.center_sin -= centre_shift[c]; m.a2.center_cos += centre_shift[c];
        m.a1.h3 = 0.02; m.a2.h3 = 0.02;

        /* No factory calibration: the compiled-in defaults are all we have. */
        encoder_calibration_set_board_defaults(&defaults);
        encoder_runtime_trim_init(&trim);
        encoder_state_init(&state);

        for (i = 0UL; i < n; i++) {
            double err;

            encoder_runtime_trim_apply(&defaults, &trim, &effective);
            make_sample(&m, theta, 3600.0, &rng, &sample);
            encoder_process(&state, &effective, NULL, &sample, &result, NULL);
            encoder_runtime_trim_update(&trim, &defaults, &sample, &result);
            if ((lock_sample == 0UL) && trim.has_locked) lock_sample = i + 1UL;

            err = serrd(result.angle_deg_raw, wrapd(theta));
            if (i < 20000UL) {
                if (err < lo_start) lo_start = err;
                if (err > hi_start) hi_start = err;
            }
            if (i >= n - 20000UL) {
                if (err < lo_end) lo_end = err;
                if (err > hi_end) hi_end = err;
            }
            theta += step;
        }
        err_start = hi_start - lo_start;
        err_end = hi_end - lo_end;

        {
            /* 2.0x drives the front end into the ADC rails, which no amount of
             * software calibration recovers -- refusing it is correct. */
            const int expected = (amp_ratio[a] <= 1.5);
            const int ok = expected ? (trim.has_locked && (err_end < 0.5))
                                    : !trim.has_locked;

            /* 600 rpm at 10 kHz is 1000 samples per revolution, so lock_sample
             * reads directly as "how far does the shaft have to turn before the
             * encoder is usable" -- the number that decides whether this is
             * plug-in-and-go or plug-in-turn-it-once. */
            printf("%-10.2f %-11.0f %-11.2f %-9lu %-13.4f %-13.4f %s%s\n",
                   amp_ratio[a], centre_shift[c],
                   lock_sample ? (double)lock_sample / 1000.0 : -1.0,
                   (unsigned long)trim.solve_count,
                   err_start, err_end,
                   trim.has_locked ? "bootstraps" : "STUCK",
                   ok ? "" : "  <-- REGRESSION");
            if (!ok) passed = 0;
        }
    }
    }
    return passed ? 0 : 1;
}

/* The bootstrap sweep spins at a constant 600 rpm, so 512 samples always span
 * several full electrical cycles and min/max is a good centre. A bench board is
 * not like that: it sits still at power-up and then gets nudged by hand. This
 * reproduces that -- dither a couple of degrees, then rotate properly -- and
 * checks the encoder ends up correct rather than latching a centre measured
 * across a fraction of one cycle. */
static int mode_benchstart(void)
{
    static const struct { const char *name; double dither_deg; unsigned long dither_samples; }
    scenario[] = {
        { "still then spin",  0.0,  20000UL },
        { "nudge 2deg",       2.0,  20000UL },
        { "nudge 10deg",     10.0,  20000UL },
        { "wiggle 30deg",    30.0,  20000UL },
    };
    unsigned int k;
    int passed = 1;

    printf("# bench start-up: stationary or hand-nudged first, then real rotation\n");
    printf("%-18s %-11s %-9s %-14s %s\n",
           "scenario", "lock_when", "solves", "span_after", "verdict");

    for (k = 0U; k < sizeof(scenario) / sizeof(scenario[0]); k++) {
        sim_model_t m;
        encoder_calibration_t defaults, effective;
        encoder_runtime_trim_t trim;
        encoder_state_t state;
        encoder_result_t result;
        encoder_raw_sample_t sample;
        rng_t rng = { 0x243F6A8885A308D3ULL };
        double theta = 0.0;
        double lo = 1e30, hi = -1e30, reference = 0.0;
        const char *lock_when = "never";
        unsigned long i;
        const unsigned long spin = 200000UL;   /* 20 s at 600 rpm = 200 revolutions */

        model_defaults(&m);
        m.a1.h3 = 0.02; m.a2.h3 = 0.02;

        encoder_calibration_set_board_defaults(&defaults);
        encoder_runtime_trim_init(&trim);
        encoder_state_init(&state);

        /* Phase 1: stationary, or dithering back and forth by a few degrees. */
        for (i = 0UL; i < scenario[k].dither_samples; i++) {
            const double phase = 2.0 * PI_D * (double)i / 3000.0;
            theta = scenario[k].dither_deg * 0.5 * sin(phase);
            encoder_runtime_trim_apply(&defaults, &trim, &effective);
            make_sample(&m, theta, 0.0, &rng, &sample);
            encoder_process(&state, &effective, NULL, &sample, &result, NULL);
            encoder_runtime_trim_update(&trim, &defaults, &sample, &result);
            if (trim.has_locked && (lock_when[0] == 'n')) lock_when = "in dither";
        }

        /* Phase 2: proper rotation -- this must leave the encoder correct. */
        for (i = 0UL; i < spin; i++) {
            double err;

            encoder_runtime_trim_apply(&defaults, &trim, &effective);
            make_sample(&m, theta, 3600.0, &rng, &sample);
            encoder_process(&state, &effective, NULL, &sample, &result, NULL);
            encoder_runtime_trim_update(&trim, &defaults, &sample, &result);
            if (trim.has_locked && (lock_when[0] == 'n')) lock_when = "in spin";

            if (i == spin - 20000UL) {
                reference = serrd(result.angle_deg_raw, wrapd(theta));
            }
            if (i >= spin - 20000UL) {
                err = serrd(serrd(result.angle_deg_raw, wrapd(theta)), reference);
                if (err < lo) lo = err;
                if (err > hi) hi = err;
            }
            theta += 3600.0 / (double)ADC_SAMPLE_RATE_HZ;
        }

        {
            const double span = hi - lo;
            const int ok = (span < 0.5);

            printf("%-18s %-11s %-9lu %-14.4f %s\n",
                   scenario[k].name, lock_when, (unsigned long)trim.solve_count,
                   span, ok ? "ok" : "BROKEN");
            if (!ok) passed = 0;
        }
    }
    return passed ? 0 : 1;
}

static void mode_inl(void)
{
    sim_model_t m;
    encoder_calibration_t cal;
    run_stats_t st;
    rng_t rng;
    double theta_zero;

    rng.s = 0xA4093822299F31D0ULL;
    model_defaults(&m);
    m.a1.h3 = 0.02; m.a1.h3_phase_deg = 20.0;
    m.a2.h3 = 0.02; m.a2.h3_phase_deg = -35.0;
    m.ecc_mech_deg = 0.05; m.ecc_phase_deg = 70.0;

    printf("config,theta_deg,fine_delta_deg,angle_err_deg\n");
    if (!run_factory_cal(&m, &rng, NULL, &cal, &theta_zero)) return;
    run_encoder(&m, &rng, &cal, NULL, theta_zero, 8000UL, &st, stdout, "inl");
    fprintf(stderr, "max_err=%.4f deg  rms_err=%.4f deg  slips=%lu\n",
            st.max_abs_err, st.rms_err, st.slips);
}

typedef struct {
    double max_control_err;
    double max_filtered_err;
    double max_step;
    unsigned long status_count;
} highspeed_stats_t;

static double counts_to_deg(uint32_t counts)
{
    return (double)counts * 360.0 / (double)ENCODER_COUNTS_PER_REV;
}

static void run_highspeed_constant(const sim_model_t *m,
                                   const encoder_calibration_t *cal,
                                   double theta_zero,
                                   double rpm,
                                   highspeed_stats_t *stats)
{
    encoder_state_t state;
    encoder_result_t result;
    encoder_raw_sample_t sample;
    rng_t rng = { 0x9E3779B97F4A7C15ULL };
    const double speed_dps = rpm * 6.0;
    double theta = theta_zero;
    uint32_t previous_counts = 0U;
    unsigned long i;

    memset(stats, 0, sizeof(*stats));
    encoder_state_init(&state);

    for (i = 0UL; i < 50000UL; i++) {
        double truth;
        double control_err;
        double filtered_err;

        make_sample(m, theta, speed_dps, &rng, &sample);
        encoder_process(&state, cal, NULL, &sample, &result, NULL);
        truth = wrapd(theta - theta_zero);
        control_err = serrd(counts_to_deg(result.angle_counts), truth);
        filtered_err = serrd(result.angle_deg_filtered, truth);

        if (fabs(control_err) > stats->max_control_err)
            stats->max_control_err = fabs(control_err);
        if ((i >= 5000UL) && (fabs(filtered_err) > stats->max_filtered_err))
            stats->max_filtered_err = fabs(filtered_err);
        if (result.status != ENCODER_STATUS_OK) stats->status_count++;

        if (i != 0UL) {
            int32_t delta = (int32_t)result.angle_counts - (int32_t)previous_counts;
            double step;

            if (delta < -((int32_t)ENCODER_COUNTS_PER_REV / 2))
                delta += (int32_t)ENCODER_COUNTS_PER_REV;
            else if (delta > ((int32_t)ENCODER_COUNTS_PER_REV / 2))
                delta -= (int32_t)ENCODER_COUNTS_PER_REV;
            step = fabs((double)delta * 360.0 / (double)ENCODER_COUNTS_PER_REV);
            if (step > stats->max_step) stats->max_step = step;
        }

        previous_counts = result.angle_counts;
        theta += speed_dps / (double)ADC_SAMPLE_RATE_HZ;
    }
}

static int test_fault_hold(const sim_model_t *m,
                           const encoder_calibration_t *cal,
                           double theta_zero)
{
    encoder_state_t state;
    encoder_result_t result;
    encoder_raw_sample_t sample;
    rng_t rng = { 0xD1B54A32D192ED03ULL };
    const double speed_dps = 36000.0;
    const double step = speed_dps / (double)ADC_SAMPLE_RATE_HZ;
    double theta = theta_zero;
    uint32_t last_counts;
    unsigned long i;

    encoder_state_init(&state);
    for (i = 0UL; i < 5000UL; i++) {
        make_sample(m, theta, speed_dps, &rng, &sample);
        encoder_process(&state, cal, NULL, &sample, &result, NULL);
        theta += step;
    }
    last_counts = result.angle_counts;

    make_sample(m, theta + 15.0, speed_dps, &rng, &sample);
    encoder_process(&state, cal, NULL, &sample, &result, NULL);
    if (((result.status & (ENCODER_STATUS_TRACK_MISMATCH | ENCODER_STATUS_HOLD_LAST)) !=
         (ENCODER_STATUS_TRACK_MISMATCH | ENCODER_STATUS_HOLD_LAST)) ||
        (result.angle_counts != last_counts)) {
        fprintf(stderr, "coherent glitch was not rejected: status=0x%lX\n",
                (unsigned long)result.status);
        return 0;
    }

    make_sample(m, theta, speed_dps, &rng, &sample);
    encoder_process(&state, cal, NULL, &sample, &result, NULL);
    if (result.status != ENCODER_STATUS_OK) {
        fprintf(stderr, "encoder did not recover after coherent glitch: status=0x%lX\n",
                (unsigned long)result.status);
        return 0;
    }

    last_counts = result.angle_counts;
    {
        const double branch_theta = theta + step;
        const double theta_a2 = branch_theta + speed_dps * (m->skew_us * 1e-6);
        const double phi15 = 15.0 * (theta_a2 + ecc_offset(m, theta_a2)) + 10.0;

        make_sample(m, branch_theta, speed_dps, &rng, &sample);
        track_sample(&m->a2, phi15, m->noise_lsb, &rng,
                     &sample.a2_sin_raw, &sample.a2_cos_raw);
        encoder_process(&state, cal, NULL, &sample, &result, NULL);
        if (((result.status & (ENCODER_STATUS_TRACK_MISMATCH | ENCODER_STATUS_HOLD_LAST)) !=
             (ENCODER_STATUS_TRACK_MISMATCH | ENCODER_STATUS_HOLD_LAST)) ||
            (result.angle_counts != last_counts)) {
            fprintf(stderr, "branch-confidence fault was not rejected: status=0x%lX\n",
                    (unsigned long)result.status);
            return 0;
        }

        make_sample(m, branch_theta + step, speed_dps, &rng, &sample);
        encoder_process(&state, cal, NULL, &sample, &result, NULL);
        if (result.status != ENCODER_STATUS_OK) {
            fprintf(stderr, "encoder did not recover after branch fault: status=0x%lX\n",
                    (unsigned long)result.status);
            return 0;
        }
    }

    last_counts = result.angle_counts;
    memset(&sample, 0, sizeof(sample));
    encoder_process(&state, cal, NULL, &sample, &result, NULL);
    if (((result.status & (ENCODER_STATUS_ADC_RAIL | ENCODER_STATUS_HOLD_LAST)) !=
         (ENCODER_STATUS_ADC_RAIL | ENCODER_STATUS_HOLD_LAST)) ||
        (result.angle_counts != last_counts)) {
        fprintf(stderr, "rail fault was not held: status=0x%lX\n",
                (unsigned long)result.status);
        return 0;
    }

    for (i = 0UL; i < ENCODER_FILTER_HOLD_RESYNC_SAMPLES; i++) {
        encoder_process(&state, cal, NULL, &sample, &result, NULL);
    }
    make_sample(m, theta + 90.0, speed_dps, &rng, &sample);
    encoder_process(&state, cal, NULL, &sample, &result, NULL);
    if (result.status != ENCODER_STATUS_OK) {
        fprintf(stderr, "encoder did not reacquire after prolonged fault: status=0x%lX\n",
                (unsigned long)result.status);
        return 0;
    }

    return 1;
}

static int test_dynamic_profile(const sim_model_t *m,
                                const encoder_calibration_t *cal,
                                double theta_zero)
{
    encoder_state_t state;
    encoder_result_t result;
    encoder_raw_sample_t sample;
    rng_t rng = { 0x94D049BB133111EBULL };
    double theta = theta_zero;
    double max_control_err = 0.0;
    double max_filtered_err = 0.0;
    double max_step = 0.0;
    uint32_t previous_counts = 0U;
    unsigned long status_count = 0UL;
    unsigned long i;

    encoder_state_init(&state);
    for (i = 0UL; i < 20000UL; i++) {
        double speed_dps;
        double truth;
        double control_err;
        double filtered_err;

        if (i < 1000UL)
            speed_dps = 36000.0 * (double)i / 999.0;
        else if (i < 6000UL)
            speed_dps = 36000.0;
        else if (i < 8000UL)
            speed_dps = 36000.0 - (72000.0 * (double)(i - 6000UL) / 1999.0);
        else if (i < 13000UL)
            speed_dps = -36000.0;
        else
            speed_dps = 36000.0; /* Deliberately abrupt reversal. */

        make_sample(m, theta, speed_dps, &rng, &sample);
        encoder_process(&state, cal, NULL, &sample, &result, NULL);
        truth = wrapd(theta - theta_zero);
        control_err = fabs(serrd(counts_to_deg(result.angle_counts), truth));
        filtered_err = fabs(serrd(result.angle_deg_filtered, truth));
        if (control_err > max_control_err) max_control_err = control_err;
        if (filtered_err > max_filtered_err) max_filtered_err = filtered_err;
        if (result.status != ENCODER_STATUS_OK) status_count++;

        if (i != 0UL) {
            int32_t delta = (int32_t)result.angle_counts - (int32_t)previous_counts;
            double step;

            if (delta < -((int32_t)ENCODER_COUNTS_PER_REV / 2))
                delta += (int32_t)ENCODER_COUNTS_PER_REV;
            else if (delta > ((int32_t)ENCODER_COUNTS_PER_REV / 2))
                delta -= (int32_t)ENCODER_COUNTS_PER_REV;
            step = fabs((double)delta * 360.0 / (double)ENCODER_COUNTS_PER_REV);
            if (step > max_step) max_step = step;
        }

        previous_counts = result.angle_counts;
        theta += speed_dps / (double)ADC_SAMPLE_RATE_HZ;
    }

    printf("dynamic   %-15.4f %-16.4f %-12.4f %lu\n",
           max_control_err, max_filtered_err, max_step, status_count);
    return (max_control_err <= 0.5) && (max_filtered_err <= 10.0) &&
           (max_step <= 5.0) && (status_count == 0UL);
}

/* candidate_matches_motion() extrapolates the prediction over hold_last_streak+1
 * samples but keeps a fixed 9 deg tolerance, so the gate gets harder to pass the
 * longer the outage lasts. If the shaft is accelerating through the outage this
 * can block recovery until the streak reaches FILTER_HOLD_RESYNC_SAMPLES, i.e. a
 * fixed 5 ms of forced HOLD_LAST after a 2-sample glitch. test_fault_hold only
 * covers the full-length path, so this measures the short-outage cases. */
static int test_short_outage_recovery(const sim_model_t *m,
                                      const encoder_calibration_t *cal,
                                      double theta_zero)
{
    static const unsigned long outage[] = { 1UL, 2UL, 5UL, 10UL, 20UL, 30UL, 40UL, 49UL };
    unsigned int k;
    int passed = 1;

    printf("%-10s %-12s %-14s %s\n", "outage", "accel_dps2", "recover_after", "verdict");
    for (k = 0U; k < sizeof(outage) / sizeof(outage[0]); k++) {
        encoder_state_t state;
        encoder_result_t result;
        encoder_raw_sample_t sample;
        rng_t rng = { 0x2545F4914F6CDD1DULL };
        double speed_dps = 3600.0;
        const double accel = 2.0e6;      /* 0 -> 6000 rpm in 18 ms */
        double theta = theta_zero;
        unsigned long i, recover = 0UL;

        encoder_state_init(&state);
        for (i = 0UL; i < 2000UL; i++) {
            make_sample(m, theta, speed_dps, &rng, &sample);
            encoder_process(&state, cal, NULL, &sample, &result, NULL);
            theta += speed_dps / (double)ADC_SAMPLE_RATE_HZ;
        }

        /* Outage: railed samples, while the shaft keeps accelerating. */
        for (i = 0UL; i < outage[k]; i++) {
            encoder_raw_sample_t dead;
            memset(&dead, 0, sizeof(dead));
            encoder_process(&state, cal, NULL, &dead, &result, NULL);
            speed_dps += accel / (double)ADC_SAMPLE_RATE_HZ;
            theta += speed_dps / (double)ADC_SAMPLE_RATE_HZ;
        }

        /* Healthy samples again: count how many before the encoder publishes. */
        for (i = 0UL; i < 200UL; i++) {
            make_sample(m, theta, speed_dps, &rng, &sample);
            encoder_process(&state, cal, NULL, &sample, &result, NULL);
            speed_dps += accel / (double)ADC_SAMPLE_RATE_HZ;
            theta += speed_dps / (double)ADC_SAMPLE_RATE_HZ;
            if (result.status == ENCODER_STATUS_OK) { recover = i + 1UL; break; }
        }

        if (recover == 0UL) { printf("%-10lu %-12.0f %-14s FAIL\n", outage[k], accel, ">200"); passed = 0; }
        else {
            const int ok = (recover <= 3UL);
            printf("%-10lu %-12.0f %-14lu %s\n", outage[k], accel, recover, ok ? "ok" : "STALL");
            if (!ok) passed = 0;
        }
    }
    return passed;   /* boolean: mode_highspeed folds this into its own flag */
}

/* turn_count and angle_counts must agree about which side of the wrap they are
 * on. They used to come from different angle sources -- counts from the raw
 * Vernier solve, the turn counter from the observer output -- so while the
 * observer lagged through a wrap the pair could differ by a whole revolution,
 * which one T-Format ID3 frame would publish as ABS and ABM disagreeing by 360
 * deg. Sweeps both directions across the boundary. */
static int test_multiturn(const sim_model_t *m,
                          const encoder_calibration_t *cal,
                          double theta_zero)
{
    static const double rpm[] = { 600.0, 6000.0, -6000.0 };
    unsigned int k;
    int passed = 1;

    printf("%-9s %-11s %-15s %-15s %s\n",
           "rpm", "turns_seen", "max_unwrap_err", "max_pair_err", "verdict");
    for (k = 0U; k < sizeof(rpm) / sizeof(rpm[0]); k++) {
        encoder_state_t state;
        encoder_result_t result;
        encoder_raw_sample_t sample;
        rng_t rng = { 0x8A5CD789635D2DFFULL };
        const double speed_dps = rpm[k] * 6.0;
        const double step = speed_dps / (double)ADC_SAMPLE_RATE_HZ;
        /* Start half a revolution away from the wrap. Zero capture parks the rotor
         * exactly on the boundary, where the first reading may land on either side
         * and so defines the multi-turn origin one revolution either way -- an
         * origin convention, not an error, but it masks real inconsistencies. */
        const double theta_start = theta_zero + 180.0;
        double theta = theta_start;
        double max_pair_err = 0.0;
        double max_unwrap_err = 0.0;
        double unwrap_ref = 0.0;
        unsigned long i;
        /* Not a whole number of revolutions: landing exactly on the wrap boundary
         * makes "how many turns should that be" genuinely ambiguous. */
        const unsigned long n = 59950UL;

        encoder_state_init(&state);
        for (i = 0UL; i < n; i++) {
            double pair_err, unwrap_err;

            make_sample(m, theta, speed_dps, &rng, &sample);
            encoder_process(&state, cal, NULL, &sample, &result, NULL);

            /* multi_turn_deg and (turn_count, angle_counts) must describe the
             * same position; a whole-revolution disagreement is the failure. */
            pair_err = fabs(result.multi_turn_deg -
                            ((double)result.turn_count * 360.0 +
                             counts_to_deg(result.angle_counts)));
            /* and the pair must track the true unwrapped shaft angle, measured
             * against the position where the run started */
            if (i == 10UL) unwrap_ref = result.multi_turn_deg - (theta - theta_start);
            unwrap_err = fabs(result.multi_turn_deg - (theta - theta_start) - unwrap_ref);
            if (i > 10UL) {
                if (pair_err > max_pair_err) max_pair_err = pair_err;
                if (unwrap_err > max_unwrap_err) max_unwrap_err = unwrap_err;
            }
            theta += step;
        }

        {
            /* float32 holds ~0.02 deg at 600 revolutions, so 0.5 deg is generous
             * for precision while still catching a 360 deg inconsistency. */
            const int ok = (max_pair_err < 1.0) && (max_unwrap_err < 0.5);
            printf("%-9.0f %-11ld %-15.4f %-15.4f %s\n",
                   rpm[k], (long)result.turn_count,
                   max_unwrap_err, max_pair_err, ok ? "ok" : "FAIL");
            if (!ok) passed = 0;
        }
    }
    return passed;   /* boolean: mode_highspeed folds this into its own flag */
}

static int mode_highspeed(void)
{
    static const double rpm[] = { 0.0, 60.0, 600.0, 3000.0, 6000.0, -6000.0 };
    sim_model_t model;
    encoder_calibration_t calibration;
    rng_t rng = { 0xA4093822299F31D0ULL };
    double theta_zero;
    unsigned int i;
    int passed = 1;

    model_defaults(&model);
    model.a1.h3 = 0.02; model.a1.h3_phase_deg = 20.0;
    model.a2.h3 = 0.02; model.a2.h3_phase_deg = -35.0;
    model.a1.h2 = 0.01; model.a2.h2 = 0.01;
    model.ecc_mech_deg = 0.05; model.ecc_phase_deg = 70.0;

    if (!run_factory_cal(&model, &rng, NULL, &calibration, &theta_zero)) return 1;

    printf("%-9s %-15s %-16s %-12s %s\n",
           "rpm", "control_max_deg", "filtered_max_deg", "max_step_deg", "status");
    for (i = 0U; i < sizeof(rpm) / sizeof(rpm[0]); i++) {
        highspeed_stats_t stats;

        run_highspeed_constant(&model, &calibration, theta_zero, rpm[i], &stats);
        printf("%-9.0f %-15.4f %-16.4f %-12.4f %lu\n",
               rpm[i], stats.max_control_err, stats.max_filtered_err,
               stats.max_step, stats.status_count);
        /* Step allowance: one sample of travel, 25% for how fast the harmonic
         * error itself moves between samples at speed, plus the standstill dither
         * band (measured at 1.65 counts peak by `standstill`, 8 counts here).
         * The point of the absolute term is to be tight at rest -- a flat 1.0 deg
         * floor is 110x the true dither and cannot detect a regression on the raw
         * path. A 22.5 deg branch slip is still far outside this at every speed. */
        const double step_allowance = (fabs(rpm[i]) * 6.0 / (double)ADC_SAMPLE_RATE_HZ) * 1.25 +
                                      (8.0 * 360.0 / (double)ENCODER_COUNTS_PER_REV);

        if ((stats.max_control_err > 0.5) ||
            (stats.max_filtered_err > 0.5) ||
            (stats.max_step > step_allowance) ||
            (stats.status_count != 0UL)) {
            passed = 0;
        }
    }

    if (!test_dynamic_profile(&model, &calibration, theta_zero)) passed = 0;
    {
        const int fault_ok = test_fault_hold(&model, &calibration, theta_zero);
        const int recovery_ok = test_short_outage_recovery(&model, &calibration, theta_zero);
        const int multiturn_ok = test_multiturn(&model, &calibration, theta_zero);

        printf("fault_hold %s\n", fault_ok ? "PASS" : "FAIL");
        if (!fault_ok) passed = 0;
        if (!recovery_ok) passed = 0;
        if (!multiturn_ok) passed = 0;
    }
    return passed ? 0 : 1;
}

int main(int argc, char **argv)
{
    const char *mode = (argc > 1) ? argv[1] : "slip";

    if (strcmp(mode, "slip") == 0)      mode_slip();
    else if (strcmp(mode, "sep") == 0)  mode_sep();
    else if (strcmp(mode, "inl") == 0)  mode_inl();
    else if (strcmp(mode, "calbias") == 0) mode_calbias();
    else if (strcmp(mode, "calcheck") == 0) mode_calcheck();
    else if (strcmp(mode, "inlfit") == 0) mode_inlfit((argc > 2) ? atof(argv[2]) : 0.02);
    else if (strcmp(mode, "faultmargin") == 0) mode_faultmargin();
    else if (strcmp(mode, "standstill") == 0) mode_standstill();
    else if (strcmp(mode, "bootstrap") == 0) return mode_bootstrap();
    else if (strcmp(mode, "benchstart") == 0) return mode_benchstart();
    else if (strcmp(mode, "highspeed") == 0) return mode_highspeed();
    else { fprintf(stderr, "usage: %s [slip|sep|inl|calbias|calcheck|inlfit|highspeed]\n", argv[0]); return 2; }
    return 0;
}
