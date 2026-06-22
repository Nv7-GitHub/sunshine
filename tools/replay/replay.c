/* tools/replay/replay.c — offline 1 kHz replay harness.
 *
 * Links the ACTUAL robot code (sunshine_core) and re-runs a .sun log's 1 kHz
 * inputs through sunshine_step(), emitting every recomputed channel as CSV at
 * the full 1 kHz input rate. No logic is duplicated here — this is purely the
 * IO/glue layer that the README's "replay" concept calls for.
 *
 * Two modes:
 *   (default)    continuous replay: seed state ONCE from the first frame, then
 *                step every input in order. Gives a faithful 1 kHz trajectory.
 *   --reseed     per-frame replay: re-seed state from each frame's stored
 *                "state at start", step that frame's inputs, and emit the
 *                stored-vs-replay vars so you can VALIDATE the replay matches
 *                what the robot actually computed (drift guard).
 *
 * The .sun on-disk layout is packed; MSVC does not honour __attribute__((packed)),
 * so we never memcpy whole structs from the file — we unpack field-by-field from
 * fixed on-disk byte offsets (which are stable regardless of in-memory padding).
 *
 * Usage:  replay.exe <file.sun> [--reseed] [--from-us N] [--to-us N] > out.csv
 */
#include <sunshine_core.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* internal core fn — used to dead-reckon across logged telemetry gaps */
void kalman_predict(SunshineState *s, float dt);

/* ---- little-endian field readers from a packed byte buffer ---- */
static uint32_t rd_u32(const uint8_t *p){ uint32_t v; memcpy(&v,p,4); return v; }
static uint16_t rd_u16(const uint8_t *p){ uint16_t v; memcpy(&v,p,2); return v; }
static int16_t  rd_i16(const uint8_t *p){ int16_t  v; memcpy(&v,p,2); return v; }
static float    rd_f32(const uint8_t *p){ float    v; memcpy(&v,p,4); return v; }

/* On-disk packed offsets (see sunshine_core.h struct definitions). */
static void unpack_input(const uint8_t *b, SunshineInput *in){
    in->time_us       = rd_u32(b+0);
    in->accel_x       = rd_i16(b+4);
    in->accel_y       = rd_i16(b+6);
    in->accel_z       = rd_i16(b+8);
    in->mag_x         = rd_i16(b+10);
    in->mag_y         = rd_i16(b+12);
    in->mag_z         = rd_i16(b+14);
    in->erpm_left     = rd_u16(b+16);
    in->erpm_right    = rd_u16(b+18);
    in->rssi          = (int8_t)b[20];
    in->ctrl_x        = (int8_t)b[21];
    in->ctrl_y        = (int8_t)b[22];
    in->ctrl_theta    = (int8_t)b[23];
    in->ctrl_throttle = b[24];
    in->batt_offset   = (int8_t)b[25];
    in->dshot_left_q  = b[26];
    in->dshot_right_q = b[27];
    in->mode          = b[28];
}
static void unpack_state(const uint8_t *b, SunshineState *s){
    s->kf_theta     = rd_f32(b+0);
    s->kf_omega     = rd_f32(b+4);
    for (int i=0;i<4;i++) s->kf_P[i]      = rd_f32(b+8 +4*i);
    s->theta_offset = rd_f32(b+24);
    for (int i=0;i<2;i++) s->mag_hp_x[i]= rd_f32(b+28+4*i);  /* state: 28..35 */
    for (int i=0;i<2;i++) s->mag_hp_y[i]= rd_f32(b+36+4*i);  /* state: 36..43 (sizeof_state=44) */
}
/* stored vars (for --reseed validation): we only need a few fields */
typedef struct { float mag_angle, est_theta, est_omega, mag_x_filt, mag_y_filt,
                       dshot_l, dshot_r, heading_deg; uint8_t led_on, mag_valid; } StoredVars;
static void unpack_vars(const uint8_t *b, StoredVars *v){
    v->mag_x_filt  = rd_f32(b+4);
    v->mag_y_filt  = rd_f32(b+8);
    v->mag_angle   = rd_f32(b+12);
    v->est_theta   = rd_f32(b+16);
    v->est_omega   = rd_f32(b+20);
    v->dshot_l     = rd_f32(b+24);
    v->dshot_r     = rd_f32(b+28);
    v->led_on      = b[48];
    v->mag_valid   = b[50];
    v->heading_deg = rd_f32(b+52);
}

int main(int argc, char **argv){
    if (argc < 2){ fprintf(stderr,"usage: %s <file.sun> [--reseed] [--from-us N] [--to-us N]\n",argv[0]); return 2; }
    const char *path = argv[1];
    int reseed = 0; long long from_us = -1, to_us = -1;
    for (int i=2;i<argc;i++){
        if (!strcmp(argv[i],"--reseed")) reseed = 1;
        else if (!strcmp(argv[i],"--from-us") && i+1<argc) from_us = atoll(argv[++i]);
        else if (!strcmp(argv[i],"--to-us")   && i+1<argc) to_us   = atoll(argv[++i]);
    }

    FILE *f = fopen(path,"rb");
    if (!f){ fprintf(stderr,"cannot open %s\n",path); return 1; }
    uint8_t hdr[95];
    if (fread(hdr,1,95,f)!=95){ fprintf(stderr,"short header\n"); return 1; }
    if (memcmp(hdr,"SHINE",5)){ fprintf(stderr,"bad magic\n"); return 1; }
    uint16_t file_ver = rd_u16(hdr+5);
    uint16_t header_sz= rd_u16(hdr+7);
    uint16_t sz_in    = rd_u16(hdr+13);
    uint16_t sz_st    = rd_u16(hdr+15);
    uint16_t sz_var   = rd_u16(hdr+17);
    uint16_t num_in   = (file_ver>=2) ? rd_u16(hdr+93) : 20;
    /* VER >= 3: two state snapshots per frame (start + midpoint → 100 Hz) and no
       vars block. Earlier versions: one state + a vars block. */
    uint16_t num_states = (file_ver>=3) ? 2 : 1;
    long frame_sz = 5 + (long)sz_st*num_states + (long)sz_in*num_in + sz_var;
    fprintf(stderr,"ver=%u header=%u sizeof(in/st/var)=%u/%u/%u num_states=%u num_inputs=%u frame_sz=%ld\n",
            file_ver, header_sz, sz_in, sz_st, sz_var, num_states, num_in, frame_sz);

    fseek(f, header_sz, SEEK_SET);
    uint8_t *fb = (uint8_t*)malloc(frame_sz);

    SunshineState st; sunshine_state_init(&st);
    SunshineVars  v;
    int seeded = 0;
    long long prev_us = -1;   /* for gap-fill dead reckoning */

    /* CSV header */
    printf("time_us,mode,kf_theta,kf_omega,omega_accel,mag_angle,est_theta,est_omega,"
           "mag_x_filt,mag_y_filt,heading_deg,led_on,mag_valid,accel_sat,dshot_l,dshot_r,"
           "mag_x,mag_y,accel_x,accel_y,theta_offset,"
           "stored_est_theta,stored_mag_angle,stored_led_on\n");

    while (fread(fb,1,frame_sz,f)==(size_t)frame_sz){
        SunshineState frame_state; unpack_state(fb+5, &frame_state);   /* state_start */
        /* Stored vars exist only in VER 2 (sz_var>0); VER 3 logs none, so the
           stored_* validation columns are left at 0 there. */
        StoredVars sv; memset(&sv, 0, sizeof(sv));
        if (sz_var > 0) unpack_vars(fb + 5 + (long)sz_st*num_states + (long)sz_in*num_in, &sv);
        if (reseed) st = frame_state;

        const uint8_t *inbase = fb + 5 + (long)sz_st*num_states;
        for (uint16_t k=0;k<num_in;k++){
            SunshineInput in; unpack_input(inbase + (long)k*sz_in, &in);
            if (from_us>=0 && (long long)in.time_us < from_us) continue;
            if (to_us  >=0 && (long long)in.time_us > to_us) { free(fb); fclose(f); return 0; }
            /* Continuous: seed ONCE from the stored (real) state at the first
               frame we actually step — so --from-us starts a faithful free-run
               at the window, not at t=0. */
            if (!reseed && !seeded){ st = frame_state; seeded = 1; prev_us = (long long)in.time_us; }
            /* Dead-reckon across logged telemetry gaps. The robot ran 1 kHz
               continuously; only the link dropped frames. Advancing the filter
               by the missing time keeps kf_theta aligned with the field so the
               next mag update isn't a spurious large jump. */
            if (prev_us >= 0) {
                long long gap = (long long)in.time_us - prev_us;
                if (gap > 1500 && gap < 100000)
                    kalman_predict(&st, (float)(gap - 1000) / 1e6f);
            }
            prev_us = (long long)in.time_us;
            sunshine_step(&in, &st, &v);
            int last = (k==num_in-1);
            printf("%u,%u,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%u,%u,%u,%.1f,%.1f,"
                   "%d,%d,%d,%d,%.6f",
                   in.time_us, in.mode, st.kf_theta, st.kf_omega, v.omega_from_accel,
                   v.mag_angle, v.est_theta, v.est_omega, v.mag_x_filt, v.mag_y_filt,
                   v.heading_deg, v.led_on, v.mag_valid, v.accel_saturated,
                   v.dshot_cmd_left, v.dshot_cmd_right,
                   in.mag_x, in.mag_y, in.accel_x, in.accel_y, st.theta_offset);
            /* stored vars only meaningful at frame end */
            if (last) printf(",%.6f,%.6f,%u\n", sv.est_theta, sv.mag_angle, sv.led_on);
            else      printf(",,,\n");
        }
    }
    free(fb); fclose(f);
    return 0;
}
