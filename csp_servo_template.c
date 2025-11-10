/* 
Servo Drive Template (CiA-402, CSP) — Omron R88D-1SN reference
==============================================================
This is a *teaching* template, master-agnostic, that shows the typical structure of a
robust CSP (Cyclic Synchronous Position) loop in C with heavy inline explanations.

READ FIRST (for beginners)
--------------------------
• Goal: help you understand WHAT to write, WHERE it goes, and WHY it’s needed.
• Safe defaults: no homing, software limits only; do NOT run on a real machine without commissioning.
• Master-agnostic: integration points are marked as TODO (map_io, sdo_write_*, state control).
• Omron block: kept as a real-world reference (R88D-1SN, CSP). You can disable it via a macro.

How to read this file
---------------------
1) “Config” section: knobs you can tune (loop period, ramp, dwell, limits, edge policy).
2) “LE helpers”: safe macros to read/write unaligned Little-Endian fields (PDOs).
3) “Process Image structs”: how we map PDOs into tight C structs.
4) “Integration layer (TODO)”: the places you must connect to *your* EtherCAT master.
5) Init(): mapping + (optional) SDOs for CSP.
6) Run(): CiA-402 enable sequence + set-point producer (+ dwell, ramp, FE monitor).
*/

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* =========================
   1) USER CONFIG “KNOBS”
   ========================= */
#define LOOP_PERIOD_MS         1      /* CSP period (ms). Must match 0x60C2:1 if you set it.          */
#define INC_STEP               300    /* Counts added per loop (1ms → 300k cnt/s).                     */
#define LIMIT_POS              200000 /* Software ±limit (counts).                                     */
#define DWELL_MS               500    /* Hold at limits before reversing (ms).                         */
#define RAMP_MS                300    /* Soft ramp on enable / after dwell (ms).                       */
#define FE_WINDOW_COUNTS       20000  /* Example following-error window (counts).                      */
#define FE_WARN_PCT            80     /* Warn at % of window (clear at ~40%).                          */
#define FAULT_COOLDOWN_MS      250    /* After fault clears, stay in Shutdown (CW=0x0006) for ms.      */
#define COMM_COOLDOWN_MS       0      /* Optional: cooldown after comm restore (usually 0 if unused).  */

/* New-setpoint edge policy on CW bit4 (Omron CSP expects an *edge*):
   0 = ON_TICK   (toggle bit4 every loop)
   1 = ON_CHANGE (toggle bit4 only when target_position changes)  ← baseline-friendly */
#define SETPOINT_EDGE_POLICY   1

/* Keep the Omron example ON (1) to document SDOs and IDs used in the field. */
#define OMRON_R88D_EXAMPLE     1

#if OMRON_R88D_EXAMPLE
  #define DRIVE_VENDOR_ID      0x00000083   /* Omron */
  #define DRIVE_PRODUCT_ID     0x00000002   /* R88D-1SN */
  #define WRITE_6060_MODE_CSP  1            /* 6060:0 = 8 (CSP) */
  #define WRITE_60C2_1_US      1            /* 60C2:1 = 1000 us (match LOOP_PERIOD_MS) */
  #define WRITE_6065_FE_WIN    1            /* 6065:0 = FE_WINDOW_COUNTS */
  #define WRITE_10F1_1_WDT     0            /* 10F1:1 (ms) — vendor-specific; leave 0 unless required */
#endif

/* ================================
   2) LITTLE-ENDIAN SAFE HELPERS
   ================================ */
static inline uint16_t le_get_u16(const uint8_t *p){ uint16_t v; memcpy(&v,p,2); return v; }
static inline void     le_set_u16(uint8_t *p, uint16_t v){ memcpy(p,&v,2); }
static inline uint32_t le_get_u32(const uint8_t *p){ uint32_t v; memcpy(&v,p,4); return v; }
static inline void     le_set_u32(uint8_t *p, uint32_t v){ memcpy(p,&v,4); }

/* Fallback macros if your master doesn’t provide EC_GET/SET (unaligned + LE safe) */
#ifndef EC_GETWORD
# define EC_GETWORD(p)        le_get_u16((const uint8_t*)(p))
# define EC_SETWORD(p,v)      le_set_u16((uint8_t*)(p),(uint16_t)(v))
# define EC_GETUINT32(p)      le_get_u32((const uint8_t*)(p))
# define EC_SETUINT32(p,v)    le_set_u32((uint8_t*)(p),(uint32_t)(v))
#endif

/* Simple logs (replace with your logger if needed) */
#ifndef LOG_TAG
# define LOG_TAG "CSP_TEMPLATE"
#endif
#define LOGF(fmt, ...)   printf("[%s] " fmt "\n", LOG_TAG, ##__VA_ARGS__)
#define DBGF(fmt, ...)   printf("[%s][DBG] " fmt "\n", LOG_TAG, ##__VA_ARGS__)
#define WARNF(fmt, ...)  printf("[%s][WRN] " fmt "\n", LOG_TAG, ##__VA_ARGS__)
#define ERRF(fmt, ...)   printf("[%s][ERR] " fmt "\n", LOG_TAG, ##__VA_ARGS__)

/* =======================================
   3) PROCESS IMAGE (PDO) → C STRUCTS
   =======================================

   WHY structs? They make the process image readable and type-safe.
   Make sure sizes and order match your ESI/ENI PDO mapping exactly. */
#pragma pack(push,1)
typedef struct {
    /* Typical minimal inputs for CSP: */
    uint16_t status_word;              /* 0x6041:0 — CiA-402 state bits (see table below)      */
    int32_t  position_actual_value;    /* 0x6064:0 — encoder counts                            */
    int32_t  following_error_actual;   /* 0x60F4:0 — position tracking error                   */
    uint16_t error_code;               /* 0x603F:0 — last error code                           */
} Drive_Inputs;

typedef struct {
    uint16_t control_word;             /* 0x6040:0 — CiA-402 commands                          */
    int32_t  target_position;          /* 0x607A:0 — desired position (counts)                 */
} Drive_Outputs;
#pragma pack(pop)

#define DRIVE_INPUTS_BITS   (sizeof(Drive_Inputs)*8u)
#define DRIVE_OUTPUTS_BITS  (sizeof(Drive_Outputs)*8u)

/* For newcomers — a few key CiA-402 bits:
   SW bit3 = Fault, bit6 = Switch-on disabled; mask 0x006F encodes state.
   Targets: ReadyToSwitchOn=(SW&0x006F)==0x0021, SwitchedOn==0x0023, OperationEnabled==0x0027.
   CW: 0x0006=Shutdown, 0x0007=SwitchOn, 0x000F=EnableOp, 0x0080=FaultReset pulse. */

/* ==========================================
   4) INTEGRATION LAYER (MASTER-SPECIFIC)
   ==========================================

   Replace these stubs with your master API. They are left as TODO on purpose. */
typedef void* EcDevice;
typedef void* EcSlave;

/* Monotonic time and sleep */
static inline long now_ms(void){
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long)(ts.tv_sec*1000L + ts.tv_nsec/1000000L);
}
static inline void sleep_ms(unsigned ms){
    struct timespec req = { ms/1000, (ms%1000)*1000000L };
    nanosleep(&req, NULL);
}

/* Map your process image pointers from ENI bit offsets. Return 0 on success. */
static int map_io(EcDevice dev, int slave_index,
                  size_t in_off_bits, size_t out_off_bits,
                  Drive_Inputs **in, Drive_Outputs **out)
{
    (void)dev; (void)slave_index; (void)in_off_bits; (void)out_off_bits;
    /* TODO:
       - Get image handle from the master
       - Translate bit offsets → byte offsets
       - Acquire pointers with size checks
       - Set *in and *out
    */
    *in = NULL; *out = NULL;
    return -1; /* Keep -1 until implemented */
}

/* CoE SDO writes. Return 0 on success. */
static int sdo_write_u8 (EcDevice dev, int slave, uint16_t idx, uint8_t sub, uint8_t  v){ (void)dev;(void)slave;(void)idx;(void)sub;(void)v; /* TODO */ return 0; }
static int sdo_write_u32(EcDevice dev, int slave, uint16_t idx, uint8_t sub, uint32_t v){ (void)dev;(void)slave;(void)idx;(void)sub;(void)v; /* TODO */ return 0; }

/* Optional: drive state control (PREOP/OP). Often handled elsewhere in your app. */
static void state_request(EcSlave s, int state){ (void)s; (void)state; /* TODO */ }
static int  state_get    (EcSlave s){ (void)s; return 0; /* TODO: DEVICE_STATE_* */ }

/* ===================================
   5) GLOBAL POINTERS + BYTE HELPERS
   =================================== */
static Drive_Inputs  *g_in  = NULL;
static Drive_Outputs *g_out = NULL;

#define IN_PTR(field)   (((uint8_t*)g_in)  + offsetof(Drive_Inputs,  field))
#define OUT_PTR(field)  (((uint8_t*)g_out) + offsetof(Drive_Outputs, field))

/* ===================================
   6) INITIALIZATION (map + SDOs)
   ===================================

   WHAT: Map PDOs and (optionally) program SDOs for CSP.
   WHY: Keep wiring + drive setup explicit and in one place.
*/
int ServoTemplate_Init(EcDevice dev,
                       int slave_index,
                       size_t eni_in_bits, size_t eni_out_bits,
                       size_t eni_in_off_bits, size_t eni_out_off_bits)
{
    if (eni_in_bits != DRIVE_INPUTS_BITS || eni_out_bits != DRIVE_OUTPUTS_BITS){
        WARNF("PDO size mismatch: ENI in=%zu out=%zu, struct in=%u out=%u",
              eni_in_bits, eni_out_bits,
              (unsigned)DRIVE_INPUTS_BITS, (unsigned)DRIVE_OUTPUTS_BITS);
        /* In a real app: abort here. For teaching: continue so code is readable. */
    }

    if (map_io(dev, slave_index, eni_in_off_bits, eni_out_off_bits, &g_in, &g_out) != 0){
        WARNF("map_io() not implemented yet — template stays no-op until you wire it.");
        g_in = g_out = NULL;
    } else {
        DBGF("PI mapped OK (bits in=%zu out=%zu)", eni_in_bits, eni_out_bits);
    }

#if OMRON_R88D_EXAMPLE
    /* Omron R88D-1SN (CSP) example SDOs. Check your manual:
       6060:0 = 8 (CSP), 60C2:1 = 1000 us (1 ms), 6065:0 = FE window.
       10F1:1 watchdog is vendor-specific; keep off unless needed. */
  #if WRITE_6060_MODE_CSP
    (void)sdo_write_u8 (dev, slave_index, 0x6060, 0, 8);
  #endif
  #if WRITE_60C2_1_US
    (void)sdo_write_u32(dev, slave_index, 0x60C2, 1, (uint32_t)(LOOP_PERIOD_MS*1000u));
  #endif
  #if WRITE_6065_FE_WIN
    (void)sdo_write_u32(dev, slave_index, 0x6065, 0, (uint32_t)FE_WINDOW_COUNTS);
  #endif
  #if WRITE_10F1_1_WDT
    (void)sdo_write_u32(dev, slave_index, 0x10F1, 1, 150);
  #endif
#endif
    return 0;
}

/* =========================================================
   7) RUNTIME LOOP (CiA-402 + CSP set-point producer)
   ========================================================= */
void ServoTemplate_Run(EcDevice dev)
{
    (void)dev;
    if (!g_in || !g_out) return; /* Not mapped yet → nothing to do */

    /* State machine bookkeeping */
    static int   st = 0;                   /* 0=Shutdown,1=SwitchOn,2=EnableOp aligning,3=Running */
    static long  t0 = 0;                   /* loop scheduler reference (ms)                       */
    static int32_t pos_tgt = 0;            /* current target position                             */
    static int   dir = 1;                  /* +1 forward, -1 backward, 0 stopped (dwell)         */
    static int   dwell_rem = 0;            /* ms remaining in dwell                               */
    static int   ramp_rem  = 0;            /* ms remaining in ramp                                */
    static int   fe_warn   = 0;            /* FE warning latched                                  */
    static int   fault_cool_rem = 0;       /* post-fault cooldown (ms in Shutdown)                */
    static int   comm_cool_rem  = COMM_COOLDOWN_MS; /* optional comm cooldown                        */
    static uint16_t edge = 0;              /* bit4 toggler for “new set-point”                    */

    /* Always read StatusWord safely; 0 means comm/state down → do nothing. */
    const uint16_t SW = EC_GETWORD(IN_PTR(status_word));
    if (SW == 0){
        t0 = now_ms(); /* avoid backlog when link returns */
        return;
    }

    /* Fault handling (SW bit3): send 0x0080 pulse; when cleared, keep FAULT_COOLDOWN_MS in Shutdown. */
    if (SW & 0x0008){
        /* Pulse Fault Reset: set 0x0080, then immediately release to 0x0006 on next tick. */
        static int need_release = 0;
        if (!need_release){
            EC_SETWORD(OUT_PTR(control_word), 0x0080);
            need_release = 1;
            return;
        } else {
            EC_SETWORD(OUT_PTR(control_word), 0x0006); /* release pulse */
            need_release = 0;
            return;
        }
    } else {
        /* If we just cleared a fault, enforce a cooldown in Shutdown. */
        if (fault_cool_rem > 0){
            EC_SETWORD(OUT_PTR(control_word), 0x0006); /* keep Shutdown */
            long now = now_ms();
            if (now - t0 >= 1){ fault_cool_rem--; t0 = now; }
            return;
        }
    }

    /* Optional: cooldown after comm restore (kept simple) */
    if (comm_cool_rem > 0){
        EC_SETWORD(OUT_PTR(control_word), 0x0006);
        long now = now_ms();
        if (now - t0 >= 1){ comm_cool_rem--; t0 = now; }
        return;
    }

    /* Switch-on disabled? Go back to Shutdown. */
    if (SW & 0x0040){
        st = 0;
        EC_SETWORD(OUT_PTR(control_word), 0x0006);
        return;
    }

    /* CiA-402 gating */
    switch (st){
        case 0: /* Want ReadyToSwitchOn (SW&0x006F)==0x0021 */
            EC_SETWORD(OUT_PTR(control_word), 0x0006); /* Shutdown */
            if ((SW & 0x006F) == 0x0021){ st = 1; DBGF("ReadyToSwitchOn"); }
            return;

        case 1: /* Want SwitchedOn (SW&0x006F)==0x0023 */
            EC_SETWORD(OUT_PTR(control_word), 0x0007); /* Switch on */
            if ((SW & 0x006F) == 0x0023){ st = 2; DBGF("SwitchedOn"); }
            return;

        case 2: { /* Align targets to avoid a jump, then EnableOperation */
            int32_t pos_act = (int32_t)EC_GETUINT32(IN_PTR(position_actual_value));
            pos_tgt = pos_act;
            EC_SETUINT32(OUT_PTR(target_position), (uint32_t)pos_tgt);
            EC_SETWORD(OUT_PTR(control_word), 0x000F); /* Enable operation */
            if ((SW & 0x006F) == 0x0027){
                st = 3; t0 = now_ms(); ramp_rem = RAMP_MS; DBGF("OperationEnabled (CSP)");
            }
            return;
        }
        default: break;
    }

    /* CSP producer (only at OperationEnabled) */
    if (st == 3){
        long now = now_ms();
        if (now - t0 >= LOOP_PERIOD_MS){
            int32_t pos_prev = pos_tgt;

            if (dwell_rem > 0){
                dwell_rem--;
                if (dwell_rem == 0){ ramp_rem = RAMP_MS; dir = (dir==0 ? -1 : dir); /* resume */ }
            } else {
                /* Mini-ramp: scale INC_STEP during the first RAMP_MS */
                int delta = dir * INC_STEP;
                if (ramp_rem > 0){
                    int ramp_total = (RAMP_MS>0 ? RAMP_MS : 1);
                    int ramp_used  = (ramp_total - ramp_rem + 1);
                    delta = (delta * ramp_used) / ramp_total;
                    if (delta == 0 && dir) delta = (dir>0)?1:-1;
                    ramp_rem--;
                }
                pos_tgt += delta;

                /* Clamp and start dwell at limits */
                if (pos_tgt >  LIMIT_POS){ pos_tgt =  LIMIT_POS; dwell_rem = DWELL_MS; dir = 0; }
                if (pos_tgt < -LIMIT_POS){ pos_tgt = -LIMIT_POS; dwell_rem = DWELL_MS; dir = 0; }
            }

            /* Write target (unaligned + LE safe) */
            EC_SETUINT32(OUT_PTR(target_position), (uint32_t)pos_tgt);

            /* New set-point edge on CW bit4 */
#if SETPOINT_EDGE_POLICY == 0 /* ON_TICK */
            edge ^= 0x0010;
            EC_SETWORD(OUT_PTR(control_word), 0x000F | edge);
#else                         /* ON_CHANGE — baseline friendly */
            if (pos_tgt != pos_prev){ edge ^= 0x0010; }
            EC_SETWORD(OUT_PTR(control_word), 0x000F | edge);
#endif

            /* Following-error monitor with hysteresis */
            const int32_t fe = (int32_t)EC_GETUINT32(IN_PTR(following_error_actual));
            const int fe_warn_th = (FE_WINDOW_COUNTS * FE_WARN_PCT)/100;
            const int fe_ok_th   = (FE_WINDOW_COUNTS * 40)/100;
            if (!fe_warn && (fe > fe_warn_th || fe < -fe_warn_th)){ WARNF("FE high: %d", fe); fe_warn = 1; }
            else if (fe_warn && (fe < fe_ok_th && fe > -fe_ok_th)){ DBGF("FE back: %d", fe); fe_warn = 0; }

            t0 = now;
        }
    }
}

/* ==============================================================
   8) WHERE TO HOOK FAULT COOLDOWN
   ==============================================================

   In your main loop, when you detect that Fault just cleared (SW bit3 went from 1 to 0),
   set fault_cool_rem = FAULT_COOLDOWN_MS. In this template we keep it simple by
   enforcing FAULT_COOLDOWN_MS right after any fault reset pulse clears (see Run()).

   In a real app you may add retries/backoff logic; this file shows the *places* to add it.
*/
