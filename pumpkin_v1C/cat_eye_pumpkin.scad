/* =====================================================================
   ANIMATRONIC CAT-EYE PUMPKIN  -  PHASE 2A  -  PARAMETRIC OPENSCAD MODEL
   ---------------------------------------------------------------------
   Inverted sector-gear plunger (flipped "MG Motion" mechanism) with a
   30-tooth printed RATCHET + flexure pawls (servo resets freely while
   the gear stays parked), MG90S deadbolt trapdoor, sculpted flame,
   sharp cat-eye LCD bezels, split pumpkin shell, sprung neck, ballast base.

   OpenSCAD 2021.01 or newer.  Units: mm / degrees.
   Animation: View > Animate, FPS 10, Steps 200  (one full 5-step cycle).

   HEAD FRAME: X = viewer's right, Y = into the pumpkin (face looks to -Y),
               Z = up, Z = 0 at the bottom of the head shell.
   MECH FRAME: origin = sector-gear centre, +x toward the rack, y = depth
               (+y = toward the MG995), z = up.  Gear turns CLOCKWISE seen
               from the front = rotate([0,+g,0]).
   TABLE FRAME (assembly): Z = 0 on the table, base at the origin.

   MECHANISM MAP (image flipped, spring folded beside the rack):
     green  backbone_frame_part  : MG995 bracket (40.7x19.7x42.9), lower spring
                                   seat SHELF, rack channel, pawl + hall posts
     green  channel_cover_part   : front channel wall + guide ridge
     black  black_plate_part     : upper guide bushing + adjustable M3 travel-stop
     red    shaft (D8x1 Al tube) : TOP TRAPDOOR STOP SHOULDER = shoulder_cap_part
     red    carrier LUG          : underside = UPPER SPRING-COMPRESSION COLLAR,
                                   top face  = SPRING-RETENTION STOP that limits
                                   travel at Z = S (hits the M3 stop screw)
     yellow rack, 9 teeth, m1.5  : pick-up tooth Z = +9.25 at full extension
     blue   sector_gear_part     : 2 x 9 teeth + 30T ratchet cup (one-way only)
   FREE RETURN: the MG995 horn returns 180 -> 0 deg through the ratchet while
     the gear stays PARKED with no tooth inside the +-11.62 mm engagement band,
     so the rack (at Z = S or anywhere in flight) is never touched; on the next
     0 -> 132 deg sweep the leading tooth picks up tooth 0 at g = 9 deg.

   v2B-print CHANGES (STL release):
     - flame base 13.5 mm above the black plate (was 2): the Step-4 tip-slip
       over-travel of 11.93 mm no longer drives the flame into the plate;
       flame 47.5 mm tall so the lid height is unchanged, still pops 44.2 mm
     - flame = flame_collar_part (grubbed to the tube) + sculpted_flame
       (open bottom, slides over a <= D13 x 12 mm LED band, clamped by the
       shoulder cap); D3.5 side hole for the LED wires
     - black plate extended to x = -4 with a 40 mm LED-wire anchor tab
     - gear front boss removed (prints flat), drive-disk horn recess removed,
       rack teeth run to the carrier front face, stem 45 deg base cone,
       TPU pads part; render_mode = "part" exports one oriented part per STL

   HARDWARE THE CAD EXPECTS: D8x1 Al tube 165.6 mm (shaft), D6x1 Al tube
     99.6 mm (spring guide), ejector spring 0.8 x 11.8 OD x 100 (cut from stock),
     neck spring 3.0 x 33 OD x 50, steel plate 230x230x4, M3 heat-set inserts
     (D4.1 holes), M3x12 stop screw + nut, M3x20 horn screw + D10 washer,
     M3x30 hinge pin, M2 screws (servo tabs, switch, IMU), 8x3 magnets x16,
     6x3 magnets x2 (hold-open), D3x10 dowels x4, A3144 hall + 3x2 magnets x2,
     KW11-3Z lever switch, 4x WS2812B-2020 LED band (<= D13 x 12 mm),
     TPU pads (tpu_pads_part) on the lug top, shoulder cap and lid.
   ===================================================================== */

/* [Render] */
render_mode = "assembly";   // [assembly, part, print_base, print_head_front, print_head_back, print_mechanism, print_flame, print_trapdoor, mesh_check]
part        = "sector_gear";// [base_shell, neck_mount, head_front, head_back, backbone_frame, channel_cover, black_plate, sector_gear, drive_disk, antireverse_pawl, rack_carrier, shoulder_cap, flame_body, flame_collar, trapdoor_lid, stem, keeper, deadbolt, lcd_retainer, tpu_pads]
quality     = "draft";      // [draft, final]
anim_t      = -1;           // -1 = follow $t ; 0..1 = freeze the cycle at this point
show_shell  = true;
shell_alpha = 0.28;
show_reserved_bays = true;
head_wobble = 0;            // [-22:22] demo tilt of the head on the neck spring
base_part   = "shell";      // [shell, neck_mount]   used by print_base
check_g     = 9;            // gear angle for mesh_check (deg)

/* [Resolution] */
FINE   = quality == "final";
$fa    = FINE ? 3 : 8;
$fs    = FINE ? 0.4 : 1.2;
SH_NT  = FINE ? 160 : 80;   // shell azimuth samples (multiple of LOBES*2 keeps grooves crisp)
SH_NP  = FINE ? 72  : 36;   // shell polar samples
STAR_N = FINE ? 120 : 60;   // flame polygon samples
EPS    = 0.01;

/* ============================ UTILITIES ============================= */
function clamp(x, a, b) = min(max(x, a), b);
function seg(t, t0, t1) = clamp((t - t0) / (t1 - t0), 0, 1);
function ease(u)        = u * u * (3 - 2 * u);
function unit(v)        = v / norm(v);
function dir2(a)        = [cos(a), sin(a)];
function arc_pts(r, a0, a1, n = 24) = [for (i = [0:n]) r * dir2(a0 + (a1 - a0) * i / n)];

// 2D sketch in the X-Z plane, extruded over y0..y1 (2D x -> X, 2D y -> Z)
module xz(y0, y1) { translate([0, y1, 0]) rotate([90, 0, 0]) linear_extrude(height = y1 - y0, convexity = 8) children(); }
// axis-aligned box from corner p0 to corner p1
module box(p0, p1) { translate(p0) cube(p1 - p0); }
// cylinder along +Y
module ycyl(d, y0, y1, c = [0, 0]) { translate([c[0], y0, c[1]]) rotate([-90, 0, 0]) cylinder(d = d, h = y1 - y0); }

/* ======================= GEAR / RACK CONSTANTS ====================== */
MOD = 1.5; PA = 20; N_FULL = 30; BACKLASH = 0.20; GEAR_FACE = 8;
R_PITCH = MOD * N_FULL / 2;           // 22.5
R_TIP   = R_PITCH + MOD;              // 24.0
R_ROOT  = R_PITCH - 1.25 * MOD;       // 20.625
R_BASE  = R_PITCH * cos(PA);          // 21.143
PITCH   = PI * MOD;                   // 4.712
TAU     = 360 / N_FULL;               // 12 deg
TEETH_PER_SECTOR = 9;
SECTOR_LEAD = 36;                     // body angle of the leading tooth (world angle when parked, g = 0)
TOOTH_CENTRES = [for (s = [0, 180]) for (i = [0:TEETH_PER_SECTOR - 1]) SECTOR_LEAD + s + TAU * i];
BAND_HALF_DEG = acos((R_PITCH - MOD) / R_TIP);       // 28.96 deg  tip circle / rack-tip-line intersection
BAND_HALF_Z   = R_TIP * sin(BAND_HALF_DEG);          // 11.62 mm  vertical engagement band

// ---- Phase-1 interference-simulation results (shapely, 0.2 mm backlash) ----
Z_T0_TOP  = 9.25;     // bottom pick-up tooth centre at FULL UPWARD EXTENSION (Z = S)
Z_T0_WIN  = [7.5, 11.0];   // clean-engagement window (jam 11.25..12.0, no engagement > 12.25)
HOLD_G    = 132;      // MG995 hold angle (last tooth on pitch line)
REL_G     = 162.25;   // slip-release angle
KIN_G0    = 6.7;      // linear-fit origin of rack drop  (drop = r * (g - 6.7deg))
DROP_HOLD = 49.20;    // rack travel at hold
DROP_MAX  = 59.13;    // max over-travel during tip-slide
EJECT     = 47.20;    // ejection stroke S (trapdoor stop -> top stop)
function rack_drop(g) = g <= KIN_G0 ? 0 : min(R_PITCH * (g - KIN_G0) * PI / 180, DROP_MAX);

// ---- rack carrier / red shaft ----
RACK_TEETH  = 9;
RACK_FACE   = GEAR_FACE - 0.8;     // rear edge of the rack teeth at +RACK_FACE/2 = 0.4 mm clear of the ratchet cup
RACK_TIP_X  = R_PITCH - MOD;          // 21.0
RACK_ROOT_X = R_PITCH + 1.25 * MOD;   // 24.375
RACK_BAR_T  = 4;
SHAFT_D     = 8;  SHAFT_ID = 6;       // Hornbach  Al tube D8 x 1
SHAFT_X     = RACK_ROOT_X + RACK_BAR_T + SHAFT_D / 2;   // 32.375
CAR_HW      = 5.6;                    // carrier half width (y)
CAR_X1      = SHAFT_X + SHAFT_D / 2 + 1.0;
CAR_BOT_L   = -(PITCH / 2 + 1.5);                         // carrier local z (tooth 0 = 0)
CAR_RACK_TOP_L = (RACK_TEETH - 1) * PITCH + PITCH / 2 + 1.5;
LUG_T       = 4;
LUG_TOP_L   = CAR_RACK_TOP_L + LUG_T;
SPRING_X    = SHAFT_X + 14;           // folded ejector-spring axis (46.375)
LUG_X1      = SPRING_X + 7;
GROOVE_X    = [25.2, 27.2];  GROOVE_D = 1.2;

// ---- ejector spring 0.8 x 11.8 OD x 100, 20 active, k = 0.1525 N/mm ----
EJ_WIRE = 0.8; EJ_OD = 11.8; EJ_L0 = 100; EJ_K = 0.1525; EJ_F_TOP = 2.5; EJ_SOLID = 17.6; EJ_TURNS = 22;
EJ_L_TOP = EJ_L0 - EJ_F_TOP / EJ_K;                // 83.6
GUIDE_D  = 6;                                       // Hornbach Al tube D6 x 1 (spring guide)

// ---- vertical stack (mech frame), all derived ----
Z_LUG_UNDER_TOP = Z_T0_TOP + CAR_RACK_TOP_L;        // 50.81  spring-compression collar (lug underside)
Z_LUG_TOP_TOP   = Z_T0_TOP + LUG_TOP_L;             // 54.81  spring-retention / travel stop face
SHELF_TOP       = Z_LUG_UNDER_TOP - EJ_L_TOP;       // -32.80 lower spring seat shelf
STOP_PROTRUDE   = 2;                                // M3 stop-screw tip below the black plate (adjust +-1.75)
BP_BOT = Z_LUG_TOP_TOP + STOP_PROTRUDE;  BP_T = 6;  BP_TOP = BP_BOT + BP_T;
OVER_TRAVEL = DROP_MAX - EJECT;                     // 11.93  plunger dips below ARMED during the Step-4 tip-slide
FLAME_CLEAR = OVER_TRAVEL + 1.57;                   // 13.5   flame base stays 1.57 above the black plate at max over-travel
FLAME_BASE_ARMED = BP_TOP + FLAME_CLEAR;            // 76.31
FLAME_BASE_TOP   = FLAME_BASE_ARMED + EJECT;
FLAME_H = 47.5;  SHOULDER_T = 3;  SHOULDER_D = 12;  // collar bottom -> spine top (protrudes EJECT - SHOULDER_T = 44.2 mm when fired)
SHOULDER_TOP_ARMED = FLAME_BASE_ARMED + FLAME_H + SHOULDER_T;   // 126.81 = lid underside
CAR_BOT_TOP = Z_T0_TOP + CAR_BOT_L;
TUBE_TOP_TOP = FLAME_BASE_TOP + FLAME_H;
TUBE_L = TUBE_TOP_TOP - CAR_BOT_TOP;
CH_BOT = CAR_BOT_TOP - DROP_MAX - 0.8;              // rack channel bottom stop
CH_TOP = BP_BOT;
LUG_UNDER_MIN = Z_LUG_UNDER_TOP - DROP_MAX;

// ---- channel / frame ----
CH_IN = 6.0; CH_WALL = 3; CH_FWALL = 5;  CH_X0 = 25.4; CH_X1 = 40; CH_END = 43;
BB_Y0 = 23; BB_Y1 = 27;                              // green backbone plate
GEAR_Y0 = -GEAR_FACE / 2; GEAR_Y1 = GEAR_FACE / 2;
CUP_Y1  = 12;                                        // ratchet cup behind the gear
DISK_Y0 = 4.6; DISK_Y1 = 11.0;                       // drive (pawl) disk inside the cup
HUB_D   = 8;                                         // drive-disk hub = gear journal (gear rides on the servo bearings)
HUB_Y0  = GEAR_Y0 - 0.4;                             // hub nose 0.4 proud of the gear face; M3x20 horn screw + D10 washer retains the gear
HALL_POS = [20, 0];                                  // A3144 hall post, faces magnets in the cup rim (body 0/180)

// ---- 30-tooth ratchet ----
RT_N = 30;
RT_EXT  = [20.8, 22.6];      // external teeth  root, tip   (anti-reverse)
RT_INT  = [17.8, 16.0];      // internal teeth  root, tip   (drive)
RT_PHASE = 0;                // steep faces at body 12k deg (indexes g = 0, 132, 180)
AR_LUG  = 60;  AR_ANCHOR = 130;                      // anti-reverse pawl (world angles)
AR_ARM  = [24.0, 25.2];  AR_LUG_TO = RT_EXT[0] + 0.25;
AR_POST_R = 28.5;
DR_LUG  = [36, 216];  DR_ARC = 70;                   // drive pawls (disk body angles)
DR_ARM  = [13.9, 15.1];  DR_LUG_TO = RT_INT[0] - 0.25;
DR_CORE = 11.8;
HORN_D = 21; HORN_T = 2.4; HORN_PCD = 14; HORN_HOLE = 2.2;

// ---- MG995 (40.7 x 19.7 x 42.9) shaft on gear axis, body pointing UP ----
MG995_L = 40.7; MG995_W = 19.7; MG995_H = 42.9;
MG995_SHAFT_OFF = 10.2;      // shaft to near body end
MG995_SPLINE_ABOVE_TAB = 15.5; MG995_CASE_ABOVE_TAB = 10;
MG995_TAB_LEN = 54; MG995_TAB_T = 2.5; MG995_HOLE_PITCH = 48.5; MG995_HOLE_SPAN = 10;
MG995_CZ = MG995_L / 2 - MG995_SHAFT_OFF;          // body centre z in mech frame (10.15)
SPLINE_TOP_Y = BB_Y1 - MG995_SPLINE_ABOVE_TAB;      // 11.5

// ---- MG90S (22.8 x 12.2 x 28.5) ----
MG90_L = 22.8; MG90_W = 12.2; MG90_BODY_H = 22.5; MG90_SPLINE = 6.0;
MG90_TAB_Z = 15.9; MG90_TAB_T = 2.5; MG90_TAB_LEN = 32.5; MG90_HOLE_PITCH = 27.6; MG90_SHAFT_OFF = 5.9;

/* ========================== HEAD CONSTANTS ========================== */
HEAD_A = 115; HEAD_C = 102.4; WALL = 1.6;
LOBES = 10; LOBE_DEPTH = 0.055; LOBE_SHARP = 3;
SPLIT_Y = -50;                                       // front "face mask" | back shell
MECH_X = -SHAFT_X; MECH_Y = 0; MECH_Z = 75;          // gear centre in head frame (shaft on head axis)
LID_UNDER = MECH_Z + SHOULDER_TOP_ARMED;             // 201.81
LID_T = 3; LID_TOP = LID_UNDER + LID_T;              // 204.81
LID_R = 37; OPEN_R = 38; LEDGE_R = [32, 46]; LEDGE_T = 3;
CROWN_R = 58; CROWN_R_TOP = 54; CROWN_Z0 = 183; CROWN_WALL = 2.4;   // stem collar, also houses the latch
KNUCKLE_D = 7; PIN_D = 3.2;
HINGE_X = -(LID_R + 4.5); HINGE_Z = LID_TOP + KNUCKLE_D / 2 + 0.3;   // knuckles ride above the crown deck
LID_OPEN_MAX = 110;
// radial deadbolt (slides along X) through a keeper plate hanging under the lid's free (right) edge
KEEPER_X = 34; KEEPER_W = 8; KEEPER_T = 2.5; KEEPER_DROP = 14;
BOLT_Z = [LID_UNDER - 11.2, LID_UNDER - 7.2]; BOLT_HW = 2;
BOLT_X = [31, 41];                                   // engaged: tip 1.75 mm past the keeper's inner face
CRANK_R = 5; BOLT_STROKE = 2 * CRANK_R * sin(45);    // 7.07 mm, crank swings -45..+45 deg
YOKE_X = [40, 45.5]; YOKE_HW = 5; YOKE_SLOT_X = 42.75;
MG90_AXIS = [YOKE_SLOT_X + CRANK_R * sin(45), -4.25];
MG90_SPLINE_TOP = BOLT_Z[0] - 2.3;
SW_BOX = [[-22, 33.2], [-2, 39.6]];                  // KW11-3Z lid switch footprint (X,Y)

// eyes (GC9A01 1.28in round, active D32.4 - Sigmanortec 45.5 x 38 x 23 module)
EYE_X = 35; EYE_Z = 128; EYE_L = 31; EYE_H = 13.5; EYE_TILT = 18;   // eye centres sit on lobe crowns (azimuth -72 / -108 deg)
LCD_PCB_D = 38.4; LCD_GLASS_D = 35.8; LCD_ACTIVE_D = 32.4; LCD_STACK = 4.5;
LCD_TAB_W = 14; LCD_TAB_REACH = 26.5; LCD_RECESS = 4.6;

// reserved bays (head frame)  [min], [max]
BAY_SMOKE = [[26, -22, 58], [96, 22, 108]];          // 70 x 44 x 50
BAY_MOTOR = [[-100, -22, 70], [-60, 8, 100]];        // 40 x 30 x 30
ESP_SHELF_Z = 113;
IMU_PAD = [[-45, -22], [-25, -2]];

/* ========================== BASE / NECK ============================= */
BASE_W = 240; BASE_H = 55; BASE_WALL = 2.4; BASE_TOP_T = 3; BASE_R = 14;
PLATE = [230, 230, 4];                               // steel ballast plate (cut locally)
NECK_WIRE = 3.0; NECK_OD = 33; NECK_L0 = 50; NECK_TURNS = 7; NECK_K = 5.95; HEAD_MASS = 0.78;
NECK_SAG = HEAD_MASS * 9.81 / NECK_K;
CUP_ID = NECK_OD + 0.6; CUP_OD = 42; CUP_DEPTH = 10;
MOUNT_D = 160; MOUNT_T = 4;
NECK_FLOOR_Z = BASE_H + MOUNT_T;
HEAD_Z0 = NECK_FLOOR_Z + NECK_L0 - NECK_SAG - CUP_DEPTH;   // head bottom in table frame
PIVOT_Z = (NECK_FLOOR_Z + CUP_DEPTH + HEAD_Z0) / 2;
TILT_STOP = 22; TILT_RHO = 58;
function head_bottom_z(rho) = HEAD_Z0 + HEAD_C - HEAD_C * sqrt(1 - pow(rho / HEAD_A, 2));
TILT_X = TILT_RHO * cos(TILT_STOP) + (head_bottom_z(TILT_RHO) - PIVOT_Z) * sin(TILT_STOP);
TILT_Z = PIVOT_Z - TILT_RHO * sin(TILT_STOP) + (head_bottom_z(TILT_RHO) - PIVOT_Z) * cos(TILT_STOP);

/* =========================== SANITY CHECKS ========================== */
assert(Z_T0_TOP >= Z_T0_WIN[0] && Z_T0_TOP <= Z_T0_WIN[1], "pick-up tooth outside clean-engagement window");
assert(Z_T0_TOP < BAND_HALF_Z, "pick-up tooth must sit inside the engagement band");
assert((TEETH_PER_SECTOR - 1) * TAU + 2 * (BAND_HALF_DEG + 1.3) < 180, "sector + band exceeds one 180 deg stroke");
assert(REL_G < 180, "release must complete inside the servo stroke");
assert(EJ_L_TOP - DROP_MAX > EJ_SOLID + 4, "ejector spring would go solid");
assert(HOLD_G % TAU == 0 && 180 % TAU == 0, "ratchet indexing must land on HOLD_G and 180");
assert(RT_EXT[1] < RACK_ROOT_X - 1, "ratchet cup hits rack carrier");
assert(abs(2 * HEAD_C - LID_TOP) < 1.5, "lid top must sit at the crown of the shell");
assert(TILT_X + 9 < MOUNT_D / 2, "tilt stop ring must fit on the neck mount");
assert(FLAME_CLEAR - OVER_TRAVEL >= 1.5, "flame base would hit the black plate during the Step-4 over-travel");
assert(min(CAV_R) > 6.5 + 0.4 && CAV_Z[1] - CAV_Z[0] >= 12, "LED cavity too small for a 13 x 12 mm LED band");
assert(SHAFT_X - ANCHOR_X[1] > FLAME_R_MAX + 3, "wire anchor tab inside the flame envelope");
echo(str("PUMPKIN | band +-", BAND_HALF_DEG, " deg / +-", BAND_HALF_Z, " mm | eject ", EJECT,
         " | lid underside Z ", LID_UNDER, " | head ", 2 * HEAD_A, "x", 2 * HEAD_A, "x", 2 * HEAD_C,
         " | head bottom (table) ", HEAD_Z0, " | tilt stop ring X ", TILT_X, " Z ", TILT_Z));

/* ============================ COLOURS =============================== */
C_GREEN = [0.30, 0.68, 0.25]; C_BLUE = [0.20, 0.25, 0.70]; C_RED = [0.85, 0.15, 0.12];
C_YELLOW = [0.98, 0.84, 0.15]; C_ORANGE = [0.95, 0.45, 0.05]; C_BLACK = [0.12, 0.12, 0.12];
C_SPRING = [0.75, 0.45, 0.15]; C_STEEL = [0.6, 0.6, 0.62]; C_GHOST = [0.0, 0.55, 0.48, 0.18];

/* =====================================================================
   1. INVOLUTE SECTOR GEAR + 30-TOOTH RATCHET
   ===================================================================== */
function inv_r(a) = tan(a) - a * PI / 180;                // involute function, a in degrees -> rad
TOOTH_S = PITCH / 2 - BACKLASH / 2;
function tooth_half_deg(rr) = (TOOTH_S / (2 * R_PITCH) + inv_r(PA) - inv_r(acos(min(1, R_BASE / rr)))) * 180 / PI;

module tooth_2d() {
    r0 = max(R_BASE, R_ROOT);
    rs = [for (i = [0:14]) r0 + (R_TIP - r0) * i / 14];
    right = [for (rr = rs) rr * dir2(-tooth_half_deg(rr))];
    left  = [for (i = [14:-1:0]) rs[i] * dir2(tooth_half_deg(rs[i]))];
    hb = tooth_half_deg(r0);
    polygon(concat([(R_ROOT - 0.6) * dir2(-hb)], right, left, [(R_ROOT - 0.6) * dir2(hb)]));
}

module sector_gear_2d() {
    difference() {
        union() { circle(r = R_ROOT, $fn = 120); for (c = TOOTH_CENTRES) rotate(c) tooth_2d(); }
        circle(d = HUB_D + 0.4, $fn = 48);
    }
}

// sawtooth ring outline: height rises with +theta, steep (radial) face at PHASE + 12k
function ratchet_pts(r_root, r_tip, n, phase) =
    [for (k = [0:n - 1]) for (j = [0:1]) let(a0 = phase + 360 * k / n)
        j == 0 ? r_root * dir2(a0) : r_tip * dir2(a0 + 360 / n - 0.8)];

module ratchet_cup_2d() {
    difference() {
        polygon(ratchet_pts(RT_EXT[0], RT_EXT[1], RT_N, RT_PHASE));
        polygon(ratchet_pts(RT_INT[0], RT_INT[1], RT_N, RT_PHASE));
    }
}

// flexure pawl: tip at a_tip pointing -theta (CW), arm runs back to a_anchor (+theta)
// lug_to = lug radius at the steep face, lug_to2 = radius 3.5 deg further (follows the tooth slope)
module pawl_2d(a_tip, a_anchor, arm, lug_to, lug_to2) {
    outward = lug_to > arm[1];
    union() {
        polygon(concat(arc_pts(arm[0], a_tip + 2, a_anchor, 30), arc_pts(arm[1], a_anchor, a_tip + 2, 30)));
        if (outward)
            polygon([arm[0] * dir2(a_tip), lug_to * dir2(a_tip), lug_to2 * dir2(a_tip + 3.5), arm[1] * dir2(a_tip + 7)]);
        else
            polygon([arm[1] * dir2(a_tip), lug_to * dir2(a_tip), lug_to2 * dir2(a_tip + 3.5), arm[0] * dir2(a_tip + 7)]);
    }
}
function slope_r(rr, frac) = rr[0] + (rr[1] - rr[0]) * frac;   // ratchet flank radius at fraction of the slope

// BLUE: sector gear (two 9-tooth sectors) + rear ratchet cup + hall index magnets
module sector_gear_part() {
    difference() {
        union() {
            xz(GEAR_Y0, GEAR_Y1) sector_gear_2d();
            xz(GEAR_Y1 - EPS, CUP_Y1) ratchet_cup_2d();
        }
        for (a = [0, 180]) xz(CUP_Y1 - 2, CUP_Y1 + 1) translate(19.3 * dir2(a)) circle(d = 3.2);   // 3x2 index magnets in the cup rim
    }
}

// drive disk with two flexure pawls, screwed to the MG995 round horn
module drive_disk_2d() {
    difference() {
        union() {
            circle(r = DR_CORE);
            for (a = DR_LUG) {
                pawl_2d(a, a + DR_ARC, DR_ARM, DR_LUG_TO, slope_r(RT_INT, 3.5 / (TAU - 0.8)) - 0.25);
                hull() { translate((DR_CORE - 1) * dir2(a + DR_ARC - 3)) circle(r = 1.6); translate(mean2(DR_ARM) * dir2(a + DR_ARC - 3)) circle(r = 1.6); }
            }
        }
        circle(d = 7.5);
        for (k = [0:3]) translate(HORN_PCD / 2 * dir2(45 + 90 * k)) circle(d = HORN_HOLE);
    }
}
function mean2(v) = (v[0] + v[1]) / 2;
module drive_disk_part() {
    difference() {
        union() { xz(DISK_Y0, DISK_Y1) drive_disk_2d(); xz(HUB_Y0, DISK_Y0 + EPS) circle(d = HUB_D); }
        xz(HUB_Y0 - 1, DISK_Y1 + 1) circle(d = 3.4);                    // M3x20 horn screw through the hub
    }
}

// anti-reverse flexure pawl (separate print, screws onto the frame post)
module antireverse_pawl_part() {
    post = AR_POST_R * dir2(AR_ANCHOR);
    difference() {
        xz(DISK_Y0, DISK_Y1 + 0.4) union() {
            pawl_2d(AR_LUG, AR_ANCHOR, AR_ARM, AR_LUG_TO, slope_r(RT_EXT, 3.5 / (TAU - 0.8)) + 0.25);
            hull() { translate(mean2(AR_ARM) * dir2(AR_ANCHOR - 4)) circle(r = 1.6); translate(post) circle(d = 8); }
        }
        xz(DISK_Y0 - 1, DISK_Y1 + 1) translate(post) circle(d = 3.4);
    }
}

/* =====================================================================
   2. RACK CARRIER (YELLOW teeth on RED shaft) - carrier local z: tooth 0 = 0
   ===================================================================== */
module rack_teeth_2d() {
    t = tan(PA); s = TOOTH_S;
    for (k = [0:RACK_TEETH - 1]) translate([0, k * PITCH])
        polygon([[RACK_TIP_X, -(s / 2 - MOD * t)], [RACK_ROOT_X + 0.3, -(s / 2 + 1.25 * MOD * t) - 0.1],
                 [RACK_ROOT_X + 0.3, s / 2 + 1.25 * MOD * t + 0.1], [RACK_TIP_X, s / 2 - MOD * t]]);
}
module rack_carrier_part() {
    difference() {
        union() {
            color(C_YELLOW) xz(-CAR_HW, RACK_FACE / 2) rack_teeth_2d();          // front side runs to the carrier face (flat on the bed)
            color(C_YELLOW) box([RACK_ROOT_X, -CAR_HW, CAR_BOT_L], [SHAFT_X, CAR_HW, CAR_RACK_TOP_L]);
            color(C_RED) box([SHAFT_X - 1, -CAR_HW, CAR_BOT_L], [CAR_X1, CAR_HW, CAR_RACK_TOP_L]);
            // LUG = upper spring-compression collar (underside) + spring-retention travel stop (top face)
            color(C_RED) box([RACK_ROOT_X, -CAR_HW, CAR_RACK_TOP_L], [LUG_X1, CAR_HW, LUG_TOP_L]);
        }
        translate([SHAFT_X, 0, CAR_BOT_L - 1]) cylinder(d = SHAFT_D + 0.25, h = LUG_TOP_L - CAR_BOT_L + 2);
        translate([SPRING_X, 0, CAR_RACK_TOP_L - 1]) cylinder(d = GUIDE_D + 0.6, h = LUG_T + 2);
        translate([SPRING_X, 0, CAR_RACK_TOP_L - 1]) cylinder(d = EJ_OD + 0.8, h = 1 + 1.2);  // spring seat
        for (s = [-1, 1]) box([GROOVE_X[0], s > 0 ? CAR_HW - GROOVE_D : -CAR_HW - 1, CAR_BOT_L - 1],
                              [GROOVE_X[1], s > 0 ? CAR_HW + 1 : -CAR_HW + GROOVE_D, LUG_TOP_L + 1]);
        for (zz = [12, 32]) translate([CAR_X1 - 6, 0, zz]) rotate([0, 90, 0]) cylinder(d = 2.6, h = 10);  // M3 grub into tube
    }
}

// red Al tube (D8x1) + trapdoor stop SHOULDER cap (load spine top)
module shaft_tube() { color(C_RED) translate([SHAFT_X, 0, CAR_BOT_L]) difference() { cylinder(d = SHAFT_D, h = TUBE_L); translate([0, 0, -1]) cylinder(d = SHAFT_ID, h = TUBE_L + 2); } }
module shoulder_cap_part() {
    difference() {
        union() { cylinder(d = SHOULDER_D, h = SHOULDER_T); translate([0, 0, -8]) cylinder(d = SHAFT_ID - 0.15, h = 8); }
        translate([0, 0, SHOULDER_T - 1]) cylinder(d = 8.4, h = 2);     // TPU pad pocket (pad bears on lid)
    }
}

/* =====================================================================
   3. GREEN BACKBONE FRAME (one print) + front axle bridge + black plate
   ===================================================================== */
BB_OUTLINE = [[-30, 66], [60, 66], [60, -42], [46, CH_BOT - 3], [24, CH_BOT - 3], [14, -30], [-22, -30], [-30, -20]];
MG995_WIN = [[-MG995_W / 2 - 0.35, -MG995_SHAFT_OFF - 0.35], [MG995_W / 2 + 0.35, MG995_L - MG995_SHAFT_OFF + 0.35]];
MG995_HOLES = [for (sz = [-1, 1]) for (sx = [-1, 1]) [sx * MG995_HOLE_SPAN / 2, MG995_CZ + sz * MG995_HOLE_PITCH / 2]];
BB_MOUNTS = [[-24, 58], [54, 58], [52, -48], [-16, -24]];
BP_HOLES = [[33, -8.5], [33, 7.5], [38, 16]];
LOW_WEB_TOP = LUG_UNDER_MIN - 2;

module backbone_frame_part() {
    color(C_GREEN) difference() {
        union() {
            // backbone plate with MG995 mounting window
            xz(BB_Y0, BB_Y1) difference() {
                polygon(BB_OUTLINE);
                translate(MG995_WIN[0]) square(MG995_WIN[1] - MG995_WIN[0]);
            }
            // rack channel: front wall, back wall, bottom stop, web to backbone, low end web
            box([CH_X0, CH_IN, CH_BOT], [CH_END, BB_Y0 + EPS, CH_TOP]);
            box([CH_X0, -CH_IN, CH_BOT - 3], [CH_END, BB_Y0 + EPS, CH_BOT]);
            box([CH_X1 - EPS, -CH_IN, CH_BOT], [CH_END, CH_IN + EPS, LOW_WEB_TOP]);
            // guide ridge (rides in the carrier groove)
            box([GROOVE_X[0] + 0.2, CH_IN - GROOVE_D, CH_BOT], [GROOVE_X[1] - 0.2, CH_IN + EPS, CH_TOP]);
            // hall-sensor post facing the cup-rim magnets
            box([HALL_POS[0] - 3, CUP_Y1 + 0.8, HALL_POS[1] - 3], [HALL_POS[0] + 3, BB_Y0 + EPS, HALL_POS[1] + 3]);
            // lower spring-seat SHELF (green) + bracket to backbone
            box([CH_X1, -9, SHELF_TOP - 5], [SPRING_X + 10, BB_Y0 + EPS, SHELF_TOP]);
            // post for the anti-reverse pawl
            ycyl(7, DISK_Y1 + 0.4, BB_Y0 + EPS, AR_POST_R * dir2(AR_ANCHOR));
        }
        // spring seat + guide tube socket
        translate([SPRING_X, 0, SHELF_TOP - 1.2]) cylinder(d = EJ_OD + 0.8, h = 2);
        translate([SPRING_X, 0, SHELF_TOP - 6]) cylinder(d = GUIDE_D + 0.1, h = 8);
        // MG995 tab screws, frame-to-shell mounts, pawl post pilot, front-bridge pilots
        for (p = MG995_HOLES) ycyl(3.4, BB_Y0 - 1, BB_Y1 + 1, p);
        for (p = BB_MOUNTS) ycyl(3.4, BB_Y0 - 1, BB_Y1 + 1, p);
        ycyl(2.6, DISK_Y1, BB_Y0 - 2, AR_POST_R * dir2(AR_ANCHOR));
        for (xx = [30, 38]) ycyl(2.6, -CH_IN - 1, -CH_IN + 8, [xx, CH_BOT - 1.5]);          // cover screws
        box([HALL_POS[0] - 2.3, CUP_Y1 + 0.7, HALL_POS[1] - 1.7], [HALL_POS[0] + 2.3, CUP_Y1 + 2.4, HALL_POS[1] + 1.7]);  // TO-92
        box([HALL_POS[0] - 1.5, CUP_Y1 + 2, HALL_POS[1] - 0.8], [HALL_POS[0] + 1.5, BB_Y1 + 1, HALL_POS[1] + 0.8]);       // leads
        // black-plate pilots in wall tops / web
        for (p = BP_HOLES) translate([p[0], p[1], CH_TOP - 10]) cylinder(d = 2.6, h = 11);
    }
}

// front channel cover (separate flat print): front wall + guide ridge + foot screwed to the bottom block
module channel_cover_part() {
    color(C_GREEN) difference() {
        union() {
            box([CH_X0, -CH_IN - CH_FWALL, CH_BOT - 3], [CH_END, -CH_IN, CH_BOT]);
            box([CH_X0, -CH_IN - CH_FWALL, CH_BOT - EPS], [CH_X1, -CH_IN, CH_TOP]);
            box([GROOVE_X[0] + 0.2, -CH_IN - EPS, CH_BOT], [GROOVE_X[1] - 0.2, -CH_IN + GROOVE_D, CH_TOP]);
        }
        for (xx = [30, 38]) ycyl(3.4, -CH_IN - CH_FWALL - 1, -CH_IN + 1, [xx, CH_BOT - 1.5]);
        translate([BP_HOLES[0][0], BP_HOLES[0][1], CH_TOP - 10]) cylinder(d = 2.6, h = 11);
    }
}

// BLACK upper guide plate: shaft bushing, spring-guide socket, adjustable M3 travel-stop screw
BP_X0 = -4;                                          // plate extended toward -x to carry the LED-wire anchor
ANCHOR_X = [BP_X0, 1]; ANCHOR_HW = 5; ANCHOR_H = 40; // tab rises 40 mm: anchor at mid-height of the wire-exit travel
module black_plate_part() {
    color(C_BLACK) difference() {
        union() {
            box([BP_X0, -CH_IN - CH_FWALL, BP_BOT], [SPRING_X + 10, BB_Y0, BP_TOP]);
            // LED-wire anchor tab (>= 31 mm from the shaft axis, outside the flame's 24.5 mm envelope)
            box([ANCHOR_X[0], -ANCHOR_HW, BP_TOP - EPS], [ANCHOR_X[1], ANCHOR_HW, BP_TOP + ANCHOR_H]);
            hull() { box([ANCHOR_X[1] - EPS, -2, BP_TOP - EPS], [ANCHOR_X[1], 2, BP_TOP + 10]);
                     box([ANCHOR_X[1], -2, BP_TOP - EPS], [ANCHOR_X[1] + 4, 2, BP_TOP]); }
        }
        translate([ANCHOR_X[0] - 1, 0, BP_TOP + ANCHOR_H - 4]) rotate([0, 90, 0]) cylinder(d = 4, h = 10);     // wire pass-through
        box([ANCHOR_X[0] - 1, -2, BP_TOP + ANCHOR_H - 12], [ANCHOR_X[1] + 1, 2, BP_TOP + ANCHOR_H - 10]);     // zip-tie slot
        translate([SHAFT_X, 0, BP_BOT - 1]) cylinder(d = SHAFT_D + 0.45, h = BP_T + 2);
        translate([SPRING_X, 0, BP_BOT - 1]) cylinder(d = GUIDE_D + 0.1, h = BP_T - 1);
        translate([40, 0, BP_BOT - 1]) cylinder(d = 3.2, h = BP_T + 2);                       // stop screw
        translate([40, 0, BP_TOP - 2.6]) cylinder(d = 6.4, h = 3, $fn = 6);                   // M3 nut trap
        for (p = BP_HOLES) { translate([p[0], p[1], BP_BOT - 1]) cylinder(d = 3.4, h = BP_T + 2);
                             translate([p[0], p[1], BP_TOP - 2]) cylinder(d = 6.2, h = 3); }
        for (yy = [-4, 4]) box([24, yy - 2, BP_BOT - 1], [27, yy + 2, BP_TOP + 1]);           // harness zip-tie
    }
}

/* =====================================================================
   4. SCULPTED MULTI-TONGUE FLAME  (flame local z = 0 at the collar bottom)
      flame_collar_part : grubs to the tube; flange carries the flame body
      sculpted_flame    : hollow body, slides over the LED band from below,
                          clamped between the collar and the shoulder cap
   ===================================================================== */
FLAME_R_MAX = 24.5;
COLLAR_FL_D = 28; COLLAR_FL_T = 2; COLLAR_HUB_D = 15.6; COLLAR_HUB_TOP = 7;
BODY_Z0  = COLLAR_FL_T;                        // flame body sits on the collar flange
BODY_BORE_D = 16.2; BODY_BORE_TOP = COLLAR_HUB_TOP + 0.3;
CORE_R0 = 18; CORE_SC = 0.30; CORE_AMP = 0.20; BODY_TW = -170;
BODY_H  = FLAME_H - BODY_Z0 - 3;              // core tip 3 mm under the spine top
CORE_TROUGH = 1 - CORE_AMP - 0.08;            // worst-case radius factor of the wave star
function core_r(z) = CORE_R0 * (1 - (1 - CORE_SC) * (z - BODY_Z0) / BODY_H);
CAV_WALL = 1.8;
CAV_Z = [BODY_BORE_TOP, 19.5];                // LED cavity (fits a 13 x 12 mm LED band around the tube)
CAV_R = [for (z = CAV_Z) CORE_TROUGH * core_r(z) - CAV_WALL];
CAV_CEIL_TOP = CAV_Z[1] + CAV_R[1] - (SHAFT_D + 0.35) / 2;   // 45 deg self-supporting ceiling
WIRE_EXIT_Z = 10; WIRE_EXIT_D = 3.5; WIRE_EXIT_ANG = 180;     // faces the anchor tab (-x)

function wave_star(r0, amp, n, ph, amp2 = 0.08, n2 = 7) =
    [for (i = [0:STAR_N - 1]) let(t = 360 * i / STAR_N, rr = r0 * (1 + amp * sin(n * t + ph) + amp2 * sin(n2 * t - 2 * ph))) rr * dir2(t)];
module tongue(r0, h, tw, sc, amp, n, ph) {
    linear_extrude(height = h, twist = tw, scale = sc, slices = max(10, round(h / (FINE ? 1.0 : 2.2))), convexity = 8)
        polygon(wave_star(r0, amp, n, ph));
}
module sculpted_flame() {
    sat_h  = [0.97, 0.80, 0.92, 0.72, 0.88, 0.66];
    sat_r  = [8.5, 9.0, 8.0, 9.5, 8.5, 9.0];
    color([1.0, 0.55, 0.08, 0.95]) difference() {
        intersection() {
            translate([0, 0, BODY_Z0]) union() {
                tongue(CORE_R0, BODY_H, BODY_TW, CORE_SC, CORE_AMP, 5, 0);                         // roaring core
                tongue(7.6, FLAME_H - BODY_Z0, -200, 0.85, 0.08, 4, 30);                         // spine sheath round the tube
                for (k = [0:5]) rotate([0, 0, 60 * k + 17 * (k % 2)]) translate([7.5, 0, 0])
                    rotate([0, 11 + 5 * (k % 3), 0])
                        tongue(sat_r[k], (FLAME_H - BODY_Z0) * sat_h[k] - 1, (k % 2 == 1 ? 1 : -1) * (130 + 25 * k), 0.06, 0.32, 3, 40 * k);
                for (k = [0:4]) rotate([0, 0, 36 + 72 * k]) translate([13, 0, 0]) rotate([0, 22, 0])
                    tongue(5.5, 22 + 4 * (k % 2), (k % 2 == 1 ? -1 : 1) * 160, 0.05, 0.35, 3, 15 * k);
            }
            translate([0, 0, BODY_Z0]) cylinder(r = FLAME_R_MAX, h = FLAME_H - BODY_Z0);
        }
        translate([0, 0, -1]) cylinder(d = SHAFT_D + 0.35, h = FLAME_H + 2);                       // tube bore
        translate([0, 0, BODY_Z0 - 1]) cylinder(d = BODY_BORE_D, h = BODY_BORE_TOP - BODY_Z0 + 1 + EPS);   // collar hub / LED entry
        rotate_extrude($fn = FINE ? 96 : 48) polygon([[0, CAV_Z[0] - EPS], [CAV_R[0], CAV_Z[0] - EPS], [CAV_R[1], CAV_Z[1]], [0, CAV_CEIL_TOP]]);
        translate([0, 0, WIRE_EXIT_Z]) rotate([0, 0, WIRE_EXIT_ANG]) rotate([0, 90, 0]) cylinder(d = WIRE_EXIT_D, h = FLAME_R_MAX + 2);
    }
}
module flame_collar_part() {
    color([1.0, 0.75, 0.3, 0.95]) difference() {
        union() { cylinder(d = COLLAR_FL_D, h = COLLAR_FL_T); cylinder(d = COLLAR_HUB_D, h = COLLAR_HUB_TOP); }
        translate([0, 0, -1]) cylinder(d = SHAFT_D + 0.35, h = COLLAR_HUB_TOP + 2);
        translate([0, 0, (COLLAR_FL_T + COLLAR_HUB_TOP) / 2]) rotate([0, 90, 0]) cylinder(d = 2.6, h = COLLAR_HUB_D);   // M3 grub (self-tap)
    }
}

/* ---------- TPU 95A pads (one print) ---------- */
module tpu_pads_part() {
    cylinder(d = 8, h = 1.5);                                  // stop pad: glued on the lug top under the M3 stop screw
    translate([14, 0, 0]) cylinder(d = 8, h = 1.0);            // shoulder-cap pad (pocket D8.4 x 1)
    translate([32, 0, 0]) cylinder(d = 12, h = 1.0);           // lid pad (pocket D12.4 x 1)
}

/* =====================================================================
   5. PUMPKIN SHELL (lobed oblate spheroid polyhedron)
   ===================================================================== */
function lobe_f(th) = 1 - LOBE_DEPTH * pow(1 - abs(cos(LOBES * th / 2)), LOBE_SHARP);
function shell_pt(th, ph, da) = let(rr = (HEAD_A - da) * sin(ph) * lobe_f(th)) [rr * cos(th), rr * sin(th), HEAD_C + (HEAD_C - da) * cos(ph)];
function sid(j, i, nt) = 1 + (j - 1) * nt + (i % nt);
module shell_solid(da = 0) {
    nt = SH_NT; np = SH_NP;
    pts = concat([[0, 0, HEAD_C + (HEAD_C - da)]],
                 [for (j = [1:np - 1]) for (i = [0:nt - 1]) shell_pt(360 * i / nt, 180 * j / np, da)],
                 [[0, 0, HEAD_C - (HEAD_C - da)]]);
    B = 1 + (np - 1) * nt;
    faces = concat([for (i = [0:nt - 1]) [0, sid(1, i + 1, nt), sid(1, i, nt)]],
                   [for (j = [1:np - 2]) for (i = [0:nt - 1]) [sid(j, i, nt), sid(j, i + 1, nt), sid(j + 1, i + 1, nt), sid(j + 1, i, nt)]],
                   [for (i = [0:nt - 1]) [B, sid(np - 1, i, nt), sid(np - 1, i + 1, nt)]]);
    polyhedron(points = pts, faces = faces, convexity = 6);
}
module inner_solid() { shell_solid(WALL); }

// crown = raised stem collar that carries the trapdoor ledge, hinge and latch
function crown_pts(r) = [for (i = [0:95]) let(t = 360 * i / 96) r * (1 - 0.06 * pow(1 - abs(cos(4 * t)), 2)) * dir2(t)];
module crown_outer() { translate([0, 0, CROWN_Z0]) linear_extrude(height = LID_TOP - CROWN_Z0, scale = CROWN_R_TOP / CROWN_R, convexity = 6) polygon(crown_pts(CROWN_R)); }
module crown_inner() { translate([0, 0, CROWN_Z0 - 1]) linear_extrude(height = LID_TOP - CROWN_Z0 + 2, scale = (CROWN_R_TOP - CROWN_WALL) / (CROWN_R - CROWN_WALL), convexity = 6) polygon(crown_pts(CROWN_R - CROWN_WALL)); }

/* ----- cat-eye geometry (eye frame: x right, y up, z out of the face) ----- */
function eye_pt(sx) = let(x = sx * EYE_X, z = EYE_Z) [x, -HEAD_A * sqrt(1 - pow(x / HEAD_A, 2) - pow((z - HEAD_C) / HEAD_C, 2)), z];
function eye_n(sx) = let(p = eye_pt(sx)) unit([p[0] / pow(HEAD_A, 2), p[1] / pow(HEAD_A, 2), (p[2] - HEAD_C) / pow(HEAD_C, 2)]);
module eye_frame(sx) {
    p = eye_pt(sx); w = eye_n(sx); u = unit(cross([0, 0, 1], w)); v = cross(w, u);
    multmatrix([[u[0], v[0], w[0], p[0]], [u[1], v[1], w[1], p[1]], [u[2], v[2], w[2], p[2]], [0, 0, 0, 1]]) children();
}
// right-eye almond: vesica + angry hood (upper lid slopes down to the nasal corner, -x)
module almond_2d(grow = 0) {
    R = EYE_H / 4 + EYE_L * EYE_L / (4 * EYE_H); d = R - EYE_H / 2;
    offset(delta = grow) rotate(EYE_TILT) intersection() {
        translate([0, -d]) circle(r = R, $fn = 180);
        translate([0, d]) circle(r = R, $fn = 180);
        polygon([[-EYE_L, -EYE_L], [EYE_L, -EYE_L], [EYE_L, EYE_H], [0.3 * EYE_L, EYE_H * 0.5], [-EYE_L / 2, 0], [-EYE_L, 0]]);
    }
}
module eye_2d(sx, grow = 0) { if (sx > 0) almond_2d(grow); else mirror([1, 0]) almond_2d(grow); }
module brow_2d(sx) {
    pts = [[-EYE_L / 2 - 4, 2.5], [EYE_L / 2 + 9, EYE_H * 0.5 + 9], [EYE_L / 2 + 3, EYE_H * 0.5 + 3.2], [-EYE_L / 2 + 1, 3.4]];
    if (sx > 0) rotate(EYE_TILT) polygon(pts); else mirror([1, 0]) rotate(EYE_TILT) polygon(pts);
}
module eye_bezel_add(sx) {
    eye_frame(sx) {
        translate([0, 0, -3.2]) linear_extrude(height = 5.4, convexity = 4) eye_2d(sx, 2.6);         // sharp raised bezel (+2.2 proud)
        translate([0, 0, -3.2]) linear_extrude(height = 5.1, convexity = 4) brow_2d(sx);             // swept brow ridge (+1.9 proud)
    }
}
module eye_cut(sx) {
    eye_frame(sx) hull() {
        translate([0, 0, -LCD_RECESS - 0.4]) linear_extrude(height = 0.1) eye_2d(sx, 0);
        translate([0, 0, 2.3]) linear_extrude(height = 0.1) eye_2d(sx, 2.0);          // chamfered, reads ~40 mm
    }
    eye_frame(sx) translate([0, 0, 2.3]) linear_extrude(height = 8) eye_2d(sx, 2.0);
}
// LCD socket fused behind each eye (module drops in from behind, retainer ring screws on)
module lcd_socket_add(sx) {
    eye_frame(sx) difference() {
        union() {
            translate([0, 0, -LCD_RECESS - LCD_STACK - 4]) cylinder(d = LCD_PCB_D + 7, h = LCD_STACK + 4 + LCD_RECESS + 2);
            for (s = [-1, 1]) translate([s * 26, 0, -LCD_RECESS - LCD_STACK - 4]) cylinder(d = 7, h = 6);
        }
        translate([0, 0, -LCD_RECESS - LCD_STACK - 5]) cylinder(d = LCD_PCB_D + 0.6, h = LCD_STACK + 5);
        translate([0, 0, -LCD_RECESS - 1]) cylinder(d = LCD_ACTIVE_D + 1.6, h = 20);
        translate([-LCD_TAB_W / 2, -LCD_TAB_REACH - 2, -LCD_RECESS - LCD_STACK - 5]) cube([LCD_TAB_W, LCD_TAB_REACH, LCD_STACK + 5]);
        for (s = [-1, 1]) translate([s * 26, 0, -LCD_RECESS - LCD_STACK - 5]) cylinder(d = 2.2, h = 6);
    }
}
module lcd_retainer_part() {
    difference() {
        union() { cylinder(d = LCD_PCB_D + 7, h = 2); for (s = [-1, 1]) hull() { translate([s * 19, 0, 0]) cylinder(d = 7, h = 2); translate([s * 26, 0, 0]) cylinder(d = 7, h = 2); } }   // solid ear lugs
        translate([0, 0, -1]) cylinder(d = 30, h = 4);
        translate([-LCD_TAB_W / 2, -LCD_TAB_REACH - 2, -1]) cube([LCD_TAB_W, LCD_TAB_REACH - 12, 4]);   // FPC-tail notch (runs 1 mm into the window)
        for (s = [-1, 1]) translate([s * 26, 0, -1]) cylinder(d = 2.8, h = 4);
    }
}
// jagged grin, engraved 0.8 mm (no through-holes)
MOUTH = [[-62, 82], [-46, 70], [-38, 79], [-26, 62], [-16, 74], [-6, 58], [6, 74], [16, 58], [26, 72], [38, 60], [48, 76], [64, 84],
         [52, 74], [44, 56], [30, 50], [16, 44], [0, 42], [-16, 44], [-30, 50], [-44, 56], [-54, 70]];
module mouth_engrave() {
    intersection() {
        difference() { shell_solid(-1); shell_solid(0.9); }
        translate([0, -40, 0]) rotate([90, 0, 0]) linear_extrude(height = 100, convexity = 6) polygon(MOUTH);
    }
}

/* ----- split-plane flange, tongue/groove, magnets ----- */
SEC_RX = (HEAD_A - WALL) * sqrt(1 - pow(SPLIT_Y / (HEAD_A - WALL), 2));
SEC_RZ = (HEAD_C - WALL) * sqrt(1 - pow(SPLIT_Y / (HEAD_A - WALL), 2));
MAG_D = 8; MAG_H = 3;
MAG_ANG = [15, 60, 105, 150, 195, 240, 300, 345];
DOWEL_ANG = [37, 128, 218, 322];                     // D3 x 10 dowels (3 mm Al rod / cut nail)
function mag_pt(a) = [(SEC_RX - 8) * cos(a), HEAD_C + (SEC_RZ - 8) * sin(a)];
module flange_band() {
    intersection() { inner_solid(); translate([-200, SPLIT_Y - 4, -10]) cube([400, 8, 400]); }
}
module flange_add() {
    difference() { flange_band(); shell_solid(WALL + 6); }
    intersection() { inner_solid(); for (a = concat(MAG_ANG, DOWEL_ANG)) ycyl(14, SPLIT_Y - 6, SPLIT_Y + 6, mag_pt(a)); }
}
module magnet_pockets() {
    for (a = MAG_ANG) ycyl(MAG_D + 0.25, SPLIT_Y - MAG_H - 0.2, SPLIT_Y + MAG_H + 0.2, mag_pt(a));
    for (a = DOWEL_ANG) ycyl(3.2, SPLIT_Y - 5.2, SPLIT_Y + 5.2, mag_pt(a));
}

/* ----- back-half internals (head frame), trimmed to the inner surface ----- */
module back_internals() {
    // head neck cup (spring captured CUP_DEPTH)
    translate([0, 0, -1]) cylinder(d = CUP_OD, h = CUP_DEPTH + 3 + 1);
    // mechanism mounting posts (shell rear wall -> backbone rear face)
    for (p = BB_MOUNTS) ycyl(11, MECH_Y + BB_Y1, 140, [MECH_X + p[0], MECH_Z + p[1]]);
    // IMU pad
    box([IMU_PAD[0][0], IMU_PAD[0][1], -10], [IMU_PAD[1][0], IMU_PAD[1][1], 14]);
    // ESP32-S3 shelf + cradle rails
    box([22, -6, ESP_SHELF_Z - 3], [140, 20, ESP_SHELF_Z]);
    for (yy = [-6, 17]) box([22, yy, ESP_SHELF_Z - EPS], [94, yy + 3, ESP_SHELF_Z + 5]);
    // reserved bay platforms
    box([BAY_SMOKE[0][0], BAY_SMOKE[0][1], BAY_SMOKE[0][2] - 3], [160, BAY_SMOKE[1][1], BAY_SMOKE[0][2]]);
    box([-160, BAY_MOTOR[0][1], BAY_MOTOR[0][2] - 3], [BAY_MOTOR[1][0], BAY_MOTOR[1][1], BAY_MOTOR[0][2]]);
    // MG90S mounting plate (tabs rest on top)
    box([38, -22, MG90_SPLINE_TOP - MG90_SPLINE - MG90_BODY_H + MG90_TAB_Z - 3],
        [160, 24, MG90_SPLINE_TOP - MG90_SPLINE - MG90_BODY_H + MG90_TAB_Z]);
}
MG90_Z0 = MG90_SPLINE_TOP - MG90_SPLINE - MG90_BODY_H;      // MG90S body bottom
MG90_BODY_Y = [MG90_AXIS[1] - MG90_SHAFT_OFF, MG90_AXIS[1] - MG90_SHAFT_OFF + MG90_L];   // body runs +Y (tangential)
module back_internal_cuts() {
    // neck cup bore, umbilical (off-centre, clear of the rack channel), grub screws
    translate([0, 0, -2]) cylinder(d = CUP_ID, h = CUP_DEPTH + 2);
    translate([-11.5, -4, -2]) cylinder(d = 8, h = 30);
    for (a = [90, 210]) translate([0, 0, 5]) rotate([0, 0, a]) rotate([0, 90, 0]) cylinder(d = 2.6, h = 30);
    // M3 heat-set inserts in the mech posts
    for (p = BB_MOUNTS) ycyl(4.1, MECH_Y + BB_Y1 - 1, MECH_Y + BB_Y1 + 8, [MECH_X + p[0], MECH_Z + p[1]]);
    // IMU M2.5 pilots
    for (xx = [-41, -29]) translate([xx, -12, 6]) cylinder(d = 2.2, h = 9);
    // bay M3 insert pilots
    for (b = [BAY_SMOKE, BAY_MOTOR]) for (xx = [b[0][0] + 5, b[1][0] - 5]) for (yy = [b[0][1] + 5, b[1][1] - 5])
        translate([xx, yy, b[0][2] - 4]) cylinder(d = 4.1, h = 5);
    // MG90S window + M2 pilots
    translate([MG90_AXIS[0] - MG90_W / 2 - 0.3, MG90_BODY_Y[0] - 0.3, MG90_Z0 - 5]) cube([MG90_W + 0.6, MG90_L + 0.6, MG90_TAB_Z + 6]);
    for (s = [-1, 1]) translate([MG90_AXIS[0], mean2(MG90_BODY_Y) + s * MG90_HOLE_PITCH / 2, MG90_Z0 + MG90_TAB_Z - 8]) cylinder(d = 1.8, h = 9);
}

/* ----- crown: ledge, hinge knuckle, hold-open stop, bolt channel, switch block ----- */
module crown_add() {
    difference() { crown_outer(); crown_inner(); }
    translate([0, 0, LID_TOP - 2.4]) difference() { linear_extrude(height = 2.4) polygon(crown_pts(CROWN_R_TOP - 0.2)); translate([0, 0, -1]) cylinder(r = OPEN_R, h = 5); }
    translate([0, 0, LID_UNDER - LEDGE_T]) difference() { cylinder(r = OPEN_R + 2.4, h = LID_TOP - LID_UNDER + LEDGE_T); translate([0, 0, -1]) cylinder(r = OPEN_R, h = 20); }
    translate([0, 0, LID_UNDER - LEDGE_T]) difference() { cylinder(r = LEDGE_R[1], h = LEDGE_T); translate([0, 0, -1]) cylinder(r = LEDGE_R[0], h = 5); }
    // shell hinge knuckle
    hull() { translate([HINGE_X, 0, HINGE_Z]) rotate([90, 0, 0]) cylinder(d = KNUCKLE_D, h = 10, center = true);
             box([HINGE_X - 3.5, -5, LID_TOP - 2.4], [HINGE_X + 2, 5, LID_TOP]); }
    // hold-open stop: contact face = lid top plane rotated LID_OPEN_MAX about the hinge
    hull() {
        translate([HINGE_X, 0, HINGE_Z]) rotate([0, -LID_OPEN_MAX, 0]) translate([7, -5, LID_TOP - HINGE_Z]) cube([12, 10, 1.2]);
        box([HINGE_X - 9, -5, LID_TOP - 2.4], [HINGE_X - 4.5, 5, LID_TOP]);
    }
    // radial deadbolt guide channel (hangs under the ledge, right side)
    box([36.4, -BOLT_HW - 1.6, BOLT_Z[0] - 2.2], [39.6, BOLT_HW + 1.6, BOLT_Z[0] - 0.2]);                // floor
    box([36.4, -BOLT_HW - 1.6, BOLT_Z[1] + 0.2], [39.6, BOLT_HW + 1.6, LID_UNDER - LEDGE_T + EPS]);      // roof
    for (s = [-1, 1]) box([36.4, s > 0 ? BOLT_HW + 0.2 : -BOLT_HW - 1.6, BOLT_Z[0] - 2.2], [39.6, s > 0 ? BOLT_HW + 1.6 : -BOLT_HW - 0.2, LID_UNDER - LEDGE_T + EPS]);
    // KW11-3Z lid micro-switch block
    box([SW_BOX[0][0] - 2, SW_BOX[0][1] - 2, LID_UNDER - LEDGE_T - 12.3], [SW_BOX[1][0] + 2, SW_BOX[1][1] + 2, LID_UNDER - LEDGE_T + EPS]);
}
module crown_cuts() {
    translate([0, 0, LID_UNDER]) cylinder(r = OPEN_R, h = 20);
    translate([0, 0, LID_UNDER - 40]) cylinder(r = LEDGE_R[0], h = 60);
    box([31.5, -KEEPER_W / 2 - 0.6, LID_UNDER - LEDGE_T - 1], [38.2, KEEPER_W / 2 + 0.6, LID_UNDER + 1]);   // keeper notch (swing clearance)
    translate([HINGE_X, 0, HINGE_Z]) rotate([90, 0, 0]) cylinder(d = 2.9, h = 40, center = true);                           // pin (self-tap M3)
    translate([HINGE_X, 0, HINGE_Z]) rotate([0, -LID_OPEN_MAX, 0]) translate([14, 0, LID_TOP - HINGE_Z - 0.1]) cylinder(d = 6.2, h = 2.4);   // hold-open magnet
    box([30, -BOLT_HW - 0.2, BOLT_Z[0] - 0.2], [42, BOLT_HW + 0.2, BOLT_Z[1] + 0.2]);                                     // bolt path
    box([SW_BOX[0][0], SW_BOX[0][1], LID_UNDER - LEDGE_T - 10.4], [SW_BOX[1][0], SW_BOX[1][1], LID_UNDER - LEDGE_T + EPS]);   // switch body
    box([SW_BOX[0][0] + 3, mean2([SW_BOX[0][1], SW_BOX[1][1]]) - 1.8, LID_UNDER - LEDGE_T - 1], [SW_BOX[1][0] - 3, mean2([SW_BOX[0][1], SW_BOX[1][1]]) + 1.8, LID_UNDER + 1]);   // lever slot
    for (xx = [SW_BOX[0][0] + 5.25, SW_BOX[0][0] + 14.75]) translate([xx, SW_BOX[0][1] - 3, LID_UNDER - LEDGE_T - 5.5]) rotate([-90, 0, 0]) cylinder(d = 1.8, h = 12);
    translate([-22, -30, LID_UNDER - 20]) cylinder(d = 10, h = 30);                                                       // smoke duct port
}

/* ----- shells ----- */
module shell_body() {
    difference() {
        union() {
            difference() {
                difference() { shell_solid(0); inner_solid(); }
                translate([0, 0, CROWN_Z0]) linear_extrude(height = 100) polygon(crown_pts(CROWN_R - CROWN_WALL - 0.3));
            }
            crown_add();
            for (sx = [-1, 1]) eye_bezel_add(sx);
            for (sx = [-1, 1]) intersection() { inner_solid(); lcd_socket_add(sx); }
            intersection() { inner_solid(); back_internals(); }
            flange_add();
        }
        for (sx = [-1, 1]) eye_cut(sx);
        mouth_engrave();
        crown_cuts();
        back_internal_cuts();
        magnet_pockets();
    }
}
module head_front_part() { intersection() { shell_body(); translate([-200, -200, -10]) cube([400, 200 + SPLIT_Y, 400]); } }
module head_back_part()  { intersection() { shell_body(); translate([-200, SPLIT_Y, -10]) cube([400, 300, 400]); } }

/* =====================================================================
   6. TRAPDOOR (LID), STEM, KEEPER, DEADBOLT
   ===================================================================== */
module lid_part() {
    color([0.45, 0.28, 0.12]) difference() {
        union() {
            translate([0, 0, LID_UNDER]) cylinder(r = LID_R, h = LID_T);
            translate([0, 0, LID_TOP - EPS]) cylinder(d = 15, h = 4);
            for (s = [-1, 1]) hull() {                                                              // knuckle straps on the lid top
                translate([HINGE_X, s * 9.5, HINGE_Z]) rotate([90, 0, 0]) cylinder(d = KNUCKLE_D, h = 7.4, center = true);
                box([-LID_R + 8, s * 9.5 - 3.7, LID_TOP - EPS], [-LID_R + 12, s * 9.5 + 3.7, LID_TOP + 2]);
            }
        }
        translate([HINGE_X, 0, HINGE_Z]) rotate([90, 0, 0]) cylinder(d = PIN_D + 0.2, h = 40, center = true);
        translate([0, 0, LID_TOP + 4 - 6]) cylinder(d = 6.2, h = 7);                                // stem socket
        translate([0, 0, LID_TOP + 4 - 3.05]) cylinder(r1 = 3.15, r2 = 6.2, h = 3.05 + EPS);        // countersink for the stem cone
        translate([0, 0, LID_UNDER - 1]) cylinder(d = 12.4, h = 2);                                // TPU pad (spine contact)
        translate([HINGE_X + 14, 0, LID_TOP - 2.2]) cylinder(d = 6.2, h = 3);                      // hold-open magnet
        box([KEEPER_X - KEEPER_T / 2 - 0.15, -3.1, LID_UNDER - 1], [KEEPER_X + KEEPER_T / 2 + 0.15, 3.1, LID_UNDER + 2]);   // keeper tenon slot
    }
}
module stem_part() {
    color([0.30, 0.42, 0.15]) union() {
        translate([0, 0, -6]) cylinder(d = 5.9, h = 6.2);
        translate([0, 0, -3.05]) cylinder(r1 = 2.95, r2 = 6.0, h = 3.05 + EPS);       // 45 deg base cone (no overhang ring)
        for (i = [0:13]) let(z = i * 2, t = i / 13)
            hull() {
                translate([3.2 * t * t * 3, 0, z]) rotate([0, 0, -9 * i]) linear_extrude(height = EPS) polygon(wave_star(6.5 - 2.3 * t, 0.12, 6, 0, 0, 7));
                translate([3.2 * pow((i + 1) / 13, 2) * 3, 0, z + 2]) rotate([0, 0, -9 * (i + 1)]) linear_extrude(height = EPS) polygon(wave_star(6.5 - 2.3 * (i + 1) / 13, 0.12, 6, 0, 0, 7));
            }
    }
}
module keeper_part() {
    color(C_BLACK) difference() {
        union() {
            box([KEEPER_X - KEEPER_T / 2, -KEEPER_W / 2, LID_UNDER - KEEPER_DROP], [KEEPER_X + KEEPER_T / 2, KEEPER_W / 2, LID_UNDER]);
            box([KEEPER_X - KEEPER_T / 2, -3, LID_UNDER - EPS], [KEEPER_X + KEEPER_T / 2, 3, LID_UNDER + 1.8]);
        }
        box([KEEPER_X - 2, -BOLT_HW - 0.4, BOLT_Z[0] - 0.3], [KEEPER_X + 2, BOLT_HW + 0.4, BOLT_Z[1] + 0.3]);
    }
}
module deadbolt_part() {
    color(C_STEEL) difference() {
        union() {
            box([BOLT_X[0], -BOLT_HW, BOLT_Z[0]], [BOLT_X[1] + EPS, BOLT_HW, BOLT_Z[1]]);
            box([YOKE_X[0], -YOKE_HW, BOLT_Z[0]], [YOKE_X[1], YOKE_HW, BOLT_Z[0] + 2.4]);
        }
        box([YOKE_SLOT_X - 1.3, -3.2, BOLT_Z[0] - 1], [YOKE_SLOT_X + 1.3, 3.2, BOLT_Z[1] + 1]);          // scotch-yoke slot (pin travels in Y)
        translate([BOLT_X[0], 0, BOLT_Z[1]]) rotate([0, 45, 0]) cube([2, 2 * BOLT_HW + 2, 2], center = true);   // lead-in chamfer
    }
}

/* =====================================================================
   7. BASE + NECK MOUNT
   ===================================================================== */
module rounded_square(w, r) { offset(r = r) square(w - 2 * r, center = true); }
module base_shell_part() {
    color([0.15, 0.15, 0.17]) difference() {
        union() {
            difference() {
                linear_extrude(height = BASE_H) rounded_square(BASE_W, BASE_R);
                translate([0, 0, -1]) linear_extrude(height = BASE_H - BASE_TOP_T + 1) rounded_square(BASE_W - 2 * BASE_WALL, BASE_R - BASE_WALL);
            }
            for (sx = [-1, 1]) for (sy = [-1, 1]) translate([sx * (PLATE[0] / 2 - 9), sy * (PLATE[1] / 2 - 9), PLATE[2] + 0.2]) cylinder(d = 13, h = BASE_H - PLATE[2] - 0.2 - EPS);
            // component cradles (hang from the top skin): 3S 18650 holder, XL4015 (6 V), LM2596 (5 V), blade fuse
            for (c = [[[-60, 35], [79, 62]], [[45, 55], [53, 28]], [[45, 5], [45, 23]], [[-60, -45], [32, 14]]])
                translate([c[0][0], c[0][1], BASE_H - BASE_TOP_T - 12]) difference() {
                    linear_extrude(height = 12 + EPS) offset(delta = 2) square(c[1], center = true);
                    translate([0, 0, -1]) linear_extrude(height = 14) square(c[1], center = true);
                    for (s = [-1, 1]) translate([s * c[1][0] / 4, 0, 3]) cube([4, c[1][1] + 10, 2.5], center = true);
                }
        }
        // plate screws (M4 heat-set), rocker switch, DC jack, vents, umbilical, neck-mount pilots
        for (sx = [-1, 1]) for (sy = [-1, 1]) translate([sx * (PLATE[0] / 2 - 9), sy * (PLATE[1] / 2 - 9), PLATE[2]]) cylinder(d = 5.6, h = 10);
        translate([-40, BASE_W / 2 - 4, 30]) cube([19.4, 10, 13.1], center = true);
        translate([10, BASE_W / 2 - 4, 30]) rotate([-90, 0, 0]) cylinder(d = 11.2, h = 10);
        for (i = [0:5]) translate([50 + i * 9, BASE_W / 2 - 4, 30]) cube([3, 10, 22], center = true);
        translate([0, 0, BASE_H - 5]) cylinder(d = 16, h = 10);
        for (k = [0:5]) translate([(MOUNT_D / 2 - 8) * cos(60 * k), (MOUNT_D / 2 - 8) * sin(60 * k), BASE_H - 8]) cylinder(d = 2.6, h = 10);
    }
}
module neck_mount_part() {
    color([0.18, 0.18, 0.2]) difference() {
        union() {
            translate([0, 0, BASE_H]) cylinder(d = MOUNT_D, h = MOUNT_T);
            translate([0, 0, BASE_H]) cylinder(d = CUP_OD, h = MOUNT_T + CUP_DEPTH);
            // tilt-limit ring: head rim lands here at TILT_STOP degrees
            translate([0, 0, BASE_H]) difference() {
                cylinder(r1 = TILT_X + 9, r2 = TILT_X + 5, h = TILT_Z - BASE_H);
                translate([0, 0, -1]) cylinder(r1 = TILT_X - 9, r2 = TILT_X - 5, h = TILT_Z - BASE_H + 2);
            }
        }
        translate([0, 0, NECK_FLOOR_Z]) cylinder(d = CUP_ID, h = CUP_DEPTH + 1);
        translate([0, 0, BASE_H - 1]) cylinder(d = 14, h = 20);
        for (a = [90, 210]) translate([0, 0, NECK_FLOOR_Z + 5]) rotate([0, 0, a]) rotate([0, 90, 0]) cylinder(d = 2.6, h = 30);
        for (k = [0:5]) translate([(MOUNT_D / 2 - 8) * cos(60 * k), (MOUNT_D / 2 - 8) * sin(60 * k), BASE_H - 1]) { cylinder(d = 3.4, h = 10); translate([0, 0, MOUNT_T - 1.8]) cylinder(d = 6.4, h = 3); }
    }
}

/* =====================================================================
   8. PURCHASED-PART STAND-INS (assembly only)
   ===================================================================== */
module coil(wire, od, turns, len) { linear_extrude(height = len, twist = -360 * turns, slices = max(48, turns * 40), convexity = 10) translate([(od - wire) / 2, 0]) circle(d = wire, $fn = 10); }
module mg995_dummy() {
    color([0.1, 0.1, 0.1]) {
        box([-MG995_W / 2, BB_Y1 - MG995_CASE_ABOVE_TAB, -MG995_SHAFT_OFF], [MG995_W / 2, BB_Y1 + (MG995_H - MG995_SPLINE_ABOVE_TAB), MG995_L - MG995_SHAFT_OFF]);
        box([-MG995_W / 2, BB_Y1, MG995_CZ - MG995_TAB_LEN / 2], [MG995_W / 2, BB_Y1 + MG995_TAB_T, MG995_CZ + MG995_TAB_LEN / 2]);
    }
    color(C_STEEL) ycyl(6, SPLINE_TOP_Y, BB_Y1 - MG995_CASE_ABOVE_TAB, [0, 0]);
    color([0.85, 0.85, 0.88]) ycyl(HORN_D, DISK_Y1, DISK_Y1 + HORN_T, [0, 0]);
}
module mg90s_dummy(phi) {   // phi = crank angle, -45 engaged .. +45 retracted
    pin = CRANK_R * [sin(phi), cos(phi)];
    translate([MG90_AXIS[0], 0, 0]) color([0.15, 0.2, 0.75]) {
        box([-MG90_W / 2, MG90_BODY_Y[0], MG90_Z0], [MG90_W / 2, MG90_BODY_Y[1], MG90_Z0 + MG90_BODY_H]);
        box([-MG90_W / 2, mean2(MG90_BODY_Y) - MG90_TAB_LEN / 2, MG90_Z0 + MG90_TAB_Z], [MG90_W / 2, mean2(MG90_BODY_Y) + MG90_TAB_LEN / 2, MG90_Z0 + MG90_TAB_Z + MG90_TAB_T]);
    }
    translate([MG90_AXIS[0], MG90_AXIS[1], MG90_SPLINE_TOP]) {
        color([0.9, 0.9, 0.9]) { hull() { cylinder(d = 6, h = 1.5); translate(pin) cylinder(d = 4, h = 1.5); } translate(pin) cylinder(d = 1.9, h = 5); }
        color(C_STEEL) translate([0, 0, -MG90_SPLINE]) cylinder(d = 4.8, h = MG90_SPLINE);
    }
}
module lcd_dummy(sx) {
    eye_frame(sx) translate([0, 0, -LCD_RECESS - LCD_STACK]) {
        color([0.05, 0.05, 0.08]) cylinder(d = LCD_PCB_D, h = LCD_STACK - 0.05);
        color([1, 0.45, 0.0]) translate([0, 0, LCD_STACK - 0.1]) linear_extrude(height = 0.12) intersection() { eye_2d(sx, 0.5); scale([0.18, 1]) circle(d = 26); }
        color([0.9, 0.12, 0.0]) translate([0, 0, LCD_STACK - 0.15]) linear_extrude(height = 0.1) eye_2d(sx, 0.5);
    }
}

/* =====================================================================
   9. MECHANISM ASSEMBLY (mech frame) and ANIMATION
   ===================================================================== */
function anim_state(T) =
    let(pb = seg(T, 0.05, 0.08), pf = seg(T, 0.08, 0.10), pc = seg(T, 0.28, 0.52), pl = seg(T, 0.55, 0.65),
        pi = seg(T, 0.66, 0.70), pr = seg(T, 0.72, 0.84), pz = seg(T, 0.86, 0.97),
        h  = T < 0.28 ? 0 : T < 0.52 ? HOLD_G * ease(pc) : T < 0.72 ? HOLD_G : T < 0.86 ? HOLD_G + (180 - HOLD_G) * pr : 180 * (1 - ease(pz)),
        g  = T < 0.28 ? 0 : T < 0.86 ? h : 180,
        dz = T < 0.08 ? -EJECT : T < 0.10 ? -EJECT * (1 - pf * pf) : T < 0.28 ? 0 : T < 0.72 ? -rack_drop(g) : (g < REL_G ? -rack_drop(g) : -EJECT),
        lid = T < 0.08 ? 0 : T < 0.55 ? LID_OPEN_MAX * ease(seg(T, 0.085, 0.12)) : LID_OPEN_MAX * (1 - ease(pl)),
        bolt = T < 0.05 ? 0 : T < 0.66 ? ease(pb) : 1 - ease(pi),
        step = T < 0.05 ? "ARMED: spring load on lid via spine, gear parked, horn at 0"
             : T < 0.28 ? "STEP 1: buzzer+strobe, MG90S bolt out, flame fires 47.2 mm (no tooth in band)"
             : T < 0.52 ? "STEP 2: MG995 0->132 deg, rack pulled 49.2 mm, hold on last tooth (pawl)"
             : T < 0.66 ? "STEP 3: user closes trapdoor -> KW11 switch"
             : T < 0.72 ? "STEP 4a: MG90S deadbolt in, wait 300 ms"
             : T < 0.86 ? "STEP 4b: MG995 132->180 deg, tip-slip release at 162.25 deg onto lid stop"
             : "RESET: horn 180->0 deg; drive pawls ratchet, gear stays parked")
    [h, g, dz, lid, bolt, step];

module mechanism(h, g, dz) {
    backbone_frame_part();
    channel_cover_part();
    black_plate_part();
    antireverse_pawl_part_c();
    color(C_BLUE) rotate([0, g, 0]) sector_gear_part();
    color([0.35, 0.45, 0.95]) rotate([0, h, 0]) drive_disk_part();
    mg995_dummy();
    color(C_STEEL) rotate([0, h, 0]) { ycyl(10, HUB_Y0 - 0.6, HUB_Y0, [0, 0]); ycyl(5.5, HUB_Y0 - 3.6, HUB_Y0 - 0.6, [0, 0]); }   // M3 horn screw + washer
    color(C_STEEL) translate([SPRING_X, 0, SHELF_TOP - 5]) cylinder(d = GUIDE_D, h = BP_TOP - 1 - (SHELF_TOP - 5));
    color(C_STEEL) translate([40, 0, BP_BOT - STOP_PROTRUDE]) cylinder(d = 3, h = BP_T + STOP_PROTRUDE + 1);
    // moving plunger assembly
    translate([0, 0, Z_T0_TOP + dz]) {
        rack_carrier_part();
        shaft_tube();
        translate([SHAFT_X, 0, CAR_BOT_L + TUBE_L]) color(C_RED) shoulder_cap_part();
        translate([SHAFT_X, 0, FLAME_BASE_TOP - Z_T0_TOP]) { flame_collar_part(); sculpted_flame(); }
    }
    // ejector spring between shelf and lug
    sl = Z_LUG_UNDER_TOP + dz - SHELF_TOP;
    color(C_SPRING) translate([SPRING_X, 0, SHELF_TOP]) coil(EJ_WIRE, EJ_OD, EJ_TURNS, sl);
}
module antireverse_pawl_part_c() { color([0.2, 0.55, 0.2]) antireverse_pawl_part(); }

module head_assembly(st) {
    translate([MECH_X, MECH_Y, MECH_Z]) mechanism(st[0], st[1], st[2]);
    translate([HINGE_X, 0, HINGE_Z]) rotate([0, -st[3], 0]) translate([-HINGE_X, 0, -HINGE_Z]) {
        lid_part(); keeper_part();
        translate([0, 0, LID_TOP + 4]) stem_part();
    }
    translate([BOLT_STROKE * st[4], 0, 0]) deadbolt_part();
    mg90s_dummy(-45 + 90 * st[4]);
    for (sx = [-1, 1]) lcd_dummy(sx);
    if (show_reserved_bays) {
        color(C_GHOST) box(BAY_SMOKE[0], BAY_SMOKE[1]);
        color(C_GHOST) box(BAY_MOTOR[0], BAY_MOTOR[1]);
        color([0.2, 0.2, 0.8, 0.5]) box([24, -4, ESP_SHELF_Z], [93, 21.5, ESP_SHELF_Z + 13]);   // ESP32-S3 devkit
        color([0.8, 0.3, 0.2, 0.7]) box([-42, -18, 14], [-26, -4, 17]);                        // MPU6050
    }
    if (show_shell) {
        color([0.95, 0.42, 0.04, shell_alpha]) head_back_part();
        color([0.95, 0.42, 0.04, shell_alpha]) head_front_part();
    }
}

module full_assembly() {
    T = anim_t >= 0 ? anim_t : $t;
    st = anim_state(T);
    echo(str("t=", T, "  horn=", st[0], "  gear=", st[1], "  rack dz=", st[2], "  lid=", st[3], "  | ", st[5]));
    base_shell_part();
    neck_mount_part();
    color([0.3, 0.3, 0.32]) translate([-PLATE[0] / 2, -PLATE[1] / 2, 0]) cube(PLATE);
    // neck spring (rendered straight; head_wobble tilts the head about the spring mid-point)
    color(C_SPRING) translate([0, 0, NECK_FLOOR_Z]) coil(NECK_WIRE, NECK_OD, NECK_TURNS, HEAD_Z0 + CUP_DEPTH - NECK_FLOOR_Z);
    translate([0, 0, PIVOT_Z]) rotate([0, head_wobble, 0]) translate([0, 0, HEAD_Z0 - PIVOT_Z]) head_assembly(st);
}

/* =====================================================================
   10. PRINT LAYOUTS (flat-oriented, 256 x 256 bed)
   ===================================================================== */
module print_mechanism_plate() {
    // backbone frame: rear face of the backbone on the bed, walls rise
    translate([-60, 30, BB_Y1]) rotate([-90, 0, 0]) backbone_frame_part();
    // sector gear: front face down, ratchet cup up
    translate([60, 80, -GEAR_Y0]) rotate([90, 0, 0]) sector_gear_part();
    // drive disk (horn face down, hub up) + anti-reverse pawl: flexures in the XY plane
    translate([60, 25, DISK_Y1]) rotate([-90, 0, 0]) drive_disk_part();
    translate([95, 40, -DISK_Y0]) rotate([90, 0, 0]) antireverse_pawl_part();
    // rack carrier on its side
    translate([-20, -60, CAR_HW]) rotate([90, 0, 0]) rack_carrier_part();
    // channel cover: outer face on the bed
    translate([-150, 0, CH_IN + CH_FWALL]) rotate([90, 0, 0]) channel_cover_part();
    // black plate
    translate([60, -60, -BP_BOT]) black_plate_part();
    // shoulder cap (upside down -> plug up)
    translate([105, -100, SHOULDER_T]) rotate([180, 0, 0]) shoulder_cap_part();
}

/* ----- one part per STL, oriented flat for printing (render_mode = "part") ----- */
PART_LIST = ["base_shell", "neck_mount", "head_front", "head_back", "backbone_frame", "channel_cover",
             "black_plate", "sector_gear", "drive_disk", "antireverse_pawl", "rack_carrier", "shoulder_cap",
             "flame_body", "flame_collar", "trapdoor_lid", "stem", "keeper", "deadbolt", "lcd_retainer", "tpu_pads"];
module print_part(name) {
    if (name == "base_shell")            translate([0, 0, BASE_H]) rotate([180, 0, 0]) base_shell_part();        // top skin down
    else if (name == "neck_mount")       translate([0, 0, -BASE_H]) neck_mount_part();                             // flange down
    else if (name == "head_front")       translate([0, 0, SPLIT_Y]) rotate([-90, 0, 0]) head_front_part();          // seam down, face up
    else if (name == "head_back")        translate([0, 0, -SPLIT_Y]) rotate([90, 0, 0]) head_back_part();           // seam down
    else if (name == "backbone_frame")   translate([0, 0, BB_Y1]) rotate([-90, 0, 0]) backbone_frame_part();       // rear face down
    else if (name == "channel_cover")    translate([0, 0, CH_IN + CH_FWALL]) rotate([90, 0, 0]) channel_cover_part();   // outer face down
    else if (name == "black_plate")      translate([0, 0, -BP_BOT]) black_plate_part();                           // underside down, tab up
    else if (name == "sector_gear")      translate([0, 0, -GEAR_Y0]) rotate([90, 0, 0]) sector_gear_part();       // gear face down, cup up
    else if (name == "drive_disk")       translate([0, 0, DISK_Y1]) rotate([-90, 0, 0]) drive_disk_part();         // horn face down, hub up
    else if (name == "antireverse_pawl") translate([0, 0, -DISK_Y0]) rotate([90, 0, 0]) antireverse_pawl_part();
    else if (name == "rack_carrier")     translate([0, 0, CAR_HW]) rotate([90, 0, 0]) rack_carrier_part();          // front face down
    else if (name == "shoulder_cap")     translate([0, 0, SHOULDER_T]) rotate([180, 0, 0]) shoulder_cap_part();    // cap down, plug up
    else if (name == "flame_body")       translate([0, 0, -BODY_Z0]) sculpted_flame();                            // upright
    else if (name == "flame_collar")     flame_collar_part();                                                      // flange down
    else if (name == "trapdoor_lid")     translate([0, 0, -LID_UNDER]) lid_part();                                 // underside down
    else if (name == "stem")             translate([0, 0, 6]) stem_part();                                         // plug down
    else if (name == "keeper")           rotate([0, -90, 0]) translate([-KEEPER_X + KEEPER_T / 2, 0, -LID_UNDER]) keeper_part();
    else if (name == "deadbolt")         translate([0, 0, -BOLT_Z[0]]) deadbolt_part();
    else if (name == "lcd_retainer")     lcd_retainer_part();                                                      // print 2
    else if (name == "tpu_pads")         tpu_pads_part();
    else assert(false, str("unknown part: ", name));
}
module print_trapdoor_plate() {
    translate([0, 0, -LID_UNDER]) lid_part();
    translate([75, 0, 6]) stem_part();
    translate([-70, 40, 0]) rotate([0, -90, 0]) translate([-KEEPER_X + KEEPER_T / 2, 0, -LID_UNDER]) keeper_part();
    translate([-100, -110, -BOLT_Z[0]]) deadbolt_part();
    for (p = [[45, -85], [-15, -100]]) translate(p) lcd_retainer_part();     // 2x LCD retainer rings
    translate([10, -60, 0]) tpu_pads_part();                                   // print in TPU separately
}
module print_head_front_plate() {
    translate([0, 0, SPLIT_Y]) rotate([-90, 0, 0]) head_front_part();    // seam face down, face up
}
module print_head_back_plate() { translate([0, 0, -SPLIT_Y]) rotate([90, 0, 0]) head_back_part(); }   // seam face down
module print_base_plate() {
    if (base_part == "neck_mount") translate([0, 0, -BASE_H]) neck_mount_part();
    else translate([0, 0, BASE_H]) rotate([180, 0, 0]) base_shell_part();       // top skin on the bed
}
module mesh_check_2d() {
    dz = -rack_drop(check_g);
    color(C_BLUE) rotate(-check_g) sector_gear_2d();
    color(C_YELLOW) translate([0, Z_T0_TOP + dz]) rack_teeth_2d();
    color([0.1, 0.4, 1]) for (s = [-1, 1]) translate([-30, s * BAND_HALF_Z - 0.1]) square([66, 0.2]);
    color(C_GREEN) for (a = TOOTH_CENTRES) rotate(a - check_g) translate([R_TIP + 1, 0]) circle(d = 0.8);
}

/* ============================ DISPATCH ============================== */
if (render_mode == "assembly")              full_assembly();
else if (render_mode == "print_base")       print_base_plate();
else if (render_mode == "print_head_front") print_head_front_plate();
else if (render_mode == "print_head_back")  print_head_back_plate();
else if (render_mode == "print_mechanism")  print_mechanism_plate();
else if (render_mode == "print_flame")      { translate([0, 0, -BODY_Z0]) sculpted_flame(); translate([45, 0, 0]) flame_collar_part(); }
else if (render_mode == "part")             print_part(part);
else if (render_mode == "print_trapdoor")   print_trapdoor_plate();
else if (render_mode == "mesh_check")       mesh_check_2d();
