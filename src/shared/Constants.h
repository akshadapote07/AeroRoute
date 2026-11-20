/*
 * ============================================================
 * FILE        : Constants.h
 * PROJECT     : AeroRoute — Autonomous Drone Delivery Simulator
 * DESCRIPTION : Shared constants used across all subjects.
 *               Cell type codes here are shared across modules.
 * ============================================================
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H
using namespace std;

// ── Grid dimensions ──────────────────────────────────────────
#define GRID_ROWS 10
#define GRID_COLS 10

// ── Cell types ────────────────────────────────────────────────
#define CELL_EMPTY     0
#define CELL_WAREHOUSE 1
#define CELL_CUSTOMER  2
#define CELL_CHARGING  3
#define CELL_OBSTACLE  4

// ── Drone models ─────────────────────────────────────────────
#define DRONE_DELIVERY "AR-D1 Delivery"
#define DRONE_SCOUT    "AR-S1 Scout"
#define DRONE_HEAVY    "AR-H1 Heavy Lift"
#define DRONE_BATTERY_RANGE_GRID_UNITS 12

// ── Route types ───────────────────────────────────────────────
#define ROUTE_SLL "SLL"   // Singly Linked List  — forward path
#define ROUTE_DLL "DLL"   // Doubly Linked List  — return path
#define ROUTE_CLL "CLL"   // Circular Linked List — patrol

// ── JSON output paths (relative to where exe runs) ───────────
#define JSON_GRID       "../../grid.json"
#define JSON_ROUTE      "../../route.json"
#define JSON_TELEMETRY  "../../telemetry.json"
#define JSON_REGISTERS  "../../registers.json"
#define JSON_MISSION    "../../mission.json"
#define JSON_CLIPPED_ROUTE "../../clipped_route.json"
#define JSON_INPUT      "../../user_input.json"

// ── Register base address ─────────────────────────────────────
#define PC_START 0x1000

#endif // CONSTANTS_H