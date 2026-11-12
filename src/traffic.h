// BeamLite — traffic: signalised junctions and AI drivers.
//
// Each map's streets form a rectangular grid of junctions (avenues along z crossing streets along
// x). Signals at every junction alternate between the two axes. AI cars drive the grid in the
// right-hand lane on ordinary soft-body physics: they pick a turn at each junction, follow a curved
// path through it, stop for red and yellow, keep a gap to whatever is ahead (the player included),
// and respawn out of sight when wrecked, flipped or stuck.
#ifndef BEAMLITE_TRAFFIC_H
#define BEAMLITE_TRAFFIC_H

#include "physics.h"

enum { LIGHT_RED, LIGHT_YELLOW, LIGHT_GREEN };

// Set up the junction grid for the world's map and spawn up to `count` AI cars into the car slots
// from w->ncars upward (w->ncars grows). Call after the map's own cars are placed.
void traffic_set_map(int map);                      // junction grid only (signals exist from here on)
void traffic_init(World *w, int count, V3 avoid);
void traffic_clear(World *w);                       // remove all AI cars (frees their slots at the top)
void traffic_forget_slot(int slot);                 // a slot was reused for something else
int traffic_is_ai(int slot);
int traffic_respawns(void);
void traffic_debug(int why[4]);                     // respawn reasons: stuck, off route, flipped, damaged                         // AI cars respawned since traffic_init (wrecked/stuck)

// Advance signal timers and drive the AI cars (sets their controls). `player` is avoided when respawning.
void traffic_update(World *w, float dt, V3 player);

// Signals, for rendering: one head per junction approach.
int traffic_signal_count(void);
// Head centre (5.4 m up, over the approach lane, on the far side of the junction), the direction it
// faces (back toward oncoming cars), the pole base at the far-right corner, and the lamp state.
void traffic_signal(int i, V3 *head, V3 *facing, V3 *pole, int *state);

#endif
