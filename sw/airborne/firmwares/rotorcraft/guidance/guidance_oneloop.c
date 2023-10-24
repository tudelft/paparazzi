/*
 * Copyright (C) 2015 Tomaso De Ponti <t.m.l.deponti@tudelft.nl>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file firmwares/rotorcraft/guidance/guidance_oneloop.c
 *
 * A dummy guidance mode to run the oneloop_andi controller
 *
 */

#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/oneloop/oneloop_andi.h"

void guidance_h_run_enter(void)
{
  oneloop_andi_enter(false);
}

void guidance_v_run_enter(void)
{
  // nothing to do
}

//static struct VerticalGuidance *_gv = &guidance_v;

struct StabilizationSetpoint guidance_h_run_pos(bool in_flight, UNUSED struct HorizontalGuidance *gh)
{
  half_loop = false;
  oneloop_andi_run(in_flight, half_loop);
  struct StabilizationSetpoint sp= { 0 };
  return sp;
}

struct StabilizationSetpoint guidance_h_run_speed(UNUSED bool in_flight, UNUSED struct HorizontalGuidance *gh)
{
   struct StabilizationSetpoint sp= { 0 };
  return sp;
}

struct StabilizationSetpoint guidance_h_run_accel(UNUSED bool in_flight, UNUSED struct HorizontalGuidance *gh)
{
   struct StabilizationSetpoint sp= { 0 };
  return sp;
}

int32_t guidance_v_run_pos(bool in_flight UNUSED, UNUSED struct VerticalGuidance *gv)
{
  return 0; // nothing to do
}

int32_t guidance_v_run_speed(bool in_flight UNUSED, UNUSED struct VerticalGuidance *gv)
{

  return 0; // nothing to do
}

int32_t guidance_v_run_accel(bool in_flight UNUSED, UNUSED struct VerticalGuidance *gv)
{
  return 0; // nothing to do
}
