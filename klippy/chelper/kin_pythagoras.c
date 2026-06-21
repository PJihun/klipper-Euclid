// Pythagoras stepper kinematics
//
// Copyright (C) 2022  Dmitry Lavrov
//
// This file may be distributed under the terms of the GNU GPLv3 license.

// This file is modified by Pjihun.
// The original file is from https://github.com/Dmytry/klipper

#include "compiler.h"  // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h"     // move_get_coord
#include <math.h>      // sqrt
#include <stddef.h>    // offsetof
#include <stdlib.h>    // malloc
#include <string.h>    // memset

struct pythagoras_stepper {
  struct stepper_kinematics sk;
  double x, y;
  double r1, r2;
};

static double square(double x) { return x * x; }

static double pythagoras_calc_belt_length(double x, double y, double pulley_x,
                                          double pulley_y, double pulley_r,
                                          double tip_r) {
  double dx = x - pulley_x;
  double dy = y - pulley_y;
  double d2 = dx * dx + dy * dy;
  double free_belt_sq = d2 - square(pulley_r + tip_r);
  if (free_belt_sq < 0.)
    free_belt_sq = 0.;
  double free_belt_length = sqrt(free_belt_sq);
  // Angles are counter clockwise
  // Left pulley has positive radius and unwinds counterclockwise. Right pulley
  // has negative radius and unwinds clockwise. The straight belt is at an extra
  // clockwise angle on the left pulley and rotated counter-clockwise on the
  // right.
  double belt_angle = atan2(dy, dx) - atan2(pulley_r + tip_r, free_belt_length);
  // Belt length is free belt length + length wrapped around the pulley
  return free_belt_length - belt_angle * pulley_r;
}

static double pythagoras_stepper_calc_position(struct stepper_kinematics *sk,
                                               struct move *m,
                                               double move_time) {
  struct pythagoras_stepper *hs =
      container_of(sk, struct pythagoras_stepper, sk);
  struct coord c = move_get_coord(m, move_time);
  return pythagoras_calc_belt_length(c.x, c.y, hs->x, hs->y, hs->r1, hs->r2);
}

int __visible pythagoras_calc_xy(double target_a, double target_b, double a_x,
                                 double a_y, double a_r1, double a_r2,
                                 double b_x, double b_y, double b_r1,
                                 double b_r2, double x_guess, double y_guess,
                                 int max_iter, double tolerance, double *x_out,
                                 double *y_out) {
  if (!x_out || !y_out || max_iter <= 0 || tolerance <= 0.)
    return -1;

  double x = x_guess;
  double y = y_guess;
  double tolerance2 = tolerance * tolerance;

  for (int i = 0; i < max_iter; i++) {
    double a0 = pythagoras_calc_belt_length(x, y, a_x, a_y, a_r1, a_r2);
    double b0 = pythagoras_calc_belt_length(x, y, b_x, b_y, b_r1, b_r2);
    double residual_a = target_a - a0;
    double residual_b = target_b - b0;
    double residual2 = residual_a * residual_a + residual_b * residual_b;
    if (residual2 <= tolerance2)
      break;

    double eps = 1.0e-6;
    double a_dx = pythagoras_calc_belt_length(x + eps, y, a_x, a_y, a_r1, a_r2);
    double b_dx = pythagoras_calc_belt_length(x + eps, y, b_x, b_y, b_r1, b_r2);
    double a_dy = pythagoras_calc_belt_length(x, y + eps, a_x, a_y, a_r1, a_r2);
    double b_dy = pythagoras_calc_belt_length(x, y + eps, b_x, b_y, b_r1, b_r2);

    double j00 = (a_dx - a0) / eps;
    double j01 = (a_dy - a0) / eps;
    double j10 = (b_dx - b0) / eps;
    double j11 = (b_dy - b0) / eps;

    double h00 = j00 * j00 + j10 * j10;
    double h01 = j00 * j01 + j10 * j11;
    double h11 = j01 * j01 + j11 * j11;
    double g0 = j00 * residual_a + j10 * residual_b;
    double g1 = j01 * residual_a + j11 * residual_b;
    double det = h00 * h11 - h01 * h01;
    if (fabs(det) < 1.0e-18)
      break;

    double step_x = (g0 * h11 - g1 * h01) / det;
    double step_y = (h00 * g1 - h01 * g0) / det;
    double step_norm = sqrt(step_x * step_x + step_y * step_y);
    if (step_norm > 10.) {
      double scale = 10. / step_norm;
      step_x *= scale;
      step_y *= scale;
    }
    x += step_x;
    y += step_y;
    if (fabs(step_x) + fabs(step_y) < 1.0e-10)
      break;
  }

  *x_out = x;
  *y_out = y;

  {
    double a0 = pythagoras_calc_belt_length(x, y, a_x, a_y, a_r1, a_r2);
    double b0 = pythagoras_calc_belt_length(x, y, b_x, b_y, b_r1, b_r2);
    double residual_a = target_a - a0;
    double residual_b = target_b - b0;
    double residual2 = residual_a * residual_a + residual_b * residual_b;
    if (residual2 > tolerance2)
      return 1;
  }
  return 0;
}

struct stepper_kinematics *__visible pythagoras_stepper_alloc(double x,
                                                              double y,
                                                              double r1,
                                                              double r2) {
  struct pythagoras_stepper *hs = malloc(sizeof(*hs));
  if (!hs)
    return NULL;
  memset(hs, 0, sizeof(*hs));
  hs->x = x;
  hs->y = y;
  hs->r1 = r1;
  hs->r2 = r2;
  hs->sk.calc_position_cb = pythagoras_stepper_calc_position;
  hs->sk.active_flags = AF_X | AF_Y;
  return &hs->sk;
}
