# Code for handling the kinematics of pythagoras robot
#
# Copyright (C) 2022  Dmitry Lavrov
#
# This file may be distributed under the terms of the GNU GPLv3 license.

# This file is modified by Pjihun.
# The original file is from https://github.com/Dmytry/klipper

import logging
import chelper
import stepper


class PythagorasKinematics:
    def __init__(self, toolhead, config):
        # Setup axis steppers
        self.printer = config.get_printer()
        printer_config = config.getsection('printer')
        self.home_x=printer_config.getfloat('home_x')
        self.home_y=printer_config.getfloat('home_y')
        # Homing parameters
        self.homing_retract_dist = printer_config.getfloat(
            'homing_retract_dist', 399., above=0.)
        self.homing_speed = printer_config.getfloat(
            'homing_speed', 200., above=0.)
        self.homing_accel = printer_config.getfloat(
            'homing_accel', 50., above=0.)

        stepper_configs = [config.getsection('stepper_' + a) for a in 'abz']
        self.homing_current = stepper_configs[0].getfloat(
            'homing_current', 0.4, above=0.)
        self.homing_dwell = stepper_configs[0].getfloat(
            'homing_dwell', 0.1, above=0.)
        rail_a = stepper.LookupMultiRail(stepper_configs[0], need_position_minmax = False)
        a_endstop = rail_a.get_homing_info().position_endstop
        rail_b = stepper.LookupMultiRail(
            stepper_configs[1], need_position_minmax = False,
            default_position_endstop=a_endstop)
        rail_z = stepper.LookupMultiRail(stepper_configs[2])
        self.rails = [rail_a, rail_b, rail_z]

        self.kin_params = []

        for i in range(0,2):
            kin_params=(
                stepper_configs[i].getfloat('pulley_x'),
                stepper_configs[i].getfloat('pulley_y'),
                stepper_configs[i].getfloat('pulley_r'),
                stepper_configs[i].getfloat('tip_r'),
                stepper_configs[i].getfloat('position_max')
                )
            self.kin_params.append(kin_params);
            self.rails[i].setup_itersolve('pythagoras_stepper_alloc',
                *kin_params[:4])

        self.rails[2].setup_itersolve('cartesian_stepper_alloc', b'z')
        # Set correct endstop values based on Inverse Kinematics calculation
        endstops=self._calc_steppers_from_xy(self.home_x, self.home_y)
        rail_a.position_endstop=endstops[0]
        rail_b.position_endstop=endstops[1]

        logging.info(f'Calculated endstops: {endstops}')

        # calculate endstops from homing position

        self.steppers=[s for rail in self.rails for s in rail.get_steppers()]

        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())

        config.get_printer().register_event_handler("stepper_enable:motor_off",
                                                    self._motor_off)
        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', max_velocity, above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat(
            'max_z_accel', max_accel, above=0., maxval=max_accel)

        # self.limits = [(1.0, -1.0)] * 3
        ranges = [r.get_range() for r in self.rails]
        self.axes_min = toolhead.Coord([r[0] for r in ranges])
        self.axes_max = toolhead.Coord([r[1] for r in ranges])
        # Internal test of calc position
        # self._test_calc_position()
        self.limit_z = (1.0, -1.0)
        self._last_xy_guess = [self.home_x, self.home_y]
        self._ik_max_iter = 30
        self._ik_tolerance = 1.0e-6
        self._ffi_main, self._ffi_lib = chelper.get_ffi()
        self._xy_out_x = self._ffi_main.new('double[1]')
        self._xy_out_y = self._ffi_main.new('double[1]')

    def get_steppers(self):
        return self.steppers

    def _get_tmc_run_current(self, stepper_name):
        """Look up the configured TMC run_current for the given stepper.
        Returns the run_current float, or None if no TMC driver is configured.
        """
        tmc_prefixes = ['tmc2209', 'tmc2130', 'tmc2208', 'tmc2660',
                        'tmc5160', 'tmc2240']
        for prefix in tmc_prefixes:
            tmc_obj = self.printer.lookup_object(
                '%s %s' % (prefix, stepper_name), None)
            if tmc_obj is not None:
                status = tmc_obj.get_status()
                return status.get('run_current')
        return None

    def _calc_steppers_from_xy(self, x, y):
        return (
            self.rails[0].calc_position_from_coord( (x,y,0)),
            self.rails[1].calc_position_from_coord( (x,y,0))
        )

    def _internal_calc_position(self, a, b):
        x, y = self._last_xy_guess
        ka = self.kin_params[0]
        kb = self.kin_params[1]
        status = self._ffi_lib.pythagoras_calc_xy(
            a, b,
            ka[0], ka[1], ka[2], ka[3],
            kb[0], kb[1], kb[2], kb[3],
            x, y,
            self._ik_max_iter, self._ik_tolerance,
            self._xy_out_x, self._xy_out_y)
        new_x = float(self._xy_out_x[0])
        new_y = float(self._xy_out_y[0])
        if status:
            logging.warning(
                'Pythagoras C inverse solve did not fully converge for a=%.6f b=%.6f status=%d; using best estimate',
                a, b, status)
        self._last_xy_guess = [new_x, new_y]
        return [new_x, new_y]

    def calc_position(self, stepper_positions):
        a = stepper_positions[self.rails[0].get_name()]
        b = stepper_positions[self.rails[1].get_name()]
        z = stepper_positions[self.rails[2].get_name()]
        x, y = self._internal_calc_position(a,b)
        return [x, y, z]

    def _test_calc_position(self):
        for x in range(-30, 31, 5):
            for y in range(10, 151, 5):
                logging.info(f'test: {x}, {y}')
                p=self._calc_steppers_from_xy(x,y)
                new_x, new_y = self._internal_calc_position(p[0], p[1])
                logging.info(f'test results: {x-new_x}, {y-new_y}')



    def set_position(self, newpos, homing_axes):
        for s in self.steppers:
            s.set_position(newpos)
        if 'z' in homing_axes:
            self.limit_z = self.rails[2].get_range()
    def note_z_not_homed(self):
        # Helper for Safe Z Home
        self.limit_z = (1.0, -1.0)
    def clear_homing_state(self, clear_axes):
        if 'z' in clear_axes:
            self.limit_z = (1.0, -1.0)
    def _home_axis(self, homing_state, axis, rail):
        # TODO: implement homing properly
        # Determine movement
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None, None, None, None]
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= hi.position_endstop - position_min
        else:
            forcepos[axis] += position_max - hi.position_endstop
        # Perform homing
        homing_state.home_rails([rail], forcepos, homepos)

    def home(self, homing_state):
        # Always home XY together
        # TODO: homing
        homing_axes = homing_state.get_axes()
        home_xy = 0 in homing_axes or 1 in homing_axes
        home_z = 2 in homing_axes
        updated_axes = []
        if home_xy:
            updated_axes = [0, 1]
            # always home both
        if home_z:
            updated_axes.append(2)

        homing_state.set_axes(updated_axes)
        if home_xy:
            x = self.home_x
            y = self.home_y

            gcode=self.printer.lookup_object('gcode')
            toolhead = self.printer.lookup_object('toolhead')
            logging.info('Pythagoras XY homing sequence start')
            fmove = self.printer.lookup_object('force_move')
            stepper_enable=self.printer.lookup_object('stepper_enable')
            stepper_b_enable=stepper_enable.lookup_enable(self.steppers[1].get_name())
            toolhead.dwell(self.homing_dwell)
            print_time = toolhead.get_last_move_time()
            stepper_b_enable.motor_disable(print_time)
            move=fmove.manual_move

            stepper_a_name=self.steppers[0].get_name()

            # Read TMC run_current before changing it, so we can restore
            restore_current = self._get_tmc_run_current(stepper_a_name)
            if restore_current is not None:
                gcode.run_script_from_command(
                    f'SET_TMC_CURRENT STEPPER={stepper_a_name}'
                    f' CURRENT={self.homing_current}\n')
            move(self.steppers[0], -self.homing_retract_dist,
                 self.homing_speed, self.homing_accel)
            if restore_current is not None:
                gcode.run_script_from_command(
                    f'SET_TMC_CURRENT STEPPER={stepper_a_name}'
                    f' CURRENT={restore_current}\n')
            curpos = toolhead.get_position()
            toolhead.set_position([x, y, curpos[2], curpos[3]])            

        if home_z:
            self._home_axis(homing_state, 2, self.rails[2])
    def _motor_off(self):
        self.limit_z = (1.0, -1.0)
        #self.limit_xy2 = -1.
    def _check_endstops(self, move):
        if not move.axes_d[2]:
            return
        zpos = move.end_pos[2]
        if zpos < self.limit_z[0] or zpos > self.limit_z[1]:
            if self.limit_z[0] > self.limit_z[1]:
                raise move.move_error("Must home axis first")
            raise move.move_error()

    def check_move(self, move):
        # TODO: check kinematic limits for XY
        if not move.axes_d[2]:
            # Normal XY move - use defaults
            return
        # Move with Z - update velocity and accel for slower Z axis
        self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(
            self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)

    def get_status(self, eventtime):
        #xy_home = "xy" if self.limit_xy2 >= 0. else ""
        # TODO: homing
        xy_home = "xy"
        z_home = "z" if self.limit_z[0] <= self.limit_z[1] else ""
        return {
            'homed_axes': xy_home + z_home,
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }
    
        

def load_kinematics(toolhead, config):
    return PythagorasKinematics(toolhead, config)
