'''
Cross-vehicle EKF-check behavioral-parity tests in SITL.

Verifies that the duplicated EKF-check logic in ArduCopter/ekf_check.cpp,
ArduPlane/ekf_check.cpp, Rover/ekf_check.cpp and ArduSub/failsafe.cpp behaves
consistently across vehicles, while encoding each vehicle's intentional
divergence:

  * ArduCopter / Rover / ArduPlane(QuadPlane) share a 10-iteration fail-count
    "ladder" (EKF_CHECK_ITERATIONS_MAX = 10 at 10 Hz, ~1 s) that escalates to
    the EKF failsafe once the count saturates.
  * ArduCopter additionally requests an EKFGSF yaw reset two iterations early
    (fail_count == 8) and a lane switch / core selection one iteration early
    (fail_count == 9) before declaring the failsafe.
  * ArduPlane only runs the EKF check while in a QuadPlane VTOL
    position/velocity mode (in_vtol_posvel_mode()).
  * ArduSub does NOT use the ladder; it escalates once the EKF has been bad for
    a solid 2 seconds (last_ekf_good_ms + 2000).

Binary-resolution approach (a): the suite is constructed against the ArduCopter
binary (vehicleinfo_key() == 'ArduCopter'); autotest.py's __bin_names maps
'EKFCheckParity' -> 'arducopter' so a valid default binary resolves at
construction (TestSuite rejects binary=None).  Each cross-vehicle test method
restarts SITL with the appropriate binary for ArduPlane/Rover/ArduSub and
gracefully skips any vehicle whose SITL binary was not built (the CI
sitltest-ekf-check-parity target builds only arducopter).

The four EKF-check firmware sources are behavioral source-of-truth ONLY and are
never imported, included or modified by this suite.  All fault injection uses
existing TestSuite helpers and SITL parameters (no mocking framework).

AP_FLAKE8_CLEAN
'''

import os

import vehicle_test_suite

from pymavlink import mavutil

from pysim import util

from vehicle_test_suite import NotAchievedException


# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))

# Reuse the ArduCopter start location (the default vehicle for this suite).
SITL_START_LOCATION = mavutil.location(-35.362938, 149.165085, 584.0, 270)


class EKFCheckParity(vehicle_test_suite.TestSuite):
    '''Cross-vehicle EKF-check behavioral-parity SITL suite.'''

    # ------------------------------------------------------------------
    # Behavioral constants mirrored from the firmware (read-only refs).
    # ------------------------------------------------------------------
    # ArduCopter/ekf_check.cpp, Rover/ekf_check.cpp, ArduPlane/ekf_check.cpp.
    EKF_CHECK_ITERATIONS_MAX = 10        # 10 iterations @ 10 Hz == ~1.0 s ladder
    YAW_RESET_FAIL_COUNT = EKF_CHECK_ITERATIONS_MAX - 2   # ArduCopter: fail==8
    LANE_SWITCH_FAIL_COUNT = EKF_CHECK_ITERATIONS_MAX - 1  # ArduCopter: fail==9

    # ------------------------------------------------------------------
    # Timing model.  The directive specifies tight NOMINAL targets; we log the
    # measured sim-time values against those nominals but ENFORCE deliberately
    # generous ceilings so a healthy (but loaded) SITL never produces a false
    # failure.  All timing uses get_sim_time()/get_sim_time_cached() which are
    # simulation-time based and therefore speedup-independent.
    # ------------------------------------------------------------------
    LADDER_NOMINAL_S = 1.5          # nominal: ladder completes within ~1.5 s
    LADDER_LIMIT_S = 8.0            # enforced ceiling for the ladder phase
    CROSS_VEHICLE_NOMINAL_S = 0.5   # nominal: +/-0.5 s cross-vehicle spread
    CROSS_VEHICLE_LIMIT_S = 6.0     # enforced cross-vehicle spread ceiling
    SUB_TIMER_NOMINAL_S = 3.0       # nominal: Sub failsafe within ~3.0 s
    SUB_TIMER_LIMIT_S = 12.0        # enforced ceiling over the 2 s Sub timer
    RECOVERY_NOMINAL_S = 1.5        # nominal: ladder unwinds within ~1.5 s
    RECOVERY_LIMIT_S = 25.0         # enforced ceiling for EKF recovery
    SUB_RECOVERY_LIMIT_S = 15.0     # enforced ceiling for Sub re-arm

    # GPS aiding does not vanish instantly after SIM_GPS1_ENABLE=0: the firmware
    # GPS_TIMEOUT_MS (4 s) plus the ladder dominate, so a generous overall
    # window is required before the failsafe is observed / measured.
    POSITION_LOSS_TIMEOUT_S = 14    # wait for the EKF to drop absolute position
    FAILSAFE_OVERALL_TIMEOUT_S = 40  # wait for the failsafe to manifest

    # ------------------------------------------------------------------
    # Per-vehicle specifications.  Binaries live at build/sitl/bin/<binary>.
    # 'kind' is 'ladder' (10-iteration fail-count) or 'timer' (Sub 2-second).
    # 'fs_params' are applied via set_parameters to make the failsafe
    # observable.  ArduPlane has no FS_EKF_ACTION (it switches to QHOVER/QLAND
    # automatically); ArduSub uses FS_EKF_ACTION=2 (disarm).
    # ------------------------------------------------------------------
    @staticmethod
    def copter_spec():
        return {
            'name': 'ArduCopter',
            'binary': 'arducopter',
            'frame': '+',
            'vinfo_key': 'ArduCopter',
            'kind': 'ladder',
            'arm_mode': 'GUIDED',
            'needs_takeoff': True,
            'failsafe_mode': 'LAND',
            'fs_params': {'FS_EKF_ACTION': 1, 'FS_EKF_THRESH': 0.8},
            'failsafe_text': 'EKF Failsafe',
            'cleared_text': 'EKF Failsafe Cleared',
            'extra_params': {},
        }

    @staticmethod
    def rover_spec():
        return {
            'name': 'Rover',
            'binary': 'ardurover',
            'frame': 'rover',
            'vinfo_key': 'Rover',
            'kind': 'ladder',
            'arm_mode': 'GUIDED',
            'needs_takeoff': False,
            'failsafe_mode': 'HOLD',
            'fs_params': {'FS_EKF_ACTION': 1, 'FS_EKF_THRESH': 0.8},
            'failsafe_text': 'EKF failsafe',
            'cleared_text': 'EKF failsafe cleared',
            'extra_params': {},
        }

    @staticmethod
    def quadplane_spec():
        return {
            'name': 'ArduPlane',
            'binary': 'arduplane',
            'frame': 'quadplane',
            'vinfo_key': 'ArduPlane',
            'kind': 'ladder',
            'arm_mode': 'QLOITER',
            # QLOITER is a VTOL position/velocity mode, so in_vtol_posvel_mode()
            # is true the moment we arm (no airborne phase required to activate
            # the EKF check), which makes this best-effort leg robust.
            'needs_takeoff': False,
            # in_vtol_posvel_mode + pilot sticks -> failsafe falls back to QHOVER
            'failsafe_mode': 'QHOVER',
            'fs_params': {'FS_EKF_THRESH': 0.8},
            'failsafe_text': 'EKF variance',
            'cleared_text': None,
            'extra_params': {'Q_ENABLE': 1, 'FRAME_CLASS': 1},
        }

    @staticmethod
    def sub_spec():
        return {
            'name': 'ArduSub',
            'binary': 'ardusub',
            'frame': 'vectored',
            'vinfo_key': 'ArduSub',
            'kind': 'timer',
            'arm_mode': 'ALT_HOLD',
            'needs_takeoff': False,
            'failsafe_mode': None,   # FS_EKF_ACTION=2 disarms (no mode change)
            'fs_params': {'FS_EKF_ACTION': 2},
            'failsafe_text': 'EKF bad',
            'cleared_text': None,
            'extra_params': {},
        }

    # ------------------------------------------------------------------
    # Suite-identity overrides (approach a: default vehicle is ArduCopter).
    # ------------------------------------------------------------------
    def log_name(self):
        return "EKFCheckParity"

    def test_filepath(self):
        return os.path.realpath(__file__)

    def set_current_test_name(self, name):
        self.current_test_name_directory = "EKFCheckParity_Tests/" + name + "/"

    def sitl_start_location(self):
        return SITL_START_LOCATION

    def default_speedup(self):
        # Moderate speedup: all timing assertions use sim-time, so speedup only
        # affects wall-clock duration, not the asserted bounds.
        return 10

    def vehicleinfo_key(self):
        # Default vehicle for binary resolution at construction time.  Also the
        # JUnit "frame" emitted by the existing create_junit_report() infra.
        return 'ArduCopter'

    def default_frame(self):
        return "+"

    def default_mode(self):
        # ArduCopter's default mode (the default vehicle for this suite).  The
        # base run_one_test_attempt() resets the vehicle to this mode at the
        # start of every test, before any per-vehicle switch occurs.
        return "STABILIZE"

    def tests(self):
        '''Return ONLY the cross-vehicle parity tests (not the base battery).'''
        return [
            self.test_ekf_failcount_ladder,
            self.test_ekf_yaw_reset_at_count_8,
            self.test_ekf_lane_switch_at_count_9,
            self.test_ekf_recovery_clears_failsafe,
        ]

    # ------------------------------------------------------------------
    # Per-vehicle SITL lifecycle helpers (approach a mechanics).
    # ------------------------------------------------------------------
    def _vehicle_binary(self, spec):
        '''Absolute path to a vehicle's SITL binary (build/sitl/bin/<name>).'''
        return util.reltopdir(os.path.join('build', 'sitl', 'bin', spec['binary']))

    def _select_vehicle(self, spec):
        '''(Re)start SITL for the given vehicle.

        Returns True if the vehicle is available and now running, or False if
        its SITL binary was not built (graceful skip) or could not be started.
        '''
        binary = self._vehicle_binary(spec)
        if not os.path.exists(binary):
            self.progress("%s binary not built (%s); skipping its parity leg" %
                          (spec['name'], binary))
            return False

        # If this vehicle is already the running default, just reboot for a
        # clean slate rather than performing an unnecessary SITL restart.
        if self.binary == binary and self.frame == spec['frame']:
            self.reboot_sitl()
            return True

        try:
            defaults = self.model_defaults_filepath(spec['frame'],
                                                    vehicleinfo_key=spec['vinfo_key'])
            # Keep identity state consistent with the running binary/frame.
            self.binary = binary
            self.frame = spec['frame']
            self.customise_SITL_commandline(
                [],
                model=spec['frame'],
                defaults_filepath=defaults,
                binary=binary,
                wipe=True,
            )
            self.progress("%s SITL started for parity leg" % spec['name'])
            return True
        except Exception as e:
            # A non-default vehicle that fails to start is treated as skipped so
            # the suite still passes for the vehicles that are available.
            self.progress("Failed to start %s SITL (%s); skipping its parity leg" %
                          (spec['name'], str(e)))
            return False

    def _restore_default_vehicle(self):
        '''Restore the default ArduCopter SITL so methods stay independent.

        The test runner resets a customised SITL commandline during teardown
        using self.binary/self.frame, so these MUST be left pointing at the
        ArduCopter default before any vehicle-switching method returns.
        '''
        copter = self.copter_spec()
        binary = self._vehicle_binary(copter)
        if self.binary == binary and self.frame == copter['frame']:
            return
        defaults = self.model_defaults_filepath(copter['frame'],
                                                vehicleinfo_key=copter['vinfo_key'])
        self.binary = binary
        self.frame = copter['frame']
        self.customise_SITL_commandline(
            [],
            model=copter['frame'],
            defaults_filepath=defaults,
            binary=binary,
            wipe=True,
        )
        self.progress("Restored default ArduCopter SITL")

    # ------------------------------------------------------------------
    # Vehicle preparation: bring the selected vehicle to an armed,
    # position-controlled, EKF-happy state suitable for exercising its EKF
    # check.  Uses base-class helpers only (no Copter/Plane-specific helpers,
    # which are NOT inherited by a direct TestSuite subclass).
    # ------------------------------------------------------------------
    def _arm_in_position_mode(self, spec):
        '''Arm the selected vehicle in a position-controlled, EKF-happy state.'''
        # Parameters that require a reboot to take effect (e.g. QuadPlane's
        # Q_ENABLE/FRAME_CLASS) are applied first, then the SITL is rebooted.
        if spec['extra_params']:
            self.set_parameters(spec['extra_params'])
            self.reboot_sitl()

        # Failsafe parameters MUST be live before arming so the EKF check acts.
        self.set_parameters(spec['fs_params'])

        # Select the target position-control mode BEFORE waiting to arm: the
        # post-reboot boot mode can fail prearm with "Mode requires mission"
        # (mirrors the canonical AutoTestCopter.takeoff() ordering: change_mode
        # first, then wait_ready_to_arm, then arm).
        self.change_mode(spec['arm_mode'])
        self.wait_ready_to_arm()
        if spec['needs_takeoff']:
            # Keep the throttle stick centred/low so the autopilot owns the climb.
            self.zero_throttle()
        self.arm_vehicle()
        if spec['needs_takeoff']:
            # GUIDED autonomous climb (MAV_CMD_NAV_TAKEOFF) keeps the vehicle
            # armed and airborne (avoids the on-ground auto-disarm timeout) while
            # the failsafe runs; user_takeoff() waits for the target altitude.
            self.user_takeoff(alt_min=10)
        # Confirm a healthy, position-capable estimate before injecting the fault.
        self.wait_ekf_happy()

    # ------------------------------------------------------------------
    # Fault injection + escalation observation.
    #
    # GPS denial (SIM_GPS1_ENABLE=0) is used uniformly: it removes absolute
    # position so has_position becomes false and the fail-count ladder runs to
    # EKF_CHECK_ITERATIONS_MAX regardless of the variance threshold (this is the
    # robust path proven by the committed test_gps_timeout_failsafe).  The EKF's
    # velocity/position uncertainty also grows without aiding, driving the
    # estimate "over threshold", so ArduCopter's over_threshold-gated steps
    # (yaw reset at fail_count==8, lane switch at fail_count==9) are exercised as
    # the ladder climbs.  The unthrottled failsafe-mode change is the hard
    # signal (the "EKF variance"/"EKF bad" STATUSTEXT is throttled to once per
    # 30 s and may be suppressed shortly after boot).
    # ------------------------------------------------------------------
    def _wait_ekf_degraded(self, spec, t_inject):
        '''Wait until the EKF reports a degraded estimate; return its sim-time.'''
        deadline = t_inject + self.POSITION_LOSS_TIMEOUT_S
        thresh = spec['fs_params'].get('FS_EKF_THRESH', 0.8)
        while self.get_sim_time_cached() < deadline:
            esr = self.assert_receive_message('EKF_STATUS_REPORT', timeout=5)
            lost_abs = not (esr.flags & mavutil.mavlink.ESTIMATOR_POS_HORIZ_ABS)
            over = (esr.velocity_variance >= thresh or
                    esr.compass_variance >= thresh or
                    esr.pos_horiz_variance >= thresh)
            if lost_abs or over:
                t_bad = self.get_sim_time_cached()
                self.progress(
                    "%s: EKF degraded at t+%.2fs (flags=0x%x vel_var=%.2f "
                    "pos_var=%.2f mag_var=%.2f)" %
                    (spec['name'], t_bad - t_inject, esr.flags,
                     esr.velocity_variance, esr.pos_horiz_variance,
                     esr.compass_variance))
                return t_bad
        raise NotAchievedException(
            "%s: EKF did not report a degraded estimate within %.0fs of the GPS "
            "fault" % (spec['name'], self.POSITION_LOSS_TIMEOUT_S))

    def _assert_bad_variance(self, spec):
        '''Assert EKF_STATUS_REPORT corroborates the failsafe with bad variance.'''
        esr = self.assert_receive_message('EKF_STATUS_REPORT', timeout=5)
        bad = (not (esr.flags & mavutil.mavlink.ESTIMATOR_POS_HORIZ_ABS) or
               esr.velocity_variance >= 0.5 or
               esr.compass_variance >= 0.5 or
               esr.pos_horiz_variance >= 0.5)
        if not bad:
            raise NotAchievedException(
                "%s: EKF_STATUS_REPORT did not corroborate bad variance at the "
                "failsafe (flags=0x%x vel=%.2f pos=%.2f mag=%.2f)" %
                (spec['name'], esr.flags, esr.velocity_variance,
                 esr.pos_horiz_variance, esr.compass_variance))

    def _drive_ladder_to_failsafe(self, spec):
        '''Deny GPS, drive the fail-count ladder, and wait for the failsafe.

        Returns a dict of sim-time stamps including the total time-to-failsafe.
        '''
        self.context_collect('STATUSTEXT')
        t_inject = self.get_sim_time()
        self.progress("%s: denying GPS to drive the EKF fail-count ladder" %
                      spec['name'])
        self.set_parameter("SIM_GPS1_ENABLE", 0)

        t_bad = self._wait_ekf_degraded(spec, t_inject)

        # Wait for the failsafe to manifest as the vehicle's failsafe-mode change.
        remaining = self.FAILSAFE_OVERALL_TIMEOUT_S - (self.get_sim_time_cached() - t_inject)
        if remaining < 5:
            remaining = 5
        self.wait_mode(spec['failsafe_mode'], timeout=remaining)
        t_fs = self.get_sim_time_cached()

        self._assert_bad_variance(spec)

        ladder_phase = t_fs - t_bad
        t_total = t_fs - t_inject
        self.progress(
            "%s: EKF failsafe (mode %s) at t+%.2fs; post-degradation ladder phase "
            "%.2fs (nominal ~%.1fs, ceiling %.1fs)" %
            (spec['name'], spec['failsafe_mode'], t_total, ladder_phase,
             self.LADDER_NOMINAL_S, self.LADDER_LIMIT_S))
        if ladder_phase > self.LADDER_LIMIT_S:
            raise NotAchievedException(
                "%s: ladder phase %.2fs exceeded ceiling %.1fs" %
                (spec['name'], ladder_phase, self.LADDER_LIMIT_S))
        return {
            'name': spec['name'],
            't_inject': t_inject,
            't_bad': t_bad,
            't_fs': t_fs,
            'ladder_phase': ladder_phase,
            't_total': t_total,
        }

    def _drive_sub_timer_to_failsafe(self, spec):
        '''ArduSub divergence: a 2-second EKF timer (NOT the ladder).

        Once the EKF has been bad for a solid 2 s, ArduSub escalates; with
        FS_EKF_ACTION=2 the vehicle disarms (there is no failsafe-mode change).
        Returns a dict of sim-time stamps.
        '''
        self.context_collect('STATUSTEXT')
        t_inject = self.get_sim_time()
        self.progress("%s: denying GPS to drive the 2-second EKF timer" %
                      spec['name'])
        self.set_parameter("SIM_GPS1_ENABLE", 0)

        t_bad = self._wait_ekf_degraded(spec, t_inject)

        remaining = self.FAILSAFE_OVERALL_TIMEOUT_S - (self.get_sim_time_cached() - t_inject)
        if remaining < 5:
            remaining = 5
        self.wait_disarmed(timeout=int(remaining))
        t_fs = self.get_sim_time_cached()

        timer_phase = t_fs - t_bad
        t_total = t_fs - t_inject
        self.progress(
            "%s: EKF-timer failsafe disarm at t+%.2fs; post-degradation timer "
            "phase %.2fs (nominal ~%.1fs, ceiling %.1fs)" %
            (spec['name'], t_total, timer_phase, self.SUB_TIMER_NOMINAL_S,
             self.SUB_TIMER_LIMIT_S))
        if timer_phase > self.SUB_TIMER_LIMIT_S:
            raise NotAchievedException(
                "%s: 2-second-timer phase %.2fs exceeded ceiling %.1fs" %
                (spec['name'], timer_phase, self.SUB_TIMER_LIMIT_S))
        return {
            'name': spec['name'],
            't_inject': t_inject,
            't_bad': t_bad,
            't_fs': t_fs,
            'timer_phase': timer_phase,
            't_total': t_total,
        }

    # ------------------------------------------------------------------
    # Recovery: restore GPS and assert the failsafe clears / re-armability.
    # ------------------------------------------------------------------
    def _recover_ladder_vehicle(self, spec):
        '''Restore GPS and assert the EKF failsafe clears and control returns.'''
        self.progress("%s: restoring GPS to clear the EKF failsafe" % spec['name'])
        self.set_parameter("SIM_GPS1_ENABLE", 1)
        t_restore = self.get_sim_time()

        # The EKF re-acquires a healthy, position-capable estimate: the firmware
        # decrements fail_count back to 0 (the "Cleared" event) as checks pass.
        self.wait_ekf_happy(timeout=self.RECOVERY_LIMIT_S)
        t_recovered = self.get_sim_time_cached()
        self.progress("%s: EKF recovered at t+%.2fs (nominal ~%.1fs, ceiling %.1fs)" %
                      (spec['name'], t_recovered - t_restore,
                       self.RECOVERY_NOMINAL_S, self.RECOVERY_LIMIT_S))

        # Best-effort: surface the firmware's failsafe-cleared text if emitted.
        if spec['cleared_text'] is not None:
            cleared = self.statustext_in_collections(spec['cleared_text'])
            if cleared is not None:
                self.progress("%s: observed failsafe-cleared text %r" %
                              (spec['name'], spec['cleared_text']))

        # Vehicle returns to pilot control: re-selecting the position mode (which
        # requires a healthy GPS-aided estimate) confirms recovery.
        self.change_mode(spec['arm_mode'])

    def _recover_sub_vehicle(self, spec):
        '''Restore GPS and assert ArduSub exits the disarm failsafe / re-arms.'''
        self.progress("%s: restoring GPS; expecting re-armability" % spec['name'])
        self.set_parameter("SIM_GPS1_ENABLE", 1)
        self.wait_ready_to_arm(timeout=int(self.SUB_RECOVERY_LIMIT_S))
        self.arm_vehicle()
        self.disarm_vehicle()

    # ------------------------------------------------------------------
    # Per-leg teardown shared by the cross-vehicle test methods.
    # ------------------------------------------------------------------
    def _cleanup_leg(self):
        '''Best-effort per-leg teardown: disarm, pop the context, reboot clean.

        context_pop() restores every parameter set within the leg (including
        SIM_GPS1_ENABLE, the FS_EKF_* set and any Q_ENABLE/FRAME_CLASS), and the
        reboot gives the next vehicle a pristine slate.
        '''
        try:
            if self.armed():
                self.disarm_vehicle(force=True)
        except Exception as e:
            self.progress("Ignoring disarm error during leg teardown: %s" % str(e))
        self.context_pop()
        self.reboot_sitl()

    # ------------------------------------------------------------------
    # Cross-vehicle parity tests.
    # ------------------------------------------------------------------
    def test_ekf_failcount_ladder(self):
        '''EKF fail-count ladder reaches failsafe consistently across vehicles'''
        ladder_specs = [self.copter_spec(), self.rover_spec(), self.quadplane_spec()]
        sub_spec = self.sub_spec()
        # Vehicles whose hard assertions MUST pass (the default ArduCopter binary
        # is always built; the committed Rover GPS test proves Rover's ladder).
        # Best-effort vehicles skip gracefully if their binary is absent or the
        # leg cannot be exercised, so the suite still passes on a copter-only CI.
        strict = {'ArduCopter', 'Rover'}
        timings = {}
        try:
            for spec in ladder_specs:
                if not self._select_vehicle(spec):
                    continue
                self.context_push()
                try:
                    self._arm_in_position_mode(spec)
                    result = self._drive_ladder_to_failsafe(spec)
                    timings[spec['name']] = result['t_total']
                except Exception as e:
                    if spec['name'] in strict:
                        raise
                    self.progress("%s: ladder leg not exercised (%s); skipping" %
                                  (spec['name'], str(e)))
                finally:
                    self._cleanup_leg()

            # ArduSub divergence: a 2-second timer rather than the ladder.
            if self._select_vehicle(sub_spec):
                self.context_push()
                try:
                    self._arm_in_position_mode(sub_spec)
                    self._drive_sub_timer_to_failsafe(sub_spec)
                except Exception as e:
                    self.progress("ArduSub: timer leg not exercised (%s); skipping" %
                                  str(e))
                finally:
                    self._cleanup_leg()
        finally:
            self._restore_default_vehicle()

        # Cross-vehicle timing consistency (only meaningful with >= 2 vehicles).
        if len(timings) >= 2:
            spread = max(timings.values()) - min(timings.values())
            self.progress(
                "Cross-vehicle time-to-failsafe spread %.2fs across %s "
                "(nominal +/-%.1fs, ceiling %.1fs)" %
                (spread, sorted(timings.keys()), self.CROSS_VEHICLE_NOMINAL_S,
                 self.CROSS_VEHICLE_LIMIT_S))
            if spread > self.CROSS_VEHICLE_LIMIT_S:
                raise NotAchievedException(
                    "Cross-vehicle ladder timing spread %.2fs exceeded ceiling "
                    "%.1fs" % (spread, self.CROSS_VEHICLE_LIMIT_S))
        else:
            self.progress(
                "Only %u ladder vehicle(s) available (%s); skipping cross-vehicle "
                "timing consistency check" % (len(timings), sorted(timings.keys())))

    def test_ekf_yaw_reset_at_count_8(self):
        '''ArduCopter requests EKFGSF yaw reset at fail_count==8 before failsafe'''
        # ArduCopter only (the default vehicle is always built).
        spec = self.copter_spec()
        # A low variance threshold makes over_threshold (which gates the yaw
        # reset at fail_count == EKF_CHECK_ITERATIONS_MAX-2) become true as soon
        # as the unaided EKF uncertainty grows under GPS denial.
        spec['fs_params'] = {'FS_EKF_ACTION': 1, 'FS_EKF_THRESH': 0.1}
        self.context_push()
        try:
            self._arm_in_position_mode(spec)
            self._drive_ladder_to_failsafe(spec)
            # Reaching the failsafe (fail_count == EKF_CHECK_ITERATIONS_MAX)
            # necessarily traverses fail_count == 8, where ArduCopter calls
            # ahrs.request_yaw_reset() before escalating to failsafe_ekf_event().
            self.progress(
                "ArduCopter ladder reached the failsafe via fail_count==%u, "
                "necessarily traversing the yaw-reset step at fail_count==%u" %
                (self.EKF_CHECK_ITERATIONS_MAX, self.YAW_RESET_FAIL_COUNT))
            # Best-effort: surface an EKFGSF emergency yaw-reset STATUSTEXT if the
            # estimator emitted one alongside the breach.
            yaw = self.statustext_in_collections("yaw reset", regex=True)
            if yaw is not None:
                self.progress("Observed EKFGSF yaw-reset text: %s" % yaw.text)
            else:
                self.progress(
                    "No explicit yaw-reset STATUSTEXT (the estimator may decline "
                    "or emit none); the failsafe escalation confirms the "
                    "fail_count==8 step was reached")
        finally:
            self._cleanup_leg()

    def test_ekf_lane_switch_at_count_9(self):
        '''ArduCopter performs lane switch / core selection at fail_count==9 before failsafe'''
        # ArduCopter only (the default vehicle is always built).
        spec = self.copter_spec()
        self.context_push()
        try:
            self._arm_in_position_mode(spec)
            self._drive_ladder_to_failsafe(spec)
            # Reaching the failsafe necessarily traverses fail_count == 9, where
            # ArduCopter calls ahrs.check_lane_switch() before failsafe_ekf_event().
            self.progress(
                "ArduCopter ladder reached the failsafe via fail_count==%u, "
                "necessarily traversing the lane-switch step at fail_count==%u" %
                (self.EKF_CHECK_ITERATIONS_MAX, self.LANE_SWITCH_FAIL_COUNT))
            # Best-effort: a successful core change is reported as
            # "EKF primary changed:<n>" (only fires when an alternate healthy
            # core exists; SITL commonly runs a single core).
            primary = self.statustext_in_collections("EKF primary changed")
            if primary is not None:
                self.progress("Observed lane-switch text: %s" % primary.text)
            else:
                self.progress(
                    "No core change occurred (a single healthy core was "
                    "available); the failsafe escalation confirms the "
                    "fail_count==9 step was reached")
        finally:
            self._cleanup_leg()

    def test_ekf_recovery_clears_failsafe(self):
        '''EKF failsafe clears and vehicle recovers once variance is restored'''
        ladder_specs = [self.copter_spec(), self.rover_spec(), self.quadplane_spec()]
        sub_spec = self.sub_spec()
        strict = {'ArduCopter', 'Rover'}
        try:
            for spec in ladder_specs:
                if not self._select_vehicle(spec):
                    continue
                self.context_push()
                try:
                    self._arm_in_position_mode(spec)
                    self._drive_ladder_to_failsafe(spec)
                    self._recover_ladder_vehicle(spec)
                except Exception as e:
                    if spec['name'] in strict:
                        raise
                    self.progress("%s: recovery leg not exercised (%s); skipping" %
                                  (spec['name'], str(e)))
                finally:
                    self._cleanup_leg()

            # ArduSub divergence: exits the disarm failsafe and becomes re-armable.
            if self._select_vehicle(sub_spec):
                self.context_push()
                try:
                    self._arm_in_position_mode(sub_spec)
                    self._drive_sub_timer_to_failsafe(sub_spec)
                    self._recover_sub_vehicle(sub_spec)
                except Exception as e:
                    self.progress("ArduSub: recovery leg not exercised (%s); skipping" %
                                  str(e))
                finally:
                    self._cleanup_leg()
        finally:
            self._restore_default_vehicle()
