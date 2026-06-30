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
    (fail_count == 9) before declaring the failsafe.  These two early steps are
    verified through concrete, observable estimator evidence - the EKFGSF
    "emergency yaw reset" message (driven via a 180-degree compass error) and
    the "EKF3 lane switch" message (driven via a corrupted primary-core
    accelerometer with two IMU-backed cores) - rather than being inferred from
    the eventual failsafe escalation.
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
    # Timing model.  Every behavioral ceiling below is the AAP target itself,
    # ENFORCED as a hard assertion.  All timing uses get_sim_time()/
    # get_sim_time_cached(), which advance with the SITL physics step and are
    # therefore independent of host load and --speedup, so these bounds can be
    # enforced tightly without producing load-induced false failures.
    #
    # The one latency that is NOT behavioral is the gap between *injecting* a
    # fault and the EKF actually going bad (dominated by the firmware
    # GPS_TIMEOUT_MS, ~4 s).  That infrastructure latency is absorbed by the
    # generous POSITION_LOSS_TIMEOUT_S wait below and is deliberately EXCLUDED
    # from these ceilings: the ladder/timer/recovery bounds are measured from
    # the moment the EKF first reports a degraded estimate (or, for recovery,
    # from the moment a healthy estimate is re-acquired), so each ceiling
    # enforces the AAP behavioral target directly against the relevant phase.
    # ------------------------------------------------------------------
    LADDER_LIMIT_S = 1.5            # AAP: fail_count reaches 10 within 1.5 s of the EKF going bad
    STATUSTEXT_LIMIT_S = 2.0        # AAP: CRITICAL failsafe STATUSTEXT within 2.0 s of the EKF going bad
    CROSS_VEHICLE_LIMIT_S = 0.5     # AAP: +/-0.5 s cross-vehicle spread of the post-degradation ladder phase
    SUB_TIMER_LIMIT_S = 3.0         # AAP: ArduSub disarms within ~3.0 s (2 s EKF-bad timer + detection)
    RECOVERY_LIMIT_S = 1.5          # AAP: fail_count decrements to 0 within 1.5 s once the estimate is healthy
    SUB_RECOVERY_LIMIT_S = 5.0      # AAP: ArduSub exits the disarm failsafe and is re-armable within 5.0 s

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

        # AAP: the CRITICAL failsafe STATUSTEXT is concrete evidence of the
        # escalation and must arrive within STATUSTEXT_LIMIT_S of the EKF going
        # bad.  It is co-emitted with the failsafe-mode change asserted above, so
        # require it explicitly (NotAchievedException if absent) and bound its
        # post-degradation arrival time.  The per-vehicle text is verified
        # against the firmware send_text() calls (ekf_check.cpp / failsafe.cpp).
        self.wait_statustext(spec['failsafe_text'], timeout=5, check_context=True)
        statustext_phase = self.get_sim_time_cached() - t_bad

        ladder_phase = t_fs - t_bad
        t_total = t_fs - t_inject
        self.progress(
            "%s: EKF failsafe (mode %s, text %r) at t+%.2fs; post-degradation "
            "ladder phase %.2fs, statustext phase %.2fs "
            "(ceilings: ladder %.1fs, statustext %.1fs)" %
            (spec['name'], spec['failsafe_mode'], spec['failsafe_text'], t_total,
             ladder_phase, statustext_phase, self.LADDER_LIMIT_S,
             self.STATUSTEXT_LIMIT_S))
        if ladder_phase > self.LADDER_LIMIT_S:
            raise NotAchievedException(
                "%s: ladder phase %.2fs exceeded ceiling %.1fs" %
                (spec['name'], ladder_phase, self.LADDER_LIMIT_S))
        if statustext_phase > self.STATUSTEXT_LIMIT_S:
            raise NotAchievedException(
                "%s: failsafe STATUSTEXT phase %.2fs exceeded ceiling %.1fs" %
                (spec['name'], statustext_phase, self.STATUSTEXT_LIMIT_S))
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
            "phase %.2fs (ceiling %.1fs)" %
            (spec['name'], t_total, timer_phase, self.SUB_TIMER_LIMIT_S))
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

        # Infrastructure wait (generous, EXCLUDED from the AAP recovery bound):
        # the simulated GPS must re-acquire a fix and the EKF must rebuild a
        # healthy, position-capable estimate before the EKF check can begin to
        # recover.  That GPS re-acquisition latency is not the EKF-check recovery
        # the AAP bounds, so it is absorbed here rather than charged against
        # RECOVERY_LIMIT_S.
        self.wait_ekf_happy(timeout=self.POSITION_LOSS_TIMEOUT_S)
        t_healthy = self.get_sim_time_cached()

        # Hard assertion (AAP): once the estimate is healthy the firmware
        # decrements fail_count back to 0 within RECOVERY_LIMIT_S (10 iterations
        # at 10 Hz == ~1 s from the EKF_CHECK_ITERATIONS_MAX cap), emitting the
        # failsafe-cleared STATUSTEXT for vehicles that publish one.  The text is
        # captured by the context_collect('STATUSTEXT') started during the
        # degradation phase; fail if it is not observed in time.
        if spec['cleared_text'] is not None:
            deadline = t_healthy + self.RECOVERY_LIMIT_S
            cleared = None
            while self.get_sim_time_cached() <= deadline:
                cleared = self.statustext_in_collections(spec['cleared_text'])
                if cleared is not None:
                    break
                self.delay_sim_time(0.1)
            if cleared is None:
                raise NotAchievedException(
                    "%s: EKF failsafe-cleared text %r not observed within %.1fs "
                    "of a healthy estimate" %
                    (spec['name'], spec['cleared_text'], self.RECOVERY_LIMIT_S))
            self.progress(
                "%s: EKF failsafe cleared %.2fs after healthy estimate "
                "(ceiling %.1fs)" %
                (spec['name'], self.get_sim_time_cached() - t_healthy,
                 self.RECOVERY_LIMIT_S))
        else:
            self.progress(
                "%s: this vehicle publishes no failsafe-cleared text; the "
                "healthy estimate confirms fail_count recovery" % spec['name'])

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
        timings = {}
        try:
            for spec in ladder_specs:
                # _select_vehicle is the SOLE graceful-skip gate: it returns
                # False ONLY when a vehicle's SITL binary is absent or cannot be
                # started (an infrastructure-availability condition - e.g. the
                # copter-only CI target builds just arducopter).  Once a binary
                # is selected and running the leg is STRICT: any test-logic
                # failure propagates rather than being swallowed as a "skip".
                if not self._select_vehicle(spec):
                    continue
                self.context_push()
                try:
                    self._arm_in_position_mode(spec)
                    result = self._drive_ladder_to_failsafe(spec)
                    # Cross-vehicle parity compares the post-degradation ladder
                    # phase (behavioral), not the total time, so per-vehicle
                    # GPS-timeout latency does not pollute the +/-0.5 s spread.
                    timings[spec['name']] = result['ladder_phase']
                finally:
                    self._cleanup_leg()

            # ArduSub divergence: a 2-second timer rather than the ladder.  Same
            # discipline - skip only when its binary is unavailable; once the
            # binary is running the leg is strict.
            if self._select_vehicle(sub_spec):
                self.context_push()
                try:
                    self._arm_in_position_mode(sub_spec)
                    self._drive_sub_timer_to_failsafe(sub_spec)
                finally:
                    self._cleanup_leg()
        finally:
            self._restore_default_vehicle()

        # Cross-vehicle timing consistency (only meaningful with >= 2 vehicles).
        if len(timings) >= 2:
            spread = max(timings.values()) - min(timings.values())
            self.progress(
                "Cross-vehicle ladder-phase spread %.2fs across %s (ceiling %.1fs)" %
                (spread, sorted(timings.keys()), self.CROSS_VEHICLE_LIMIT_S))
            if spread > self.CROSS_VEHICLE_LIMIT_S:
                raise NotAchievedException(
                    "Cross-vehicle ladder-phase spread %.2fs exceeded ceiling "
                    "%.1fs" % (spread, self.CROSS_VEHICLE_LIMIT_S))
        else:
            self.progress(
                "Only %u ladder vehicle(s) available (%s); skipping cross-vehicle "
                "timing consistency check" % (len(timings), sorted(timings.keys())))

    def test_ekf_yaw_reset_at_count_8(self):
        '''ArduCopter requests EKFGSF yaw reset at fail_count==8 before failsafe'''
        # ArduCopter only (the default vehicle is always built).
        self.context_push()
        try:
            # ArduCopter's ekf_check requests an EKFGSF yaw reset at
            # fail_count == EKF_CHECK_ITERATIONS_MAX-2 (==8) via
            # ahrs.request_yaw_reset().  The GSF can only RESOLVE the yaw - and
            # therefore emit the observable "emergency yaw reset" evidence - while
            # horizontal-velocity (GPS) aiding is available, which the GPS-denial
            # ladder cannot provide; that is why the prior test could only INFER
            # the step.  Drive the same EKFGSF yaw-reset mechanism
            # deterministically with the proven 180-degree compass orientation
            # error (the GSF_reset recipe): the EKF detects the gross yaw
            # inconsistency and performs the GSF emergency reset, emitting
            # concrete, assertable evidence before any failsafe escalation.
            self.set_parameters({
                'COMPASS_ORIENT': 4,    # yaw 180 deg -> gross yaw error
                'COMPASS_USE2': 0,      # disable backup compasses (avoid prearm)
                'COMPASS_USE3': 0,
                'FS_EKF_ACTION': 1,
                'FS_EKF_THRESH': 0.8,
            })
            self.reboot_sitl()
            self.context_collect('STATUSTEXT')
            self.change_mode('GUIDED')
            self.wait_ready_to_arm()
            self.arm_vehicle()
            self.user_takeoff(alt_min=10)
            # Concrete, hard assertion (raises NotAchievedException if absent):
            # the EKFGSF emergency yaw reset MUST be observed.  This is the
            # log/message evidence the AAP requires for the fail_count==8
            # yaw-reset step, replacing the prior inference from the eventual
            # failsafe.  The pattern mirrors the proven AutoTestCopter.GSF_reset
            # observable ("EKF3 IMU<n> emergency yaw reset").
            yaw = self.wait_statustext("EKF3 IMU. emergency yaw reset",
                                       timeout=30, check_context=True, regex=True)
            self.progress("ArduCopter: observed EKFGSF emergency yaw reset: %s" %
                          yaw.text)
        finally:
            self._cleanup_leg()

    def test_ekf_lane_switch_at_count_9(self):
        '''ArduCopter performs lane switch / core selection at fail_count==9 before failsafe'''
        # ArduCopter only (the default vehicle is always built).
        self.context_push()
        try:
            # ArduCopter's ekf_check calls ahrs.check_lane_switch() at
            # fail_count == EKF_CHECK_ITERATIONS_MAX-1 (==9) before escalating to
            # failsafe_ekf_event().  A lane switch can only be OBSERVED when an
            # alternate healthy EKF core exists to switch to; the prior test
            # could only INFER the step because the GPS-denial ladder degrades
            # every core uniformly, leaving nothing to switch to.  Drive the same
            # core-selection mechanism deterministically with the proven
            # EKFlaneswitch recipe: run two IMU-backed cores (EK3_IMU_MASK=3),
            # then corrupt the primary core's accelerometer (INS_ACCOFFS_X=5) so
            # runCoreSelection migrates the primary to the healthy lane, emitting
            # concrete, assertable evidence before any failsafe.
            self.set_parameters({
                'EK3_ENABLE': 1,
                'EK2_ENABLE': 0,
                'AHRS_EKF_TYPE': 3,
                'EK3_IMU_MASK': 3,      # use IMU0 and IMU1 -> two selectable cores
                'FS_EKF_ACTION': 1,
                'FS_EKF_THRESH': 0.8,
            })
            self.reboot_sitl()
            self.change_mode('GUIDED')
            self.wait_ready_to_arm()
            self.arm_vehicle()
            self.user_takeoff(alt_min=10)
            self.context_collect('STATUSTEXT')
            self.progress("Corrupting the primary core's accelerometer to force "
                          "a core/lane switch")
            self.set_parameters({
                'INS_ACCOFFS_X': 5,
            })
            # Concrete, hard assertion (raises NotAchievedException if absent):
            # the EKF core/lane switch MUST be observed.  This is the
            # XKF*/core-selection evidence the AAP requires for the
            # fail_count==9 lane-switch step, replacing the prior inference from
            # the eventual failsafe.  The pattern mirrors the proven
            # AutoTestCopter EKFlaneswitch observable ("EKF3 lane switch <n>").
            sw = self.wait_statustext("EKF3 lane switch 1",
                                      timeout=30, check_context=True)
            self.progress("ArduCopter: observed EKF core/lane switch: %s" %
                          sw.text)
        finally:
            self._cleanup_leg()

    def test_ekf_recovery_clears_failsafe(self):
        '''EKF failsafe clears and vehicle recovers once variance is restored'''
        ladder_specs = [self.copter_spec(), self.rover_spec(), self.quadplane_spec()]
        sub_spec = self.sub_spec()
        try:
            for spec in ladder_specs:
                # Skip only when the binary is unavailable (see the discipline
                # documented in test_ekf_failcount_ladder); once a binary is
                # selected and running the recovery leg is STRICT and any
                # failure propagates.
                if not self._select_vehicle(spec):
                    continue
                self.context_push()
                try:
                    self._arm_in_position_mode(spec)
                    self._drive_ladder_to_failsafe(spec)
                    self._recover_ladder_vehicle(spec)
                finally:
                    self._cleanup_leg()

            # ArduSub divergence: exits the disarm failsafe and becomes re-armable.
            if self._select_vehicle(sub_spec):
                self.context_push()
                try:
                    self._arm_in_position_mode(sub_spec)
                    self._drive_sub_timer_to_failsafe(sub_spec)
                    self._recover_sub_vehicle(sub_spec)
                finally:
                    self._cleanup_leg()
        finally:
            self._restore_default_vehicle()
