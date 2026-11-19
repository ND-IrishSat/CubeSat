"""Per-mode control dispatch.

Allocation and torquer timing happen after this controller.
"""
from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from cloversat.control.attitude_pd_law import attitude_pd_torque
from cloversat.control.bcross_law import bcross_dipole
from cloversat.control.momentum_dump_law import (
    dump_active,
    momentum_dump_dipole,
)
from cloversat.datatypes.command_types import (
    ActuatorCommand,
    Mode,
    ModeCommand,
    SubMode,
)
from cloversat.datatypes.state_types import AttitudeTarget, StateEstimate
from cloversat.datatypes.vector_types import Vec3

if TYPE_CHECKING:
    from cloversat.config.control_params import ControlParams
    from cloversat.config.spacecraft_params import SpacecraftParams



class Controller:
    #Chooses a control law and returns an actuator request.

    def __init__(self, params: ControlParams, spacecraft: SpacecraftParams) -> None:
        self.params = params
        self.spacecraft = spacecraft
        self._dump_is_active = False


# The main decision-making function, determines which control algorithm to use under a certain mode.
    def step( self, command: ModeCommand, estimate: StateEstimate, B_body: Vec3) -> ActuatorCommand:
    # BOOT MODE

    # During spacecraft startup, do not request any actuator movement. Return zero reaction-wheel torque and zero magnetic dipole.
    if command.mode is Mode.BOOT:
        return self.zero_command("boot")

    # DETUMBLE MODE
    # The spacecraft may be rotating uncontrollably after deployment. Use the B-cross control law to calculate a magnetic dipole that helps reduce its angular velocity using Earth's magnetic field.Reaction-wheel torque is not requested during detumbling.
    if command.mode is Mode.DETUMBLE:
        return self._detumble(estimate, B_body, "detumble")

    # ORIENTATION TRANSITION MODE
    # Transition from detumbling to controlled attitude pointing. Before using reaction-wheel attitude control, verify that:
    # 1. The estimated spacecraft orientation is valid.
    # 2. The estimated angular velocity is valid.
    # 3. The state manager has supplied a pointing target.
    # 4. The supplied pointing target is valid.
    if command.mode is Mode.ORIENT_TRANSITION:
        if (
            estimate.attitude_valid
            and estimate.rates_valid
            and command.target is not None
            and command.target.valid
        ):
            # All required information is available.
            # Use PD attitude control to calculate the reaction-wheel
            # torque needed to move toward the desired orientation.
            return self._point(command.target, estimate, "orient_transition")

        # If any of the conditions above are not satisfied, precise
        # attitude control cannot be performed reliably.
        # Fall back to B-cross detumbling using the magnetorquers.
        # If angular-rate data is also invalid, _detumble returns zeros.
        return self._detumble(estimate, B_body, "orient_transition_bcross")

    # OPERATIONAL MODE
    # The spacecraft is performing its normal mission operations.
    # Select the appropriate control behavior based on its submode.
    if command.mode is Mode.OPERATIONAL:

        # MOMENTUM DUMPING
        # Reaction wheels can accumulate angular momentum over time.
        # Use magnetorquers to help remove this accumulated momentum.
        # If valid attitude information and a target are available,
        # the spacecraft can continue pointing while dumping momentum.
        if command.submode is SubMode.MOMENTUM_DUMP:
            return self._momentum_dump(command, estimate, B_body)

        # ATTITUDE POINTING
        # The spacecraft may need to point toward the Sun, a GPS-related
        # target, or a ground target, depending on its mission submode.
        if command.submode in (
            SubMode.SUN_POINT,
            SubMode.GPS_POINT,
            SubMode.GROUND_POINT,
        ):
            # Retrieve and validate the attitude target provided
            # by the state manager.
            # This function does not calculate the target itself.
            target = self.resolve_target(command, estimate)

            # Apply PD attitude control to calculate the reaction-wheel
            # torque required to reach or maintain the desired orientation.
            # The submode's string value identifies the command source.
            return self._point(target, estimate, command.submode.value)

        # If the operational submode is not recognized, do not request
        # any actuation. Return zero torque and zero magnetic dipole.
        return self.zero_command("operational_invalid_submode")

    # SAFE MODE
    # Enter a protective control state when the spacecraft cannot
    # continue normal operations or needs to recover.
    # Prefer maintaining a valid Sun-pointing target when available.
    if command.mode is Mode.SAFE:

        # Check whether valid orientation, angular-rate information,
        # and a pointing target are available.
        if (
            estimate.attitude_valid
            and estimate.rates_valid
            and command.target is not None
            and command.target.valid
        ):
            # Use PD attitude control to hold the supplied target.
            # The state manager is responsible for ensuring that
            # this is the correct Sun-pointing target for SAFE mode.
            return self._point(command.target, estimate, "safe_sun_hold")

        # If valid attitude pointing is not possible, fall back
        # to B-cross detumbling to reduce unwanted spacecraft rotation.
        # If rate information is invalid, zero actuation is requested.
        return self._detumble(estimate, B_body, "safe_bcross")

    # UNKNOWN MODE
    # If the received mode does not match any supported operating mode,
    # return zero actuation rather than executing an undefined control law.
    return self.zero_command("unknown_mode")

    def zero_command(self, source: str) -> ActuatorCommand:
        """Return an all-zero actuator request."""
        return ActuatorCommand(
            wheel_torque_body=np.zeros(3, dtype=np.float64),
            dipole_body=np.zeros(3, dtype=np.float64),
            source=source,
        )

    def resolve_target(
        self,
        command: ModeCommand,
        estimate: StateEstimate,
    ) -> AttitudeTarget:
        """Return the target supplied by the state manager."""
        if command.submode not in (
            SubMode.SUN_POINT,
            SubMode.GPS_POINT,
            SubMode.GROUND_POINT,
        ):
            raise ValueError(f"Cannot resolve target for submode: {command.submode}")

        if command.target is None or not command.target.valid:
            raise ValueError(f"Missing or invalid target for {command.submode.value}")

        if not estimate.attitude_valid:
            raise ValueError("Attitude estimate is invalid")

        return command.target

    def _detumble(
        self,
        estimate: StateEstimate,
        B_body: Vec3,
        source: str,
    ) -> ActuatorCommand:
        if not estimate.rates_valid:
            return self.zero_command(f"{source}_rates_invalid")

        if self.params.bcross_gain is None:
            raise ValueError("ControlParams.bcross_gain is required for B-cross control")

        dipole = bcross_dipole(
            omega_body=estimate.omega_body,
            B_body=B_body,
            gain=self.params.bcross_gain,
        )
        return ActuatorCommand(
            wheel_torque_body=np.zeros(3, dtype=np.float64),
            dipole_body=dipole,
            source=source,
        )

    def _point(
        self,
        target: AttitudeTarget,
        estimate: StateEstimate,
        source: str,
    ) -> ActuatorCommand:
        if not estimate.attitude_valid or not estimate.rates_valid or not target.valid:
            return self.zero_command(f"{source}_state_invalid")

        if self.params.kp is None or self.params.kd is None:
            raise ValueError("ControlParams.kp and kd are required for attitude control")

        torque = attitude_pd_torque(
            q=estimate.q,
            q_target=target.q_target,
            omega_body=estimate.omega_body,
            omega_target_body=target.omega_target_body,
            kp=self.params.kp,
            kd=self.params.kd,
        )
        return ActuatorCommand(
            wheel_torque_body=torque,
            dipole_body=np.zeros(3, dtype=np.float64),
            source=source,
        )

    def _momentum_dump(
        self,
        command: ModeCommand,
        estimate: StateEstimate,
        B_body: Vec3,
    ) -> ActuatorCommand:
        if not estimate.wheels_valid:
            self._dump_is_active = False
            return self.zero_command("momentum_dump_wheels_invalid")

        required = (
            self.params.dump_gain,
            self.params.dump_h_start,
            self.params.dump_h_stop,
        )
        if any(value is None for value in required):
            raise ValueError(
                "dump_gain, dump_h_start, and dump_h_stop are required "
                "for momentum dumping"
            )

        self._dump_is_active = dump_active(
            h_wheels_body=estimate.h_wheels_body,
            was_active=self._dump_is_active,
            h_start=self.params.dump_h_start,
            h_stop=self.params.dump_h_stop,
        )

        dipole = np.zeros(3, dtype=np.float64)
        if self._dump_is_active:
            dipole = momentum_dump_dipole(
                B_body=B_body,
                h_wheels_body=estimate.h_wheels_body,
                gain=self.params.dump_gain,
            )

        # Keep attitude control active during dumping when a target is available.
        if (
            estimate.attitude_valid
            and estimate.rates_valid
            and command.target is not None
            and command.target.valid
        ):
            pointed = self._point(command.target, estimate, "momentum_dump")
            return ActuatorCommand(
                wheel_torque_body=pointed.wheel_torque_body,
                dipole_body=dipole,
                source="momentum_dump",
            )

        return ActuatorCommand(
            wheel_torque_body=np.zeros(3, dtype=np.float64),
            dipole_body=dipole,
            source="momentum_dump",
        )