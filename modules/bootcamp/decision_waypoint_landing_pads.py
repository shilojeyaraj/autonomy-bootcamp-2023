"""
BOOTCAMPERS TO COMPLETE.

Travel to designated waypoint and then land at a nearby landing pad.
"""

from .. import commands
from .. import drone_report

# Disable for bootcamp use
# pylint: disable-next=unused-import
from .. import drone_status
from .. import location
from ..private.decision import base_decision


# Disable for bootcamp use
# No enable
# pylint: disable=duplicate-code,unused-argument


class DecisionWaypointLandingPads(base_decision.BaseDecision):
    """
    Travel to the designed waypoint and then land at the nearest landing pad.
    """

    def __init__(self, waypoint: location.Location, acceptance_radius: float) -> None:
        """
        Initialize all persistent variables here with self.
        """
        self.waypoint = waypoint
        print(f"Waypoint: {waypoint}")

        self.acceptance_radius = acceptance_radius

        # ============
        # ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
        # ============

        # Add your own
        self.has_landed = False
        self.phase = "to_wp"  # "to_wp" or "to_pad"
        self.pad_target = None
        # ============
        # ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
        # ============

    def run(
        self, report: drone_report.DroneReport, landing_pad_locations: "list[location.Location]"
    ) -> commands.Command:
        """
        Make the drone fly to the waypoint and then land at the nearest landing pad.

        You are allowed to create as many helper methods as you want,
        as long as you do not change the __init__() and run() signatures.

        This method will be called in an infinite loop, something like this:

        ```py
        while True:
            report, landing_pad_locations = get_input()
            command = Decision.run(report, landing_pad_locations)
            put_output(command)
        ```
        """
        # Default command
        command = commands.Command.create_null_command()

        # ============
        # ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
        # ============
        status = report.status.name  # "HALTED" | "MOVING" | "LANDED"
        pos = report.position

        # If the sim already says we're landed, just noop
        if status == "LANDED":
            return command

        # ---- Phase 1: go to waypoint ----
        if self.phase == "to_wp":
            dx = self.waypoint.x - pos.x
            dy = self.waypoint.y - pos.y
            dist = (dx * dx + dy * dy) ** 0.5

            if dist <= self.acceptance_radius:
                # We're within radius of the waypoint:
                # If moving, HALT first; if halted, switch to pad-seeking
                if status == "HALTED":
                    self.phase = "to_pad"
                else:
                    return commands.Command.create_halt_command()
            else:
                # Need to move toward waypoint; only valid when HALTED
                if status == "HALTED":
                    return commands.Command.create_set_relative_destination_command((dx, dy))
                # If already moving, do nothing this tick

        # ---- Phase 2: go to nearest landing pad, then land ----
        if self.phase == "to_pad":
            # Pick nearest pad once (or re-pick if list changes/was empty)
            if (self.pad_target is None) and landing_pad_locations:
                # choose argmin distance
                self.pad_target = min(
                    landing_pad_locations,
                    key=lambda p: (p.x - pos.x) * (p.x - pos.x) + (p.y - pos.y) * (p.y - pos.y),
                )

            # If no pads detected yet, just hold (null or halt if moving)
            if self.pad_target is None:
                if status != "HALTED":
                    return commands.Command.create_halt_command()
                return command  # wait for detections

            # Move toward chosen pad
            pdx = self.pad_target.x - pos.x
            pdy = self.pad_target.y - pos.y
            pdist_squared = pdx * pdx + pdy * pdy

            if pdist_squared <= self.acceptance_radius**2:
                # Inside landing radius: HALT if moving; LAND if halted
                if status == "HALTED":
                    return commands.Command.create_land_command()
                return commands.Command.create_halt_command()
            else:
                # Need to move toward pad; only valid when HALTED
                if status == "HALTED":
                    return commands.Command.create_set_relative_destination_command((pdx, pdy))

        # Default: advance sim with no change
        return command

        # ============
        # ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
        # ============
