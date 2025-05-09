/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.parameters.ElevatorLevel;
import frc.robot.subsystems.Subsystems;

public final class CoralAndElevatorCommands {
  /** Raises elevator and sets the coral arm angle when the elevator reaches the pivot height. */
  public static Command raiseElevatorAndTipCoralArm(Subsystems subsystems, ElevatorLevel level) {
    return Commands.sequence(
            ElevatorCommands.seekToElevatorLevel(subsystems, level),
            CoralCommands.waitForElevatorToReachArmHeight(subsystems),
            CoralCommands.setArmAngleForReefLevel(subsystems, level),
            ElevatorCommands.waitForElevatorToReachGoalHeight(subsystems.elevator),
            Commands.idle(subsystems.coralArm)
                .until(subsystems.coralArm::atGoalAngle)
                .withTimeout(0.5))
        .withName("RaiseElevatorAndCoralArm");
  }

  public static Command stowAll(Subsystems subsystems) {
    return Commands.parallel(
        ElevatorCommands.stowElevatorAndArm(subsystems),
        CoralCommands.stowGroundIntake(subsystems));
  }
}
