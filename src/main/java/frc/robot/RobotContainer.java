// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Arm;

public class RobotContainer {
  Arm arm = new Arm();
  CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    controller.x().whileTrue(arm.sysIdQuasistatic(Direction.kForward));
    controller.y().whileTrue(arm.sysIdQuasistatic(Direction.kReverse));
    controller.a().whileTrue(arm.sysIdDynamic(Direction.kForward));
    controller.b().whileTrue(arm.sysIdDynamic(Direction.kReverse));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
