// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final Drive m_Drive = new Drive();
  // private final Joystick controller1 = new Joystick(0);
  // JoystickButton button1 = new JoystickButton(controller1, 1);
  // JoystickButton button3 = new JoystickButton(controller1, 2);
  CommandXboxController driverController = new CommandXboxController(0);
  // XboxController XBOXTHING = new XboxController(0); // 0 is the USB Port to be
  // used as indicated on the Driver Station

  public RobotContainer() {
    configureBindings();

  }

  private void configureBindings() {

    // button1.onTrue(new MotorGo(m_Drive));//can also be onFalse
    // button1.whileTrue(new StartEndCommand(() -> m_Drive.motorSpin(.5), () ->
    // m_Drive.motorSpin(0), m_Drive));
    // button1.onFalse(new RunCommand(() -> m_Drive.printAThing("NoThing"),
    // m_Drive));

    // InstantCommand runMotorCommand = new InstantCommand(() -> {
    // m_Drive.motorSpin(.5);
    // });

    // InstantCommand stopMotorCommand = new InstantCommand(() -> {
    // m_Drive.motorSpin(0);
    // });

    // button1.onTrue(new SequentialCommandGroup(runMotorCommand, new
    // WaitCommand(5), stopMotorCommand));
    // double controller1Y = controller1.getRawAxis(1);
    // System.out.println(controller1Y);

    // double controller1X = controller1.getRawAxis(0);
    // System.out.println(controller1X);
    //

    // button1.whileTrue(new joystickYPrint(controller1.getRawAxis(0),
    // controller1.getRawAxis(1)));
    // driverController.a().whileTrue(new RunCommand(() ->
    // m_Drive.printAThing(driverController.getLeftY()), m_Drive));
    double YVlaue = -driverController.getLeftY();
    m_Drive.setDefaultCommand(new RunCommand(() -> m_Drive.printAThing(-driverController.getLeftY()), m_Drive));
    driverController.a().whileTrue(new RunCommand(() -> m_Drive.shooterGo(), m_Drive));
    driverController.a().whileFalse(new RunCommand(() -> m_Drive.shooterNoGo(), m_Drive));
    
    driverController.b().whileTrue(new RunCommand(() -> m_Drive.climberUp(), m_Drive));
    driverController.b().whileFalse(new RunCommand(() -> m_Drive.climberStop(), m_Drive));

    driverController.y().whileTrue(new RunCommand(() -> m_Drive.climberDown(), m_Drive));
    driverController.y().whileFalse(new RunCommand(() -> m_Drive.climberStop(), m_Drive));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
