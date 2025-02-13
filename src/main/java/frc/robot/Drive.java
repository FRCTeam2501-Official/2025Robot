// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
  /** Creates a new Drive. */
  private final SparkMax sparky = new SparkMax(56, MotorType.kBrushless);

  private final SparkMax shooterLower = new SparkMax(19, MotorType.kBrushless); // shooter lower
  private final SparkMax shooterUpper;
  private final SparkMaxConfig shooterUpper_config = new SparkMaxConfig();
  SparkClosedLoopController shooterUpper_pid;

  private final SparkMax climberLeft = new SparkMax(5, MotorType.kBrushless); // shooter lower
  private final SparkMax climberRight = new SparkMax(13, MotorType.kBrushless); // shooter upper

  RelativeEncoder lowerEncoder = shooterLower.getEncoder();
  RelativeEncoder upperEncoder;

  RelativeEncoder LeftEncoder = climberLeft.getEncoder();
  RelativeEncoder rightEncoder = climberRight.getEncoder();

  public Drive() {
    shooterUpper_config.closedLoop
      .p(0)
      .i(0)
      .d(0)
      .velocityFF(.0002)
      .outputRange(-1, 1);

    shooterUpper = new SparkMax(15, MotorType.kBrushless); // shooter upper
      shooterUpper.configure(shooterUpper_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      shooterUpper_pid = shooterUpper.getClosedLoopController();
      upperEncoder = shooterUpper.getEncoder();
  }

  public void printAThing(double controller1x) {
    System.out.println("Drive.PrintAThing: " + controller1x);
    SmartDashboard.putNumber("Joystick X value", controller1x);
    motorSpin(controller1x);

  }

  public void motorSpin(double speed) {
    System.out.println("Drive.motorSpin.speed " + speed);
    sparky.set(speed);
  }

  public void shooterGo() {
    double setRPM = 1600;
    //PIDController pidThing = new PIDController(0.0002, 0, 0.000001); // KP KI increases initial y KD overshoot
    PIDController pidThing = new PIDController(0.0004, 0.000012, 0.0000003); // KP KI increases initial y KD overshoot
//shooterUpper.set(.5);
//shooterLower.set(pidThing.calculate(lowerEncoder.getVelocity(), setRPM));
    //shooterUpper.set(pidThing.calculate(upperEncoder.getVelocity(), setRPM));
    double currentRPMLow = lowerEncoder.getVelocity();
    double currentRPMUp = upperEncoder.getVelocity();
    double currentCalculateUp = pidThing.calculate(upperEncoder.getVelocity(), setRPM);
    SmartDashboard.putBoolean("Button A", true);
    SmartDashboard.putNumber("Set RPM", setRPM);
    SmartDashboard.putNumber("Lower Shooter RPM", currentRPMLow);
    SmartDashboard.putNumber("Upper Shooter RPM", currentRPMUp);
   // SmartDashboard.putNumber("Calc Upper", currentCalculateUp);
    // shooterLower.set(.5);
    // shooterUpper.set(.5);


  }
  public void shooterNoGo() {
    double setRPM = 1000;
    double currentRPMLow = lowerEncoder.getVelocity();
    double currentRPMUp = upperEncoder.getVelocity();

    SmartDashboard.putBoolean("Button A", false);
    SmartDashboard.putNumber("Set RPM", setRPM);
    SmartDashboard.putNumber("Lower Shooter RPM", currentRPMLow);
    SmartDashboard.putNumber("Upper Shooter RPM", currentRPMUp);

    shooterLower.set(0);
    shooterUpper.set(0);


  }
  public void climberUp() {

    double CurrentLeftEncoderValue = LeftEncoder.getPosition();
    double CurrentRightEncoderValue = LeftEncoder.getPosition();

    SmartDashboard.putNumber("LeftPosition", CurrentLeftEncoderValue);
    SmartDashboard.putNumber("RightPosition", CurrentRightEncoderValue);

    if(((CurrentRightEncoderValue + CurrentLeftEncoderValue) /2) <= 300){
      climberLeft.set(.5);
      climberRight.set(-.5);
      SmartDashboard.putBoolean("Left Climber Up", true);

    } else{
      climberLeft.set(0);
      climberRight.set(0);

      SmartDashboard.putBoolean("Left Climber Up", false);


    }


    
  }
  public void climberDown() {

    double CurrentLeftEncoderValue = LeftEncoder.getPosition();
    double CurrentRightEncoderValue = LeftEncoder.getPosition();

    SmartDashboard.putNumber("LeftPosition", CurrentLeftEncoderValue);
    SmartDashboard.putNumber("RightPosition", CurrentRightEncoderValue);

    if(((CurrentRightEncoderValue + CurrentLeftEncoderValue) /2) >= 40){
      climberRight.set(.5);
      climberLeft.set(-.5);

    SmartDashboard.putBoolean("Right Climber Down", true);
    } else{
      climberRight.set(0);
      climberLeft.set(0);

      SmartDashboard.putBoolean("Right Climber Down", false);
    }


  }
  public void climberStop() {

    climberLeft.set(0);
    climberRight.set(0); // inverted


  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
