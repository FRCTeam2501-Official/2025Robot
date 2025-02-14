// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.ctre.phoenix.sensors.CANCoderConfiguration;
// // import com.ctre.phoenix6.hardware.CANcoder;
// // import com.ctre.phoenix6.hardware.core.CoreCANcoder;
// import com.ctre.phoenix.sensors.WPI_CANCoder;
// import com.ctre.phoenix6.StatusSignal;
// //import com.ctre.phoenix6.configs.MagnetSensorConfigs;
// import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

public class Swerve extends SubsystemBase {
  /** Creates a new Swerve. */

  double initialposition;

  //Front Left // module 1
  private final SparkMax FLTurn = new SparkMax(18, MotorType.kBrushless); //correct
  private final SparkMax FLDrive = new SparkMax(7, MotorType.kBrushless); //correct

  RelativeEncoder frontLeftTurnEncoder = FLTurn.getEncoder();
  RelativeEncoder frontLeftDriveEncoder = FLTurn.getEncoder();

  PIDController FrontLeftPID = new PIDController(0, 0, 0);


  // private final PIDController PIDFrontLeft = new PIDController(0, 0, 0);
  // private final SparkMax shooterUpper;
  //  private final SparkMaxConfig shooterUpper_config = new SparkMaxConfig();
  // SparkClosedLoopController shooterUpper_pid;

// pigeon 2 29
// 25 BR, 26 Bl, 27 FR, 28 FL

CANcoder m_cancoderFrontRight = new CANcoder(27);
CANcoder m_cancoderFrontLeft = new CANcoder(28);
CANcoder m_cancoderBackLeft = new CANcoder(26);
CANcoder m_cancoderBackRight = new CANcoder(25);


public void CANCoderAngle(){

StatusSignal<Angle> canCoderFrontRightAngle = m_cancoderFrontRight.getPosition();
StatusSignal<Angle> canCoderFrontLeftAngle = m_cancoderFrontLeft.getPosition();
StatusSignal<Angle> canCoderBackLeftAngle = m_cancoderBackLeft.getPosition();
StatusSignal<Angle> canCoderBackRightAngle = m_cancoderBackRight.getPosition();

//.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinus180;

SmartDashboard.putNumber("Front Right Angle", canCoderFrontRightAngle.getValueAsDouble()*1100/Math.PI);
SmartDashboard.putNumber("Front Left Angle", canCoderFrontLeftAngle.getValueAsDouble()*180/Math.PI);
SmartDashboard.putNumber("Back Right Angle", canCoderBackRightAngle.getValueAsDouble()*180/Math.PI);
SmartDashboard.putNumber("Back Right Angle", canCoderBackLeftAngle.getValueAsDouble()*180/Math.PI);


}

  //CoreCANcoder m_CaNcoder = new CoreCANcoder(27);
  // final CANcoder m_CANCoder = new CANcoder(27);
  // private final CANcoder cancodernumber1 = new CANcoder(26);

  // final WPI_CANCoder m_cancoder = new WPI_CANCoder(27);
  // final CANCoderConfiguration m_cancoderConfig = new CANCoderConfiguration();


   //final WPI_CANCoder m_cancoder = new WPI_CANCoder(27);
//  CANCoderConfiguration config = new CANCoderConfiguration();
// CANcoder

 //Front Right // module 1
  private final SparkMax FRTurn = new SparkMax(4, MotorType.kBrushless); //correct
  private final SparkMax FRDrive = new SparkMax(20, MotorType.kBrushless); // correct

  RelativeEncoder frontRightTurnEncoder = FRTurn.getEncoder();
  RelativeEncoder frontRightDriveEncoder = FRDrive.getEncoder();

 //Back Left // module 1
  private final SparkMax BLTurn = new SparkMax(3, MotorType.kBrushless);
  private final SparkMax BLDrive = new SparkMax(1, MotorType.kBrushless);

  RelativeEncoder backLeftTurnEncoder = BLTurn.getEncoder();
  RelativeEncoder backLeftDriveEncoder = BLDrive.getEncoder();

 //Back Right // module 1
  private final SparkMax BRTurn = new SparkMax(17, MotorType.kBrushless);
  private final SparkMax BRDrive = new SparkMax(10, MotorType.kBrushless);

  RelativeEncoder backRightTurnEncoder = BRTurn.getEncoder();
  RelativeEncoder backRightDriveEncoder = BRDrive.getEncoder();


  public Swerve() {}

  public void motorSpin(double speed) { // tester for motors

    double spinAngle = frontLeftTurnEncoder.getPosition();
        SmartDashboard.putNumber("Front Left Angle", spinAngle);

    //FLTurn.set(speed);
    //FLDrive.set(speed);
    //FRDrive.set(speed);
   //FRTurn.set(speed);

   //BLTurn.set(speed);
   //BLDrive.set(speed);
   //BRTurn.set(speed);
   BRDrive.set(speed);
  }

  public void ResetEncoders1() { // tester for motors
    
    frontLeftTurnEncoder.setPosition(0);
    frontRightTurnEncoder.setPosition(0);

    backLeftTurnEncoder.setPosition(0);
    backLeftTurnEncoder.setPosition(0);

 initialposition = frontLeftTurnEncoder.getPosition();

  }

  public void motorStop() { // all swerve stop

    //double spinAngle = frontLeftTurnEncoder.getPosition();
    // var spingThing = cancodernumber1.getPosition();
    // SmartDashboard.putNumber("springThing",springThing);
        //SmartDashboard.putNumber("Front Left Angle", spinAngle);

    //FLTurn.set(speed);
    FLDrive.set(0);
    FRDrive.set(0);
    BLDrive.set(0);
    BRDrive.set(0);

    FLTurn.set(0);
    FRTurn.set(0);
    BLTurn.set(0);
    BRTurn.set(0);
  }

public void SwerveSequence(double x, double y){
  SmartDashboard.putNumber("Left Axis Y", y);
  SmartDashboard.putNumber("Left Axis X", x);

  double angle = Math.toDegrees((Math.atan2(y,-x))) + 180;
  SmartDashboard.putNumber("Degree Of joystick", angle); // gets angle of left joystick

  double distance2 = Math.sqrt((Math.pow(y, 2) + Math.pow(x, 2)));
  if(distance2 > 1) distance2 = 1;
  SmartDashboard.putNumber("distance 2", distance2); // gets angle of left joystick

  SwerveSpeed(distance2 * .1);
  SwerveTurn(angle);


}

public void SwerveSpeed(double speed){ //swerveDrive
  FLDrive.set(speed);
  FRDrive.set(speed);
  BLDrive.set(speed);
  BRDrive.set(-speed);
}



public void SwerveTurn(double Angle){ //swervespin
  double SwerveSetPoint = Angle;

  double spinAngle1 = frontLeftTurnEncoder.getPosition();
  double spinAngle2 = frontRightTurnEncoder.getPosition();
  double spinAngle3 = backLeftTurnEncoder.getPosition();
  double spinAngle4 = backRightTurnEncoder.getPosition();

  // SmartDashboard.putNumber("Front Left Angle", spinAngle1);
  // SmartDashboard.putNumber("Front Right Angle", spinAngle2);
  // SmartDashboard.putNumber("Back Left Angle", spinAngle3);
  // SmartDashboard.putNumber("Back Right Angle", spinAngle4);

  //double SwervePosition = m_CANCoder.getPosition();
  //StatusSignal<Angle> swerveAngle = number1s();
  //double swerveTest = number1s();

//System.out.println(swerveAngle);
//SmartDashboard.putNumber("Position FR", swerveAngle); // gets angle of left joystick

  // FLDrive.set(speed);
  // FRDrive.set(speed);
  // BLDrive.set(speed);
  // BRDrive.set(speed);
  //number1s().getValue()*360;
FLTurn.set(FrontLeftPID.calculate(1, SwerveSetPoint));
  SmartDashboard.putNumber("Front Left PID", FrontLeftPID.calculate(SwerveSetPoint-spinAngle1 , SwerveSetPoint));
Double AngleOffset = Angle -180;


if(AngleOffset >= spinAngle1 ) FLTurn.set(.1);
if(AngleOffset<= spinAngle1 ) FLTurn.set(-.1);

if(AngleOffset >= spinAngle2 ) FRTurn.set(.1);
if(AngleOffset <= spinAngle2 ) FRTurn.set(-.1);

if(AngleOffset >= spinAngle3 ) BLTurn.set(.1);
if(AngleOffset <= spinAngle3 ) BLTurn.set(-.1);

if(AngleOffset >= spinAngle4 ) BRTurn.set(.1);
if(AngleOffset <= spinAngle4 ) BRTurn.set(-.1);

shooterGo(SwerveSetPoint , spinAngle2);
}

public void shooterGo(double joystickangle, double FrontRightAngle) {
  SmartDashboard.putBoolean("Bumper",true);

  double SwerveSetPoint = (joystickangle - 180);

  SmartDashboard.putNumber("SwerveSetpoint", SwerveSetPoint);

  //PIDController pidThing = new PIDController(0.0002, 0, 0.000001); // KP KI increases initial y KD overshoot
  PIDController pidThing = new PIDController(0.0004, 0.0, 0.00001); // KP KI increases initial y KD overshoot
//shooterUpper.set(.5);
//shooterLower.set(pidThing.calculate(lowerEncoder.getVelocity(), setRPM));
  //shooterUpper.set(pidThing.calculate(upperEncoder.getVelocity(), setRPM));
  double currentRPMLow = frontLeftTurnEncoder.getPosition();
  double currentCalculateUp = pidThing.calculate(frontLeftTurnEncoder.getPosition(), SwerveSetPoint);
//FRTurn.set(currentCalculateUp);
}

public void FRTest(){
  FLTurn.set(.2);

}

public void FRTest2(){
  FLTurn.set(-.2);

}

// public StatusSignal<Angle> number1s(){
//   return cancodernumber1.getPosition();
// }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
