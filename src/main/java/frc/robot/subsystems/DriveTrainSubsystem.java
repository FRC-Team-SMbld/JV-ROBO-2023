// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.NavX;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.SPI;

public class DriveTrainSubsystem extends SubsystemBase {
    private WPI_TalonSRX rightDriveMotor1;
    private WPI_TalonSRX rightDriveMotor2;
    private MotorControllerGroup rightMotors;
    private WPI_TalonSRX leftDriveMotor1;
    private WPI_TalonSRX leftDriveMotor2;
    private MotorControllerGroup leftMotors;
    private DifferentialDrive drive;
    private final NavX navX;
    private SlewRateLimiter filtermove;
    private SlewRateLimiter filterturn;


    public DriveTrainSubsystem() {
        rightDriveMotor1 = new WPI_TalonSRX(DriveTrainConstants.rightMotor1);
        rightDriveMotor1.configFactoryDefault();
        rightDriveMotor1.configPeakCurrentLimit(30);
        rightDriveMotor2 = new WPI_TalonSRX(DriveTrainConstants.rightMotor2);
        rightDriveMotor2.configFactoryDefault();
        rightDriveMotor2.configPeakCurrentLimit(30);
        rightMotors = new MotorControllerGroup(rightDriveMotor1, rightDriveMotor2);

        leftDriveMotor1 = new WPI_TalonSRX(DriveTrainConstants.leftMotor1);
        leftDriveMotor1.configFactoryDefault();
        leftDriveMotor1.configPeakCurrentLimit(30);
        leftDriveMotor2 = new WPI_TalonSRX(DriveTrainConstants.leftMotor2);
        leftDriveMotor2.configFactoryDefault();
        leftDriveMotor2.configPeakCurrentLimit(30);
        leftMotors = new MotorControllerGroup(leftDriveMotor1, leftDriveMotor2);

        drive = new DifferentialDrive(leftMotors, rightMotors);
        navX = new NavX(SPI.Port.kMXP);

        filtermove = new SlewRateLimiter(3.5);
        filterturn = new SlewRateLimiter(3.5);
    }
    public void manualDrive(double move, double turn){
        System.err.println("Manuel Drive Called " + move + " " + turn);
        move *= 0.89;
        turn *= 0.89;
        drive.arcadeDrive(filtermove.calculate(move), filterturn.calculate(turn));
    }
    // public void setMotors(double leftSpeed, double rightSpeed){
    //     rightMotors.set(0.8 * rightSpeed);
    //     leftMotors.set(0.8 * leftSpeed);
    // }

    public double getPitch() {
        return navX.getPitch();
    }

    public void stop(){
        rightMotors.set(0.0);
        leftMotors.set(0.0);
    }
}
