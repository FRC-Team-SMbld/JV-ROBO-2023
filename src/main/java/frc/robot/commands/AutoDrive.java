package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

// This command self=balances on the charging station using gyroscope pitch as feedback
public class AutoDrive extends CommandBase {

  private DriveTrainSubsystem m_DriveSubsystem;

  private double drivePower;
  private Timer time;
  private double driveTime;

  /** Command to use Gyro data to resist the tip angle from the beam - to stabalize and balanace */
  public AutoDrive(DriveTrainSubsystem in, double drivePower_, double dtime) {
    this.m_DriveSubsystem = in;
    drivePower = drivePower_;
    time = new Timer();
    driveTime = dtime;
    addRequirements(m_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.reset();
    time.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(time.get() < driveTime){
        m_DriveSubsystem.manualDrive(drivePower, 0);
        System.err.println("auto execute");
    }
    else{
        m_DriveSubsystem.manualDrive(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    // m_DriveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return time.get() > driveTime; // End the command when we are within the specified threshold of being 'flat' (gyroscope pitch of 0 degrees)
  }
}