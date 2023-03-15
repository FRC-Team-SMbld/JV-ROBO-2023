package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

// This command self=balances on the charging station using gyroscope pitch as feedback
public class ArmGoToLow extends CommandBase {

  private ArmSubsystem m_arm;

  private Timer time;
  private double goTime;

  /** Command to use Gyro data to resist the tip angle from the beam - to stabalize and balanace */
  public ArmGoToLow(ArmSubsystem arm_, double g_time) {
    m_arm = arm_;
    goTime = g_time;
    addRequirements(m_arm);
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
    if(time.get() < goTime){
        m_arm.set(0.1);
    }
    else{
        m_arm.set(0);
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
    return time.get() > goTime; // End the command when we are within the specified threshold of being 'flat' (gyroscope pitch of 0 degrees)
  }
}