 package frc.robot.commands;

 import edu.wpi.first.wpilibj.PS4Controller;
 import edu.wpi.first.wpilibj2.command.CommandBase;
 import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
 import edu.wpi.first.wpilibj2.command.button.JoystickButton;
 import edu.wpi.first.wpilibj2.command.button.Trigger;
 import frc.robot.subsystems.ClawSubsystem;
 import frc.robot.subsystems.DriveTrainSubsystem;

 public class ClawCommandExtend extends CommandBase{
     private final ClawSubsystem m_ClawSubsystem;

     public ClawCommandExtend(ClawSubsystem claw){
         m_ClawSubsystem = claw;
         addRequirements(m_ClawSubsystem);
     }
     @Override
     public void execute() {
         m_ClawSubsystem.extend();
     }
 }
