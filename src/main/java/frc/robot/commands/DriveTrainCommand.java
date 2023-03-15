package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveTrainCommand extends CommandBase{
    private final DriveTrainSubsystem m_DriveTrainSubsystem;
    private final CommandPS4Controller m_controller;

    public DriveTrainCommand(DriveTrainSubsystem drive, CommandPS4Controller controller){
        m_DriveTrainSubsystem = drive;
        m_controller = controller;
        addRequirements(m_DriveTrainSubsystem);
    }
    @Override
    public void execute() {
        if(!DriverStation.isAutonomous()){
            double turn = -m_controller.getRawAxis(0);
            double move = -m_controller.getRawAxis(1);
    
            m_DriveTrainSubsystem.manualDrive(move, turn);
        }
        
        // double left = turn + move;
        // double right = turn - move;
        // m_DriveTrainSubsystem.setMotors(left, right);
    }
}
