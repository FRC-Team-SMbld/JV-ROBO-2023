package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.TriangleSubsystem;

public class TriangleTurnTo extends CommandBase{
    private final TriangleSubsystem m_TriangleSubsystem;
    private CommandPS4Controller m_controller;

    public TriangleTurnTo(TriangleSubsystem triangle, CommandPS4Controller controller){
        m_TriangleSubsystem = triangle;
        addRequirements(m_TriangleSubsystem);
    }

    @Override
    public void execute() 
    {
        if(Math.abs(m_controller.getRawAxis(4)) > 0.2){
            m_TriangleSubsystem.setAngle();
        }
    }
}
