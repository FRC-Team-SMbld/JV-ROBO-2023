package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.TriangleSubsystem;

public class TriangleCommandTurn extends CommandBase{
    private final TriangleSubsystem m_TriangleSubsystem;
    private final CommandPS4Controller m_controller;
    private SlewRateLimiter filter;
    private boolean crossflag;
    private Timer time;
    private double desired;

    public TriangleCommandTurn(TriangleSubsystem triangle, CommandPS4Controller controller, double want){
        m_TriangleSubsystem = triangle;
        m_controller = controller;
        desired = want;
        filter = new SlewRateLimiter(5);
        addRequirements(m_TriangleSubsystem);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Encoder position arm:", m_TriangleSubsystem.getEncoderPos());
        //SmartDashboard.putNumber("Velocity", m_TriangleSubsystem.getVelocity());

        double angle = -m_controller.getRawAxis(5);
        angle *= 0.8;
        m_TriangleSubsystem.set(filter.calculate(angle));
        // if(!crossflag && m_controller.circle().getAsBoolean()){
        //     time.reset();
        //     time.start();
        //     crossflag = true;
        // }
        // if(crossflag){
        //     while(time.get() < desired){
        //         m_TriangleSubsystem.set(0.5);
        //     }
        //     m_TriangleSubsystem.set(0);
        //     crossflag = false;
        // }
    }

    
}
