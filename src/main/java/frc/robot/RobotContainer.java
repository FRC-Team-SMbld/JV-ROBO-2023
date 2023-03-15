// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmCommandExtend;
import frc.robot.commands.ArmGoToLow;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.ClawCommandExtend;
import frc.robot.commands.ClawCommandRetract;
import frc.robot.commands.DriveTrainCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TriangleCommandTurn;
import frc.robot.commands.TriangleTurnTo;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.TriangleSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller m_driverController =
      new CommandPS4Controller(OperatorConstants.kDriverControllerPort);
  private final DriveTrainSubsystem m_driveTrain = new DriveTrainSubsystem();
 private final ClawSubsystem m_claw = new ClawSubsystem();
  private final TriangleSubsystem m_triangle = new TriangleSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final SendableChooser<Command> auto = new SendableChooser<>();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_driveTrain.setDefaultCommand(new DriveTrainCommand(m_driveTrain, m_driverController));
    
    m_triangle.setDefaultCommand(new TriangleCommandTurn(m_triangle, m_driverController, 0.5));
    m_arm.setDefaultCommand(new ArmCommandExtend(m_arm, m_driverController));
    // m_arm.setDefaultCommand(new ArmGoToLow(m_arm, 1, m_driverController));
    m_claw.setDefaultCommand(new ClawCommandRetract(m_claw));
    // m_arm.setDefaultCommand(new Triang);

    auto.addOption("Taxi", new AutoDrive(m_driveTrain, 0.63, 2.5));

    auto.addOption("test auto", new AutoBalance(m_driveTrain));
    
    SmartDashboard.putData("AUTOCHOICES",auto);
    // Configure the trigger bindings
    configureBindings();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  private void configureBindings() {
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));
    m_driverController.R1()
        .onTrue(new ClawCommandExtend(m_claw));
    m_driverController.L1()
        .onTrue(new ClawCommandRetract(m_claw));
    // m_driverController.L2()
    //     .onTrue(new ArmCommandExtend(m_arm, m_driverController));
     
//         .onTrue(new TriangleTurnTo(m_triangle, m_driverController));

      //  m_driverController.square()
        //.onTrue(new TriangleCommandTurn(m_triangle, m_driverController));
  }

  /**-=-pl,
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return (Command)auto.getSelected();
  }
}
