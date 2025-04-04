package frc.robot.commands.automatics;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SubsystemSwerveDrivetrain;
import frc.robot.subsystems.swerve.control.DriveWithSpeeds;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TaxiCommand extends Command {
  Timer timer = new Timer();
  SubsystemSwerveDrivetrain drivetrain;
  double time;

  /** Creates a new CommandSwerveTaxi. */
  public TaxiCommand(SubsystemSwerveDrivetrain drivetrain, double time) {
    this.drivetrain = drivetrain;
    this.time = time;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.setControlRequest(
        DriveWithSpeeds.newRequest(0.7, 0, 0).fieldRelative(false).limitSpeed(false).limitAccelaration(false));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControlRequest(DriveWithSpeeds.newStopDrivetrainRequest().limitAccelaration(false));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(time);
  }
}