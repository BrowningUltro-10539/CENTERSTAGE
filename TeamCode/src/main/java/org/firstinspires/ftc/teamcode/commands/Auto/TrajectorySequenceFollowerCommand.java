package org.firstinspires.ftc.teamcode.commands.Auto;



import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class TrajectorySequenceFollowerCommand extends CommandBase {

    private final MecanumDriveSubsystem drive;
    private final TrajectorySequence trajectory;

    public TrajectorySequenceFollowerCommand(MecanumDriveSubsystem drive, TrajectorySequence trajectory) {
        this.drive = drive;
        this.trajectory = trajectory;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.followTrajectorySequence(trajectory);
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            drive.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !drive.isBusy();
    }

}