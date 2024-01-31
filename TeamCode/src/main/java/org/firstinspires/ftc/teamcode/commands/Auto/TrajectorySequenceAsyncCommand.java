package org.firstinspires.ftc.teamcode.commands.Auto;


import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

public class TrajectorySequenceAsyncCommand extends CommandBase {
    private SampleMecanumDrive drive;
    private TrajectorySequence sequence;

    public TrajectorySequenceAsyncCommand(SampleMecanumDrive drive, TrajectorySequence sequence){
        this.drive = drive;
        this.sequence = sequence;
    }

    @Override
    public void initialize(){
        drive.followTrajectorySequenceAsync(sequence);
    }

    @Override
    public void execute(){
        drive.update();
    }

    @Override
    public boolean isFinished(){
        return !drive.isBusy();
    }



}
