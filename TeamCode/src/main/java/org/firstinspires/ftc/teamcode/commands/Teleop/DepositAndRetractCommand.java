package org.firstinspires.ftc.teamcode.commands.Teleop;


import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

public class DepositAndRetractCommand extends SequentialCommandGroup {
    public DepositAndRetractCommand(Robot robot){
        super(
                new InstantCommand(() -> robot.outtake.update(OuttakeSubsystem.ArmState.RELEASE)),
                new WaitCommand(500),
                new SequentialCommandGroup(
                        new InstantCommand(() -> robot.outtake.update(OuttakeSubsystem.ArmState.INTAKE)),
                        new WaitCommand(500),
                        new LiftPositionCommand(robot.lift, 0, 40, 50, 2)
                )
        );
    }
}
