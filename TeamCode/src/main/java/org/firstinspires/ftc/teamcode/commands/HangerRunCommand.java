package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.HangerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.concurrent.TimeUnit;

public class HangerRunCommand extends CommandBase {
    private final HangerSubsystem hanger;
    private double power;
    private double time;
    private ElapsedTime timer;

    public HangerRunCommand(HangerSubsystem hanger, double power, double time){
        this.hanger = hanger;
        this.power = power;
        this.time = time;
    }

    @Override
    public void execute(){
        if (timer == null) {
            timer = new ElapsedTime();
            hanger.setPower(this.power);
        }

    }

    @Override
    public boolean isFinished() {
        boolean value = false;

        if (timer.time(TimeUnit.SECONDS) >= this.time) {
            hanger.setPower(0);
            value = true;
        }

        return value;
    }
}


