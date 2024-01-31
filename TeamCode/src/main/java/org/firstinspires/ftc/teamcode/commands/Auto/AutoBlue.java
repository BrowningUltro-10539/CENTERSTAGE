package org.firstinspires.ftc.teamcode.commands.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous
public class AutoBlue extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    static final double FORWARD_SPEED = 0.6;
    static final double BACKWARD_SPEED = -0.6;

    @Override
    public void runOpMode() {
        DcMotor LT = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor LB = hardwareMap.get(DcMotor.class, "leftRear");
        DcMotor RT = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor RB = hardwareMap.get(DcMotor.class, "rightRear");

        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        waitForStart();

        runtime.reset();
        LB.setPower(BACKWARD_SPEED);
        RB.setPower(BACKWARD_SPEED);
        LT.setPower(FORWARD_SPEED);
        RT.setPower(FORWARD_SPEED);

        //change time later
        while (opModeIsActive() && (runtime.seconds() < .5)) {
            telemetry.addData("Path", "Leg 1: %2f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        LB.setPower(0);
        RB.setPower(0);
        LT.setPower(0);
        RT.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);

    }


}
