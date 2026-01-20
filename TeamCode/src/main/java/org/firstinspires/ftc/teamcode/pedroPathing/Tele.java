package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.RobotConstants.*;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

@Configurable
@TeleOp(name = "Tele", group = "Linear OpMode")
public class Tele extends RobotBase {
    public static double targetVelocity = 3650, test2 = 3800, angle = 30;
    int temp = 1;
    boolean last = false;

//    public static double velocityU,

    @Override
    public void robotInit() {

    }

    @Override
    protected void robotInitLoop() {
    }

    @Override
    public void robotStart() {
        follower.startTeleopDrive();
    }

    @Override
    public void robotLoop() {
        //teleop
        shooter.setVelocity(targetVelocity, true);

        if (gamepad1.right_trigger > 0.3) {
            shooter.shooting(2, true);
            colorSpinner.spin.setPower(1);
            intake.off();
            shooter.elevatorUp();
        } else {
            shooter.elevatorOff();
            shooter.shooting(2, false);
            if (gamepad1.left_bumper) {
                intake.out();
                colorSpinner.spin.setPower(-1);
            } else if (gamepad1.left_trigger > 0.3) {
                intake.on();
                colorSpinner.spin.setPower(0.18);
            } else {
                intake.off();
                colorSpinner.spin.setPower(0.18);
            }
        }
        shooter.visionTracking(2);

        if (gamepad1.y) angle += 0.5;
        else if (gamepad1.x) angle -= 0.5;
        angle = clamp(angle, 24, 49);
        shooter.pitchDegree(angle);

//        if(gamepad1.y) angle+=0.05;
//        else if(gamepad1.a) angle-=0.05;
//        angle = clamp(angle,0,1);
//        shooter.arm.setPosition(angle);
//        colorSpinner.detect3posePRO();
//        test3 = clamp(test3, 24, 49);
//        shooter.pitchDegree(test3);


//        shooter.pitchDegree(test1);
//        shooter.setVelocity(test2, true);
//        intake.on();
//        colorSpinner.on(1);
//        shooter.setVelocity(test2, true);
//        shooter.visionTracking((int)test1);
        // 底盤遙控
        double axial = -gamepad1.left_stick_y;
        double lateral = -gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;
        follower.update();
        follower.setTeleOpDrive(axial, lateral, -yaw * 0.8, true);

        // 顯示數據
        telemetryM.addData("target velocity", targetVelocity);
        telemetryM.addData("up velocity", shooter.shooterU.getVelocity() / 28.0 * 60.0);
        telemetryM.addData("down velocity", shooter.shooterD.getVelocity() / 28.0 * 60.0);
        telemetryM.update();

        telemetry.addData("angle", angle);
//        telemetry.addData("3 blue", colorSpinner.color3.blue());
//        telemetry.addData("3 green", colorSpinner.color3.green());
//        telemetry.addData("4 blue", colorSpinner.color4.blue());
//        telemetry.addData("4 green", colorSpinner.color4.green());
//        telemetry.addData("5 blue", colorSpinner.color5.blue());
//        telemetry.addData("5 green", colorSpinner.color5.green());
//        telemetry.addData("6 blue", colorSpinner.color6.blue());
//        telemetry.addData("6 green", colorSpinner.color6.green());
//        telemetry.addData("tagNumber", shooter.tagNumber());
//        telemetry.addData("getLatestResult", shooter.limelight.getLatestResult().getTx());
        telemetry.addData("pid up calculate", shooter.ShooterUPID.calculate(shooter.shooterU.getVelocity() / 2800, targetVelocity / 4600));
        telemetry.addData("pid down calculate", shooter.ShooterDPID.calculate(shooter.shooterD.getVelocity() / 2800, targetVelocity / 4300));
        telemetry.addData("shooterU.getPower", shooter.shooterU.getPower());
        telemetry.addData("shooterD.getPower", shooter.shooterD.getPower());
//        telemetry.addData("1", colorSpinner.place1);
//        telemetry.addData("2", colorSpinner.place2);
//        telemetry.addData("3", colorSpinner.place3);
//        telemetry.addData("shooter.getPose()", shooter.getPose());
        telemetry.addData("Up Velocity ", shooter.uVelocity);
        telemetry.addData("Down Velocity ", shooter.dVelocity);
        telemetry.addData("Up Velocity rpm", shooter.shooterU.getVelocity() / 28.0 * 60.0);
        telemetry.addData("Down Velocity rpm", shooter.shooterD.getVelocity() / 28.0 * 60.0);
//        telemetry.addData("shooterU.getVelocity()", shooter.shooterU.getVelocity());
//        telemetry.addData("shooterD.getVelocity()", shooter.shooterD.getVelocity());
//        telemetry.addData("delta", colorSpinner.delta);
//        telemetry.addData("test1", test1);
//        telemetry.addData("color spin pose", colorSpinner.getPose());
//        telemetry.addData("getCurrentPose", colorSpinner.getDegree());
//        telemetry.addData("sh pose", shooter.getPose());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getHeading()));
        telemetry.update();
    }

    public void robotStop() {
    }
}




