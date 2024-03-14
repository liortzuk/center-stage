package org.firstinspires.ftc.teamcode;


import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;import static java.lang.Math.abs;
import org.firstinspires.ftc.teamcode.DriveTrain.DriveTrain;

import org.firstinspires.ftc.teamcode.Elevator.Elevator;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode{

    @Override
    protected void postInit() {
        Imu.resetYaw();
    }
    Elevator elevator = new Elevator(armL, armR, intake, ANGLE, LeftServo, RightServo, trigger, angle);
    @Override
    public void run(){

        Elevator elevator = new Elevator(armL, armR, intake, ANGLE, LeftServo, RightServo, trigger, angle);
        while (opModeIsActive()) {
            DriveTrain driveTrain = new DriveTrain(DriveBackRight, DriveBackLeft, DriveFrontRight, DriveFrontLeft, telemetry, Imu);
            double forward = -gamepad1.left_stick_y;//-1 to 1
            double turn = gamepad1.right_stick_x;
            double drift = gamepad1.left_stick_x;
            double botHeading = Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double botAngle = Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            if (hold){
                angle.setPosition(0.575);

            }

            if (gamepad1.a){
                driveTrain.noa(forward, drift, turn, botHeading);
            }else {
                driveTrain.Roni(forward, drift, turn, botHeading);
            }

            if (Math.abs(-gamepad2.left_stick_y) > 0.2){
                armR.setPower(-gamepad2.left_stick_y);
                armL.setPower(-gamepad2.left_stick_y);
            }else if(Math.abs(-gamepad2.left_stick_y) < 0.2 && state == 1){
                armR.setPower(0);
                armL.setPower(0);
            }

            if (gamepad1.options) {
                Imu.resetYaw();
            }

            if (gamepad1.x){
                hold = false;
                angle.setPosition(.75);
                sleep(1000);
                trigger.setPosition(0);
            }

            if (gamepad1.b){
                angle.setPosition(0.5);
            }

            ///take in
            if(gamepad2.b){
                intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                intake.setPower(1);

            }
            ///put out
            else if(gamepad2.a){
                intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                intake.setPower(-1);
            }
            else if (state == 1){
                intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                intake.setPower(0);
            }

            if(gamepad2.left_bumper != lastFrame3){
                Thread t = new Thread(() -> {elevator.AngleLift(0,1);;});
                t.start();

            }
            else if(gamepad2.right_bumper != lastFrame2){
                Thread t = new Thread(() -> {elevator.AngleLift(800,-1);;;});
                t.start();

            }

            if(gamepad2.left_trigger != 0 || gamepad2.right_trigger != 0) {
                RightServo.setPower(gamepad2.left_trigger);
                LeftServo.setPower(-gamepad2.right_trigger);
            }else {
                RightServo.setPower(0);
                LeftServo.setPower(0);
            }

            if (gamepad2.x){
                RightServo.setPower(-1);
            }
            if(gamepad2.y){
                LeftServo.setPower(1);
            }

            if(gamepad2.dpad_up != lastFrame){
                state = 0;
                elevator.ResetAngle();
                elevator.reset_all();
                //Thread t = new Thread(() -> {elevator.daria_thread();});
                Thread t = new Thread(() -> {

                    elevator.Elevator_function(750);
                    sleep(200);
                    elevator.AngleLift(657, -1);
                    sleep(100);
                    elevator.Elevator_function_down(675);
                    sleep(200);
                    telemetry.addData("position: ", armL.getCurrentPosition());
                    telemetry.update();
                    elevator.IntakePower(3000, 1);
                    elevator.servo_left_and_right(1,.12);
                    sleep(500);
                    elevator.Elevator_function(1300);
                    state = 1;
                });
                t.start();

            }

            if (gamepad2.dpad_down){
                elevator.check_State = true;
                elevator.teleop_lihi();
                elevator.check_State = false;
            }

            if(gamepad2.dpad_right){
                elevator.ResetAngle();
                elevator.AngleLift(800,-1);
            }


            telemetry.addData("Left Lift: ", armL.getCurrentPosition());
            telemetry.addData("Right Lift: ", armR.getCurrentPosition());
            telemetry.addData("ANGLE: ", ANGLE.getCurrentPosition());
            telemetry.addData("Gyro: ", botAngle);
            telemetry.addData("is finished:  ", elevator.is_finished);
            telemetry.addData("angle position: ", angle.getPosition());
            telemetry.update();


            lastFrame = gamepad2.dpad_up;
            lastFrame2 = gamepad2.right_bumper;
            lastFrame3 = gamepad2.left_bumper;

        }
    }
    @Override
    protected void end() {

    }

    public boolean lastFrame;
    public boolean lastFrame2;
    public boolean lastFrame3;
    public int state = 1;

    public boolean hold = true;

}
