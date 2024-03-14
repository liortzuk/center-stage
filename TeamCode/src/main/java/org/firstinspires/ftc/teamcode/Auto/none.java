package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.vision.VisionPortal.easyCreateWithDefaults;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AI.Arava;
import org.firstinspires.ftc.teamcode.AI.examplePipeline;
import org.firstinspires.ftc.teamcode.AI.golda;
import org.firstinspires.ftc.teamcode.OpMode;



import org.firstinspires.ftc.teamcode.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.Elevator.Elevator;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class none extends OpMode{
    Elevator elevator = new Elevator(armL, armR, intake, ANGLE, LeftServo, RightServo, trigger, angle);
    DriveTrain driveTrain = new DriveTrain(DriveBackRight, DriveBackLeft, DriveFrontRight, DriveFrontLeft, telemetry, Imu);

    @Override
    protected void run() {
        driveTrain.driveTo(20);
        elevator.AngleLift(-790, 1);
    }

    @Override
    protected void end() {

    }
}
