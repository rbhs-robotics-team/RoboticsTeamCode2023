package org.firstinspires.ftc.teamcode.Autonomous.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Autonomous.old.AutoOp2023;
import org.firstinspires.ftc.teamcode.Autonomous.old.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "20pt: Signal Sleeve", group="Autonomous")
public class SSPark extends AutoOp2023 {

    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;

    // Name of the Webcam to be set in the config
    private String webcamName = "Webcam 1";

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(hardwareMap, telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        while (!isStarted()) {

            telemetry_.addData("ROTATION: ", sleeveDetection.getPosition());
            telemetry_.update();
        }
        
        waitForStart();
        SleeveDetection.ParkingPosition position = sleeveDetection.getPosition();
        telemetry_.addData("Position", position);
        telemetry_.update();
        forward(1.2, 0.2);
        if(position.equals(SleeveDetection.ParkingPosition.LEFT)) {
            strafeLeft(1.3, 0.2);
        } else if(position.equals(SleeveDetection.ParkingPosition.RIGHT)) {
            strafeRight(1.3, 0.2);
        }
    }
}
