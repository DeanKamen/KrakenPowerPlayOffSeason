/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * This 2022-2023 subsystem illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 *
 * IMPORTANT: In order to use this subsystem, you need to obtain your own Vuforia license key as
 * is explained below.
 */

public class WebcamObjectDetection {

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_ASSET = "coneModel1.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";


    private static final String[] LABELS = {
            "one",
            "two",
            "three"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */

    //This is Currently Hunter Messners Key
    private static final String VUFORIA_KEY =
            //"Ab2KZyn/////AAABmeuSPzBMbUdQpilf1SvrgHs5NsXkqaqiLJjGAaaP1bbe/4WVCKrUv0Fo3DFkiR7yWAD2dfvIXXOI8GVn0mC2SMudg+aSqD7VvNPzFmNuryyQsZ5N/p9WKddwzOugUUWkmDugGPX19+JDjKFcdU6wOSSjrmglIQz5/yZkuiXMK0kiDCAsVAJkPDoz83VKbXtuK9ItbT0Q80HAwN7GP8oB8nJ1gBPqKZYwvIQse9DLPLIsoSx0RvVGW8i+RgbgM/wg+AnvGJeOxNDyuoPvMDzNURTVeoe9qr+JIZlJMVEF9/pWE4zU4dH4PZ6EhVu+/WvpRdchWvxwLTB9omxNbxjDlKN7CFM9dSFYcQGcxxGT2f/K";
            //"Abg3PNz/////AAAAGWNcVXMODUM7pQASJjERHvgGsKSLk01jQ8GUfwIIhXvFq3v9f88OkuUy0PzNTC7QEZoQqTTzBW8ZP/cs8Y2l374MhQsGSfKBvBYjaO73FoHIZWhVoLIVw4AHC1PDOOFwku6pNk/GDx4n/a0hXruBenwO5d7PNFOPf8aC9Irao/OtA7Ot/6p8I8VtgmnWA/YHlgy2s9+JgGXps3MT3pvLCQe8kI/b/emCXGPA5khcD0Gmd4/Kp1KjVz/1hGQQMUngTeBxajvRaj5ACIKR7V8DsdGFZFykqkXlFNUluO583awvbtDPJ4OdAvlM+w8qM1+ILm++Tq27/QOuftKBKEBJTZcL3iWhRagxWd8sODDtgdYq";
            "AZj6OjT/////AAABmTYI9Vsm70l8lXIS0WH3h75BCwvek9Twey0T7xkTVV/MOg2rwRYxklWO81Al7LL92HF/qp93G2263TsCAx4IW6xfY4hA045obOJYVs7nsohKXxG/OlSzkwQAiK8SXUB5OpnicWtWWNNUAZBxYgUH4F5JIO25Gcp87xx+K38PovbSY/Vibf/N3IDRPYUxT/Bjl/6gmwMM5a8s/7e7H2PBoCNI2OtkGu7X81UuREd2JvKlrNhnMIifCHMhH1OZ+lvG8k3HDs/NbM3ry1YmHytkn9XRwoE2BffKgko0I6Rl2ilAaHOagtIULVOogTfyJZJHqUTvPQK314lT5+6JFhUL6rI2ZIBojSsag4Z2VvPxxiTp";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    public static WebcamObjectDetection init (HardwareMap hardwareMapin, Telemetry telemetryin) {
        WebcamObjectDetection cam = new WebcamObjectDetection();
        cam.telemetry = telemetryin;
        cam.hardwareMap = hardwareMapin;
        cam.initVuforia();
        cam.initTfod(cam.hardwareMap, cam.telemetry);
        // Return the initialized drive.
        return cam;
    }

    public String getRecognizedObject() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.


        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }

        telemetry.update();
        String recognizedObject = "";
        int oneCount = 0;
        int twoCount = 0;
        int threeCount = 0;
        int totalTries = 0;

        float highestConfidence = 0.0f;
        if ((tfod != null)&&(totalTries<10)) {

            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            while (updatedRecognitions != null) {
                telemetry.addData("# Objects Detected", updatedRecognitions.size());

                // step through the list of recognitions and display image position/size information for each one
                // Note: "Image number" refers to the randomized image orientation/number
                for (Recognition recognition : updatedRecognitions) {
                    //First, record highest confidence object
                    if (highestConfidence < recognition.getConfidence()) {
                        highestConfidence = recognition.getConfidence();
                        recognizedObject = recognition.getLabel();

                        switch (recognizedObject) {
                            case "one": oneCount++; break;
                            case "two": twoCount++; break;
                            case "three": threeCount++; break;
                        }
                    }
                    //Now print out all of the recognized images
                    double col = (recognition.getLeft() + recognition.getRight()) / 2;
                    double row = (recognition.getTop() + recognition.getBottom()) / 2;
                    double width = Math.abs(recognition.getRight() - recognition.getLeft());
                    double height = Math.abs(recognition.getTop() - recognition.getBottom());

                    telemetry.addData("", " ");
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                    telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                    telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
                }

                // Increment total tries so a robot moving across the field doesn't stall Proto.
                totalTries++;

                telemetry.update();

            }
        }

        // This conditional compares the counts of each recognition type and sets the return string
        // to the most recognized object.
        if ((oneCount>twoCount)&&(oneCount>threeCount)) {
            recognizedObject = "one";
        } else if ((twoCount>oneCount)&&(twoCount>threeCount)) {
            recognizedObject = "two";
        } else if ((threeCount>oneCount)&&(threeCount>twoCount)) {
            recognizedObject = "three";
        } else {
            recognizedObject = "dunno";
            // just move forward for a 33% chance of being correct?
        }

        return recognizedObject;
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcrap"); //webcam

        //parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; //phone cam
        Deadline deadline = new Deadline(10, TimeUnit.SECONDS);
        telemetry.addData("Vuforia camera permission:", "%s", parameters.cameraName.requestCameraPermission(deadline));
        //  Instantiate the Vuforia engine
        try{
            vuforia = ClassFactory.getInstance().createVuforia(parameters);
        }catch(Exception e){
            telemetry.addData("Vuforia camera Problem:", "%s", parameters.cameraName.toString());
            telemetry.addData("Vuforia camera is webcam:", "%s", parameters.cameraName.isWebcam());
            telemetry.addData("Vuforia Instantiation Problem:", "%s", e.toString());
            telemetry.update();
            while(true){
                if(false){
                    break;
                }
            };
        }

    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod(HardwareMap hardwareMap, Telemetry telemetry) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}
