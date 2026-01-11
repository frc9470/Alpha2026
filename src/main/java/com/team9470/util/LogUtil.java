package com.team9470.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.ejml.data.Complex_F64;
import org.ejml.simple.SimpleEVD;
import org.ejml.simple.SimpleMatrix;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import java.util.stream.DoubleStream;










// MAKE THIS PROPER FOR 2025 SEASON









public class LogUtil {
    private static Map<Supplier<Boolean>, String> periodicLogs = new HashMap<>();
    private static int count = 0;

    /**
     * should be called periodically (every 20 ms)
     */
    public static void periodic(){
        for(Map.Entry<Supplier<Boolean>, String> entry : periodicLogs.entrySet()){
            if(entry.getKey().get() && count % 100 == 0){
                System.err.println("[WARNING] " + entry.getValue());
            }
        }
        count += 1;
    }

    public static void recordTranslation2d(String key, Translation2d translation) {
        SmartDashboard.putNumberArray(key, new double[] {translation.getX(), translation.getY()});
    }

    public static void recordRotation2d(String key, Rotation2d rotation) {
        SmartDashboard.putNumber(key, rotation.getDegrees());
    }

    public static void recordPose2d(String key, Pose2d... poses) {
        final double[] doubleArray = Arrays.stream(poses)
                .flatMapToDouble(pose -> DoubleStream.of(
                        pose.getTranslation().getX(),
                        pose.getTranslation().getY(),
                        pose.getRotation().getRadians()))
                .toArray();
        SmartDashboard.putNumberArray(key, doubleArray);
    }

    public static void recordPose3d(String key, Pose3d... poses) {
        final double[] doubleArray = Arrays.stream(poses)
                .flatMapToDouble(pose -> DoubleStream.of(
                        pose.getTranslation().getX(),
                        pose.getTranslation().getY(),
                        pose.getTranslation().getZ(),
                        pose.getRotation().getQuaternion().getW(),
                        pose.getRotation().getQuaternion().getX(),
                        pose.getRotation().getQuaternion().getY(),
                        pose.getRotation().getQuaternion().getZ()))
                .toArray();
        SmartDashboard.putNumberArray(key, doubleArray);
    }


    /**
     *
     * @param key Logged poses will be called 'key + " confidence interval a1...b2"'
     * @param confidence Probablity robot is within the ellipse
     * @param covMatrix Covariance matrix of measurement
     * @param x x-value of measurement
     * @param y y-value of measurement
     *
     */
    public static void logConfidenceEllipse(
            String key, double confidence, Matrix<N2, N2> covMatrix, double x, double y) {

        SimpleEVD<SimpleMatrix> deconstructor =
                new SimpleEVD<SimpleMatrix>(covMatrix.getStorage().getMatrix());
        List<Complex_F64> eigenvalues = deconstructor.getEigenvalues();
        if (eigenvalues.get(0).imaginary != 0 | eigenvalues.get(1).imaginary != 0) {
            System.out.println("Not real");
        } else {
            double a;
            double b;
            double scaleFactor = Math.sqrt(-2 * Math.log(1 - confidence));
            // SimpleMatrix eigVec;

            if (eigenvalues.get(0).real > eigenvalues.get(1).real) {
                a = scaleFactor * Math.sqrt(eigenvalues.get(0).real);
                b = scaleFactor * Math.sqrt(eigenvalues.get(1).real);

                // eigVec = deconstructor.getEigenVector(0);
            } else {
                b = scaleFactor * Math.sqrt(eigenvalues.get(0).real);
                a = scaleFactor * Math.sqrt(eigenvalues.get(1).real);

                // eigVec = deconstructor.getEigenVector(1);
            }
            // double theta = Math.atan(eigVec.get(1)/eigVec.get(0)) % (2*Math.PI);
            // (theta = PI/2)
            Pose2d a1 = new Pose2d(new Translation2d(x, y + a), Rotation2d.fromDegrees(90));
            Pose2d a2 = new Pose2d(new Translation2d(x, y - a), Rotation2d.fromDegrees(270));
            Pose2d b1 = new Pose2d(new Translation2d(x - b, y), Rotation2d.fromDegrees(180));
            Pose2d b2 = new Pose2d(new Translation2d(x + b, y), Rotation2d.fromDegrees(0));

            recordPose2d(key + " confidence interval a1", a1);
            recordPose2d(key + " confidence interval a2", a2);
            recordPose2d(key + " confidence interval b1", b1);
            recordPose2d(key + " confidence interval b2", b2);
        }
    }

    public static void periodicWarning(Supplier<Boolean> trigger, String message) {
        periodicLogs.put(trigger, message);
    }

    public static void recordTransform3d(String s, Transform3d transform) {
        recordPose3d(s, new Pose3d(transform.getTranslation(), transform.getRotation()));
    }
}