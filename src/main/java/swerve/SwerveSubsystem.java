// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package swerve;

import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
    public SwerveDrivetrainModel dt;
    FeedForwardCharacterizationData characterizationData = new FeedForwardCharacterizationData();
    double timeCharacterizing;

    public SwerveSubsystem(SwerveDrivetrainModel dt) {
        this.dt = dt;
        Timer.delay(1.0);
        for (SwerveModule mod : dt.getSwerveModules()) {
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic() {
        dt.updateOdometry();
    }

    /**
     * 
     * @param quas_voltage  the quasiastic voltage per second
     * @param quas_duration the quasiastic test duration
     * @return the command to characterize the swerve drivetrain
     */
    public Command characterizeDrive(double quas_voltage, double quas_duration) {
        return runOnce(this::characterizeDriveInit).andThen(runEnd(() -> {
            timeCharacterizing += 0.020;
            characterizeDrive(timeCharacterizing * quas_voltage);
        }, () -> {
            dt.setVoltageToZero();
            printData();
        })).until(() -> timeCharacterizing >= quas_duration);
    }

    private void characterizeDriveInit() {
        dt.setVoltageToZero();
        characterizationData = new FeedForwardCharacterizationData();
        timeCharacterizing = 0;
    }

    private void characterizeDrive(double voltage) {
        for (int i = 0; i < SwerveDrivetrainModel.NUM_MODULES; i++) {
            dt.getSwerveModules()[i].setDriveVoltageForCharacterization(voltage);
        }
        var fl = dt.getSwerveModules()[0];
        var data = fl.getDriveVoltageAndRate();
        characterizationData.add(data.getSecond(), data.getFirst());
    }

    private void printData() {
        characterizationData.print();
    }

}

class FeedForwardCharacterizationData {
    private final List<Double> velocityData = new LinkedList<>();
    private final List<Double> voltageData = new LinkedList<>();

    public void add(double velocity, double voltage) {
        if (Math.abs(velocity) > 1E-4) {
            velocityData.add(Math.abs(velocity));
            voltageData.add(Math.abs(voltage));
        }
    }

    public void print() {
        double velocityDataArray[] = velocityData.stream().mapToDouble(Double::doubleValue).toArray();
        double voltageDataArray[] = voltageData.stream().mapToDouble(Double::doubleValue).toArray();
        double accelerationDataArray[] = new double[velocityDataArray.length];
        for (int i = 0; i < velocityDataArray.length - 1; i++) {
            accelerationDataArray[i] = (velocityDataArray[i + 1] - velocityDataArray[i]) / 0.020;
        }
        accelerationDataArray[accelerationDataArray.length - 1] = accelerationDataArray[accelerationDataArray.length
                - 2];
        PolynomialRegression regression = new PolynomialRegression(velocityDataArray,
                voltageDataArray, 1);
        double residualsVoltageVelocityWise[] = new double[velocityDataArray.length];
        for (int i = 0; i < velocityDataArray.length; i++) {
            residualsVoltageVelocityWise[i] = voltageDataArray[i] - regression.predict(velocityDataArray[i]);
        }
        PolynomialRegression accelerationRegression = new PolynomialRegression(accelerationDataArray,
                residualsVoltageVelocityWise,
                1);
        System.out.println("FF Characterization Results:");
        System.out.println("\tCount=" + Integer.toString(velocityData.size())
                + "");
        System.out.println(String.format("\tR2=%.5f", regression.R2(velocityDataArray, voltageDataArray)));
        System.out.println(String.format("\tkS=%.5f", regression.beta(0)));
        System.out.println(String.format("\tkV=%.5f", regression.beta(1)));
        System.out.println(String.format("\tkA=%.5f", accelerationRegression.beta(1)));
    }
}

class PolynomialRegression {
    private SimpleMatrix coefficients;

    public PolynomialRegression(double[] x, double[] y, int degree) {
        // Add a column of ones to the x matrix for the y-intercept
        SimpleMatrix xMatrix = new SimpleMatrix(x.length, degree + 1);
        for (int i = 0; i < x.length; i++) {
            xMatrix.set(i, 0, 1);
            for (int j = 1; j <= degree; j++) {
                xMatrix.set(i, j, Math.pow(x[i], j));
            }
        }
        // Use the Moore-Penrose pseudoinverse to find the coefficients
        SimpleMatrix yMatrix = new SimpleMatrix(y.length, 1, true, y);
        SimpleMatrix transpose = xMatrix.transpose();
        SimpleMatrix xTx = transpose.mult(xMatrix);
        SimpleMatrix xTxInv = xTx.invert();
        SimpleMatrix xTy = transpose.mult(yMatrix);
        coefficients = xTxInv.mult(xTy);
    }

    public double predict(double x) {
        double y = 0;
        for (int i = 0; i < coefficients.numRows(); i++) {
            y += coefficients.get(i) * Math.pow(x, i);
        }
        return y;
    }

    public double R2(double[] x, double[] y) {
        double sst = 0, sse = 0, yMean = 0;
        for (int i = 0; i < y.length; i++) {
            yMean += y[i];
        }
        yMean /= y.length;
        for (int i = 0; i < y.length; i++) {
            double yPred = predict(x[i]);
            sst += (y[i] - yMean) * (y[i] - yMean);
            sse += (y[i] - yPred) * (y[i] - yPred);
        }
        return 1 - sse / sst;
    }

    public double beta(int degree) {
        return coefficients.get(degree);
    }
}