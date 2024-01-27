package com.swervedrivespecialties.swervelib.ctre;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
//import com.ctre.phoenix.sensors.AbsoluteSensorRange;
//import com.ctre.phoenix.sensors.CANCoder;
//import com.ctre.phoenix.sensors.CANCoderConfiguration;
//import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.swervedrivespecialties.swervelib.AbsoluteEncoder;
import com.swervedrivespecialties.swervelib.AbsoluteEncoderFactory;

public class CanCoderFactoryBuilder {
    private SensorDirectionValue direction = SensorDirectionValue.CounterClockwise_Positive;
    private int periodMilliseconds = 10;

    public CanCoderFactoryBuilder withReadingUpdatePeriod(int periodMilliseconds) {
        this.periodMilliseconds = periodMilliseconds;
        return this;
    }

    public CanCoderFactoryBuilder withDirection(SensorDirectionValue direction) {
        this.direction = direction;
        return this;
    }

    public AbsoluteEncoderFactory<CanCoderAbsoluteConfiguration> build() {
        return configuration -> {
            CANcoderConfiguration config = new CANcoderConfiguration();
            config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
            config.MagnetSensor.MagnetOffset = configuration.getOffset();//360;
            this.direction = SensorDirectionValue.Clockwise_Positive;
            config.MagnetSensor.SensorDirection = this.direction;
            CANcoder encoder = new CANcoder(configuration.getId());
            encoder.getConfigurator().apply(config,250);
            encoder.getAbsolutePosition().setUpdateFrequency(periodMilliseconds, 500);
            encoder.getVelocity().setUpdateFrequency(periodMilliseconds, 500);
            //CtreUtils.checkCtreError(encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, periodMilliseconds, 250), "Failed to configure CANCoder Posistion update rate");
            return new EncoderImplementation(encoder);
        };
    }

    private static class EncoderImplementation implements AbsoluteEncoder {
        private final CANcoder encoder;

        private EncoderImplementation(CANcoder encoder) {
            this.encoder = encoder;
        }

        @Override
        public double getAbsoluteAngle() {
            double angle = Math.toRadians(encoder.getAbsolutePosition().getValueAsDouble());
            angle %= 2.0 * Math.PI;
            if (angle < 0.0) {
                angle += 2.0 * Math.PI;
            }

            return angle;
        }
    }
}
