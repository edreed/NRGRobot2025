/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

public class SparkEncoderAdapter implements RelativeEncoder {
  private final com.revrobotics.RelativeEncoder encoder;

  public SparkEncoderAdapter(com.revrobotics.RelativeEncoder encoder) {
    this.encoder = encoder;
  }

  @Override
  public double getPosition() {
    return encoder.getPosition();
  }

  @Override
  public double getVelocity() {
    return encoder.getVelocity();
  }
}