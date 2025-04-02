// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Actions;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Sensors;

public class LEDController extends SubsystemBase {
  LEDPattern m_gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kBlue, Color.kWhite);
  Distance kLedSpacing = Meters.of(1/120.0);
  LEDPattern m_scrollingGradient = m_gradient.scrollAtAbsoluteSpeed(MetersPerSecond.of(2.5), kLedSpacing);
  

  public LEDController() {
    Sensors.m_led.setLength(Sensors.m_ledBuffer.getLength());
    Sensors.m_led.start();
  }

  @Override
  public void periodic() {
    m_scrollingGradient.applyTo(Sensors.m_ledBuffer);
    Sensors.m_led.setData(Sensors.m_ledBuffer);
  }

  public void flash(){
    
  }
}
