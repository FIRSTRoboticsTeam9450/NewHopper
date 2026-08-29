package frc.robot.util;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.HashMap;
import java.util.Map;

public class PIDVals {

    public enum PIDVars {
        kS,
        kV,
        kA,
        kP,
        kI,
        kD,
        kG
    }

    Map<PIDVars, LoggedNetworkNumber> vals;

    public PIDVals(String path, double... vals) {

        assert vals.length == 7;

        this.vals = new HashMap<>();

        PIDVars[] vars = PIDVars.values();

       for(int i = 0; i < vals.length && i < vars.length; i++) {

           this.vals.put(vars[i], new LoggedNetworkNumber(path + vars[i].toString(), vals[i]));

       }

    }

    public double get(PIDVars var) {

        return vals.get(var).get();

    }

    public void setMotorPID0(TalonFXConfiguration motorConfig) {

        Slot0Configs slot = motorConfig.Slot0;

        slot.kS = vals.get(PIDVars.kS).get();
        slot.kV = vals.get(PIDVars.kV).get();
        slot.kA = vals.get(PIDVars.kA).get();
        slot.kP = vals.get(PIDVars.kP).get();
        slot.kI = vals.get(PIDVars.kI).get();
        slot.kD = vals.get(PIDVars.kD).get();
        slot.kG = vals.get(PIDVars.kG).get();

    }

    public void setMotorPID1(TalonFXConfiguration motorConfig) {

        Slot1Configs slot = motorConfig.Slot1;

        slot.kS = vals.get(PIDVars.kS).get();
        slot.kV = vals.get(PIDVars.kV).get();
        slot.kA = vals.get(PIDVars.kA).get();
        slot.kP = vals.get(PIDVars.kP).get();
        slot.kI = vals.get(PIDVars.kI).get();
        slot.kD = vals.get(PIDVars.kD).get();
        slot.kG = vals.get(PIDVars.kG).get();

    }

    public boolean updateMotorPID(TalonFXConfiguration motorConfig) {

        Slot0Configs slot0 = motorConfig.Slot0;

        boolean changed = false;

        if(slot0.kS != vals.get(PIDVars.kS).get()) {
            slot0.kS = vals.get(PIDVars.kS).get();
            changed = true;
        }
        if(slot0.kV != vals.get(PIDVars.kV).get()) {
            slot0.kV = vals.get(PIDVars.kV).get();
            changed = true;
        }
        if(slot0.kA != vals.get(PIDVars.kA).get()) {
            slot0.kA = vals.get(PIDVars.kA).get();
            changed = true;
        }
        if(slot0.kP != vals.get(PIDVars.kP).get()) {
            slot0.kP = vals.get(PIDVars.kP).get();
            changed = true;
        }
        if(slot0.kI != vals.get(PIDVars.kI).get()) {
            slot0.kI = vals.get(PIDVars.kI).get();
            changed = true;
        }
        if(slot0.kD != vals.get(PIDVars.kD).get()) {
            slot0.kD = vals.get(PIDVars.kD).get();
            changed = true;
        }
        if(slot0.kG != vals.get(PIDVars.kG).get()) {
            slot0.kG = vals.get(PIDVars.kG).get();
            changed = true;
        }

        return changed;

    }



}
