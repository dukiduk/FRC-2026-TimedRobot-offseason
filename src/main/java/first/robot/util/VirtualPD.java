package first.robot.util;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Joules;
import static org.wpilib.units.Units.Seconds;

import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Energy;
import org.wpilib.units.measure.Power;
import org.wpilib.units.measure.Voltage;
import org.wpilib.system.RobotController;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class VirtualPD {
    private static ArrayList<Supplier<Current>> motors = new ArrayList<>();
    private static ArrayList<String> groups = new ArrayList<>();
    private static HashMap<String, Energy> groupEnergyTotals = new HashMap<>();
    private static ArrayList<Energy> motorEnergyTotals = new ArrayList<>();
    private static Energy totalEnergy = Joules.zero();

    private VirtualPD() {}

    public static void registerMotor(Supplier<Current> currentSupplier, String group) {
        motors.add(currentSupplier);
        groups.add(group);
        motorEnergyTotals.add(Joules.zero());
        if (!groupEnergyTotals.containsKey(group)) {
            groupEnergyTotals.put(group, Joules.zero());
        }
    }

    public static void logTotalCurrent() {
        Current total = Amps.zero();
        HashMap<String, Current> groupCurrentTotals = new HashMap<>();
        Voltage batteryVoltage = RobotController.getMeasureBatteryVoltage();

        for (int i = 0; i < motors.size(); i++) {
            Current current = motors.get(i).get();
            Power power = current.times(batteryVoltage);
            Energy energy = power.times(Seconds.of(0.02));
            motorEnergyTotals.set(i, motorEnergyTotals.get(i).plus(energy));
            total = total.plus(current);
            totalEnergy = totalEnergy.plus(energy);
            String group = groups.get(i);
            if (groupCurrentTotals.containsKey(group)) {
                groupCurrentTotals.put(group, groupCurrentTotals.get(group).plus(current));
            } else {
                groupCurrentTotals.put(group, current);
            }
            groupEnergyTotals.put(group, groupEnergyTotals.get(group).plus(energy));
        }

        Logger.recordOutput("VirtualPD/Total/Current", total);
        Logger.recordOutput("VirtualPD/Total/Energy", totalEnergy);
        
        for (String group : groupCurrentTotals.keySet()) {
            Logger.recordOutput("VirtualPD/Current/" + group, groupCurrentTotals.get(group));
        }
        for (String group : groupEnergyTotals.keySet()) {
            Logger.recordOutput("VirtualPD/Energy/" + group, groupEnergyTotals.get(group));
        }
    }

    public static Current getCurrent(String group) {
        Current groupCurrent = Amps.zero();
        for (int i = 0; i < motors.size(); i++) {
            if (groups.get(i).equals(group)) {
                groupCurrent = groupCurrent.plus(motors.get(i).get());
            }
        }
        return groupCurrent;
    }

    public static double getCurrentAmps(String group) {
        return getCurrent(group).in(Amps);
    }
}