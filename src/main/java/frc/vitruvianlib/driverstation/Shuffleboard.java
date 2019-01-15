package frc.vitruvianlib.driverstation;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;

/**
 * This is a custom interface that allows us to place NetworkTable items into
 * separate folders so that we can organize our Shuffleboard dashboard. This is
 * basically a shameless copy/paste of the SmartDashboard functions, modified to
 * accept an additional argument to specify where to place each piece of data.
 */
public class Shuffleboard {
    private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("");

    /**
     * Returns the boolean the key maps to. If the key does not exist or is of
     * different type, it will return the default value.
     *
     * @param tabName      the Shuffleboard tab the key is under
     * @param key          the key to look up
     * @param defaultValue the value to be returned if no value is found
     * @return the value associated with the given key or the given default value
     * if there is no value associated with the key
     */
    public static boolean getBoolean(String tabName, String key, boolean defaultValue) {
        return table.getSubTable(tabName).getEntry(key).getBoolean(defaultValue);
    }

    /**
     * Returns the value at the specified key.
     *
     * @param key the key
     * @return the value
     * @throws IllegalArgumentException if the key is null
     */
    public static synchronized Sendable getData(String tabName, String key) {
        String tableKey = tabName + "-" + key;
        Data data = tablesToData.get(tableKey);
        if (data == null) {
            throw new IllegalArgumentException("Shuffleboard data does not exist: " + tableKey);
        } else {
            return data.m_sendable;
        }
    }

    /**
     * Returns the number the key maps to. If the key does not exist or is of
     * different type, it will return the default value.
     *
     * @param tabName      the Shuffleboard tab the key is under
     * @param key          the key to look up
     * @param defaultValue the value to be returned if no value is found
     * @return the value associated with the given key or the given default value
     * if there is no value associated with the key
     */
    public static double getNumber(String tabName, String key, double defaultValue) {
        return table.getSubTable(tabName).getEntry(key).getDouble(defaultValue);
    }

    /**
     * Returns the number array the key maps to. If the key does not exist or is
     * of different type, it will return the default value.
     *
     * @param tabName      the Shuffleboard tab the key is under
     * @param key          the key to look up
     * @param defaultValue the value to be returned if no value is found
     * @return the value associated with the given key or the given default value
     * if there is no value associated with the key
     */
    public static double[] getNumberArray(String tabName, String key, double[] defaultValue) {
        return table.getSubTable(tabName).getEntry(key).getDoubleArray(defaultValue);
    }

    /**
     * Returns the string the key maps to. If the key does not exist or is of
     * different type, it will return the default value.
     *
     * @param tabName      the Shuffleboard tab the key is under
     * @param key          the key to look up
     * @param defaultValue the value to be returned if no value is found
     * @return the value associated with the given key or the given default value
     * if there is no value associated with the key
     */
    public static String getString(String tabName, String key, String defaultValue) {
        return table.getSubTable(tabName).getEntry(key).getString(defaultValue);
    }

    /**
     * Put a boolean under a Shuffleboard tab.
     *
     * @param tabName the Shuffleboard tab the key is under
     * @param key     the key to be assigned to
     * @param value   the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean putBoolean(String tabName, String key, boolean value) {
        return table.getSubTable(tabName).getEntry(key).setBoolean(value);
    }

    /**
     * Maps the specified key to the specified value in this table. The key can not be null. The value
     * can be retrieved by calling the get method with a key that is equal to the original key.
     *
     * @param tabName the Shuffleboard tab the key is under
     * @param key     the key
     * @param data    the value
     * @throws IllegalArgumentException If key is null
     */
    public static synchronized void putData(String tabName, String key, Sendable data) {
        String tableKey = tabName + "-" + key;
        Data sddata = tablesToData.get(tableKey);
        if (sddata == null || sddata.m_sendable != data) {
            if (sddata != null) {
                sddata.m_builder.stopListeners();
            }
            sddata = new Data(data);
            tablesToData.put(tableKey, sddata);
            NetworkTable dataTable = table.getSubTable(tabName).getSubTable(tableKey);
            sddata.m_builder.setTable(dataTable);
            data.initSendable(sddata.m_builder);
            sddata.m_builder.updateTable();
            sddata.m_builder.startListeners();
            dataTable.getSubTable(tabName).getEntry(".name").setString(key);
        }
    }

    /**
     * Maps the specified key (where the key is the name of the {@link NamedSendable}
     * to the specified value in this table. The value can be retrieved by
     * calling the get method with a key that is equal to the original key.
     *
     * @param tabName the Shuffleboard tab the key is under
     * @param value   the value
     * @throws IllegalArgumentException If key is null
     */
    public static void putData(String tabName, Sendable value) {
        putData(tabName, value.getName(), value);
    }

    /**
     * Put a number in the table.
     *
     * @param tabName the Shuffleboard tab the key is under
     * @param key     the key to be assigned to
     * @param value   the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean putNumber(String tabName, String key, double value) {
        return table.getSubTable(tabName).getEntry(key).setDouble(value);
    }

    /**
     * Returns the number array the key maps to. If the key does not exist or is
     * of different type, it will return the default value.
     *
     * @param tabName      the Shuffleboard tab the key is under
     * @param key          the key to look up
     * @param defaultValue the value to be returned if no value is found
     * @return the value associated with the given key or the given default value
     * if there is no value associated with the key
     */
    public static boolean putNumberArray(String tabName, String key, double value[]) {
        return table.getSubTable(tabName).getEntry(key).setDoubleArray(value);
    }

    /**
     * Put a string in the table.
     *
     * @param tabName the Shuffleboard tab the key is under
     * @param key     the key to be assigned to
     * @param value   the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean putString(String tabName, String key, String value) {
        return table.getSubTable(tabName).getEntry(key).setString(value);
    }

    private static class Data {
        Data(Sendable sendable) {
            m_sendable = sendable;
        }

        final Sendable m_sendable;
        final SendableBuilderImpl m_builder = new SendableBuilderImpl();
    }

    private static final Map<String, Data> tablesToData = new HashMap<>();
}
