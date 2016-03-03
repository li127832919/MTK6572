package android.bluetooth;

import android.content.Context;

/**
 * Represents a BluetoothCmnAdapter
 *
 * @hide
 */
public class BluetoothCmnAdapter{
    public static Object getBluetoothService(Context ctxt){
        return BluetoothAdapter.getDefaultAdapter();
    }
}
