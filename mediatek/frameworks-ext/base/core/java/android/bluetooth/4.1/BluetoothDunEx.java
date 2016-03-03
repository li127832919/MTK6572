package android.bluetooth;

import android.content.Context;
import android.content.ServiceConnection;
import android.content.Intent;
import android.os.IBinder;

import android.bluetooth.IBluetoothDun;

import android.util.Log;

public final class BluetoothDunEx
{
    private static final String TAG = "BluetoothDunEx 4.1";
    
    public static synchronized void bindDunService(Context context, ServiceConnection connection) {
        if (!context.bindService(new Intent(IBluetoothDun.class.getName()), connection, context.BIND_AUTO_CREATE)) {
            Log.e(TAG, "Could not bind to Bluetooth Dun Service");
        }
    }
}


