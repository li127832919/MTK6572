package android.bluetooth;

import android.content.Context;
import android.content.ServiceConnection;
import android.content.Intent;
import android.os.IBinder;

import android.bluetooth.IBluetoothDun;
import android.bluetooth.ConfigHelper;
import android.bluetooth.ProfileConfig;

import android.util.Log;

public final class BluetoothDunEx
{
    private static final String TAG = "BluetoothDunEx 4.2";
    
    public static synchronized void bindDunService(Context context, ServiceConnection connection) {
        if(ConfigHelper.checkSupportedProfiles(ProfileConfig.PROFILE_ID_DUN)) {
            if (!context.bindService(new Intent(IBluetoothDun.class.getName()), connection, 0)) {
                Log.e(TAG, "Could not bind to Bluetooth Dun Service");
            }
        }
        else {
            Log.e(TAG, "Bluetooth Dun is not supported!");
        }
    }
}


