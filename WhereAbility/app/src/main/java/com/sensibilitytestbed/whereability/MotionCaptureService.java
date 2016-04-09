/*
        <Program Name>
            MotionCaptureService.java

        <Purpose>
            Captures inertial and magnetic sensor data in the background,
            and sends it to another background service for backhauling to
            a remote server.
 */


package com.sensibilitytestbed.whereability;


import android.app.Service;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.content.SharedPreferences;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.IBinder;

import org.json.JSONException;
import org.json.JSONObject;



public class MotionCaptureService extends Service implements SensorEventListener{

    private long deviceID;
    private SensorManager mSensorManager;
    private BackhaulService mBackhaulService;
    private ServiceConnection mBackhaulConnection;
    private JSONObject entry;


    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {

        // Load the user's unique ID
        SharedPreferences setupPrefs = getSharedPreferences(MainActivity.SETUP, MODE_PRIVATE);
        deviceID = setupPrefs.getLong(MainActivity.DEVICE_ID, -1);



        /****************  Start listening for sensor events.  *****************************/

        mSensorManager = (SensorManager) this.getSystemService(Context.SENSOR_SERVICE);
        Sensor accelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        Sensor gyroscope = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        Sensor magnetometer = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        mSensorManager.registerListener(this, accelerometer, 5000);
        mSensorManager.registerListener(this, gyroscope, 5000);
        mSensorManager.registerListener(this, magnetometer, 5000);



        /****************  Start backhauling in a background service.  **************************/

        // Android requires a connection to bind to the backhauling service to communicate.
        mBackhaulConnection = new ServiceConnection() {
            @Override
            public void onServiceConnected(ComponentName name, IBinder service) {
                // Get a reference to the backhauling service to communicate with it.
                BackhaulService.BackhaulBinder binder = (BackhaulService.BackhaulBinder) service;
                mBackhaulService = binder.getService();
            }

            @Override
            public void onServiceDisconnected(ComponentName name) {

            }
        };
        Intent backhaulIntent = new Intent(this, BackhaulService.class);
        bindService(backhaulIntent, mBackhaulConnection, Context.BIND_AUTO_CREATE);



        return START_STICKY;
    }





    @Override
    public IBinder onBind(Intent intent) {
        // Not bound to an activity, so do nothing...
        return null;
    }





    @Override
    public void onDestroy() {
        // Something (i.e. user or system) told us to stop...
        mSensorManager.unregisterListener(this);
        // Tell the backhauling service to stop.
        unbindService(mBackhaulConnection);

        super.onDestroy();
    }





    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // Don't care about changes in accuracy, so do nothing...
    }





    @Override
    public void onSensorChanged(SensorEvent event) {
        try {
            switch (event.sensor.getType()) {
                case Sensor.TYPE_ACCELEROMETER:
                    /*
                        Because the sensors were registered at the
                        same time, with the same polling rate, they
                        all have the same timestamp.
                        But the accelerometer goes through the pipeline
                        first, so we'll save its timestamp now, along
                        with the deviceID.
                     */
                    double time = event.timestamp;
                    entry = new JSONObject();
                    entry.put("time", time);
                    entry.put("deviceID", deviceID);

                    entry.put("ax", event.values[0]);
                    entry.put("ay", event.values[1]);
                    entry.put("az", event.values[2]);
                    break;
                case Sensor.TYPE_GYROSCOPE:
                    entry.put("gx", event.values[0]);
                    entry.put("gy", event.values[1]);
                    entry.put("gz", event.values[2]);
                    break;
                case Sensor.TYPE_MAGNETIC_FIELD:
                    entry.put("mx", event.values[0]);
                    entry.put("my", event.values[1]);
                    entry.put("mz", event.values[2]);

                    // Magnetometer comes last,
                    // so backhaul this data entry.
                    mBackhaulService.put(entry);
                    break;
            }
        } catch (JSONException e) {
            // Shouldn't get a JSON exception if Android does it's job...
        }
    }
}
