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
import android.os.SystemClock;
import android.util.Log;

import org.json.JSONException;
import org.json.JSONObject;

import java.util.LinkedList;
import java.util.Queue;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;


public class MotionCaptureService extends Service implements SensorEventListener{

    private int deviceID;
    private SensorManager mSensorManager;
    private BackhaulService mBackhaulService;
    private ServiceConnection mBackhaulConnection;
    private boolean bound = false;
    private long boot = 0;
    private JSONObject entry;
    private Queue<JSONObject> queue = new LinkedList<JSONObject>();


    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {

        /****************  Start backhauling in a background service.  **************************/

        // Android requires a connection to bind to the backhauling service to communicate.
        mBackhaulConnection = new ServiceConnection() {
            @Override
            public void onServiceConnected(ComponentName name, IBinder service) {
                // Get a reference to the backhauling service to communicate with it.
                BackhaulService.BackhaulBinder binder = (BackhaulService.BackhaulBinder) service;
                mBackhaulService = binder.getService();
                bound = true;
            }

            @Override
            public void onServiceDisconnected(ComponentName name) {
                Log.d("MOCAP", "unbound");
                bound = false;
            }
        };
        Intent backhaulIntent = new Intent(this, BackhaulService.class);
        bindService(backhaulIntent, mBackhaulConnection, Context.BIND_AUTO_CREATE);

        // Load the user's randomized ID
        SharedPreferences setupPrefs = getSharedPreferences(MainActivity.SETUP, MODE_PRIVATE);
        deviceID = setupPrefs.getInt(MainActivity.DEVICE_ID, -1);


        boot = System.currentTimeMillis() - SystemClock.elapsedRealtime();
        Log.d("fw", "boot: " + boot);

        try {
            entry = new JSONObject();
            entry.put("time", 0L);
        }
        catch (JSONException e) {
            Log.d("UHOH", e.toString());
        }

        /****************  Start listening for sensor events.  *****************************/

        mSensorManager = (SensorManager) this.getSystemService(Context.SENSOR_SERVICE);
        Sensor accelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        Sensor gyroscope = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        Sensor magnetometer = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        mSensorManager.registerListener(this, accelerometer, 5000);
        mSensorManager.registerListener(this, gyroscope, 5000);
        mSensorManager.registerListener(this, magnetometer, 5000);

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

            long elapsed = SystemClock.elapsedRealtime();
            long timestamp;

            // A sensor event's timestamp can be
            // given in four different forms. Which one?

            // Epoch nanosec
            if (event.timestamp > 1e18)
                timestamp = event.timestamp;
            // ms since boot
            else if (Math.abs(elapsed - event.timestamp) < 1e3)
                timestamp = boot + event.timestamp;
            // nanosec since boot
            else if (Math.abs(elapsed - (event.timestamp / 1000000L)) < 1e3)
                timestamp = (boot * 1000000L) + event.timestamp;
            // Epoch ms
            else
                timestamp = event.timestamp * 1000000L;


            if (timestamp != entry.getLong("time")) {
                entry = new JSONObject();
                entry.put("time", timestamp);
                entry.put("deviceID", deviceID);
            }

            switch (event.sensor.getType()) {
                case Sensor.TYPE_ACCELEROMETER:
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
                    break;
            }

            queue.add(entry);

            if (bound) {
                while (!queue.isEmpty())
                    mBackhaulService.put(queue.remove());
            }

        } catch (Exception e) {
            Log.d("fw", e.toString());
        }

    }
}
