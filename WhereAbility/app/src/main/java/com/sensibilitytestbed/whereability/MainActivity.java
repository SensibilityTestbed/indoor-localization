/*
        <Program Name>
            MainActivity.java

        <Purpose>
            This activity provides a GUI for stopping and starting
            background services that collect and backhaul sensor data.
            It also provides a button for starting an activity to scan
            QR encoded locations. These ground truth locations are
            intended to be combined with inertial sensor data for the
            purpose of indoor localization (although not in this app).
 */

package com.sensibilitytestbed.whereability;

import android.app.Notification;
import android.app.NotificationManager;
import android.app.PendingIntent;
import android.app.TaskStackBuilder;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.content.SharedPreferences;
import android.os.IBinder;
import android.os.SystemClock;
import android.support.v7.app.ActionBar;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.support.v4.app.NotificationCompat;
import android.util.Log;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.Switch;
import android.widget.Toast;

import com.gnzlt.AndroidVisionQRReader.QRActivity;

import org.json.JSONException;
import org.json.JSONObject;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;


public class MainActivity extends AppCompatActivity {

    public static final String SETUP = "setup_prefs";
    public static final String DEVICE_ID = "deviceID";
    public static final String HEIGHT = "height_cm";
    public static final String HEIGHT_SENT = "height_sent";
    public static final String STARTED = "mocap_started";
    public static final String SCANNED = "scanned";
    public static final int QR_REQUEST = 111;

    private Switch mSwitch;
    private Button button;
    private boolean started = false, bound = false, scanned = false, fromCode = false;
    private int deviceID = -1;
    private float height = -1;
    private ServiceConnection mBackhaulConnection;
    private BackhaulService mBackhaulService;
    private Lock lock = new ReentrantLock();
    private Condition notBound = lock.newCondition();
    private Intent mocapIntent, backhaulIntent;
    private Notification mNotification;
    private NotificationManager mNotificationManager;



    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        // Load the user's height and deviceID
        SharedPreferences setupPrefs = getSharedPreferences(SETUP, MODE_PRIVATE);
        height = setupPrefs.getFloat(HEIGHT, -1);
        deviceID = setupPrefs.getInt(DEVICE_ID, -1);

        // Has the user input their height yet?
        if (height < 0 || deviceID < 0) {
            startActivity(new Intent(this, SetupActivity.class));
            finish();
            Log.d("MAIN", "Went directly to Setup");
            return;
        }




        /*********  Prepare intents to start sensing and backhauling data in the background  ***********/

        mocapIntent = new Intent(this, MotionCaptureService.class);
        backhaulIntent = new Intent(this, BackhaulService.class);

        // Android requires a connection to bind to the backhauling service to communicate.
        mBackhaulConnection = new ServiceConnection() {
            @Override
            public void onServiceConnected(ComponentName name, IBinder service) {
                Log.d("MAIN", "connected to service");
                // Get a reference to the backhauling service to communicate with it.
                BackhaulService.BackhaulBinder binder = (BackhaulService.BackhaulBinder) service;
                mBackhaulService = binder.getService();
                bound = true;
                lock.lock();
                try {
                    notBound.signalAll();
                } finally {
                    lock.unlock();
                }
            }

            @Override
            public void onServiceDisconnected(ComponentName name) {
                Log.d("MAIN", "service died");
                bound = false;
            }
        };
        bindService(backhaulIntent, mBackhaulConnection, Context.BIND_AUTO_CREATE);




        // Has the height been sent yet?
        if (setupPrefs.getBoolean(HEIGHT_SENT, false))

            Log.d("MAIN", "send height");
            new Thread(new Runnable() {
                @Override
                public void run() {
                    android.os.Process.setThreadPriority(android.os.Process.THREAD_PRIORITY_BACKGROUND);
                    Log.d("MAIN", "ready to send height");
                    lock.lock();
                    try {
                        while (!bound) {
                            Log.d("MAIN", "waiting to bind");
                            notBound.awaitUninterruptibly();
                        }
                    } finally {
                        lock.unlock();
                    }

                    Log.d("MAIN", "sending height");
                    try {
                        JSONObject entry = new JSONObject();
                        entry.put("time", SystemClock.elapsedRealtime() * 1e6).put("deviceID", deviceID).put("height", height);
                        mBackhaulService.put(entry);

                        SharedPreferences.Editor prefsEditor = getSharedPreferences(SETUP, MODE_PRIVATE).edit();
                        prefsEditor.putBoolean(HEIGHT_SENT, true);
                        prefsEditor.commit();
                    } catch (Exception e) {
                        // Shouldn't happen
                        Log.d("MAIN", "send failed");
                    }
                    Log.d("MAIN", "height thread done");
                }
            }).start();

        Log.d("MAIN", "bound: " + bound);

        // Build the UI
        setContentView(R.layout.activity_main);

        // Setup Up navigation to the height activity
        ActionBar mActionBar = getSupportActionBar();
        mActionBar.setDisplayHomeAsUpEnabled(true);
        mActionBar.setTitle("Height");


        // Load previous UI state
        if (savedInstanceState != null) {
            started = savedInstanceState.getBoolean(STARTED, false);
            bound = savedInstanceState.getBoolean(SCANNED, false);
        }
        started = setupPrefs.getBoolean(STARTED, started);
        scanned = setupPrefs.getBoolean(SCANNED, scanned);




        /*******************  Build a notification, so the user is aware of background services  ***********************/

        mNotificationManager = (NotificationManager) getSystemService(Context.NOTIFICATION_SERVICE);

        NotificationCompat.Builder mBuilder = new NotificationCompat.Builder(this);
        mBuilder.setSmallIcon(R.mipmap.ic_launcher);
        mBuilder.setContentTitle("WhereAbility");
        mBuilder.setContentText("App is running in background...");
        // Prevent dismissal
        mBuilder.setOngoing(true);

        // Push the main activity (this) onto the task stack,
        // so the user can resurrect the UI to turn the app off.
        Intent resurrect = new Intent(this, MainActivity.class);
        TaskStackBuilder stackBuilder = TaskStackBuilder.create(this);
        stackBuilder.addParentStack(MainActivity.class);
        stackBuilder.addNextIntent(resurrect);

        // Add the resurrection intent to the notification builder.
        PendingIntent resultPendingIntent = stackBuilder.getPendingIntent(0, PendingIntent.FLAG_UPDATE_CURRENT);
        mBuilder.setContentIntent(resultPendingIntent);

        mNotification = mBuilder.build();



        /********************  Create the OFF switch to kill background services  *********************/

        mSwitch = (Switch) findViewById(R.id.switch1);
        // Switch should be created with last state
        if (started)
            mSwitch.setText("ON");
        else
            mSwitch.setText("OFF");
        mSwitch.setChecked(started);
        mSwitch.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                // The user can't turn on background services via
                // the OFF switch, because they haven't checked in yet.
                /*if (fromCode) {
                    fromCode = false;
                    return;
                }*/
                if (isChecked && !scanned) {
                    // Undo attempts to turn the switch ON manually
                    //fromCode = true;
                    mSwitch.setChecked(false);
                    Toast.makeText(getBaseContext(), "Please scan a QR code first.", Toast.LENGTH_LONG).show();

                } else if (!isChecked) {
                    started = false;
                    scanned = false;
                    //fromCode = true;
                    mSwitch.setText("OFF");
                    Toast.makeText(getBaseContext(), "Thank you for participating!", Toast.LENGTH_LONG).show();


                    // Kill background services
                    stopService(mocapIntent);
                    /*if (bound)
                        unbindService(mBackhaulConnection);
                        */


                    // Remove background services notification
                    mNotificationManager.cancel(0);
                }
            }
        });





        // Setup Check-in button
        button = (Button) findViewById(R.id.checkin);
        button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                // Start QR code scanner
                Intent intent = new Intent(getBaseContext(), QRActivity.class);
                startActivityForResult(intent, QR_REQUEST);
            }
        });

    }





    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);


        if (requestCode == QR_REQUEST) {
            // Did the scan work?

            if (resultCode == RESULT_OK) {
                // Get the JSON encoded location from the QR code

                String qrData = data.getStringExtra(QRActivity.EXTRA_QR_RESULT);


                // Need to be bound to the backhaul
                // service to send the location.
                if (!bound)
                    bindService(new Intent(this, BackhaulService.class), mBackhaulConnection, Context.BIND_AUTO_CREATE);


                try {
                    // Append a timestamp and the device ID to the location, and send it to the server.
                    final JSONObject entry = new JSONObject(qrData).put("time", SystemClock.elapsedRealtime() * 1e6).put("deviceID", deviceID);

                    new Thread(new Runnable() {
                        @Override
                        public void run() {
                            android.os.Process.setThreadPriority(android.os.Process.THREAD_PRIORITY_BACKGROUND);
                            Log.d("MAIN", "service not running");
                            lock.lock();
                            try {
                                while (!bound)
                                    notBound.awaitUninterruptibly();
                            } finally {
                                lock.unlock();
                            }

                            try {
                                mBackhaulService.put(entry);
                            } catch (Exception e) {

                            }
                        }
                    }).start();

                    Toast.makeText(this, "Location saved", Toast.LENGTH_SHORT).show();

                } catch (JSONException e) {
                    // The JSON location was malformed,
                    // so ask the user to try again...
                    Toast.makeText(this, "Please re-scan", Toast.LENGTH_LONG).show();
                    return;
                }

                scanned = true;

                // Start sensing if we haven't already.
                if (!started) {
                    started = true;
                    startService(mocapIntent);

                    // Show the user the app is ON
                    mSwitch.setText("ON");
                    mSwitch.setChecked(true);

                    // Show the user a persistent notification,
                    // telling them background services are running.
                    mNotificationManager.notify(0, mNotification);

                }

            } else {
                // Tell the user the scan failed
                Toast.makeText(this, "Please re-scan", Toast.LENGTH_LONG).show();
            }
        }
    }





    @Override
    public void onSaveInstanceState(Bundle savedInstanceState) {
        Log.d("END", "SAVE");
        // Save the UI state
        savedInstanceState.putBoolean(STARTED, started);
        savedInstanceState.putBoolean(SCANNED, scanned);

        super.onSaveInstanceState(savedInstanceState);
    }

    @Override
    public void onDestroy() {
        Log.d("MYEND", "DESTROY");

        if (bound) {
            unbindService(mBackhaulConnection);
        }



        SharedPreferences.Editor editor = getSharedPreferences(SETUP, MODE_PRIVATE).edit();
        editor.putBoolean(STARTED, started);
        editor.putBoolean(SCANNED, scanned);
        editor.commit();



        super.onDestroy();

    }





    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        startActivity(new Intent(this, SetupActivity.class));
        finish();
        return false;
    }


}

