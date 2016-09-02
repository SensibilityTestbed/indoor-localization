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

import android.Manifest;
import android.app.Notification;
import android.app.NotificationManager;
import android.app.PendingIntent;
import android.app.TaskStackBuilder;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.graphics.PorterDuff;
import android.graphics.drawable.Drawable;
import android.os.Environment;
import android.os.IBinder;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.support.v7.app.ActionBar;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.support.v4.app.NotificationCompat;
import android.support.v7.widget.Toolbar;
import android.text.Editable;
import android.text.Html;
import android.util.Log;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.EditText;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;


import com.google.zxing.integration.android.IntentIntegrator;
import com.google.zxing.integration.android.IntentResult;

import org.json.JSONException;
import org.json.JSONObject;

import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
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

    private Switch mSwitch, onlineSwitch, qrSwitch;
    private EditText path, location;
    private TextView loclabel, pathlabel;
    private Button button;
    private boolean started = false, bound = false, scanned = false, fromCode = false;
    private String mDatabase = "";
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
                Log.d("fw", "unbound");
                bound = false;
            }
        };
        bindService(backhaulIntent, mBackhaulConnection, Context.BIND_AUTO_CREATE);



        // Has the height been sent yet?
        if (!setupPrefs.getBoolean(HEIGHT_SENT, false)) {

            new Thread(new Runnable() {
                @Override
                public void run() {
                    android.os.Process.setThreadPriority(android.os.Process.THREAD_PRIORITY_BACKGROUND);
                    lock.lock();
                    try {
                        while (!bound) {
                            notBound.awaitUninterruptibly();
                        }
                    } finally {
                        lock.unlock();
                    }

                    try {
                        JSONObject entry = new JSONObject();
                        entry.put("time", System.currentTimeMillis() * 1000000L).put("deviceID", deviceID).put("height", height);
                        mBackhaulService.put(entry);

                        SharedPreferences.Editor prefsEditor = getSharedPreferences(SETUP, MODE_PRIVATE).edit();
                        prefsEditor.putBoolean(HEIGHT_SENT, true);
                        prefsEditor.commit();
                    } catch (Exception e) {
                        // Shouldn't happen, right?
                        Log.d("UHOH", e.toString());
                    }
                }
            }).start();

        }


        // Build the UI
        setContentView(R.layout.activity_main);

        // Setup Up navigation to the height activity
        ActionBar mActionBar = getSupportActionBar();
        mActionBar.setDisplayHomeAsUpEnabled(true);

        final Drawable upArrow = ContextCompat.getDrawable(this, R.drawable.abc_ic_ab_back_mtrl_am_alpha);
        upArrow.setColorFilter(ContextCompat.getColor(this, R.color.colorPrimaryDark), PorterDuff.Mode.SRC_ATOP);
        mActionBar.setHomeAsUpIndicator(upArrow);

        mActionBar.setTitle(Html.fromHtml("<font color='#000'> WhereAbility </font>"));


        // Load previous UI state
        if (savedInstanceState != null) {
            started = savedInstanceState.getBoolean(STARTED, false);
            bound = savedInstanceState.getBoolean(SCANNED, false);
        }
        started = setupPrefs.getBoolean(STARTED, started);
        scanned = setupPrefs.getBoolean(SCANNED, scanned);




        /*******************  Build a notification, so the user is aware of background services  ***********************/

        mNotificationManager = (NotificationManager) getSystemService(Context.NOTIFICATION_SERVICE);

        final NotificationCompat.Builder mBuilder = new NotificationCompat.Builder(this);
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
            mSwitch.setText("SENSORS ON");
        else
            mSwitch.setText("SENSORS OFF");

        mSwitch.setChecked(started);

        mSwitch.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                // The user can't turn on background services via
                // the OFF switch, because they haven't checked in yet.
                if (isChecked && !scanned) {
                    mSwitch.setChecked(false);
                    if (qrSwitch.isChecked())
                        Toast.makeText(getBaseContext(), "Please scan a QR code first.", Toast.LENGTH_LONG).show();
                    else
                        Toast.makeText(getBaseContext(), "Please enter your location first.", Toast.LENGTH_LONG).show();

                } else if (!isChecked) {
                    started = false;
                    scanned = false;
                    mSwitch.setText("SENSORS OFF");
                    Toast.makeText(getBaseContext(), "Your data has been stored.", Toast.LENGTH_LONG).show();


                    // Kill background services
                    stopService(mocapIntent);
                    if (bound) {
                        unbindService(mBackhaulConnection);
                        bound = false;
                    }

                    // Remove background services notification
                    mNotificationManager.cancel(0);
                }
            }
        });


        pathlabel = (TextView) findViewById(R.id.pathlabel);

        onlineSwitch = (Switch) findViewById(R.id.switch2);
        onlineSwitch.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (onlineSwitch.isChecked()) {

                    onlineSwitch.setText("ONLINE");

                    Log.d("SWITCH", "ONLINE");
                    mBackhaulService.setOnline(true, null);

                    if (qrSwitch.isChecked()) {
                        path.setVisibility(EditText.INVISIBLE);
                        pathlabel.setVisibility(TextView.INVISIBLE);
                    }
                    else {
                        pathlabel.setText("Send to Sensevis path:");
                        path.setText("username/experiment/dataset");
                    }
                }
                else {
                    Log.d("SWITCH", "OFFLINE");
                    mBackhaulService.setOnline(false, "sdcard/" + path.getText().toString());
                    onlineSwitch.setText("OFFLINE");
                    pathlabel.setText("Save file as:");
                    pathlabel.setVisibility(TextView.VISIBLE);
                    path.setText("filename.csv");
                    path.setVisibility(EditText.VISIBLE);
                }
            }
        });


        loclabel = (TextView) findViewById(R.id.loclabel);
        location = (EditText) findViewById(R.id.location);


        // Setup Check-in button
        button = (Button) findViewById(R.id.checkin);

        final View.OnClickListener qrListener =  new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                // Start QR code scanner
                IntentIntegrator integrator = new IntentIntegrator(MainActivity.this);
                integrator.initiateScan(IntentIntegrator.QR_CODE_TYPES);
            }
        };

        final View.OnClickListener submitListener =  new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                if (onlineSwitch.isChecked()) {

                    String db = path.getText().toString();
                    String[] strArray = db.split("/");
                    db = "user=" + strArray[0] + "&exp=" + strArray[1] + "&set=" + strArray[2];

                    if (!mDatabase.equals(db)) {
                        SharedPreferences.Editor editor = getSharedPreferences(SETUP, MODE_PRIVATE).edit();
                        editor.putString("DATABASE", db);
                        editor.commit();
                        mDatabase = db;
                    }
                }



                long timestamp = System.currentTimeMillis() * 1000000L;
                try {
                    store(timestamp, new JSONObject("{'location':" + location.getText() + "}"));
                }
                catch (JSONException e) { Log.d("UHOH", e.toString()); }
            }
        };

        button.setOnClickListener(submitListener);



        path = (EditText) findViewById(R.id.path);

        qrSwitch = (Switch) findViewById(R.id.switch3);
        qrSwitch.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {

            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                /*if (mSwitch.isChecked()) {
                    qrSwitch.setChecked(!isChecked);
                    return;
                }*/
                if (isChecked) {

                    qrSwitch.setText("QR ON");

                    button.setText("SCAN QR CODE");
                    button.setOnClickListener(qrListener);

                    loclabel.setVisibility(TextView.INVISIBLE);
                    location.setVisibility(EditText.INVISIBLE);

                    if (onlineSwitch.isChecked()) {
                        pathlabel.setVisibility(TextView.INVISIBLE);
                        path.setVisibility(EditText.INVISIBLE);
                    }

                } else {
                    qrSwitch.setText("QR OFF");

                    button.setText("SUBMIT LOCATION");
                    button.setOnClickListener(submitListener);

                    loclabel.setVisibility(TextView.VISIBLE);
                    location.setVisibility(EditText.VISIBLE);

                    if (onlineSwitch.isChecked()) {
                        pathlabel.setText("Send to Sensevis path:");
                        path.setText("username/experiment/dataset");
                    }

                    pathlabel.setVisibility(TextView.VISIBLE);
                    path.setVisibility(EditText.VISIBLE);

                }
            }
        });

        ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.WRITE_EXTERNAL_STORAGE}, 0);
    }


    public void onRequestPermissionsResult(int requestCode, String permissions[], int[] grantResults) {
        switch (requestCode) {
            case 0: {
                // If request is cancelled, the result arrays are empty.
                if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                    Log.d("PERMS", "Yay, we can write. :P");
                }
                return;
            }
        }
    }




    private void store(long timestamp, JSONObject data) {

        if (!bound)
            bindService(new Intent(this, BackhaulService.class), mBackhaulConnection, Context.BIND_AUTO_CREATE);

        try {
            // Append a timestamp and the device ID to the location, and send it to the server.
            final JSONObject entry = data.put("time", timestamp).put("deviceID", deviceID);

            new Thread(new Runnable() {
                @Override
                public void run() {
                    android.os.Process.setThreadPriority(android.os.Process.THREAD_PRIORITY_BACKGROUND);

                    lock.lock();
                    try {
                        while (!bound)
                            notBound.awaitUninterruptibly();
                    } finally {
                        lock.unlock();
                    }

                    if (!started && !onlineSwitch.isChecked()) {
                        mBackhaulService.setOnline(false, "sdcard/" + path.getText().toString());
                        try {
                            entry.put("height", height);
                        }
                        catch (JSONException e) { Log.d("UHOH", e.toString()); }
                    }

                    try {
                        mBackhaulService.put(entry);
                    } catch (Exception e) {
                        Log.d("UHOH",e.toString());
                    }
                }
            }).start();

        } catch (JSONException e) {
            // The JSON location was malformed,
            // so ask the user to try again...
            if (qrSwitch.isChecked())
                Toast.makeText(this, "Please re-scan", Toast.LENGTH_LONG).show();
            else
                Toast.makeText(this, "Please re-enter your location", Toast.LENGTH_LONG).show();

            return;
        }


        Toast.makeText(this, "Location saved", Toast.LENGTH_SHORT).show();
        scanned = true;

        // Start sensing if we haven't already.
        if (!started) {
            started = true;
            startService(mocapIntent);

            // Show the user the app is ON
            mSwitch.setText("SENSORS ON");
            mSwitch.setChecked(true);

            // Show the user a persistent notification,
            // telling them background services are running.
            mNotificationManager.notify(0, mNotification);

        }
    }


    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        long timestamp = System.currentTimeMillis() * 1000000L;
        super.onActivityResult(requestCode, resultCode, data);


        try {
            IntentResult result = IntentIntegrator.parseActivityResult(requestCode, resultCode, data);
            if (result != null) {

                String contents = result.getContents();
                if (contents != null) {

                    JSONObject qrData = new JSONObject(contents);
                    String database = qrData.getString("database");
                    if (!mDatabase.equals(database)) {
                        SharedPreferences.Editor editor = getSharedPreferences(SETUP, MODE_PRIVATE).edit();
                        editor.putString("DATABASE", database);
                        editor.commit();
                    }

                    store(timestamp, qrData.getJSONObject("data"));
                    return;
                }
            }
        }
        catch (Exception e) { Log.d("UHOH", e.toString()); }
        // Tell the user the scan failed
        Toast.makeText(this, "Please re-scan", Toast.LENGTH_LONG).show();

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

        if (bound) {
            unbindService(mBackhaulConnection);
            bound = false;
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

