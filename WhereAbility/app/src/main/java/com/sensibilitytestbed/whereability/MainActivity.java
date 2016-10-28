/*
        <Program Name>
            MainActivity.java

        <Author>
            Seth Miller

        <Purpose>
            This activity provides a GUI for stopping and starting
            background services that collect and backhaul sensor data.
            It also provides a button for starting an activity to scan
            QR encoded locations. These ground truth locations are
            intended to be combined with inertial sensor data for the
            purpose of indoor localization (although not within this app).
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
import android.os.IBinder;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.support.v7.app.ActionBar;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.support.v4.app.NotificationCompat;
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

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;


public class MainActivity extends AppCompatActivity {

    public static final String SETUP = "setup_prefs";
    public static final String DEVICE_ID = "deviceID";
    public static final String HEIGHT = "height_cm";
    public static final String STARTED = "mocap_started";
    public static final String SCANNED = "scanned";
    public static final String ONLINE = "online";
    public static final String QR = "qr";
    public static final String PATH = "path";

    private Switch mSwitch, onlineSwitch, qrSwitch;
    private EditText path, location;
    private TextView loclabel, pathlabel;
    private Button button;
    private boolean started = false, bound = false, scanned = false;
    private int deviceID = -1;
    private float height = -1;
    private ServiceConnection mBackhaulConnection;
    private BackhaulService mBackhaulService;
    private Lock lock = new ReentrantLock();
    private Condition notBound = lock.newCondition();
    private Intent mocapIntent;
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
            return;
        }


        /*********  Prepare intents to start sensing and backhauling data in the background  ***********/

        mocapIntent = new Intent(this, MotionCaptureService.class);
        Intent backhaulIntent = new Intent(this, BackhaulService.class);

        // Android requires a connection to bind to the backhauling service to communicate.
        mBackhaulConnection = new ServiceConnection() {
            @Override
            public void onServiceConnected(ComponentName name, IBinder service) {
                Log.d("WhereAbility", "Connected to BackaulService");
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
                Log.d("WhereAbility", "Disconnected from BackhaulService");
                bound = false;
            }
        };
        bindService(backhaulIntent, mBackhaulConnection, Context.BIND_AUTO_CREATE);


        // Build the UI
        setContentView(R.layout.activity_main);

        // Setup Up navigation to the height activity
        ActionBar mActionBar = getSupportActionBar();
        try {
            mActionBar.setDisplayHomeAsUpEnabled(true);

            // draw the back arrow for navigation
            final Drawable upArrow = ContextCompat.getDrawable(this, R.drawable.abc_ic_ab_back_mtrl_am_alpha);
            upArrow.setColorFilter(ContextCompat.getColor(this, R.color.colorPrimaryDark), PorterDuff.Mode.SRC_ATOP);
            mActionBar.setHomeAsUpIndicator(upArrow);

            // add style to the app title
            mActionBar.setTitle(Html.fromHtml("<font color='#000'> WhereAbility </font>"));
        } catch (NullPointerException e) {
            Log.d("WhereAbility", "onCreate()", e);
        }


        onlineSwitch = (Switch) findViewById(R.id.switch2);
        qrSwitch = (Switch) findViewById(R.id.switch3);
        path = (EditText) findViewById(R.id.path);


        // Load previous or saved UI state
        /*if (savedInstanceState != null) {
            started = savedInstanceState.getBoolean(STARTED, false);
            scanned = savedInstanceState.getBoolean(SCANNED, false);
            onlineSwitch.setChecked(savedInstanceState.getBoolean(ONLINE, false));
            qrSwitch.setChecked(savedInstanceState.getBoolean(QR, false));
            path.setText(savedInstanceState.getString(PATH, "sdcard/filename.csv"));
        } else { */
            started = setupPrefs.getBoolean(STARTED, started);
            scanned = setupPrefs.getBoolean(SCANNED, scanned);
            onlineSwitch.setChecked(setupPrefs.getBoolean(ONLINE, false));
            qrSwitch.setChecked(setupPrefs.getBoolean(QR, false));
            path.setText(setupPrefs.getString(PATH, path.getText().toString()));
        //}

        if (onlineSwitch.isChecked())
            onlineSwitch.setText("ONLINE");


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
        if (started) {
            mSwitch.setText("SENSORS ON");
            mSwitch.setChecked(true);
            path.setClickable(false);
        } else {
            mSwitch.setText("SENSORS OFF");
            mSwitch.setChecked(false);
        }

        mSwitch.setChecked(started);

        mSwitch.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                // The user can't turn on background services via
                // the OFF switch, because they haven't checked in yet.
                try {
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
                        path.setClickable(true);
                        Toast.makeText(getBaseContext(), "Your data has been stored.", Toast.LENGTH_LONG).show();


                        // Turn off sensors
                        stopService(mocapIntent);

                        // Remove background services notification
                        mNotificationManager.cancel(0);

                        /*
                        // Notify background backhauling service to finish up
                        if (bound) {
                            unbindService(mBackhaulConnection);
                            bound = false;
                        }
                        */
                    }
                } catch (Exception e) {
                    Log.e("WhereAbility", "onCreate()", e);
                }
            }
        });


        pathlabel = (TextView) findViewById(R.id.pathlabel);

        if (onlineSwitch.isChecked())
            onlineSwitch.setText("ONLINE");
        else
            onlineSwitch.setText("OFFLINE");

        onlineSwitch.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (onlineSwitch.isChecked()) {

                    onlineSwitch.setText("ONLINE");

                    Log.d("WhereAbility", "Switch online");

                    if (qrSwitch.isChecked()) {
                        path.setVisibility(EditText.INVISIBLE);
                        pathlabel.setVisibility(TextView.INVISIBLE);
                    } else {
                        pathlabel.setText("Send to URL:");
                        path.setText("");
                    }
                } else {
                    Log.d("WhereAbility", "Switch offline");
                    onlineSwitch.setText("OFFLINE");
                    pathlabel.setText("Save as:");
                    pathlabel.setVisibility(TextView.VISIBLE);
                    path.setText("sdcard/filename.csv");
                    path.setVisibility(EditText.VISIBLE);
                }
            }
        });


        loclabel = (TextView) findViewById(R.id.loclabel);
        location = (EditText) findViewById(R.id.location);


        // Setup Check-in button
        button = (Button) findViewById(R.id.checkin);

        final View.OnClickListener qrListener = new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                // Start QR code scanner
                IntentIntegrator integrator = new IntentIntegrator(MainActivity.this);
                integrator.initiateScan(IntentIntegrator.QR_CODE_TYPES);
            }
        };

        final View.OnClickListener submitListener = new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.d("WhereAbility", "Submit location");
                long timestamp = System.currentTimeMillis() * 1000000L;
                try {
                    store(timestamp, new JSONObject("{'location':" + location.getText() + "}"));
                } catch (JSONException e) {
                    Log.d("WhereAbility", "onCreate()", e);
                }
            }
        };

        button.setOnClickListener(submitListener);


        if (qrSwitch.isChecked()) {
            setQrChecked(qrListener);
        }

        qrSwitch.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {

            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (isChecked) {

                    setQrChecked(qrListener);

                } else {
                    qrSwitch.setText("QR OFF");

                    button.setText("SUBMIT LOCATION");
                    button.setOnClickListener(submitListener);

                    loclabel.setVisibility(TextView.VISIBLE);
                    location.setVisibility(EditText.VISIBLE);

                    if (onlineSwitch.isChecked()) {
                        pathlabel.setText("Send to URL:");
                        path.setText("");
                    }

                    pathlabel.setVisibility(TextView.VISIBLE);
                    path.setVisibility(EditText.VISIBLE);

                }
            }
        });

        ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.WRITE_EXTERNAL_STORAGE}, 0);
    }

    private void setQrChecked(View.OnClickListener qrListener) {

        qrSwitch.setText("QR ON");

        button.setText("SCAN QR CODE");
        button.setOnClickListener(qrListener);

        loclabel.setVisibility(TextView.INVISIBLE);
        location.setVisibility(EditText.INVISIBLE);

        if(onlineSwitch.isChecked())

        {
            pathlabel.setVisibility(TextView.INVISIBLE);
            path.setVisibility(EditText.INVISIBLE);
        }

    }

    public void onRequestPermissionsResult(int requestCode, String permissions[], int[] grantResults) {
        switch (requestCode) {
            case 0: {
                // If request is cancelled, the result arrays are empty.
                if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                    Log.d("WhereAbility", "Write permission granted");
                }
                break;
            }
        }
    }

    private void store(long timestamp, JSONObject data) {

        Log.d("WhereAbility", "In store()");

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

                    if (!started) {
                        try {
                            mBackhaulService.addPath(path.getText().toString(), onlineSwitch.isChecked());
                            entry.put("height", height);
                        }
                        catch(JSONException e) {
                            Log.d("WhereAbility", "store()", e);
                        }
                    }

                    try {
                        mBackhaulService.put(entry);
                    } catch (Exception e) {
                        Log.d("WhereAbility", "store()", e);
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

        Log.d("WhereAbility", "Created thread in store()");


        Toast.makeText(this, "Location saved", Toast.LENGTH_SHORT).show();
        scanned = true;

        // Start sensing if we haven't already.
        if (!started) {
            started = true;
            startService(mocapIntent);

            // Show the user the app is ON
            mSwitch.setText("SENSORS ON");
            mSwitch.setChecked(true);

            // path shouldn't change while running
            path.setClickable(false);

            // Show the user a persistent notification,
            // telling them background services are running.
            mNotificationManager.notify(0, mNotification);

        }

        Log.d("WhereAbility", "End of store()");
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

                    store(timestamp, qrData.getJSONObject("data"));
                    return;
                }
            }
        }
        catch (Exception e) { Log.d("WhereAbility", "onActivityResult()", e); }
        // Tell the user the scan failed
        Toast.makeText(this, "Please re-scan", Toast.LENGTH_LONG).show();

    }





    @Override
    public void onSaveInstanceState(Bundle savedInstanceState) {
        Log.d("WhereAbility", "Save state");
        // Save the UI state
        savedInstanceState.putBoolean(STARTED, started);
        savedInstanceState.putBoolean(SCANNED, scanned);
        savedInstanceState.putBoolean(ONLINE, onlineSwitch.isChecked());
        savedInstanceState.putBoolean(QR, qrSwitch.isChecked());
        savedInstanceState.putString(PATH, path.getText().toString());

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
        try {
            editor.putBoolean(ONLINE, onlineSwitch.isChecked());
            editor.putBoolean(QR, qrSwitch.isChecked());
            editor.putString(PATH, path.getText().toString());
        } catch (NullPointerException e) {
            editor.putBoolean(ONLINE, false);
            editor.putBoolean(QR, false);
            editor.putString(PATH, "sdcard/filename.csv");
        }
        editor.apply();

        super.onDestroy();
    }





    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        startActivity(new Intent(this, SetupActivity.class));
        finish();
        return false;
    }


}

