/*
    <Program Name>
        SetupActivity.java

    <Purpose>
        This activity collects the user's height via text input, saving the data
        both locally and remotely with a unique ID. It is intended that the
        height will be used to estimate the user's stride (although not in this app).
 */

package com.sensibilitytestbed.whereability;


import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.os.IBinder;
import android.os.SystemClock;
import android.support.v7.app.AppCompatActivity;
import android.text.Editable;
import android.text.TextWatcher;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;

import org.json.JSONException;
import org.json.JSONObject;

import java.util.Random;


public class SetupActivity extends AppCompatActivity{

    public static final float CM_TO_FT = 0.0328084f;

    private SharedPreferences setupPrefs;
    private Button button;
    private EditText cmInput, ftInput, inInput;
    private float height = 175;
    private boolean isMetric = false;
    private BackhaulService mBackhaulService;
    private ServiceConnection mBackhaulConnection;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_setup);


        /**************** Load the most recently saved height ********************/

        setupPrefs = getSharedPreferences(MainActivity.SETUP, MODE_PRIVATE);
        if (savedInstanceState != null)
            height =  savedInstanceState.getFloat(MainActivity.HEIGHT, height);
        else
            height = setupPrefs.getFloat(MainActivity.HEIGHT, height);



        /******** Fill the TextViews with the height ***********/

        cmInput = (EditText) findViewById(R.id.cmInput);
        cmInput.setText("" + Math.round(height));

        float ft = height * CM_TO_FT;

        ftInput = (EditText) findViewById(R.id.ftInput);
        ftInput.setText("" + ((int) ft));

        inInput = (EditText) findViewById(R.id.inInput);
        inInput.setText("" + Math.round((ft - (int) ft) * 12));



        /**************  Setup event listeners for changes to the height  **********************/

        // Need to which height input is being changed by the user, not the code.
        cmInput.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                isMetric = true;
                return false;
            }
        });

        // Change the English height on changes to the metric height
        cmInput.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {

            }

            @Override
            public void afterTextChanged(Editable s) {
                // Is the text being
                // changed by code?
                if (!isMetric)
                    return;
                try {
                    // Convert the metric height to English system
                    height = Float.parseFloat(s.toString());
                    float ft = height * CM_TO_FT;
                    ftInput.setText("" + ((int) ft));
                    int in = Math.min(Math.round((ft - (int) ft) * 12), 11);
                    inInput.setText("" + in);
                } catch (Exception e) {
                    // The input is currently the null string
                }
            }
        });

        // Need to which height input is being changed by the user, not the code.
        View.OnTouchListener touchListener = new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                isMetric = false;
                return false;
            }
        };

        ftInput.setOnTouchListener(touchListener);
        inInput.setOnTouchListener(touchListener);

        // Change the metric height on changes to the English height
        TextWatcher watcher = new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {

            }

            @Override
            public void afterTextChanged(Editable s) {
                // Is the text being
                // changed by code?
                if (isMetric)
                    return;
                try {
                    // Update the metric height
                    height = (Integer.parseInt(ftInput.getText().toString()) + (Float.parseFloat(inInput.getText().toString()) / 12)) / CM_TO_FT;
                    cmInput.setText("" + Math.round(height));
                } catch (NumberFormatException e) {
                    // One of the inputs is currently the null string
                }
            }
        };

        // Feet and inches both change the metric
        // height, so they can use the same listener.
        ftInput.addTextChangedListener(watcher);
        inInput.addTextChangedListener(watcher);




        /********  Android requires a connection to bind to the backhauling service to communicate.  ********/

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



        /*************************************  Setup the Continue button  **********************************************/

        button = (Button) findViewById(R.id.continueButton);
        button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Context context = getBaseContext();

                // Has the height changed?
                if (height != setupPrefs.getFloat(MainActivity.HEIGHT, -1)) {

                    // The user is new, so save the new height with a new device ID
                    SharedPreferences.Editor prefsEditor = setupPrefs.edit();
                    prefsEditor.putFloat(MainActivity.HEIGHT, height);
                    long deviceID = new Random().nextLong() & 0xffffffffL;
                    prefsEditor.putLong(MainActivity.DEVICE_ID, deviceID);
                    prefsEditor.commit();

                    // Send the new user's information to the server via the dedicated backhaul service.
                    bindService(new Intent(context, BackhaulService.class), mBackhaulConnection, Context.BIND_AUTO_CREATE);
                    try {
                        JSONObject entry = new JSONObject();
                        entry.put("time", SystemClock.elapsedRealtime() * 1e6).put("deviceID", deviceID).put("height", height);
                        mBackhaulService.put(entry);
                    } catch (JSONException e) {
                        // No JSON exceptions will occur if the QR code is correct...
                    }
                    // Only need to send this one thing
                    unbindService(mBackhaulConnection);
                }

                // Finally, continue.
                Intent intent = new Intent(context, MainActivity.class);
                startActivity(intent);
            }
        });
    }





    @Override
    public void onSaveInstanceState(Bundle savedInstanceState) {
        // Remember the last height input when the activity is killed.
        savedInstanceState.putFloat(MainActivity.HEIGHT, height);
        super.onSaveInstanceState(savedInstanceState);
    }
}
