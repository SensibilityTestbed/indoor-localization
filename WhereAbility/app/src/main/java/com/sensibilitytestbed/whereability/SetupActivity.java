/*
    <Program Name>
        SetupActivity.java

    <Author>
        Seth Miller

    <Purpose>
        This activity collects the user's height via text input, saving the data
        both locally and/or remotely with a unique ID. It is intended that the
        height will be used to estimate the user's stride (although not in this app).
 */

package com.sensibilitytestbed.whereability;



import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.support.v7.app.ActionBar;
import android.support.v7.app.AppCompatActivity;
import android.text.Editable;
import android.text.Html;
import android.text.TextWatcher;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;



import java.util.Random;



public class SetupActivity extends AppCompatActivity{

    public static final float CM_TO_FT = 0.0328084f;
    public static final float ROUNDS_T0_FOOT = 11.5f / 12;

    private SharedPreferences setupPrefs;
    private Button button;
    private EditText cmInput, ftInput, inInput;
    private float height = 175;
    private boolean isMetric = false;



    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_setup);

        ActionBar actionBar = getSupportActionBar();
        actionBar.setTitle(Html.fromHtml("<font color='#000'> Setup </font>"));


        /**************** Load the most recently saved height ********************/

        setupPrefs = getSharedPreferences(MainActivity.SETUP, MODE_PRIVATE);
        if (savedInstanceState != null)
            height =  savedInstanceState.getFloat(MainActivity.HEIGHT, height);
        else
            height = setupPrefs.getFloat(MainActivity.HEIGHT, height);



        /******** Fill the TextViews with the height ***********/

        cmInput = (EditText) findViewById(R.id.cmInput);
        ftInput = (EditText) findViewById(R.id.ftInput);
        inInput = (EditText) findViewById(R.id.inInput);

        cmInput.setText("" + Math.round(height));

        float ft = height * CM_TO_FT;
        int ftInt = (int) ft;

        if (ft - ftInt >= ROUNDS_T0_FOOT) {
            ftInput.setText("" + (ftInt + 1));
            inInput.setText("0");
        }
        else {
            ftInput.setText("" + ftInt);
            inInput.setText("" + Math.round((ft - ftInt) * 12));
        }





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
                    int ftInt = (int) ft;
                    if (ft - ftInt >= ROUNDS_T0_FOOT) {
                        ftInput.setText("" + (ftInt + 1));
                        inInput.setText("0");
                    }
                    else {
                        ftInput.setText("" + ftInt);
                        inInput.setText("" + Math.round((ft - ftInt) * 12));
                    }
                } catch (Exception e) {
                    // The input is currently the null string
                }
            }
        });

        // Need to know the height input is being changed by the user, not the code.
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
                    int deviceID = Math.abs(new Random().nextInt());
                    prefsEditor.putInt(MainActivity.DEVICE_ID, deviceID);
                    prefsEditor.commit();

                }

                // Finally, continue to MainActivity.
                Intent intent = new Intent(context, MainActivity.class);
                startActivity(intent);

            }
        });

        Log.d("WhereAbility", "Setup complete");
    }





    @Override
    public void onSaveInstanceState(Bundle savedInstanceState) {
        // Remember the last height input when the activity is killed.
        savedInstanceState.putFloat(MainActivity.HEIGHT, height);
        super.onSaveInstanceState(savedInstanceState);
    }
}
