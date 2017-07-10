package com.example.test1.Ui.Activity;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;

import com.example.test1.R;

public class WelcomeActivity extends Activity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_welcome);

        Runnable runnable = new Runnable() {
            @Override
            public void run() {
                goToMain();
            }
        };
        new Handler().postDelayed(runnable,5000);

    }

    private void goToMain() {
        startActivity(new Intent(WelcomeActivity.this, MainActivity.class));
        finish();

    }
}
