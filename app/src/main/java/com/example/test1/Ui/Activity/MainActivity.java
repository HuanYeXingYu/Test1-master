package com.example.test1.Ui.Activity;

import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.support.v4.app.FragmentActivity;
import android.support.v4.view.ViewPager;
import android.util.Log;
import android.view.View;
import android.widget.EditText;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.TextView;
import android.widget.Toast;

import com.example.test1.Adapter.MyViewPagerAdapter;
import com.example.test1.R;
import com.example.test1.Serial.Serial;
import com.example.test1.Serial.SerialPort;
import com.example.test1.Serial.SerialPortFinder;
import com.example.test1.Ui.Fragment.FragmentFive;
import com.example.test1.Ui.Fragment.FragmentFour;
import com.example.test1.Ui.Fragment.FragmentOne;
import com.example.test1.Ui.Fragment.FragmentSix;
import com.example.test1.Ui.Fragment.FragmentThree;
import com.example.test1.Ui.Fragment.FragmentTwo;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import butterknife.BindView;
import butterknife.ButterKnife;

public class MainActivity extends FragmentActivity {
    Serial serial = new Serial();
    TextView U1;

    FileOutputStream mOutputStream;
    FileInputStream mInputStream;
    SerialPort sp;
    @BindView(R.id.right1)
    RadioButton right1;
    @BindView(R.id.right2)
    RadioButton right2;
    @BindView(R.id.right3)
    RadioButton right3;
    @BindView(R.id.right4)
    RadioButton right4;
    @BindView(R.id.right5)
    RadioButton right5;
    @BindView(R.id.right6)
    RadioButton right6;
    @BindView(R.id.rGroup)
    RadioGroup radioGroup;
    @BindView(R.id.viewpager)
    ViewPager viewPager;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        ButterKnife.bind(this);
        initView();

    }

    private void initView() {
        U1 = (TextView) findViewById(R.id.U1);
        radioGroup.check(R.id.right1);
        //RadioGroup选中状态改变监听
        radioGroup.setOnCheckedChangeListener(new RadioGroup.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(RadioGroup group, int checkedId) {
                switch (checkedId) {
                    case R.id.right1:
                        /**
                         * setCurrentItem第二个参数控制页面切换动画
                         * true:打开/false:关闭
                         */
                        viewPager.setCurrentItem(0, false);
                        break;
                    case R.id.right2:
                        viewPager.setCurrentItem(1, false);
                        break;
                    case R.id.right3:
                        viewPager.setCurrentItem(2, false);
                        break;
                    case R.id.right4:
                        viewPager.setCurrentItem(3, false);
                        break;
                    case R.id.right5:
                        viewPager.setCurrentItem(4, false);
                        break;
                    case R.id.right6:
                        viewPager.setCurrentItem(5, false);
                        break;
                }
            }
        });

        Fragment one = new FragmentOne();
        Fragment three = new FragmentThree();
        Fragment four = new FragmentFour();
        Fragment five = new FragmentFive();
        Fragment six = new FragmentSix();
        Fragment two = new FragmentTwo();


        List<Fragment> alFragment = new ArrayList<>();
        alFragment.add(one);
        alFragment.add(two);
        alFragment.add(three);
        alFragment.add(four);
        alFragment.add(five);
        alFragment.add(six);


        //ViewPager设置适配器
        viewPager.setAdapter(new MyViewPagerAdapter(getSupportFragmentManager(), alFragment));
        //ViewPager显示第一个Fragment
        viewPager.setCurrentItem(0);
        //ViewPager页面切换监听
        viewPager.addOnPageChangeListener(new ViewPager.OnPageChangeListener() {
            @Override
            public void onPageScrolled(int position, float positionOffset, int positionOffsetPixels) {

            }

            @Override
            public void onPageSelected(int position) {
                switch (position) {
                    case 0:
                        radioGroup.check(R.id.right1);
                        break;
                    case 1:
                        radioGroup.check(R.id.right2);
                        break;
                    case 2:
                        radioGroup.check(R.id.right3);
                        break;
                    case 3:
                        radioGroup.check(R.id.right4);
                        break;
                    case 4:
                        radioGroup.check(R.id.right5);
                        break;
                    case 5:
                        radioGroup.check(R.id.right6);
                        break;
                }
            }

            @Override
            public void onPageScrollStateChanged(int state) {

            }
        });
//        SerialPortFinder finder = new SerialPortFinder();
//        List<String> list = Arrays.asList(finder.getAllDevices());
//        for (String a : list
//                ) {
//            Log.e("SerialPort=", a);
//        }

        try {
            sp = new SerialPort(new File("/dev/ttyS2"), 9600);
            mOutputStream = (FileOutputStream) sp.getOutputStream();
            mInputStream = (FileInputStream) sp.getInputStream();
        } catch (SecurityException | IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        U1.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                int size;
                try {
                    byte[] buffer = new byte[64];
                    if (mInputStream == null) {
                        Log.e("SerialPort=", "null");
                        return;
                    }
                    size = mInputStream.read(buffer);
                    if (size > 0) {
                        onDataReceived(buffer, size);
                    }
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });


//
//
//U1.setText("ddddddddddd");
//
//        serial.Open(3, 9600);
//        new Thread(new Runnable() {
//            @Override
//            public void run() {
//                while (true) {
//                    final int[] RX = serial.Read();
//
//                    if (RX != null) {
//                        runOnUiThread(new Runnable() {
//                            @Override
//                            public void run() {
//                             //   U1.setText("");
//                                U1.setText(RX.length+"");
//                                Toast.makeText(MainActivity.this, ""+RX.length, Toast.LENGTH_SHORT).show();
//                            }
//                        });
//
//                    }
//                }
//            }
//        }).start();


    }

    static {
        System.loadLibrary("native-lib");
    }

    void onDataReceived(final byte[] buffer, final int size) {
        runOnUiThread(new Runnable() {
            public void run() {
                if (U1 != null) {
                    U1.append(new String(buffer, 0, size));
                }
            }
        });
    }

}

