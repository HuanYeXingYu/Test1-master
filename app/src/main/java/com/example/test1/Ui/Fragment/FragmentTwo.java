package com.example.test1.Ui.Fragment;


import android.app.Activity;
import android.content.Context;
import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.text.Editable;
import android.text.TextWatcher;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import com.example.test1.Entity.UType;
import com.example.test1.R;
import com.example.test1.Serial.Serial;
import com.example.test1.View.ClockView;

import java.util.ArrayList;
import java.util.List;

import butterknife.BindView;
import butterknife.ButterKnife;
import butterknife.OnTextChanged;

public class FragmentTwo extends Fragment {
    private View rootView;//缓存Fragment view 
    ClockView clock;
    @BindView(R.id.U1)
    TextView U1;
    @BindView(R.id.U2)
    EditText U2;
    @BindView(R.id.U3)
    EditText U3;
    @BindView(R.id.I1)
    EditText I1;
    @BindView(R.id.I2)
    EditText I2;
    @BindView(R.id.I3)
    EditText I3;
    @BindView(R.id.d1)
    EditText d1;
    @BindView(R.id.d2)
    EditText d2;
    @BindView(R.id.d3)
    EditText d3;
    @BindView(R.id.p1)
    TextView p1;
    @BindView(R.id.p2)
    TextView p2;
    @BindView(R.id.p3)
    TextView p3;
    @BindView(R.id.pt)
    TextView pt;
    List<UType> typeList = new ArrayList<>();
    List<UType> typeList1 = new ArrayList<>();
    float dianliu, r, U11, I11, U12, I12, U13, I13, P1, P2, P3;
    EditText jiao12, jiao23, jiao31;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        rootView = inflater.inflate(R.layout.right2, container, false);
        jiao12 = (EditText) rootView.findViewById(R.id.jiao_12);
        jiao23 = (EditText) rootView.findViewById(R.id.jiao_23);
        jiao31 = (EditText) rootView.findViewById(R.id.jiao_31);
        clock = (ClockView) rootView.findViewById(R.id.clock);

        r = clock.getR();
        initJiao();

        return rootView;


    }

    private void initJiao() {
        jiao12.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {

            }

            @Override
            public void afterTextChanged(Editable s) {
                drawLineI("I1", s, 1);
            }
        });
        jiao23.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {

            }

            @Override
            public void afterTextChanged(Editable s) {
                drawLineI("I2", s, 2);
            }
        });
        jiao31.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {

            }

            @Override
            public void afterTextChanged(Editable s) {
                drawLineI("I3", s, 3);
            }
        });
    }

    @Override
    public void onAttach(Activity activity) {
        super.onAttach(activity);
        //获取activity根视图,rootView设为全局变量
        rootView = activity.getWindow().getDecorView();
        ButterKnife.bind(this, rootView);
        typeList.add(new UType("U1", 0, 1));
        typeList.add(new UType("U2", 0, 2));
        typeList.add(new UType("U3", 0, 3));
        typeList1.add(new UType("U1", 0, 1));
        typeList1.add(new UType("U2", 0, 2));
        typeList1.add(new UType("U3", 0, 3));

        d1.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {

            }

            @Override
            public void afterTextChanged(Editable s) {
                drawLineU("U1", s, 1);
                String angle = s.toString();
                if (angle.equals("")) {
                    angle = "0";
                }
                float D11 = Float.parseFloat(angle);

                P1 = Math.abs((float)(U11 * I11 *  Math.cos(D11 * Math.PI / 180)));
                p1.setText(String.valueOf(P1));
                pt.setText(String.valueOf(P1 + P2 + P3));
            }
        });
        d2.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {

            }

            @Override
            public void afterTextChanged(Editable s) {
                drawLineU("U2", s, 2);
                String angle = s.toString();
                if (angle.equals("")) {
                    angle = "0";
                }
                float D12 = Float.parseFloat(angle);

                P2 =Math.abs(U12 * I12 * (float) Math.cos(D12 * Math.PI / 180)) ;
                p2.setText(String.valueOf(P2));
                pt.setText(String.valueOf(P1 + P2 + P3));
            }
        });
        d3.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {

            }

            @Override
            public void afterTextChanged(Editable s) {
                drawLineU("U3", s, 3);
                String angle = s.toString();
                if (angle.equals("")) {
                    angle = "0";
                }
                float D13 = Float.parseFloat(angle);

                P3 =Math.abs( U13 * I13 * (float) Math.cos(D13 * Math.PI / 180));
                p3.setText(String.valueOf(P3));
                pt.setText(String.valueOf(P1 + P2 + P3));

            }
        });




     //   U1.setText("220.01");
        U2.setText("220.05");
        U3.setText("219.95");
        I1.setText("5.0004");
        I2.setText("4.9995");
        I3.setText("5.0001");
      //  U11 = Float.parseFloat(U1.getText().toString());
        U12 = Float.parseFloat(U2.getText().toString());
        U13 = Float.parseFloat(U3.getText().toString());
        I11 = Float.parseFloat(I1.getText().toString());
        I12 = Float.parseFloat(I2.getText().toString());
        I13 = Float.parseFloat(I3.getText().toString());


    }

    public void drawLineU(String name, Editable s, int i) {
        String angle = s.toString();
        if (angle.equals("")) {
            angle = "0";
        }
        if (Float.parseFloat(angle) > 360) {
            Toast.makeText(getContext(), "请输入合理的电压值！", Toast.LENGTH_SHORT).show();
            return;
        }
        float a = Float.parseFloat(angle);
        typeList.remove(i - 1);
        typeList.add(i - 1, new UType(name, a, i));
        clock.setA(typeList);
        clock.setIfPre(true);

    }

    public void drawLineI(String name, Editable s, int i) {
        String angle = s.toString();
        if (angle.equals("")) {
            angle = "0";
        }
        if (Float.parseFloat(angle) > 360) {
            Toast.makeText(getContext(), "角度越界！", Toast.LENGTH_SHORT).show();
            return;
        }
        float a = Float.parseFloat(angle);
        typeList1.remove(i - 1);
        typeList1.add(i - 1, new UType(name, a, i));
        clock.setB(typeList1);


    }

}
