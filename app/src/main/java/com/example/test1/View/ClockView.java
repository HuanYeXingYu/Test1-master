package com.example.test1.View;

import android.content.Context;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.RectF;
import android.os.Handler;
import android.os.Message;
import android.util.AttributeSet;
import android.util.Log;
import android.view.View;
import android.widget.Toast;


import com.example.test1.Entity.UType;
import com.example.test1.R;

import java.util.Calendar;
import java.util.List;

public class ClockView extends View {
    //View默认最小宽度
    private static final int DEFAULT_MIN_WIDTH = 200;
    //秒针长度
    private float secondPointerLength;
    //分针长度
    private float minutePointerLength;
    //时针长度
    private float hourPointerLength;
    //圆半径
    private float rPointerLength;
    //外圆边框宽度
    private static final float DEFAULT_BORDER_WIDTH = 3f;
    //指针反向超过圆点的长度
    private static final float DEFAULT_POINT_BACK_LENGTH = 3f;
    //长刻度线
    private static final float DEFAULT_LONG_DEGREE_LENGTH = 20f;
    //短刻度线
    private static final float DEFAULT_SHORT_DEGREE_LENGTH = 0f;

    private Thread timeThread = new Thread() {
        @Override
        public void run() {

            try {
                while (true) {

                    updateHandler.sendEmptyMessage(0);
                    Thread.sleep(1000);

                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    };

    private Handler updateHandler = new Handler() {
        @Override
        public void handleMessage(Message msg) {

            invalidate();
        }
    };


    public ClockView(Context context) {
        super(context);

        init();
    }

    public ClockView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    public ClockView(Context context, AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        init();
    }

    //启动线程让指针动起来
    private void init() {
        timeThread.setDaemon(true);
        timeThread.start();
    }

    /**
     * 计算时针、分针、秒针的长度
     */
    private void reset() {
        float r = (Math.min(getHeight() / 2, getWidth() / 2) - DEFAULT_BORDER_WIDTH / 2);
        secondPointerLength = r * 0.7f;
        minutePointerLength = r * 0.5f;
        hourPointerLength = r * 0.4f;
        rPointerLength = r;
    }

    /**
     * 根据角度和长度计算线段的起点和终点的坐标
     *
     * @param angle
     * @param length
     * @return
     */
    private float[] calculatePoint(float angle, float length) {
        float[] points = new float[4];
        if (angle <= 90f) {
            points[0] = -(float) Math.sin(angle * Math.PI / 180) * DEFAULT_POINT_BACK_LENGTH;
            points[1] = (float) Math.cos(angle * Math.PI / 180) * DEFAULT_POINT_BACK_LENGTH;
            points[2] = (float) Math.sin(angle * Math.PI / 180) * length;
            points[3] = -(float) Math.cos(angle * Math.PI / 180) * length;
        } else if (angle <= 180f) {
            points[0] = -(float) Math.cos((angle - 90) * Math.PI / 180) * DEFAULT_POINT_BACK_LENGTH;
            points[1] = -(float) Math.sin((angle - 90) * Math.PI / 180) * DEFAULT_POINT_BACK_LENGTH;
            points[2] = (float) Math.cos((angle - 90) * Math.PI / 180) * length;
            points[3] = (float) Math.sin((angle - 90) * Math.PI / 180) * length;
        } else if (angle <= 270f) {
            points[0] = (float) Math.sin((angle - 180) * Math.PI / 180) * DEFAULT_POINT_BACK_LENGTH;
            points[1] = -(float) Math.cos((angle - 180) * Math.PI / 180) * DEFAULT_POINT_BACK_LENGTH;
            points[2] = -(float) Math.sin((angle - 180) * Math.PI / 180) * length;
            points[3] = (float) Math.cos((angle - 180) * Math.PI / 180) * length;
        } else if (angle <= 360f) {
            points[0] = (float) Math.cos((angle - 270) * Math.PI / 180) * DEFAULT_POINT_BACK_LENGTH;
            points[1] = (float) Math.sin((angle - 270) * Math.PI / 180) * DEFAULT_POINT_BACK_LENGTH;
            points[2] = -(float) Math.cos((angle - 270) * Math.PI / 180) * length;
            points[3] = -(float) Math.sin((angle - 270) * Math.PI / 180) * length;
        }
        return points;
    }

    Canvas canvas;
    Calendar now;
    float r;

    public float getR() {
        return r;
    }

    public void setR(float r) {
        this.r = r;
    }

    @Override
    protected void onDraw(Canvas canvas) {
        this.canvas = canvas;
        reset();
        //画外圆
        float borderWidth = DEFAULT_BORDER_WIDTH;

        float r = Math.min(getHeight() / 2, getWidth() / 2) - borderWidth / 2;
        setR(r);
        Paint paintCircle = new Paint();
        paintCircle.setStyle(Paint.Style.STROKE);
        paintCircle.setAntiAlias(true);
        paintCircle.setStrokeWidth(borderWidth);
        canvas.drawCircle(getWidth() / 2, getHeight() / 2, r, paintCircle);

        //画刻度线
        float degreeLength = 0f;
        Paint paintDegree = new Paint();
        paintDegree.setAntiAlias(true);
        for (int i = 0; i < 60; i++) {
            if (i % 5 == 0) {
                paintDegree.setStrokeWidth(2);
                degreeLength = DEFAULT_LONG_DEGREE_LENGTH;
            } else {
                paintDegree.setStrokeWidth(3);
                degreeLength = DEFAULT_SHORT_DEGREE_LENGTH;
            }
            canvas.drawLine(getWidth() / 2, Math.abs(getHeight() / 2 - r), getWidth() / 2, Math.abs(getHeight() / 2 - r) + degreeLength, paintDegree);
            canvas.rotate(360 / 60, getWidth() / 2, getHeight() / 2);
        }

        //刻度数字
        int degressNumberSize = 30;
        canvas.translate(getWidth() / 2, getHeight() / 2);
        Paint paintDegreeNumber = new Paint();
        paintDegreeNumber.setTextAlign(Paint.Align.CENTER);
        paintDegreeNumber.setTextSize(degressNumberSize - 10);
        paintDegreeNumber.setFakeBoldText(true);
//        for(int i=0;i<12;i++){
//            float[] temp = calculatePoint((i+1)*30, r - DEFAULT_LONG_DEGREE_LENGTH - degressNumberSize/2 - 15);
//            canvas.drawText((i+1)+"", temp[2], temp[3] + degressNumberSize/2-6, paintDegreeNumber);
//        }

        //画指针
        Paint paintHour = new Paint();
        float[] hourPoints, temp, PointsStart, PointMid, PointsEnd;
        Path path = new Path();
        Paint paintHour2 = new Paint();
        paintHour2.setStyle(Paint.Style.STROKE);
        paintHour2.setStrokeWidth(3);
        paintHour2.setAntiAlias(true);
        paintHour2.setColor(getResources().getColor(R.color.purple));
        paintHour.setAntiAlias(true);
        paintHour.setStrokeWidth(3);
        paintHour.setColor(getResources().getColor(R.color.gry));
        hourPoints = calculatePoint(0, rPointerLength);
        canvas.drawLine(hourPoints[0], hourPoints[1], hourPoints[2], hourPoints[3], paintHour);


        //刻画标识
        temp = calculatePoint(0, r - degressNumberSize / 2 - 32);
        canvas.drawText("U1", temp[2] - 20, temp[3] + degressNumberSize / 2 - 6, paintDegreeNumber);
        drawArrow(canvas, hourPoints[2], hourPoints[3], 0, rPointerLength, 3, R.color.gry, 2);
        //I
        hourPoints = calculatePoint(60, secondPointerLength);
        canvas.drawLine(hourPoints[0], hourPoints[1], hourPoints[2], hourPoints[3], paintHour);
        //刻画标识I
        temp = calculatePoint(60, r - degressNumberSize / 2 - 32);
        canvas.drawText("I1", temp[2] - 20, temp[3] + degressNumberSize / 2 - 6, paintDegreeNumber);
        //划线
        PointsStart = calculatePoint(0, r * 0.3f);
        PointMid = calculatePoint(30, r * 0.3f);
        PointsEnd = calculatePoint(60, r * 0.3f);
        path.moveTo(PointsStart[2], PointsStart[3]);
        path.quadTo(PointMid[2] * 1.2f, PointMid[3] * 1.2f, PointsEnd[2], PointsEnd[3]);
        canvas.drawPath(path, paintHour2);
        //画箭头
//        hourPoints = calculatePoint(45, 0.1f * rPointerLength);
//        canvas.drawLine(hourPoints[0], hourPoints[1], hourPoints[2], hourPoints[3], paintHour);
        drawArrow(canvas, hourPoints[2], hourPoints[3], 60, rPointerLength, 3, R.color.gry, 1);
        drawArrow(canvas,PointsStart[2], PointsStart[3], 0, rPointerLength, 3, R.color.gry);

        paintHour.setColor(getResources().getColor(R.color.green));
        hourPoints = calculatePoint(120, rPointerLength);
        canvas.drawLine(hourPoints[0], hourPoints[1], hourPoints[2], hourPoints[3], paintHour);
        //刻画标识
        temp = calculatePoint(120, r - degressNumberSize / 2 - 32);
        canvas.drawText("U2", temp[2] - 20, temp[3] + degressNumberSize / 2 - 6, paintDegreeNumber);
        drawArrow(canvas, hourPoints[2], hourPoints[3], 120, rPointerLength, 3, R.color.green, 2);
        //I
        hourPoints = calculatePoint(180, secondPointerLength);
        canvas.drawLine(hourPoints[0], hourPoints[1], hourPoints[2], hourPoints[3], paintHour);
        //刻画标识I
        temp = calculatePoint(180, r - degressNumberSize / 2 - 32);
        canvas.drawText("I2", temp[2] - 20, temp[3] + degressNumberSize / 2 - 6, paintDegreeNumber);
        //划线
        PointsStart = calculatePoint(120, r * 0.3f);
        PointMid = calculatePoint(150, r * 0.3f);
        PointsEnd = calculatePoint(180, r * 0.3f);
        path.moveTo(PointsStart[2], PointsStart[3]);
        path.quadTo(PointMid[2] * 1.2f, PointMid[3] * 1.2f, PointsEnd[2], PointsEnd[3]);
        canvas.drawPath(path, paintHour2);
        drawArrow(canvas, hourPoints[2], hourPoints[3], 180, rPointerLength, 3, R.color.green, 1);
        drawArrow(canvas,PointsStart[2], PointsStart[3],120, rPointerLength, 3, R.color.green);

        paintHour.setColor(getResources().getColor(R.color.red));
        hourPoints = calculatePoint(240, rPointerLength);
        canvas.drawLine(hourPoints[0], hourPoints[1], hourPoints[2], hourPoints[3], paintHour);
        //刻画标识
        temp = calculatePoint(240, r - degressNumberSize / 2 - 32);
        canvas.drawText("U3", temp[2] - 20, temp[3] + degressNumberSize / 2 - 6, paintDegreeNumber);
        drawArrow(canvas, hourPoints[2], hourPoints[3], 240, rPointerLength, 3, R.color.red, 2);
        //I
        hourPoints = calculatePoint(300, secondPointerLength);
        canvas.drawLine(hourPoints[0], hourPoints[1], hourPoints[2], hourPoints[3], paintHour);
        //刻画标识I
        temp = calculatePoint(300, r - degressNumberSize / 2 - 32);
        canvas.drawText("I3", temp[2] - 20, temp[3] + degressNumberSize / 2 - 6, paintDegreeNumber);
        //划线
        PointsStart = calculatePoint(240, r * 0.3f);
        PointMid = calculatePoint(270, r * 0.3f);
        PointsEnd = calculatePoint(300, r * 0.3f);
        path.moveTo(PointsStart[2], PointsStart[3]);
        path.quadTo(PointMid[2] * 1.2f, PointMid[3] * 1.2f, PointsEnd[2], PointsEnd[3]);
        canvas.drawPath(path, paintHour2);
        drawArrow(canvas, hourPoints[2], hourPoints[3], 300, rPointerLength, 3, R.color.red, 1);
        drawArrow(canvas,PointsStart[2], PointsStart[3],240, rPointerLength, 3, R.color.red);
        //画圆心
        Paint paintCenter = new Paint();
        paintCenter.setColor(Color.WHITE);
        canvas.drawCircle(0, 0, 2, paintCenter);
    }

    /**
     * 当布局为wrap_content时设置默认长宽
     *
     * @param widthMeasureSpec
     * @param heightMeasureSpec
     */
    @Override
    protected void onMeasure(int widthMeasureSpec, int heightMeasureSpec) {
        setMeasuredDimension(measure(widthMeasureSpec), measure(heightMeasureSpec));

    }

    private int measure(int origin) {

        int result = DEFAULT_MIN_WIDTH;
        int specMode = MeasureSpec.getMode(origin);
        int specSize = MeasureSpec.getSize(origin);
        if (specMode == MeasureSpec.EXACTLY) {
            result = specSize;
        } else {
            if (specMode == MeasureSpec.AT_MOST) {
                result = Math.min(result, specSize);
            }
        }
        return result;
    }


    private List<UType> a;
    private List<UType> b;
    private boolean ifPre = false;


    public boolean isIfPre() {
        return ifPre;
    }

    public void setIfPre(boolean ifPre) {
        this.ifPre = ifPre;
    }

    public List<UType> getA() {
        return a;
    }

    public List<UType> getB() {
        return b;
    }

    public void setB(List<UType> b) {
        this.b = b;
    }

    public void setA(List<UType> a) {

        this.a = a;


    }

    //    ctx ：Canvas绘图环境
//    fromX, fromY ：起点坐标（也可以换成 p1 ，只不过它是一个数组）
//    toX, toY ：终点坐标 (也可以换成 p2 ，只不过它是一个数组)
//    theta ：三角斜边一直线夹角
//    headlen ：三角斜边长度
//    width ：箭头线宽度
//    color ：箭头颜色

    /**
     * @param canvs
     * @param toX
     * @param toY
     * @param toAngle 相对边角度
     * @param r       圆半径
     * @param width   线宽
     * @param color
     */
    void drawArrow(Canvas canvs, float toX, float toY, int toAngle, float r, int width, int color, int id) {
        Paint paintRow = new Paint();
        paintRow.setAntiAlias(true);
        paintRow.setStrokeWidth(width);
        paintRow.setColor(getResources().getColor(color));
        if (id == 1) {
            float[] arrowPoints = calculatePoint(toAngle - 3, 0.6f * r);
            canvs.drawLine(toX, toY, arrowPoints[2], arrowPoints[3], paintRow);
            arrowPoints = calculatePoint(toAngle + 3, 0.6f * r);
            canvs.drawLine(toX, toY, arrowPoints[2], arrowPoints[3], paintRow);
        } else {
            float[] arrowPoints = calculatePoint(toAngle - 2, 0.9f * r);
            canvs.drawLine(toX, toY, arrowPoints[2], arrowPoints[3], paintRow);
            arrowPoints = calculatePoint(toAngle + 2, 0.9f * r);
            canvs.drawLine(toX, toY, arrowPoints[2], arrowPoints[3], paintRow);
        }


    }

    void drawArrow(Canvas canvs, float toX, float toY, int toAngle, float r, int width, int color) {
        Paint paintRow = new Paint();
        paintRow.setAntiAlias(true);
        paintRow.setStrokeWidth(width);
        paintRow.setColor(getResources().getColor(color));
        float[] arrowPoints = calculatePoint(toAngle+10, 0.34f * r);
        canvs.drawLine(toX, toY, arrowPoints[2], arrowPoints[3], paintRow);
        arrowPoints = calculatePoint(toAngle +12, 0.27f * r);
        canvs.drawLine(toX, toY, arrowPoints[2], arrowPoints[3], paintRow);

    }
}

