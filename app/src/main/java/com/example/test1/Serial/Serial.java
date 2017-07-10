package com.example.test1.Serial;


public class Serial  {

    /**
     * A native method that is implemented by the 'native-lib' native library,
     * which is packaged with this application.
     */
    public native String stringFromJNI();



    ////
    public native int Open(int Port, int Rate);

    public native int Close();

    public native int[] Read();

    public native int Write(int[] buffer, int len);
}
