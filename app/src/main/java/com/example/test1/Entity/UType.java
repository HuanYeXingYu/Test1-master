package com.example.test1.Entity;

/**
 * Created by 夜墨 on 2017/5/4.
 */

public class UType {
    private  String text;
    private  float a;
    private  int id;

    public int getId() {
        return id;
    }

    public void setId(int id) {
        this.id = id;
    }

    public String getText() {
        return text;
    }

    public void setText(String text) {
        this.text = text;
    }

    public float getA() {
        return a;
    }

    public UType(String text, float a,int id) {
        this.id=id;
        this.text = text;
        this.a = a;
    }

    public void setA(float a) {
        this.a = a;
    }

    @Override
    public String toString() {
        return "avcd{" +
                "text='" + text + '\'' +
                ", a=" + a +
                '}';
    }
}
