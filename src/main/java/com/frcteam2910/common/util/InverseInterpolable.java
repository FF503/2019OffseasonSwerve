package com.frcteam2910.common.util;

public interface InverseInterpolable<T> {
    double inverseInterpolate(T upper, T query);
}
