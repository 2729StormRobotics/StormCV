/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpijavacv;

import com.googlecode.javacv.cpp.opencv_core;
import com.googlecode.javacv.cpp.opencv_core.CvSeq;
import com.googlecode.javacv.cpp.opencv_core.IplImage;

/**
 *
 * @author Joe
 */
public class StormCVUtil {
    public static IplImage getIplImage(WPIImage image)
    {
        return image.image;
    }
    
    public static CvSeq getCvSeq(WPIContour contour)
    {
        return contour.getCVSeq();
    }

    public static WPIContour makeWPIContour(opencv_core.CvSeq seq)
    {
        return new WPIContour(seq);
    }

    public static WPIGrayscaleImage makeWPIGrayscaleImage(opencv_core.IplImage arr)
    {
        opencv_core.IplImage tempImage = opencv_core.IplImage.create(arr.cvSize(), arr.depth(), 1);
        opencv_core.cvCopy(arr, tempImage);
        return new WPIGrayscaleImage(tempImage);
    }
}
