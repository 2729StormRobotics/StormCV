package edu.wpi.first.wpijavacv;

import static com.googlecode.javacv.cpp.opencv_core.*;

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

    public static WPIContour makeWPIContour(CvSeq seq)
    {
        return new WPIContour(seq);
    }

    public static WPIGrayscaleImage makeWPIGrayscaleImage(IplImage arr)
    {
        IplImage tempImage = IplImage.create(arr.cvSize(), arr.depth(), 1);
        cvCopy(arr, tempImage);
        return new WPIGrayscaleImage(tempImage);
    }

    public static WPIColorImage makeWPIColorImage(IplImage arr)
    {
        IplImage tempImage = IplImage.create(arr.cvSize(), arr.depth(), 1);
        cvCopy(arr, tempImage);
        return new WPIColorImage(tempImage);
    }
    
    public static void copyImage(WPIImage out,IplImage image) {
        boolean allocateNew = false;
        if(out.image == null || out.image.depth() != image.depth()) {
            allocateNew = true;
        } else {
            CvSize outSize = out.image.cvSize(),
                   imgSize = image.cvSize();
            if(outSize.width() != imgSize.width() || outSize.height() != imgSize.height()) {
                allocateNew = true;
            }
        }
        if(allocateNew) {
            System.out.println("Allocating new");
            out.image = IplImage.create(image.cvSize(), image.depth(), 1);
        }
        cvCopy(image,out.image);
    }
    
    public static WPIImage dupImage(WPIImage in) {
        IplImage newImage = IplImage.create(in.image.cvSize(), in.image.depth(), 1);
        cvCopy(in.image, newImage);
        return new WPIImage(newImage);
    }
}
