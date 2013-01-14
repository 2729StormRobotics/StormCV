package storm2013.smartdashboard;

import static com.googlecode.javacv.cpp.opencv_core.*;
import static com.googlecode.javacv.cpp.opencv_imgproc.*;
import edu.wpi.first.smartdashboard.camera.WPICameraExtension;
import edu.wpi.first.smartdashboard.camera.WPILaptopCameraExtension;
import edu.wpi.first.smartdashboard.properties.BooleanProperty;
import edu.wpi.first.smartdashboard.properties.IntegerProperty;
import edu.wpi.first.smartdashboard.properties.MultiProperty;
import edu.wpi.first.smartdashboard.properties.Property;
import edu.wpi.first.wpijavacv.StormCVUtil;
import edu.wpi.first.wpijavacv.WPIColorImage;
import edu.wpi.first.wpijavacv.WPIImage;

/**
 *
 * @author Joe
 */
public class StormCV extends WPILaptopCameraExtension {
//public class StormCV extends WPICameraExtension {
    public static final String NAME = "StormCV Target Tracker";
    
    IntegerProperty r0Property,r1Property,
                    g0Property,g1Property,
                    b0Property,b1Property;
    MultiProperty   processProperty;
    // Dummy objects representing steps of the process (for _processProperty)
    Object _process_nothing   = new Object(),
           _process_threshold = new Object();
    
    // Store these and update them in propertyChanged() because getValue()
    // has too much overhead to call it every time.
    private int _r0,_r1,
                _g0,_g1,
                _b0,_b1;
    private Object _process;
    
    // Keep temporaries around so they aren't constantly being reallocated
    private CvSize   _size;
    private IplImage _bin;
    private IplImage _redLow,  _redHigh;
    private IplImage _greenLow,_greenHigh;
    private IplImage _blueLow, _blueHigh;

    @Override
    public void init() {
        super.init();
        r0Property      = new IntegerProperty(this, "Low Red threshold",    0);
        r1Property      = new IntegerProperty(this, "High Red threshold",   255);
        g0Property      = new IntegerProperty(this, "Low Green threshold",  0);
        g1Property      = new IntegerProperty(this, "High Green threshold", 255);
        b0Property      = new IntegerProperty(this, "Low Blue threshold",   0);
        b1Property      = new IntegerProperty(this, "High Blue threshold",  255);
        processProperty = new MultiProperty  (this, "Process until?");
        
        processProperty.add("Nothing",         _process_nothing);
        processProperty.add("Apply Threshold", _process_threshold);
        
        processProperty.setDefault("Nothing");
        
        _r0 = r0Property.getValue();
        _r1 = r1Property.getValue();
        _g0 = g0Property.getValue();
        _g1 = g1Property.getValue();
        _b0 = b0Property.getValue();
        _b1 = b1Property.getValue();
        
        _process = processProperty.getValue();
    }

    @Override
    public void propertyChanged(Property property) {        
        if(property == r0Property) {
            _r0 = r0Property.getValue();
        } else if(property == r1Property) {
            _r1 = r1Property.getValue();
        } else if(property == g0Property) {
            _g0 = g0Property.getValue();
        } else if(property == g1Property) {
            _g1 = g1Property.getValue();
        } else if(property == b0Property) {
            _b0 = b0Property.getValue();
        } else if(property == b1Property) {
            _b1 = b1Property.getValue();
        } else if(property == processProperty) {
            _process = processProperty.getValue();
        }
    }    

    @Override
    public WPIImage processImage(WPIColorImage rawImage) {
        // If we aren't doing any processing, leave the image as-is
        if(_process == _process_nothing) {
            return rawImage;
        }
        
        // Reallocate temporaries if the size has changed
        if(_size == null || _size.width() != rawImage.getWidth() || _size.height() != rawImage.getHeight()) {
            _size = cvSize(rawImage.getWidth(),rawImage.getHeight());
            _bin = IplImage.create(_size, 8, 1);
            _redLow = IplImage.create(_size, 8, 1);
            _redHigh = IplImage.create(_size, 8, 1);
            _greenLow = IplImage.create(_size, 8, 1);
            _greenHigh = IplImage.create(_size, 8, 1);
            _blueLow = IplImage.create(_size, 8, 1);
            _blueHigh = IplImage.create(_size, 8, 1);
        }
        
        IplImage image = StormCVUtil.getIplImage(rawImage);
        
        // Split into individual color channels (from BGR)
        cvSplit(image, _blueLow, _greenLow, _redLow, null);
        
        // Apply thresholds
        // OpenCV can only do one-way threshold (less than or greater than
        // a specific value), so to have a ranged threshold, both thresholds
        // are performed then the results are ANDed together
        // The -1 is to make it an inclusive range (a >= n is equivalent to
        // a > n-1 for integers)
        cvThreshold(_redLow,   _redHigh,   _r0-1, 255, CV_THRESH_BINARY);
        cvThreshold(_redLow,   _redLow,    _r1,   255, CV_THRESH_BINARY_INV);
        
        cvThreshold(_greenLow, _greenHigh, _g0-1, 255, CV_THRESH_BINARY);
        cvThreshold(_greenLow, _greenLow,  _g1,   255, CV_THRESH_BINARY_INV);
        
        cvThreshold(_blueLow,  _blueHigh,  _b0-1, 255, CV_THRESH_BINARY);
        cvThreshold(_blueLow,  _blueLow,   _b1,   255, CV_THRESH_BINARY_INV);
        
        cvAnd(_redLow, _redHigh,   _bin, null);
        cvAnd(_bin,    _blueLow,   _bin, null);
        cvAnd(_bin,    _blueHigh,  _bin, null);
        cvAnd(_bin,    _greenLow,  _bin, null);
        cvAnd(_bin,    _greenHigh, _bin, null);
        
        // return thresholded image
        return StormCVUtil.makeWPIGrayscaleImage(_bin);
    }
}
