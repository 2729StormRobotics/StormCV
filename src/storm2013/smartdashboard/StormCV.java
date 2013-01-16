package storm2013.smartdashboard;

import com.googlecode.javacpp.Pointer;
import static com.googlecode.javacv.cpp.opencv_core.*;
import static com.googlecode.javacv.cpp.opencv_imgproc.*;
import edu.wpi.first.smartdashboard.camera.WPICameraExtension;
import edu.wpi.first.smartdashboard.camera.WPILaptopCameraExtension;
import edu.wpi.first.smartdashboard.properties.IntegerProperty;
import edu.wpi.first.smartdashboard.properties.MultiProperty;
import edu.wpi.first.smartdashboard.properties.Property;
import edu.wpi.first.wpijavacv.StormCVUtil;
import edu.wpi.first.wpijavacv.WPIColorImage;
import edu.wpi.first.wpijavacv.WPIImage;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 *
 * @author Joe
 */
public class StormCV extends WPILaptopCameraExtension {
//public class StormCV extends WPICameraExtension {
    public static final String NAME = "StormCV Target Tracker";
    
    // Dummy objects representing steps of the process (for processProperty)
    private static final Object _process_nothing     = new Object(),
                                _process_threshold   = new Object(),
                                _process_closeHoles  = new Object(),
                                _process_contours    = new Object();
    
    // Default settings. Once the values are tuned live with the different
    // properties, record them here so they stick around
    private static final int DEFAULT_R0 = 0,DEFAULT_R1 = 255,
                             DEFAULT_G0 = 0,DEFAULT_G1 = 255,
                             DEFAULT_B0 = 0,DEFAULT_B1 = 255;
    private static final int DEFAULT_HOLECLOSING_ITERS = 2;
    
    // Having aspects of the process editable as properties allows for quick
    // and easy tuning and testing.
    IntegerProperty r0Property,r1Property,
                    g0Property,g1Property,
                    b0Property,b1Property;
    IntegerProperty holeClosingIterationsProperty;
    MultiProperty   processProperty;
    
    
    // Store these and update them in propertyChanged() because getValue()
    // has too much overhead to be called every time.
    private int _r0,_r1,
                _g0,_g1,
                _b0,_b1;
    private int holeClosingIterations;
    private Object _process;
    
    // This holds the image returned from processImage() (if the selected
    // processing mode replaces the rawImage instead of drawing on top of
    // it). It prevents SmartDashboard from crashing without any indication 
    // whatsoever.
    private WPIImage _ret;
    
    // Keep temporaries around so they aren't constantly being reallocated
    private CvSize   _size;
    private IplImage _bin;
    private IplImage _redLow,  _redHigh;
    private IplImage _greenLow,_greenHigh;
    private IplImage _blueLow, _blueHigh;
    private IplConvKernel _morphology = IplConvKernel.create(3, 3, 1, 1, CV_SHAPE_RECT, null);;
    private CvMemStorage _storage;

    @Override
    public void init() {
        try {
            // Since SmartDashboard apparently doesn't provide any output by default
            System.setOut(new PrintStream("stdout.txt"));
            System.setErr(new PrintStream("stderr.txt"));
        } catch (FileNotFoundException ex) {
            Logger.getLogger(StormCV.class.getName()).log(Level.SEVERE, null, ex);
        }
        super.init();
        r0Property      = new IntegerProperty(this, "Low Red threshold",    DEFAULT_R0);
        r1Property      = new IntegerProperty(this, "High Red threshold",   DEFAULT_R1);
        g0Property      = new IntegerProperty(this, "Low Green threshold",  DEFAULT_G0);
        g1Property      = new IntegerProperty(this, "High Green threshold", DEFAULT_G1);
        b0Property      = new IntegerProperty(this, "Low Blue threshold",   DEFAULT_B0);
        b1Property      = new IntegerProperty(this, "High Blue threshold",  DEFAULT_B1);
        
        holeClosingIterationsProperty = new IntegerProperty(this, "Hole Closing Iterations",  DEFAULT_HOLECLOSING_ITERS);
        
        processProperty = new MultiProperty  (this, "Process until?");
        
        processProperty.add("Nothing",         _process_nothing);
        processProperty.add("Apply Threshold", _process_threshold);
        processProperty.add("Close Holes",     _process_closeHoles);
        processProperty.add("Find Contours",   _process_contours);
        
        processProperty.setDefault("Nothing");
        
        _r0 = r0Property.getValue();
        _r1 = r1Property.getValue();
        _g0 = g0Property.getValue();
        _g1 = g1Property.getValue();
        _b0 = b0Property.getValue();
        _b1 = b1Property.getValue();
        
        holeClosingIterations = holeClosingIterationsProperty.getValue();
        
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
        } else if(property == holeClosingIterationsProperty) {
            holeClosingIterations = holeClosingIterationsProperty.getValue();
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
            _size      = cvSize(rawImage.getWidth(),rawImage.getHeight());
            _bin       = IplImage.create(_size, 8, 1);
            _redLow    = IplImage.create(_size, 8, 1);
            _redHigh   = IplImage.create(_size, 8, 1);
            _greenLow  = IplImage.create(_size, 8, 1);
            _greenHigh = IplImage.create(_size, 8, 1);
            _blueLow   = IplImage.create(_size, 8, 1);
            _blueHigh  = IplImage.create(_size, 8, 1);
        }
        
        // Extract the IplImage so we can do OpenCV magic.
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
        
        // ANDing the images leaves only the pixels within all of the ranges
        cvAnd(_redLow, _redHigh,   _bin, null);
        cvAnd(_bin,    _blueLow,   _bin, null);
        cvAnd(_bin,    _blueHigh,  _bin, null);
        cvAnd(_bin,    _greenLow,  _bin, null);
        cvAnd(_bin,    _greenHigh, _bin, null);
        
        if(_process == _process_closeHoles) {
            cvMorphologyEx(_bin, _bin, null, _morphology, CV_MOP_CLOSE, holeClosingIterations);
        }
        
        if(_process == _process_threshold || _process == _process_closeHoles) {
            // Allocate _ret if it's the first time, otherwise reuse it.
            if(_ret == null) {
                _ret = StormCVUtil.makeWPIGrayscaleImage(_bin);
            } else {
                StormCVUtil.copyImage(_ret, _bin);
            }
            
            return _ret;
        }
        
        if(_storage == null) {
//            _storage = CvMemStorage.create();
        }
        
        return rawImage;
    }
    
    private static void deallocIfNull(Pointer p) {
        if(p != null && !p.isNull()) {
            p.deallocate();
        }
    }

    @Override
    protected void finalize() throws Throwable {
        super.finalize();
        deallocIfNull(_size);
        deallocIfNull(_bin);
        deallocIfNull(_redLow);
        deallocIfNull(_redHigh);
        deallocIfNull(_greenLow);
        deallocIfNull(_greenHigh);
        deallocIfNull(_blueLow);
        deallocIfNull(_blueHigh);
    }
    
    public static void main(String[] args) {
        if(args.length == 0) {
            System.out.println("Usage: [FILE1] ... [FILEN]");
        }
        
    }
}
