package storm2013.smartdashboard;

import com.googlecode.javacpp.Pointer;
import com.googlecode.javacv.CanvasFrame;
import static com.googlecode.javacv.cpp.opencv_core.*;
import static com.googlecode.javacv.cpp.opencv_imgproc.*;
import edu.wpi.first.smartdashboard.camera.WPICameraExtension;
import edu.wpi.first.smartdashboard.camera.WPILaptopCameraExtension;
import edu.wpi.first.smartdashboard.properties.*;
import edu.wpi.first.smartdashboard.robot.Robot;
import edu.wpi.first.wpijavacv.StormCVUtil;
import edu.wpi.first.wpijavacv.WPIColorImage;
import edu.wpi.first.wpijavacv.WPIImage;
import edu.wpi.first.wpilibj.tables.ITable;
import java.awt.Color;
import java.awt.Container;
import java.awt.Point;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.Scanner;
import javax.imageio.ImageIO;

/**
 *
 * @author Joe
 */
public class StormCV extends WPILaptopCameraExtension {
//public class StormCV extends WPICameraExtension {
    public static final String NAME = "StormCV Target Tracker";
    
    // Dummy objects representing steps of the process (for processProperty)
    private static final Object _process_nothing    = new Object(),
                                _process_threshold  = new Object(),
                                _process_closeHoles = new Object(),
                                _process_contours   = new Object(),
                                _process_convexHull = new Object(),
                                _process_select     = new Object(),
                                _process_calculate  = new Object(),
                                _process_all        = new Object();
    
    // Default settings. Once the values are tuned live with the different
    // properties, record them here so they stick around
    private static final int DEFAULT_R0 = 0,  DEFAULT_R1 = 100,
                             DEFAULT_G0 = 120,DEFAULT_G1 = 255,
                             DEFAULT_B0 = 80,DEFAULT_B1 = 200;
    private static final int DEFAULT_HOLECLOSING_ITERS = 5;
    private static final double DEFAULT_MIN_AREA_RATIO = 0.5/100;
    private static final Color DEFAULT_CONTOUR_COLOR = Color.red,
                               DEFAULT_LINE_COLOR    = Color.pink;
    
    private static final ITable outputTable = Robot.getTable();
    
    // Having aspects of the process editable as properties allows for quick
    // and easy tuning and testing.
    
    public final IntegerProperty 
        r0Property = new IntegerProperty(this, "Low Red threshold",    DEFAULT_R0),
        r1Property = new IntegerProperty(this, "High Red threshold",   DEFAULT_R1),
        g0Property = new IntegerProperty(this, "Low Green threshold",  DEFAULT_G0),
        g1Property = new IntegerProperty(this, "High Green threshold", DEFAULT_G1),
        b0Property = new IntegerProperty(this, "Low Blue threshold",   DEFAULT_B0),
        b1Property = new IntegerProperty(this, "High Blue threshold",  DEFAULT_B1);
        
    public final IntegerProperty
        holeClosingIterationsProperty = new IntegerProperty(this, "Hole Closing Iterations",  DEFAULT_HOLECLOSING_ITERS);
    
    public final DoubleProperty
        minAreaRatioProperty = new DoubleProperty(this,"Minimum contour/image area ratio",DEFAULT_MIN_AREA_RATIO);
        
    public final ColorProperty
        contourColorProperty = new ColorProperty(this, "Contour color", DEFAULT_CONTOUR_COLOR),
        lineColorProperty    = new ColorProperty(this, "Line color",    DEFAULT_LINE_COLOR);
        
    public final MultiProperty
        processProperty = new MultiProperty(this, "Process until?");
    
    
    // Store these and update them in propertyChanged() because getValue()
    // has too much overhead to be called every time.
    private int _r0,_r1,
                _g0,_g1,
                _b0,_b1;
    private int _holeClosingIterations;
    private double _minAreaRatio;
    private Color _contourColor,
                  _lineColor;
    private Object _process;
    
    private CvPoint2D32f _desiredLocNormed = new CvPoint2D32f(0,0);
    
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
            
    private boolean  _sendResults = true,
                     _displayIntermediate       = false;

    private void _initVars() {
        processProperty.add("Nothing",           _process_nothing);
        processProperty.add("Apply Threshold",   _process_threshold);
        processProperty.add("Close Holes",       _process_closeHoles);
        processProperty.add("Find Contours",     _process_contours);
        processProperty.add("Apply Convex Hull", _process_convexHull);
        processProperty.add("Select Contour",    _process_select);
        processProperty.add("Calculate data",    _process_calculate);
        processProperty.add("Everything",        _process_all);
        
        processProperty.setDefault("Everything");
        
        _r0 = r0Property.getValue();
        _r1 = r1Property.getValue();
        _g0 = g0Property.getValue();
        _g1 = g1Property.getValue();
        _b0 = b0Property.getValue();
        _b1 = b1Property.getValue();
        
        _holeClosingIterations = holeClosingIterationsProperty.getValue();
        
        _minAreaRatio = minAreaRatioProperty.getValue();
        
        _contourColor = contourColorProperty.getValue();
        _lineColor = lineColorProperty.getValue();
        
        _process = processProperty.getValue();
    }
    
    @Override
    public void init() {
        super.init();
        
        _initVars();
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
            _holeClosingIterations = holeClosingIterationsProperty.getValue();
        } else if(property == minAreaRatioProperty) {
            _minAreaRatio = minAreaRatioProperty.getValue();
        } else if(property == contourColorProperty) {
            _contourColor = contourColorProperty.getValue();
        } else if(property == lineColorProperty) {
            _lineColor = lineColorProperty.getValue();
        } else if(property == processProperty) {
            _process = processProperty.getValue();
        }
    }
    
    private void _sendData(boolean found,double x,double y) {
        if(_sendResults) {
            outputTable.putBoolean("targetFound", found);
            outputTable.putNumber ("targetX",     x);
            outputTable.putNumber ("targetY",     y);
        } else {
            if(found) {
                System.out.println("targetX: " + x);
                System.out.println("targetY: " + y);
            } else {
                System.out.println("Target not found");
            }
        }
    }

    @Override
    public WPIImage processImage(WPIColorImage rawImage) {
        if(_displayIntermediate) {
            _displayImage("Raw",StormCVUtil.getIplImage(rawImage));
        }
        
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
        
        if(_displayIntermediate) {
            _displayImage("Threshold",_bin);
        }
        
        if(_process != _process_threshold) {
            // Apply repeated dilations followed by repeated erosions in order
            // to close holes
            cvMorphologyEx(_bin, _bin, null, _morphology, CV_MOP_CLOSE, _holeClosingIterations);
        }
        
        if(_displayIntermediate) {
            _displayImage("Hole Closing",_bin);
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
            _storage = CvMemStorage.create();
        } else {
            cvClearMemStorage(_storage);
        }
        
        CvSeq contours = new CvSeq();
        
        // Detects any contours in _bin. CV_RETR_EXTERNAL makes it only find the
        // outer contours of a shape, CV_CHAIN_APPROX_TC89_KCOS uses "Teh-Chin
        // Chain Approximation" -- I have no idea what that means yet.
        cvFindContours(_bin, _storage, contours, 256, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_KCOS);
        
        if(contours == null || contours.isNull() || contours.total() == 0) {
            _sendData(false,0,0);
            return rawImage;
        }
        
        CvScalar color = CV_RGB(_contourColor.getRed(),_contourColor.getGreen(),_contourColor.getBlue());
        
        if(_displayIntermediate) {
            IplImage raw = StormCVUtil.getIplImage(rawImage);
            IplImage copy = IplImage.create(raw.cvSize(),raw.depth(),raw.nChannels());
            cvCopy(raw, copy);
            cvDrawContours(copy, contours, color, color, 1, 2, 8);
            
            _displayImage("Find Contours",copy);
            
            copy.deallocate();
        }
        
        if(_process == _process_contours) {
            cvDrawContours(StormCVUtil.getIplImage(rawImage), contours, color, color, 1, 2, 8);
            return rawImage;
        }
        
        ArrayList<CvSeq> convexContours = new ArrayList<>();
        while(contours != null && !contours.isNull()) {
            convexContours.add(cvConvexHull2(contours, _storage, CV_CLOCKWISE, 1));
            contours = contours.h_next();
        }
        
        if(_process == _process_convexHull) {
            IplImage target = StormCVUtil.getIplImage(rawImage);
            for(CvSeq contour: convexContours) {
                cvDrawContours(target,contour,color,color,0,2,8);
            }
            return rawImage;
        } else if(_displayIntermediate) {
            IplImage raw = StormCVUtil.getIplImage(rawImage);
            IplImage copy = IplImage.create(raw.cvSize(),raw.depth(),raw.nChannels());
            cvCopy(raw, copy);
            
            for(CvSeq contour: convexContours) {
                cvDrawContours(copy,contour,color,color,0,2,8);
            }
            _displayImage("Convex Hull", copy);
            
            copy.deallocate();
        }
        
        double maxArea = 0;
        int    maxIndex = -1;
        double minArea = rawImage.getWidth()*rawImage.getHeight()*_minAreaRatio;
        for(int i=0;i<convexContours.size();++i) {
            CvRect boundingRect = cvBoundingRect(convexContours.get(i), 1);
            double area = boundingRect.width()*boundingRect.height();
            if(area > minArea && (maxIndex == -1 || area > maxArea)) {
                maxIndex = i;
                maxArea = area;
            }
        }
        
        if(maxIndex == -1) {
            _sendData(false, 0, 0);
            return rawImage;
        }
        
        CvSeq selectedContour = convexContours.get(maxIndex);
        
        IplImage target = StormCVUtil.getIplImage(rawImage);
        cvDrawContours(target,selectedContour,color,color,0,2,8);
        
        if(_process != _process_convexHull) {
            double desiredXNormed = _desiredLocNormed.x(),
                   desiredYNormed = _desiredLocNormed.y();
            // desired location in screen pixels, for drawing purposes
            CvPoint desiredLoc = new CvPoint((int)((desiredXNormed+1)/2*rawImage.getWidth()),
                                             (int)((desiredYNormed+1)/2*rawImage.getHeight()));
            
            CvSeq polygon = cvApproxPoly(selectedContour,selectedContour.header_size(),_storage,CV_POLY_APPROX_DP,20,0);
            
            int totalPoints = polygon.total();
            if(totalPoints <= 0) {
                // Something is REALLY wrong
                System.err.println("No points in selected contour");
                return rawImage;
            }
            
            CvPoint points = new CvPoint(polygon.total());
            cvCvtSeqToArray(polygon, points, CV_WHOLE_SEQ);
            
            double centerX = 0,
                   centerY = 0;
            
            for(int i=0;i<totalPoints;++i) {
                CvPoint point = points.position(i);
                centerX += point.x();
                centerY += point.y();
            }
            
            centerX /= totalPoints;
            centerY /= totalPoints;
            
            cvLine(StormCVUtil.getIplImage(rawImage),
                   desiredLoc, 
                   new CvPoint((int)centerX,(int)centerY),
                   CV_RGB(_lineColor.getRed(),_lineColor.getGreen(),_lineColor.getBlue()),
                   2,8,0);
            
            double centerXNormed = centerX/rawImage.getWidth() *2-1,
                   centerYNormed = -(centerY/rawImage.getHeight()*2-1);
            
            double offsetX = centerXNormed-_desiredLocNormed.x(),
                   offsetY = centerYNormed-_desiredLocNormed.y();
            
            _sendData(true,offsetX,offsetY);
        }
        
        return rawImage;
    }
    
    private ArrayList<IplImage> _displayedImages = new ArrayList<>();
    private ArrayList<CanvasFrame> _frames = new ArrayList<>();
    private void _displayImage(String title,IplImage image) {
        IplImage newImage = IplImage.create(image.cvSize(), image.depth(), image.nChannels());
        cvCopy(image, newImage);
        _displayedImages.add(newImage);
        CanvasFrame result = new CanvasFrame(title);
        result.showImage(newImage.getBufferedImage());
        _frames.add(result);
    }
    
    private static void _deallocateIfNonNull(Pointer p) {
        if(p != null && !p.isNull()) {
            p.deallocate();
        }
    }

    @Override
    protected void finalize() throws Throwable {
        super.finalize();
        _deallocateIfNonNull(_size);
        _deallocateIfNonNull(_bin);
        _deallocateIfNonNull(_redLow);
        _deallocateIfNonNull(_redHigh);
        _deallocateIfNonNull(_greenLow);
        _deallocateIfNonNull(_greenHigh);
        _deallocateIfNonNull(_blueLow);
        _deallocateIfNonNull(_blueHigh);
        for(IplImage image:_displayedImages) {
            _deallocateIfNonNull(image);
        }
        for(CanvasFrame frame:_frames) {
            if(frame.isVisible()) {
                frame.setVisible(false);
                frame.dispose();
            }
        }
    }
    
    public static void main(String[] args) {
        boolean showUsage = (args.length == 0);
        boolean flagShow  = false;
        if(!showUsage && args[0].equals("--show")) {
            if(args.length == 1) {
                showUsage = true;
                flagShow  = true;
            }
        }
        
        if(showUsage) {
            System.out.println("Usage: [--show] [FILE1] ... [FILEN]");
            return;
        }
        
        StormCV cv = new StormCV();
        
        cv._sendResults = false;
        
        cv._displayIntermediate = flagShow;
        cv._initVars();
        
        Scanner scanner = new Scanner(System.in);
        
        int start = flagShow ? 1 : 0;
        
        for(int i=start;i<args.length;++i) {
            String filename = args[i];
            
            System.out.println(filename + ":");
            
            WPIColorImage rawImage;
            try {
                rawImage = new WPIColorImage(ImageIO.read(new File(filename)));
            } catch (IOException e) {
                System.err.println("Could not find file!");
                return;
            }
            
            WPIImage result;
            
            long startTime = System.nanoTime();
            
            result = cv.processImage(rawImage);
            
            long totalTime = System.nanoTime()-startTime;
            
            cv._displayImage("Result", StormCVUtil.getIplImage(result));
            
            System.out.format("Processed in %f ms\n",totalTime/1.0e6);
            
            System.out.println("Press Enter to Continue...");
            scanner.nextLine();
        }
        System.out.println("Done!");
        System.out.println("Press Enter to Continue...");
        scanner.nextLine();
        System.exit(0);
    }
}
