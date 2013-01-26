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
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Scanner;
import javax.imageio.ImageIO;

/**
 * This project depends on the following files from the SmartDashboard 
 * installation:
 *     SmartDashboard.jar
 *     extensions/WPICameraExtension.jar
 *     lib/NetworkTable_Client.jar
 *     extensions/lib/javacpp.jar
 *     extensions/lib/javacv-<environment>.jar (eg. javacv-windows-x86.jar)
 *     extensions/lib/javacv.jar
 *     extensions/lib/WPIJavaCV.jar
 * @author Joe
 */
//public class StormCV extends WPILaptopCameraExtension {
public class StormCV extends WPICameraExtension {
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
    
    private static final Object _select_biggest     = new Object(),
                                _select_closest     = new Object(),
                                _select_closest_3pt = new Object(),
                                _select_closest_2pt = new Object();
        
    private static final ITable outputTable = Robot.getTable();
    
    // Having aspects of the process editable as properties allows for quick
    // and easy tuning and testing.
    public final IntegerProperty 
        h0Property = new IntegerProperty(this, "Low Hue threshold",        60),
        h1Property = new IntegerProperty(this, "High Hue threshold",       90),
        s0Property = new IntegerProperty(this, "Low Saturation threshold", 70),
        s1Property = new IntegerProperty(this, "High Saturation threshold",255),
        v0Property = new IntegerProperty(this, "Low Value threshold",      90),
        v1Property = new IntegerProperty(this, "High Value threshold",     255);
        
    public final IntegerProperty
        holeClosingIterationsProperty = new IntegerProperty(this, "Hole Closing Iterations",9);
    
    public final DoubleProperty
        minAreaRatioProperty = new DoubleProperty(this,"Minimum contour/image area ratio",0.5/100);
    public final DoubleProperty
        min3ptAspectRatioProperty = new DoubleProperty(this,"Minimum 3-pt aspect ratio",4),
        max3ptAspectRatioProperty = new DoubleProperty(this,"Maximum 3-pt aspect ratio",5),
        min2ptAspectRatioProperty = new DoubleProperty(this,"Minimum 2-pt aspect ratio",2),
        max2ptAspectRatioProperty = new DoubleProperty(this,"Maximum 2-pt aspect ratio",3);
    public final DoubleProperty
        nearHorizAngleProperty = new DoubleProperty(this,"Nearly horizontal angle",20),
        nearVertAngleProperty  = new DoubleProperty(this,"Nearly vertical angle",  70);
        
    public final ColorProperty
        contourColorProperty = new ColorProperty(this, "Contour color", Color.red),
        lineColorProperty    = new ColorProperty(this, "Line color",    Color.pink);
        
    public final MultiProperty
        processProperty = new MultiProperty(this, "Process until?"),
        selectProperty  = new MultiProperty(this, "Select for?");
    
    
    // Store these and update them in propertyChanged() because getValue()
    // has too much overhead to be called every time.
    private int _h0,_h1,
                _s0,_s1,
                _v0,_v1;
    private int _holeClosingIterations;
    private double _minAreaRatio;
    private double _min3ptAspectRatio,
                   _max3ptAspectRatio,
                   _min2ptAspectRatio,
                   _max2ptAspectRatio;
    private double _nearHorizSlope,
                   _nearVertSlope;
    private Color _contourColor,
                  _lineColor;
    private Object _process,
                   _select;
    
    private CvPoint2D32f _desiredLocNormed = new CvPoint2D32f(0,0);
    
    // This holds the image returned from processImage() (if the selected
    // processing mode replaces the rawImage instead of drawing on top of
    // it). It prevents SmartDashboard from crashing without any indication 
    // whatsoever.
    private WPIImage _ret;
    
    // Keep temporaries around so they aren't constantly being reallocated
    private CvSize   _size;
    private IplImage _hsv;
    private IplImage _bin;
    private IplImage _hueLow, _hueHigh;
    private IplImage _satLow, _satHigh;
    private IplImage _valLow, _valHigh;
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
        
        selectProperty.add("Largest",    _select_biggest);
        selectProperty.add("Closest",    _select_closest);
        selectProperty.add("Closest 3pt",_select_closest_3pt);
        selectProperty.add("Closest 2pt",_select_closest_2pt);
        
        selectProperty.setDefault("Largest");
        
        _h0 = h0Property.getValue();
        _h1 = h1Property.getValue();
        _s0 = s0Property.getValue();
        _s1 = s1Property.getValue();
        _v0 = v0Property.getValue();
        _v1 = v1Property.getValue();
        
        _holeClosingIterations = holeClosingIterationsProperty.getValue();
        
        _minAreaRatio = minAreaRatioProperty.getValue();
        
        _min3ptAspectRatio = min3ptAspectRatioProperty.getValue();
        _max3ptAspectRatio = max3ptAspectRatioProperty.getValue();
        _min2ptAspectRatio = min2ptAspectRatioProperty.getValue();
        _max2ptAspectRatio = max2ptAspectRatioProperty.getValue();
        
        _nearHorizSlope = Math.tan(Math.toRadians(nearHorizAngleProperty.getValue()));
        _nearVertSlope = Math.tan(Math.toRadians(nearVertAngleProperty.getValue()));

        _contourColor = contourColorProperty.getValue();
        _lineColor = lineColorProperty.getValue();
        
        _process = processProperty.getValue();
        _select  = selectProperty.getValue();
    }
    
    @Override
    public void init() {
        super.init();
        
        _initVars();
    }

    @Override
    public void propertyChanged(Property property) {        
        if(property == h0Property) {
            _h0 = h0Property.getValue();
        } else if(property == h1Property) {
            _h1 = h1Property.getValue();
        } else if(property == s0Property) {
            _s0 = s0Property.getValue();
        } else if(property == s1Property) {
            _s1 = s1Property.getValue();
        } else if(property == v0Property) {
            _v0 = v0Property.getValue();
        } else if(property == v1Property) {
            _v1 = v1Property.getValue();
        } else if(property == holeClosingIterationsProperty) {
            _holeClosingIterations = holeClosingIterationsProperty.getValue();
        } else if(property == minAreaRatioProperty) {
            _minAreaRatio = minAreaRatioProperty.getValue();
        } else if(property == min3ptAspectRatioProperty) {
            _min3ptAspectRatio = min3ptAspectRatioProperty.getValue();
        } else if(property == max3ptAspectRatioProperty) {
            _max3ptAspectRatio = max3ptAspectRatioProperty.getValue();
        } else if(property == min2ptAspectRatioProperty) {
            _min2ptAspectRatio = min2ptAspectRatioProperty.getValue();
        } else if(property == max2ptAspectRatioProperty) {
            _max2ptAspectRatio = max2ptAspectRatioProperty.getValue();
        } else if(property == nearHorizAngleProperty) {
            _nearHorizSlope = Math.tan(Math.toRadians(nearHorizAngleProperty.getValue()));
        } else if(property == nearVertAngleProperty) {
            _nearVertSlope = Math.tan(Math.toRadians(nearVertAngleProperty.getValue()));
        } else if(property == contourColorProperty) {
            _contourColor = contourColorProperty.getValue();
        } else if(property == lineColorProperty) {
            _lineColor = lineColorProperty.getValue();
        } else if(property == processProperty) {
            _process = processProperty.getValue();
        } else if(property == selectProperty) {
            _select = selectProperty.getValue();
        }
    }
    
    private void _sendData(boolean found,double x,double y) {
        if(_sendResults) {
            outputTable.putBoolean("Target Found?", found);
            outputTable.putNumber ("Target X",     x);
            outputTable.putNumber ("Target Y",     y);
        } else {
            if(found) {
                System.out.println("Target X: " + x);
                System.out.println("Target Y: " + y);
            } else {
                System.out.println("Target not found");
            }
        }
    }
    
    private void _sendTime(long nanoTime) {
        double msTime = nanoTime/1.0e6;
        if(_sendResults) {
            outputTable.putNumber("ms per Frame",msTime);
        } else {
            System.out.format("Processed in %f ms\n",msTime);
        }
    }
    
    private double _aspectRatio(CvPoint points) {
        float horizTotal = 0,
              vertTotal  = 0;
        int horizCount = 0,
            vertCount  = 0;
        for(int i=0;i<4;++i) {
            CvPoint start = points.position(i),
                    end   = points.position((i+1)%4);
            double dx = end.x()-start.x(),
                   dy = end.y()-start.y();
            double slope = Double.MAX_VALUE;
            if(dx != 0) {
                slope = dy/dx;
            }
            slope = Math.abs(slope);
            if(slope < _nearHorizSlope) {
                horizTotal += Math.sqrt(dx*dx+dy*dy);
                horizCount++;
            } else if(slope > _nearVertSlope) {
                vertTotal += Math.sqrt(dx*dx+dy*dy);
                vertCount++;
            }
        }
        if(vertCount == 0 || horizCount == 0 || vertTotal == 0) {
            return 0;
        }
        return (horizTotal/horizCount)/(vertTotal/vertCount);
    }

    @Override
    public WPIImage processImage(WPIColorImage rawImage) {
        long startTime = System.nanoTime();
        if(_displayIntermediate) {
            _displayImage("Raw",StormCVUtil.getIplImage(rawImage));
        }
        
        // If we aren't doing any processing, leave the image as-is
        if(_process == _process_nothing) {
            return rawImage;
        }
        
        // Reallocate temporaries if the size has changed
        if(_size == null || _size.width() != rawImage.getWidth() || _size.height() != rawImage.getHeight()) {
            _size    = cvSize(rawImage.getWidth(),rawImage.getHeight());
            _hsv     = IplImage.create(_size, 8, 3);
            _bin     = IplImage.create(_size, 8, 1);
            _hueLow  = IplImage.create(_size, 8, 1);
            _hueHigh = IplImage.create(_size, 8, 1);
            _satLow  = IplImage.create(_size, 8, 1);
            _satHigh = IplImage.create(_size, 8, 1);
            _valLow  = IplImage.create(_size, 8, 1);
            _valHigh = IplImage.create(_size, 8, 1);
        }
        
        // Extract the IplImage so we can do OpenCV magic.
        IplImage image = StormCVUtil.getIplImage(rawImage);
        
        long nanoPreHSV = System.nanoTime();
        // Convert to HSV
        cvCvtColor(image, _hsv, CV_BGR2HSV);
        
        System.out.println("HSV conversion: " + (System.nanoTime()-nanoPreHSV)/1.0e6);
                
        // Split into individual color channels (from HSV)
        cvSplit(_hsv, _hueLow, _satLow, _valLow, null);
        
        // Apply thresholds
        // OpenCV can only do one-way threshold (less than or greater than
        // a specific value), so to have a ranged threshold, both thresholds
        // are performed then the results are ANDed together
        // The -1 is to make it an inclusive range (a >= n is equivalent to
        // a > n-1 for integers)
        cvThreshold(_hueLow,   _hueHigh,   _h0-1, 255, CV_THRESH_BINARY);
        cvThreshold(_hueLow,   _hueLow,    _h1,   255, CV_THRESH_BINARY_INV);
        
        cvThreshold(_satLow, _satHigh, _s0-1, 255, CV_THRESH_BINARY);
        cvThreshold(_satLow, _satLow,  _s1,   255, CV_THRESH_BINARY_INV);
        
        cvThreshold(_valLow,  _valHigh,  _v0-1, 255, CV_THRESH_BINARY);
        cvThreshold(_valLow,  _valLow,   _v1,   255, CV_THRESH_BINARY_INV);
        
        // ANDing the images leaves only the pixels within all of the ranges
        cvAnd(_hueLow, _hueHigh,   _bin, null);
        cvAnd(_bin,    _valLow,   _bin, null);
        cvAnd(_bin,    _valHigh,  _bin, null);
        cvAnd(_bin,    _satLow,  _bin, null);
        cvAnd(_bin,    _satHigh, _bin, null);
        
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
            CvSeq convexHull = cvConvexHull2(contours, _storage, CV_CLOCKWISE, 1);
            CvSeq polygon    = cvApproxPoly(convexHull,convexHull.header_size(),_storage,CV_POLY_APPROX_DP,20,0);
            convexContours.add(polygon);
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
        
        int selectedIndex = -1;
        
        double minArea = rawImage.getWidth()*rawImage.getHeight()*_minAreaRatio;
        double largestArea = 0;
        double smallestDistance = 0;
        for(int i=0;i<convexContours.size();++i) {
            CvSeq contour = convexContours.get(i);
            CvRect boundingRect = cvBoundingRect(contour, 1);
            double area = boundingRect.width()*boundingRect.height();
            if(area < minArea) {
                continue;
            }
            if(_select == _select_biggest) {
                if(selectedIndex == -1 || area > largestArea) {
                    selectedIndex = i;
                    largestArea = area;
                }
            } else {
                CvPoint points = new CvPoint(contour.total());
                cvCvtSeqToArray(contour, points, CV_WHOLE_SEQ);
                if(_select == _select_closest_2pt || _select == _select_closest_3pt) {
                    if(contour.total() != 4) {
                        continue;
                    }
                    double aspectRatio = _aspectRatio(points);

                    double minRatio,maxRatio;
                    if(_select == _select_closest_2pt) {
                        minRatio = _min2ptAspectRatio;
                        maxRatio = _max2ptAspectRatio;
                    } else {
                        minRatio = _min3ptAspectRatio;
                        maxRatio = _max3ptAspectRatio;
                    }
                    if(aspectRatio < minRatio || aspectRatio > maxRatio) {
                        continue;
                    }
                }
                double centroidX = 0,centroidY = 0;
                for(int j=0;j<contour.total();++j) {
                    centroidX += points.position(j).x();
                    centroidY += points.position(j).y();
                }
                centroidX /= contour.total();
                centroidY /= contour.total();
                
                // normalize
                centroidX = centroidX*2/rawImage.getWidth()-1;
                centroidY = centroidY*2/rawImage.getWidth()-1;
                
                double dx = centroidX-_desiredLocNormed.x(),
                       dy = centroidY-_desiredLocNormed.y();
                
                double dist = Math.sqrt(dx*dx+dy*dy);
                if(selectedIndex == -1 || dist < smallestDistance) {
                    selectedIndex = i;
                    smallestDistance = dist;
                }
            }
        }
        
        if(selectedIndex == -1) {
            _sendData(false, 0, 0);
            return rawImage;
        }
        
        CvSeq selectedContour = convexContours.get(selectedIndex);
        
        IplImage target = StormCVUtil.getIplImage(rawImage);
        cvDrawContours(target,selectedContour,color,color,0,2,8);
        
        if(_process != _process_convexHull) {
            double desiredXNormed = _desiredLocNormed.x(),
                   desiredYNormed = _desiredLocNormed.y();
            // desired location in screen pixels, for drawing purposes
            CvPoint desiredLoc = new CvPoint((int)((desiredXNormed +1)/2*rawImage.getWidth()),
                                             (int)((-desiredYNormed+1)/2*rawImage.getHeight()));
            
            int totalPoints = selectedContour.total();
            if(totalPoints <= 0) {
                // Something is REALLY wrong
                System.err.println("No points in selected contour");
                return rawImage;
            }
            
            CvPoint points = new CvPoint(selectedContour.total());
            cvCvtSeqToArray(selectedContour, points, CV_WHOLE_SEQ);
            
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
        
        long totalTime = System.nanoTime()-startTime;
        
        _sendTime(totalTime);
        
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
        _deallocateIfNonNull(_hueLow);
        _deallocateIfNonNull(_hueHigh);
        _deallocateIfNonNull(_satLow);
        _deallocateIfNonNull(_satHigh);
        _deallocateIfNonNull(_valLow);
        _deallocateIfNonNull(_valHigh);
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
            
            result = cv.processImage(rawImage);
            
            cv._displayImage("Result", StormCVUtil.getIplImage(result));
            
            System.out.println("Press Enter to Continue...");
            scanner.nextLine();
        }
        System.out.println("Done!");
        System.out.println("Press Enter to Continue...");
        scanner.nextLine();
        System.exit(0);
    }
}
