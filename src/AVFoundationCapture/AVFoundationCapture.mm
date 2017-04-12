/*
 * Ubitrack - Library for Ubiquitous Tracking
 * Copyright 2006, Technische Universitaet Muenchen, and individual
 * contributors as indicated by the @authors tag. See the
 * copyright.txt in the distribution for a full listing of individual
 * contributors.
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software; if not, write to the Free
 * Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA, or see the FSF site: http://www.fsf.org.
 */

/**
 * @ingroup vision_components
 * @file
 * Reads camera images using Apples AVFoundation Framework.
 *
 * @author Ulrich Eck <ueck@net-labs.de>
 *
 * implementation taken from opencv/modules/videoio/cap_qtkit.mm
 *
 */

#include <string>
#include <list>
#include <iostream>
#include <strstream>
#include <log4cpp/Category.hh>

#ifdef __APPLE__
// workaround for compiling variadict templates - seems to be a compiler problem
#define BOOST_NO_CXX11_VARIADIC_TEMPLATES
#endif

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>

#include <utDataflow/PushSupplier.h>
#include <utDataflow/PullSupplier.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utMeasurement/TimestampSync.h>
#include <utVision/Image.h>
#include <utVision/Undistortion.h>
#include <utUtil/OS.h>
#include <utUtil/TracingProvider.h>
#include <opencv/cv.h>
#include <utVision/OpenCLManager.h>


#import <AVFoundation/AVFoundation.h>
#import <Foundation/NSException.h>

#define DISABLE_AUTO_RESTART 999

// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Vision.AVFoundationCapture" ) );

using namespace Ubitrack;
using namespace Ubitrack::Vision;





/*****************************************************************************
 *
 * UTCaptureDelegate Implementation.
 *
 * UTCaptureDelegate is notified on a separate thread by the OS whenever there
 *   is a new frame. When "updateImage" is called from the main thread, it
 *   copies this new frame into an IplImage, but only if this frame has not
 *   been copied before. When "getOutput" is called from the main thread,
 *   it gives the last copied IplImage.
 *
 *****************************************************************************/

namespace Ubitrack {
    namespace Drivers {
        class AVFoundationCapture;
    }
}


@interface UTCaptureDelegate : NSObject <AVCaptureVideoDataOutputSampleBufferDelegate>
{
    Ubitrack::Drivers::AVFoundationCapture* _owner;
    NSLock* _lock;
}

- (void)captureOutput:(AVCaptureOutput *)captureOutput
didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer
fromConnection:(AVCaptureConnection *)connection;

- (void)registerOwner:(Ubitrack::Drivers::AVFoundationCapture*)owner;

@end



namespace Ubitrack { namespace Drivers {

/**
 * @ingroup vision_components
 * Reads camera images using Apple AVFoundation
 *
 * @par Input Ports
 * None.
 *
 * @par Output Ports
 * \c Output push port of type Ubitrack::Vision::ImageMeasurement.
 *
 * @par Configuration
 */
class AVFoundationCapture
	: public Dataflow::Component
{
public:

	/** constructor */
    AVFoundationCapture( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph >  );

	/** destructor, waits until thread stops */
	~AVFoundationCapture();

    /** starts the camera */
    void start();

    /** actually start capturing **/
    void startCapturing();

    /** stops the camera */
    void stop();
    
    /** handler method for incoming pull requests */
	Measurement::Matrix3x3 getIntrinsic( Measurement::Timestamp t )
	{
		if (m_undistorter) {
			return Measurement::Matrix3x3( t, m_undistorter->getMatrix() );
		} else {
			UBITRACK_THROW( "No undistortion configured for AVFoundationCapture" );
		}
	}

    void receiveFrame (void*pixelBufferBase, size_t width, size_t height, size_t size, Ubitrack::Measurement::Timestamp timestamp);

protected:
	// qtkit stuff
    AVCaptureSession            *mCaptureSession;
    AVCaptureDeviceInput        *mCaptureDeviceInput;
    AVCaptureVideoDataOutput    *mCaptureDecompressedVideoOutput;
    UTCaptureDelegate           *m_CaptureDelegate;

    unsigned char* m_imageBuffer;
    size_t         m_imageBufferSize;

    void initializeCamera();
    void configureOutput ();
    void destroySession();

    // camera UUID
    std::string m_cameraUUID;
    
	// the image width
	int m_width;

	// the image height
	int m_height;

	// trigger flash
	bool m_disable_autostart;

	// shift timestamps (ms)
	int m_timeOffset;

    /** automatic upload of images to the GPU*/
    bool m_autoGPUUpload;

    // thread main loop
    void ThreadProc();

    // the thread
    boost::scoped_ptr< boost::thread > m_Thread;

    // stop the thread?
    volatile bool m_bStop;
    volatile bool m_bCaptureThreadReady;


	/** undistorter */
	boost::shared_ptr<Vision::Undistortion> m_undistorter;

	// the ports
	Dataflow::PushSupplier< Measurement::ImageMeasurement > m_outPort;
	Dataflow::PushSupplier< Measurement::ImageMeasurement > m_colorOutPort;
	Dataflow::PullSupplier< Measurement::Matrix3x3 > m_intrinsicsPort;
};


AVFoundationCapture::AVFoundationCapture( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
	: Dataflow::Component( sName )
	, m_cameraUUID( "" )
	, m_width( 0 )
	, m_height( 0 )
    , m_disable_autostart( false )
    , m_bStop( false )
    , m_bCaptureThreadReady( false )
	, m_timeOffset( 0 )
	, m_outPort( "Output", *this )
	, m_colorOutPort( "ColorOutput", *this )
	, m_intrinsicsPort( "Intrinsics", *this, boost::bind( &AVFoundationCapture::getIntrinsic, this, _1 ) )
    , m_imageBufferSize(0)
    , mCaptureSession(NULL)
    , mCaptureDeviceInput(NULL)
    , mCaptureDecompressedVideoOutput(NULL)
    , m_CaptureDelegate(NULL)
    , m_imageBuffer(NULL)
    , m_autoGPUUpload(false)
{

    m_cameraUUID = subgraph->m_DataflowAttributes.getAttributeString( "cameraUUID" );
	subgraph->m_DataflowAttributes.getAttributeData( "width", m_width );
	subgraph->m_DataflowAttributes.getAttributeData( "height", m_height );

	if ( subgraph->m_DataflowAttributes.getAttributeString( "disableAutostart" ) == "true")
	{
        m_disable_autostart = true;
	}

	subgraph->m_DataflowAttributes.getAttributeData( "timeOffset", m_timeOffset );


    Vision::OpenCLManager& oclManager = Vision::OpenCLManager::singleton();
    if (oclManager.isEnabled()) {
        if (subgraph->m_DataflowAttributes.hasAttribute("uploadImageOnGPU")){
            m_autoGPUUpload = subgraph->m_DataflowAttributes.getAttributeString("uploadImageOnGPU") == "true";
            LOG4CPP_INFO(logger, "Upload to GPU enabled? " << m_autoGPUUpload);
        }
        if (m_autoGPUUpload) {
            oclManager.activate();
        }
    }


    if (subgraph->m_DataflowAttributes.hasAttribute("cameraModelFile")){
        std::string cameraModelFile = subgraph->m_DataflowAttributes.getAttributeString("cameraModelFile");
        m_undistorter.reset(new Vision::Undistortion(cameraModelFile));
    }
    else {
        std::string intrinsicFile = subgraph->m_DataflowAttributes.getAttributeString("intrinsicMatrixFile");
        std::string distortionFile = subgraph->m_DataflowAttributes.getAttributeString("distortionFile");


        m_undistorter.reset(new Vision::Undistortion(intrinsicFile, distortionFile));
    }
}

AVFoundationCapture::~AVFoundationCapture()
{
}

void AVFoundationCapture::initializeCamera() {
    // initialize qtkit m_CaptureDelegate
    LOG4CPP_INFO(logger, "Initialize AVFoundation.");

    mCaptureSession = nil;
    mCaptureDeviceInput = nil;
    mCaptureDecompressedVideoOutput = nil;
    m_CaptureDelegate = nil;


    m_CaptureDelegate = [[UTCaptureDelegate alloc] init];
    [m_CaptureDelegate registerOwner:this];

    NSArray* devices = [AVCaptureDevice devicesWithMediaType:AVMediaTypeVideo];

    if ([devices count] == 0) {
        LOG4CPP_ERROR( logger, "AVFoundation didn't find any attached Video Input Devices!" );
        return;
    }

    AVCaptureDevice *device = NULL;
    NSUInteger nCameras = [devices count];
    NSString* uid = [NSString stringWithUTF8String:m_cameraUUID.c_str()];
    for (int i = 0; i < nCameras; ++i) {
        device = [devices objectAtIndex:i];

        LOG4CPP_INFO(logger, "Found camera: "
                << ([[device localizedName] UTF8String])
                << " UUID "<< ([[device uniqueID] UTF8String]));

        if ([[device uniqueID] rangeOfString:uid].location != NSNotFound) {
            uid = [device uniqueID];
        }
    }

    if (! uid) {
        LOG4CPP_INFO(logger, "Requesting default device ");
        device = [[AVCaptureDevice defaultDeviceWithMediaType: AVMediaTypeVideo] retain];
    } else {
        LOG4CPP_INFO(logger, "Requesting uid " << [uid UTF8String]);
        AVCaptureDevice *device_tmp = NULL;
        for (int i = 0; i < nCameras; ++i) {
            device_tmp = [devices objectAtIndex:i];
            const char * cuid = [[ device_tmp uniqueID ] UTF8String ];
            if (cuid && ( strcmp( [uid UTF8String], cuid ) == 0)) {
                device = device_tmp;
                break;
            }}

        if (!device) {
            LOG4CPP_ERROR( logger, "AVFoundation didn't find Video Input Device!" );
            return;
        }
    }

    if (! device ) {
        LOG4CPP_WARN(logger, "Invalid Camera Handle for AVFoundation");
        return;
    }


    LOG4CPP_INFO(logger, "Selected camera: "
            << ([[device localizedName] UTF8String])
            << " UUID "<< ([[device uniqueID] UTF8String]));


    int success;
    NSError* error;

    if (device) {
        LOG4CPP_INFO(logger, "AVFoundation - Found device.");

        mCaptureDeviceInput = [[AVCaptureDeviceInput alloc] initWithDevice:device error:&error];
        if (! mCaptureDeviceInput ) {
            LOG4CPP_ERROR(logger, "AVFoundation input not received");
            return;
        }

        mCaptureSession = [[AVCaptureSession alloc] init];

        mCaptureDecompressedVideoOutput = [[AVCaptureVideoDataOutput alloc] init];

        dispatch_queue_t queue = dispatch_queue_create("cameraQueue", NULL);
        [mCaptureDecompressedVideoOutput setSampleBufferDelegate:m_CaptureDelegate queue:queue];
        dispatch_release(queue);

        configureOutput();

        [mCaptureSession addInput:mCaptureDeviceInput];
        [mCaptureSession addOutput:mCaptureDecompressedVideoOutput];
        
        LOG4CPP_INFO(logger, "AVFoundation - device setup complete.");

    }
}



void AVFoundationCapture::configureOutput ()
{

    NSDictionary *pixelBufferOptions;
    if (m_width > 0 && m_height > 0) {
        pixelBufferOptions = [NSDictionary dictionaryWithObjectsAndKeys: [NSNumber numberWithDouble: 1.0 * m_width], (id) kCVPixelBufferWidthKey,
        [NSNumber numberWithDouble: 1.0 * m_height], (id) kCVPixelBufferHeightKey,
        [NSNumber numberWithUnsignedInt: kCVPixelFormatType_32BGRA], (id) kCVPixelBufferPixelFormatTypeKey, nil];
    } else {
        pixelBufferOptions = [NSDictionary dictionaryWithObjectsAndKeys: [NSNumber numberWithUnsignedInt: kCVPixelFormatType_32BGRA], (id) kCVPixelBufferPixelFormatTypeKey, nil];
    }
    [mCaptureDecompressedVideoOutput setVideoSettings: pixelBufferOptions];
    mCaptureDecompressedVideoOutput.alwaysDiscardsLateVideoFrames = YES;

}

void AVFoundationCapture::destroySession ()
{
    LOG4CPP_INFO( logger, "Camera session ends.");
    if (mCaptureSession) {
        if ([mCaptureSession isRunning]) [mCaptureSession stopRunning ];
        if (mCaptureDecompressedVideoOutput) [mCaptureSession removeOutput: mCaptureDecompressedVideoOutput ];
        if (mCaptureDeviceInput) [ mCaptureSession removeInput: mCaptureDeviceInput ];

        [mCaptureSession release];
        mCaptureSession = nil;
    }
    if (mCaptureDeviceInput) {
        [mCaptureDeviceInput release];
        mCaptureDeviceInput = nil;
    }
    if (mCaptureDecompressedVideoOutput) {
        [mCaptureDecompressedVideoOutput release];
        mCaptureDecompressedVideoOutput = nil;
    }
    if (m_CaptureDelegate) {
        [m_CaptureDelegate registerOwner:nil];
        [m_CaptureDelegate
        release];
        m_CaptureDelegate = nil;
    }
}


void AVFoundationCapture::startCapturing() {

    LOG4CPP_DEBUG(logger, "Start Capturing.");
    // run the main-loop until the capture thread is ready
    // then he'll maintain the loop until stop is requested
    double sleepTime = 0.005;

    m_Thread.reset( new boost::thread( boost::bind( &AVFoundationCapture::ThreadProc, this ) ) );

    // If the capture is launched in a separate thread, then
    // [NSRunLoop currentRunLoop] is not the same as in the main thread, and has no timer.
    //see https://developer.apple.com/library/mac/#documentation/Cocoa/Reference/Foundation/Classes/nsrunloop_Class/Reference/Reference.html
    // "If no input sources or timers are attached to the run loop, this
    // method exits immediately"
    // using usleep() is not a good alternative, because it may block the GUI.
    // Create a dummy timer so that runUntilDate does not exit immediately:
    [NSTimer scheduledTimerWithTimeInterval:100 target:nil selector:@selector(doFireTimer:) userInfo:nil repeats:YES];
    while (!m_bCaptureThreadReady) {
        [[NSRunLoop currentRunLoop] runUntilDate:[NSDate dateWithTimeIntervalSinceNow:sleepTime]];
    }

}


void AVFoundationCapture::start()
{
    if ( !m_running ) {
        if (m_autoGPUUpload){
            // wait for the notification of OpenCL Manager to start capturing
            LOG4CPP_DEBUG(logger, "AVFoundation - Waiting for OpenCLManager to be initialized.");
            Vision::OpenCLManager& oclManager = Vision::OpenCLManager::singleton();
            oclManager.registerInitCallback(boost::bind(&AVFoundationCapture::startCapturing, this));
        } else {
            // start capturing right away
            startCapturing();
        }
    }
    Component::start();
}


void AVFoundationCapture::stop()
{
    if ( m_running ) {
        if ( m_Thread )
        {
            m_bStop = true;
            m_Thread->join();
        }
    }
}


void AVFoundationCapture::receiveFrame(void *pixelBufferBase, size_t width, size_t height, size_t size, Ubitrack::Measurement::Timestamp timestamp) {

    if (size != 0) {
        if ((width == 0) || (height == 0)) {
            LOG4CPP_WARN(logger, "Received empty image - skipping (" << width << "," << height << "," << size << ").");
            return;
        }

        Vision::Image::ImageFormatProperties fmt;
        fmt.channels = 4;
        fmt.depth = CV_8U;
        fmt.imageFormat = Vision::Image::BGRA;
        fmt.origin = 0;
        fmt.bitsPerPixel = 32;
        fmt.matType = CV_8UC4;

        boost::shared_ptr<Vision::Image> pColorImage(new Image((int)width, (int)height, fmt, static_cast< char* >(pixelBufferBase)));

#ifdef ENABLE_EVENT_TRACING
        TRACEPOINT_MEASUREMENT_CREATE(getEventDomain(), timestamp, getName().c_str(), "VideoCapture")
#endif

        if (m_autoGPUUpload){
            Vision::OpenCLManager& oclManager = Vision::OpenCLManager::singleton();
            if (oclManager.isInitialized()) {
                //force upload to the GPU
                pColorImage->uMat();
            }
        }

        pColorImage = m_undistorter->undistort( pColorImage );
//        pColorImage->set_pixelFormat(Vision::Image::BGRA);

        if ( m_colorOutPort.isConnected() )
            m_colorOutPort.send( Measurement::ImageMeasurement( timestamp, pColorImage ) );
        if ( m_outPort.isConnected() )
            m_outPort.send( Measurement::ImageMeasurement( timestamp, pColorImage->CvtColor( CV_RGBA2GRAY, 1 ) ) );
    }
}

void AVFoundationCapture::ThreadProc() {


    double sleepTime = 0.005;

    initializeCamera();

    if (mCaptureSession) {
        if (! [mCaptureSession isRunning]) {
            LOG4CPP_INFO(logger, "Start AVFoundation Capturing.");
            [ mCaptureSession startRunning ];
        }
    } else {
        LOG4CPP_ERROR( logger, "Run: no session to run.");
        return;
    }

    m_bCaptureThreadReady = true;


    // If the capture is launched in a separate thread, then
    // [NSRunLoop currentRunLoop] is not the same as in the main thread, and has no timer.
    //see https://developer.apple.com/library/mac/#documentation/Cocoa/Reference/Foundation/Classes/nsrunloop_Class/Reference/Reference.html
    // "If no input sources or timers are attached to the run loop, this
    // method exits immediately"
    // using usleep() is not a good alternative, because it may block the GUI.
    // Create a dummy timer so that runUntilDate does not exit immediately:
    [NSTimer scheduledTimerWithTimeInterval:100 target:nil selector:@selector(doFireTimer:) userInfo:nil repeats:YES];
    while (!m_bStop) {
        [[NSRunLoop currentRunLoop] runUntilDate:[NSDate dateWithTimeIntervalSinceNow:sleepTime]];
    }

    LOG4CPP_INFO(logger, "Stop AVFoundation Capturing.");
    destroySession();

}

} } // namespace Ubitrack::Driver


// implementation of delegate

@implementation UTCaptureDelegate

- (id)init {
    LOG4CPP_INFO(logger, "Initialize AVFoundation delegate.");
    self = [super init];
    if (self) {
        _owner = NULL;
    }
    return self;
}


-(void)dealloc {
    _owner = NULL;
    [super dealloc];
}

- (void)captureOutput:(AVCaptureOutput *)captureOutput
didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer
fromConnection:(AVCaptureConnection *)connection {

    LOG4CPP_TRACE(logger, "AVFoundation received image.");
    (void)captureOutput;
    (void)connection;
    CVImageBufferRef videoFrame = CMSampleBufferGetImageBuffer(sampleBuffer);

    @synchronized (self) {
        // get timestamp early
        Ubitrack::Measurement::Timestamp timestamp = Ubitrack::Measurement::now();

        void* base;
        size_t size, width, height;
        if (CVPixelBufferLockBaseAddress(videoFrame, 0)) {
            LOG4CPP_ERROR(logger, "Cannot lock frame buffer.");
            return;
        }

        // Get info about the raw pixel-buffer data.
        base = (void*) CVPixelBufferGetBaseAddress(videoFrame);
        width = CVPixelBufferGetWidth(videoFrame);
        height = CVPixelBufferGetHeight(videoFrame);
        size = height * CVPixelBufferGetBytesPerRow(videoFrame);

        if (_owner != NULL) {
            _owner->receiveFrame (base, width, height, size, timestamp);
        }

        // We're done with the pixel-buffer
        CVPixelBufferUnlockBaseAddress(videoFrame, 0);
    }

}


#pragma mark Public methods

- (void)registerOwner:(Ubitrack::Drivers::AVFoundationCapture*)owner {
    @synchronized (self) {
        _owner = owner;
    }
}

@end

UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< Ubitrack::Drivers::AVFoundationCapture > ( "device_camera_avfoundation" );
}

