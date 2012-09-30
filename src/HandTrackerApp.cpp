// Cinder
#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/params/Params.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/Camera.h"
// OpenNI
#include "VOpenNIHeaders.h"
#include <XnCppWrapper.h>
// NITE
#include <XnVSessionManager.h>
#include <XnVMultiProcessFlowClient.h>
#include <XnVWaveDetector.h>
// Internal
#include "CinderOpenCV.h"

using namespace ci;
using namespace ci::app;
using namespace std;

#define SAMPLE_XML_FILE "SampleTracking.xml"
#define WIDTH 1280
#define HEIGHT 480
Vec3f handCoords(0,0,0);

class ImageSourceKinectColor : public ImageSource
{
public:
	ImageSourceKinectColor( uint8_t *buffer, int width, int height )
  : ImageSource(), mData( buffer ), _width(width), _height(height)
	{
		setSize( _width, _height );
		setColorModel( ImageIo::CM_RGB );
		setChannelOrder( ImageIo::RGB );
		setDataType( ImageIo::UINT8 );
	}
  
	~ImageSourceKinectColor()
	{
		// mData is actually a ref. It's released from the device.
		/*if( mData ) {
     delete[] mData;
     mData = NULL;
     }*/
	}
  
	virtual void load( ImageTargetRef target )
	{
		ImageSource::RowFunc func = setupRowFunc( target );
    
		for( uint32_t row	 = 0; row < _height; ++row )
			((*this).*func)( target, row, mData + row * _width * 3 );
	}
  
protected:
	uint32_t					_width, _height;
	uint8_t						*mData;
};


class ImageSourceKinectDepth : public ImageSource
{
public:
	ImageSourceKinectDepth( uint16_t *buffer, int width, int height )
  : ImageSource(), mData( buffer ), _width(width), _height(height)
	{
		setSize( _width, _height );
		setColorModel( ImageIo::CM_GRAY );
		setChannelOrder( ImageIo::Y );
		setDataType( ImageIo::UINT16 );
	}
  
	~ImageSourceKinectDepth()
	{
		// mData is actually a ref. It's released from the device.
		/*if( mData ) {
     delete[] mData;
     mData = NULL;
     }*/
	}
  
	virtual void load( ImageTargetRef target )
	{
		ImageSource::RowFunc func = setupRowFunc( target );
    
		for( uint32_t row = 0; row < _height; ++row )
			((*this).*func)( target, row, mData + row * _width );
	}
  
protected:
	uint32_t					_width, _height;
	uint16_t					*mData;
};

class HandTrackerApp : public AppBasic
{
public:
  ImageSourceRef getColorImage();
  ImageSourceRef getDepthImage();
  xn::DepthGenerator* getDepthGenerator();
  virtual void setup();
  virtual void update();
  virtual void draw();
  virtual void prepareSettings(AppBasic::Settings *settings);
  virtual void shutdown();
  void addUser(const XnUserID &user, const XnPoint3D *handPosition);
  void removeUser(const XnUserID &user);
  void updateUser(const XnUserID &user, const XnPoint3D *handPosition);
  
protected:
  params::InterfaceGl mParams;
  float mThreshold,
  mMinBlobSize,
  mMaxBlobSize;
  gl::Texture mColorTexture,
  mDepthTexture,
  mCvTexture;
  Surface mDepthSurface;
  Vec3f mTargetPosition;
  V::OpenNIDeviceManager*	mManager;
	V::OpenNIDevice::Ref	mDevice0;
  ci::gl::GlslProg mShader;
  XnCallbackHandle mCallbackHandle;
  std::map<XnUserID, XnPoint3D> mHandPositions;
  
  virtual void mouseDown( MouseEvent event );
  virtual void keyDown(KeyEvent event);
};

//-----------------------------------------------------------------------------
// Callbacks
//-----------------------------------------------------------------------------

// Callback for when the focus is in progress
void XN_CALLBACK_TYPE SessionProgress(const XnChar* strFocus, const XnPoint3D& ptFocusPoint, XnFloat fProgress, void* UserCxt)
{
	// printf("Session progress (%6.2f,%6.2f,%6.2f) - %6.2f [%s]\n", ptFocusPoint.X, ptFocusPoint.Y, ptFocusPoint.Z, fProgress,  strFocus);
}
// callback for session start
void XN_CALLBACK_TYPE SessionStart(const XnPoint3D& ptFocusPoint, void* UserCxt)
{
	// printf("Session started. Please wave (%6.2f,%6.2f,%6.2f)...\n", ptFocusPoint.X, ptFocusPoint.Y, ptFocusPoint.Z);
}
// Callback for session end
void XN_CALLBACK_TYPE SessionEnd(void* UserCxt)
{
	printf("Session ended. Please perform focus gesture to start session\n");
}
// Callback for wave detection
void XN_CALLBACK_TYPE OnWaveCB(void* cxt)
{
	printf("Wave!\n");
}
// callback for a new position of any hand
void XN_CALLBACK_TYPE OnPointUpdate(const XnVHandPointContext* pContext, void* cxt)
{
	handCoords = Vec3f(pContext->ptPosition.X + WIDTH/2, -pContext->ptPosition.Y + HEIGHT/2, -pContext->ptPosition.Z);
	// printf("%d: (%f,%f,%f) [%f]\n", pContext->nID, pContext->ptPosition.X, pContext->ptPosition.Y, pContext->ptPosition.Z, pContext->fTime);
}

void XN_CALLBACK_TYPE OnHandCreate(xn::HandsGenerator& generator, XnUserID user, const XnPoint3D* pPosition, XnFloat fTime, void* pCookie)
{
  HandTrackerApp *app = static_cast<HandTrackerApp*>(pCookie);
  app->addUser(user, pPosition);
  std::cout << "Created hand for XnUser: " << user << std::endl;
}

void XN_CALLBACK_TYPE OnHandDestroy(xn::HandsGenerator &generator, XnUserID user, XnFloat fTime, void *pCookie)
{
  HandTrackerApp *app = static_cast<HandTrackerApp*>(pCookie);
  app->removeUser(user);
  std::cout << "Destroyed hand for XnUser: " << user << std::endl;
}

void XN_CALLBACK_TYPE OnHandUpdate(xn::HandsGenerator &generator, XnUserID user, const XnPoint3D *pPosition, XnFloat fTime, void *pCookie)
{
  HandTrackerApp *app = static_cast<HandTrackerApp*>(pCookie);
  
  XnMapOutputMode mode;
	app->getDepthGenerator()->GetMapOutputMode(mode);
  
  /*
  XnPoint3D p(*pPosition);
  // p.X *= -1.0f;
  p.Y *= -1.0f;
  p.Z *= -1.0f;
  app->updateUser(user, &p);
  std::cout << "Updated hand for XnUser: " << user << " @ screen={ " << pPosition->X << ", " << pPosition->Y << ", " << pPosition->Z << " }" << std::endl;
  //*/
  XnPoint3D projective(*pPosition);
  app->getDepthGenerator()->ConvertRealWorldToProjective(1, &projective, &projective);
  app->updateUser(user, &projective);
  
  std::cout << "Updated hand for XnUser: " << user
    << " @ screen={ " << projective.X << ", " << projective.Y << ", " << projective.Z << " }"
    << ", real={ " << pPosition[0].X << ", " << pPosition[0].Y << ", " << pPosition[0].Z << " }"
    << std::endl;
   
}

ImageSourceRef HandTrackerApp::getColorImage()
{
  // register a reference to the active buffer
  uint8_t *activeColor = this->mDevice0->getColorMap();
  return ImageSourceRef( new ImageSourceKinectColor( activeColor, 640, 480 ) );
}

ImageSourceRef HandTrackerApp::getDepthImage()
{
  // register a reference to the active buffer
  uint16_t *activeDepth = this->mDevice0->getDepthMap();
  return ImageSourceRef( new ImageSourceKinectDepth( activeDepth, 640, 480 ) );
}

xn::DepthGenerator* HandTrackerApp::getDepthGenerator()
{
  if (!this->mDevice0.get())
  {
    return NULL;
  }
  return this->mDevice0->getDepthGenerator();
}

void HandTrackerApp::addUser(const XnUserID &user, const XnPoint3D *handPosition)
{
  this->mHandPositions.insert(std::make_pair(user, XnPoint3D(*handPosition)));
}

void HandTrackerApp::removeUser(const XnUserID &user)
{
  this->mHandPositions.erase(this->mHandPositions.find(user));
}

void HandTrackerApp::updateUser(const XnUserID &user, const XnPoint3D *handPosition)
{
  this->mHandPositions[user] = XnPoint3D(*handPosition);
}


void HandTrackerApp::prepareSettings(AppBasic::Settings *settings)
{
  settings->setWindowSize(WIDTH, HEIGHT);
}

void HandTrackerApp::setup()
{  
  // OPENNI ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  this->mThreshold = 70.0f;
  this->mMinBlobSize = 20.0f;
  this->mMaxBlobSize = 80.0f;
  /*
  this->mParams = params::InterfaceGl("Hand Tracking", Vec2i(10, 10));
  this->mParams.addParam("Threshold", &this->mThreshold, "min=0.0 max=255.0, step=1.0, keyIncr=s keyDecr=w");
  this->mParams.addParam("Blob Minimum Radius", &this->mMinBlobSize, "min=1.0 max=200.0, step=1.0, keyIncr=e keyDecr=d");
  this->mParams.addParam("Blob Maximum Radius", &this->mMaxBlobSize, "min=2.0 max=200.0, step=1.0, keyIncr=r keyDecr=f");
  */
  V::OpenNIDeviceManager::USE_THREAD = false;
	this->mManager = V::OpenNIDeviceManager::InstancePtr();
	this->mManager->createDevices( 1, V::NODE_TYPE_DEPTH | V::NODE_TYPE_IMAGE | V::NODE_TYPE_HANDS /* | XN_NODE_TYPE_GESTURE */, V::RES_640x480 );
	this->mDevice0 = this->mManager->getDevice( 0 );
	if (!this->mDevice0)
	{
		DEBUG_MESSAGE( "[ERROR] Can't find the Kinect device! Exiting...\n" );
		shutdown();
		quit();
		return;
	}
  
  void *pCookie = this;
	this->mDevice0->setDepthInvert( false );
  this->mDevice0->setDepthShiftMul( 4 );
  this->mDevice0->getHandsGenerator()->RegisterHandCallbacks(OnHandCreate, OnHandUpdate, OnHandDestroy, pCookie, this->mCallbackHandle);
	this->mColorTexture = gl::Texture( 640, 480 );
	this->mDepthTexture = gl::Texture( 640, 480 );
  this->mDepthSurface = Surface(640, 480, false);
  
	this->mManager->SetPrimaryBuffer( 0, V::NODE_TYPE_DEPTH );
	this->mManager->start();
  
  try
  {
    this->mShader = ci::gl::GlslProg(loadResource("vertex.glsl"), loadResource("fragment.glsl") );
  }
  catch( const std::exception &e )
  {
    console() << "[ERROR] " << e.what() << std::endl;
  }
}

void HandTrackerApp::shutdown()
{
  this->mDevice0->getHandsGenerator()->UnregisterHandCallbacks(this->mCallbackHandle);
}

void HandTrackerApp::update()
{
  if (!V::OpenNIDeviceManager::USE_THREAD)
	{
		this->mManager->update();
	}
  
	// Update textures
  // @ref https://github.com/pixelnerve/BlockOpenNI/issues/5
	ImageSourceRef colorImage = getColorImage();
  ImageSourceRef depthImage = getDepthImage();
  this->mColorTexture = colorImage;
  this->mDepthSurface = depthImage;
  
  // make a surface for opencv
  this->mDepthTexture = this->mDepthSurface.clone();
  
  /*
  // once the surface is avalable pass it to opencv
  // had trouble here with bit depth. surface comes in full color, needed to crush it down
  cv::Mat input( toOcv( Channel8u( this->mDepthSurface )  ) ), blurred, thresholded, thresholded2, output;
  
  cv::blur(input, blurred, cv::Size(10,10));
  
  // make two thresholded images one to display and one
  // to pass to find contours since its process alters the image
  cv::threshold( blurred, thresholded, mThreshold, 255, CV_THRESH_BINARY);
  cv::threshold( blurred, thresholded2, mThreshold, 255, CV_THRESH_BINARY);
  
  // 2d vector to store the found contours
  vector<vector<cv::Point> > contours;
  // find em
  cv::findContours(thresholded, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
  
  // convert theshold image to color for output
  // so we can draw blobs on it
  cv::cvtColor( thresholded2, output, CV_GRAY2RGB );
  
  // loop the stored contours
  for (vector<vector<cv::Point> >::iterator it=contours.begin() ; it < contours.end(); it++ )
  {
    // center abd radius for current blob
    cv::Point2f center;
    float radius;
    // convert the cuntour point to a matrix
    vector<cv::Point> pts = *it;
    cv::Mat pointsMatrix = cv::Mat(pts);
    // pass to min enclosing circle to make the blob
    cv::minEnclosingCircle(pointsMatrix, center, radius);
    
    cv::Scalar color( 0, 255, 0 );
    
    if (radius > this->mMinBlobSize && radius < this->mMaxBlobSize)
    {
      // draw the blob if it's in range
      cv::circle(output, center, radius, color);
      
      //update the target position
      mTargetPosition.x = 640 - center.x;
      mTargetPosition.y = center.y;
      mTargetPosition.z = 0;
    }
  }
  
  this->mCvTexture = gl::Texture( fromOcv( output ) );
  //*/

  // this->mDepthTexture = this->mDepthSurface;
}

void HandTrackerApp::draw()
{
  gl::clear(Color(0, 0, 0));
  
  if (this->mDepthSurface)
  {
    gl::draw(this->mDepthTexture);

    /*
    int X_OFFSET = 0;
    int Y_OFFSET = 0;
    float FACTOR = 50.0f;
    int PERIOD = (int)(this->mDepthSurface.getWidth() / FACTOR);
    float MAX_SIZE = 10.0f;
    int l = 0, p = 0;
    Area area( 0, 0, this->mDepthSurface.getWidth(), this->mDepthSurface.getHeight() );
    Surface::Iter iter = this->mDepthSurface.getIter(area);
    while (iter.line())
    {
      if (l++ % PERIOD != 0)
      {
        continue;
      }
      
      p = 0;
      while(iter.pixel())
      {
        if (p++ % PERIOD != 0) { continue; }
        Vec2i pos(X_OFFSET + p, Y_OFFSET + l);
        
        float intensity = (float)iter.r() / 255.0f;
        //*
        gl::color((1.0f - intensity), 0.0, 0.0);
        gl::drawSolidCircle(pos, (MAX_SIZE * (1.0f - intensity)));
        //* /
        
        /*
        int ASCII_OFFSET = 33;
        int ASCII_ADJUSTED_RANGE_END = 125 - ASCII_OFFSET;
        char ascii = (char)((intensity * ASCII_ADJUSTED_RANGE_END) + ASCII_OFFSET);
        std::string strAscii(1, ascii);
        
        ColorA color((1.0f - intensity), 0.0, 0.0, 1.0f);
        Font font("Arial", 14.0f * intensity);
        gl::drawString(strAscii, pos, color, font);
        //* /
      }
    }
     */
  }
  /*
  this->mShader.bind();
  this->mShader.uniform("tex0", 0);
  this->mShader.uniform("tex1", 1);
  
  // enable the use of textures
  gl::enable( GL_TEXTURE_2D );
  
  // assumes you have two textures, named mTexture0 and mTexture1
  this->mColorTexture.bind(0);
  this->mDepthTexture.bind(1);
  
  // params::InterfaceGl::draw();
  
  // now run the shader for every pixel in the window
  // by drawing a full screen rectangle
  Area area = getWindowBounds();
  Vec2i lr(area.getLR());
  lr.x -= area.getWidth() / 2.0f;
  area.setX2(lr.x);
  area.setY2(lr.y);
  
  gl::drawSolidRect(area, false);
  
  this->mShader.unbind();
  //*/

  /*
  glLineWidth( 10.0f );
  glEnable(GL_LINE_SMOOTH);
  glHint(GL_LINE_SMOOTH_HINT,  GL_NICEST);
  //*/
  
  gl::enableAlphaBlending();
  
  static const float MAX_POINT_SIZE = 25.0f;

  std::map<XnUserID, XnPoint3D>::iterator
    iter = this->mHandPositions.begin(),
    end = this->mHandPositions.end();
  for (; iter != end; ++iter)
  {
    gl::color(ColorA(1.0f, 0.0f, 0.0f, 0.5f));
    
    XnMapOutputMode mode;
    this->getDepthGenerator()->GetMapOutputMode(mode);
    
    std::stringstream ss;
    ss << iter->first;
    
    Font font("Helvetica-Bold", 24);
    Vec2i handPos((int)iter->second.X, (int)iter->second.Y);
    gl::drawSolidCircle(handPos, MAX_POINT_SIZE);
    handPos.y -= (font.getSize() / 4.0f);
    gl::drawStringCentered(ss.str(), handPos, ColorA(1,1,1,0.5), font);
  }
  
  gl::disableAlphaBlending();
  
  gl::color(ColorA(1.0f, 1.0f, 1.0f, 1.0f));
}

void HandTrackerApp::mouseDown(MouseEvent event)
{
}

void HandTrackerApp::keyDown(KeyEvent event)
{
	if (event.getCode() == KeyEvent::KEY_ESCAPE)
	{
		this->quit();
		this->shutdown();
	}
}


CINDER_APP_BASIC( HandTrackerApp, RendererGl )
