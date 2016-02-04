/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description: Class which enables to project an image in the 3D space
 * and get the view of a virtual camera.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file vpFlyCaptureGrabber.cpp
  \brief Wrapper over PointGrey FlyCapture SDK to capture images from PointGrey cameras.
*/

#include <visp3/core/vpException.h>
#include <visp3/flycapture/vpFlyCaptureGrabber.h>

#ifdef VISP_HAVE_FLYCAPTURE

#include <visp3/core/vpTime.h>

/*!
   Default constructor that consider the first camera found on the bus as active.
 */
vpFlyCaptureGrabber::vpFlyCaptureGrabber()
  : m_camera(), m_guid(), m_index(0), m_numCameras(0), m_rawImage(), m_connected(false), m_capture(false)
{
  m_numCameras = this->getNumCameras();
}

/*!
   Default destructor that closes the connection with the camera.
 */
vpFlyCaptureGrabber::~vpFlyCaptureGrabber()
{
  close();
}
/*!
  \return Return the number of cameras connected on the bus.
*/
unsigned int vpFlyCaptureGrabber::getNumCameras()
{
  unsigned int numCameras;
  FlyCapture2::BusManager busMgr;
  FlyCapture2::Error error = busMgr.GetNumOfCameras(&numCameras);
  if (error != FlyCapture2::PGRERROR_OK) {
    numCameras = 0;
  }
  return numCameras;
}

/*!
  Print to the output stream active camera information (serial number, camera model,
  camera vendor, sensor, resolution, firmaware version, ...).
  */
std::ostream &vpFlyCaptureGrabber::getCameraInfo(std::ostream & os)
{
  this->connect();

  FlyCapture2::CameraInfo camInfo;
  FlyCapture2::Error error = m_camera.GetCameraInfo(&camInfo);
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
  }

  os << "Serial number -" << camInfo.serialNumber << std::endl;
  os << "Camera model - " << camInfo.modelName << std::endl;
  os << "Camera vendor - " << camInfo.vendorName << std::endl;
  os << "Sensor - " << camInfo.sensorInfo << std::endl;
  os << "Resolution - " << camInfo.sensorResolution << std::endl;
  os << "Firmware version - " << camInfo.firmwareVersion << std::endl;
  os << "Firmware build time - " << camInfo.firmwareBuildTime << std::endl;
  return os;
}


/*!
  Return the handler to the active camera or NULL if the camera is not connected.
  This function was designed to provide a direct access to the FlyCapture SDK to
  get access to advanced functionalities that are not implemented in this class.

  We provide here after and example that shows how to use this function to access
  to the camera and check if a given video mode and framerate are supported by the
  camera.

\code
#include <visp3/core/vpImage.h>
#include <visp3/flycapture/vpFlyCaptureGrabber.h>

int main()
{
#if defined(VISP_HAVE_FLYCAPTURE)
  int nframes = 100;
  vpImage<unsigned char> I;

  vpFlyCaptureGrabber g;
  g.connect();
  FlyCapture2::Camera *handler = g.getCameraHandler();
  bool supported = false;
  handler->GetVideoModeAndFrameRateInfo(FlyCapture2::VIDEOMODE_1280x960Y8, FlyCapture2::FRAMERATE_60, &supported);
  if (supported)
    g.setVideoModeAndFrameRate(FlyCapture2::VIDEOMODE_1280x960Y8, FlyCapture2::FRAMERATE_60);
  g.startCapture();

  for(int i=0; i< nframes; i++) {
    g.acquire(I);
  }
#endif
}
\endcode


  The following code shows how to use this function to check if a given format7
  (here MODE_0, PIXEL_FORMAT_MONO8) is supported by the camera:
  \code
#include <visp3/flycapture/vpFlyCaptureGrabber.h>

int main()
{
#if defined(VISP_HAVE_FLYCAPTURE)
  vpFlyCaptureGrabber g;
  g.connect();
  FlyCapture2::Camera *handler = g.getCameraHandler();

  // Query for available Format 7 modes
  const FlyCapture2::Mode k_fmt7Mode = FlyCapture2::MODE_0;
  const FlyCapture2::PixelFormat k_fmt7PixFmt = FlyCapture2::PIXEL_FORMAT_MONO8;

  FlyCapture2::Format7Info fmt7Info;
  bool supported;
  fmt7Info.mode = k_fmt7Mode;
  FlyCapture2::Error error = handler->GetFormat7Info( &fmt7Info, &supported );
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
    return -1;
  }
  if (supported) {
    std::cout << "Max image pixels: (" << fmt7Info.maxWidth << ", " << fmt7Info.maxHeight << ")" << std::endl;
    std::cout << "Image Unit size: (" << fmt7Info.imageHStepSize << ", " << fmt7Info.imageVStepSize << ")" << std::endl;
    std::cout << "Offset Unit size: (" << fmt7Info.offsetHStepSize << ", " << fmt7Info.offsetVStepSize << ")" << std::endl;
    std::cout << "Pixel format bitfield: 0x" << fmt7Info.pixelFormatBitField << std::endl;

    if ( (k_fmt7PixFmt & fmt7Info.pixelFormatBitField) == 0 ) {
      // Pixel format not supported!
      std::cout << "Pixel format is not supported" << std::endl;
      return -1;
    }
  }
#endif
}
  \endcode

*/
FlyCapture2::Camera *vpFlyCaptureGrabber::getCameraHandler()
{
  this->connect();

  if (m_connected == true) {
    return &m_camera;
  }
  else {
    return NULL;
  }
}

/*!
  Return camera capture framerate. The camera needs to be connected using connect(),
  open(vpImage<unsigned char> &) or open(vpImage<vpRGBa> &).
  If the camera doesn't support framerate property, return -1.

  \code
  vpFlyCaptureGrabber g;
  g.setCameraIndex(0);
  g.connect();
  std::cout << "Frame rate: " << std::fixed << std::setprecision(3) << g.getFrameRate() << " fps" << std::endl;
  \endcode

  \sa setFrameRate()
 */
float vpFlyCaptureGrabber::getFrameRate()
{
  try {
    this->connect();

    FlyCapture2::PropertyInfo propInfo;
    propInfo = this->getPropertyInfo(FlyCapture2::FRAME_RATE);

    if (propInfo.present == true) {
      FlyCapture2::Property prop = this->getProperty(FlyCapture2::FRAME_RATE);
      return (prop.absValue);
    }
    else {
      return -1.f;
    }
  }
  catch(...) {
    return -1.f;
  }
}

/*!
  Return the serial id of a camera with \e index.
  \param index : Camera index.

  The following code shows how to retrieve the serial id of all the cameras that
  are connected on the bus.
  \code
#include <visp3/flycapture/vpFlyCaptureGrabber.h>

int main()
{
#if defined(VISP_HAVE_FLYCAPTURE)
  vpFlyCaptureGrabber g;
  unsigned int num_cameras = vpFlyCaptureGrabber::getNumCameras();
  for (unsigned int i=0; i<num_cameras; i++) {
    unsigned int serial_id = vpFlyCaptureGrabber::getCameraSerial(i);
    std::cout << "Camera with index " << i << " has serial id: " << serial_id << std::endl;
  }
#endif
}
  \endcode

  When two cameras are connected (PGR Flea3 in our case), we get the following:
  \code
Camera with index 0 has serial id: 15372913
Camera with index 1 has serial id: 15290004
  \endcode

  \sa setCameraSerial()
 */
unsigned int vpFlyCaptureGrabber::getCameraSerial(const unsigned int &index)
{
  unsigned int num_cameras = vpFlyCaptureGrabber::getNumCameras();
  if(index >= num_cameras) {
    throw (vpException(vpException::badValue,
                       "The camera with index %u is not present. Only %d cameras connected.",
                       index, num_cameras) );
  }
  unsigned int serial_id;
  FlyCapture2::BusManager busMgr;
  FlyCapture2::Error error;
  error = busMgr.GetCameraSerialNumberFromIndex(index, &serial_id);
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
    throw (vpException(vpException::fatalError,
                       "Cannot get camera with index %d serial id.", index) );
  }
  return serial_id;
}

/*!

  If multiples cameras are connected on the bus, select the camero to dial
  with.

  \param index : Current camera index, a value comprised between 0 (the first
  camera found on the bus) and the number of cameras found on the bus and returned by
  getNumCameras() minus 1. If two cameras are connected on the bus,
  setting \e index to one allows to communicate with the second
  one. This identifier is not unique. That is why, it is also possible
  to select a camera by its serial number, which is unique using
  setCameraSerial().

  \exception vpException::badValue : If the index is greater or equal to the number of
  cameras connected to the bus.
  */
void vpFlyCaptureGrabber::setCameraIndex(const unsigned int &index)
{
  if(index >= m_numCameras) {
    throw (vpException(vpException::badValue,
                       "The camera with index %u is not present. Only %d cameras connected.",
                       index, m_numCameras) );
  }

  m_index = index;
}

/*!
   Set the current camera from its unique serial identifier.
   \param serial_id : Camera serial id.

   The following example shows how to capture images from a camera that has seial id 15290004.
   \code
#include <visp3/flycapture/vpFlyCaptureGrabber.h>

int main()
{
#if defined(VISP_HAVE_FLYCAPTURE)
  int nframes = 100;
  vpImage<unsigned char> I;

  vpFlyCaptureGrabber g;
  g.setCameraSerial(15290004); // Set camera with serial id
  g.open(I);
  g.getCameraInfo(std::cout);

  for(int i=0; i< nframes; i++) {
    g.acquire(I);
  }
#endif
}
  \endcode

  \sa getCameraSerial()
 */
void vpFlyCaptureGrabber::setCameraSerial(const unsigned int &serial_id)
{
  FlyCapture2::BusManager busMgr;
  FlyCapture2::Error error;
  m_numCameras = this->getNumCameras();
  for (unsigned int i=0; i<m_numCameras; i++) {
    if (vpFlyCaptureGrabber::getCameraSerial(i) == serial_id) {
      m_index = i;
      return;
    }
  }
  throw (vpException(vpException::badValue,
                     "The camera with serial id %u is not present.",
                     serial_id) );
}

/*!
  Set camera property.

  \param propType : Property type.
  \param on : true to turn property on.
  \param auto_on : true to turn on auto mode, false to turn manual mode.
  \param value : value to set.
 */
void vpFlyCaptureGrabber::setProperty(const FlyCapture2::PropertyType &propType,
                                      const bool &on, const bool &auto_on,
                                      double value)
{
  this->connect();

  FlyCapture2::PropertyInfo propInfo;
  propInfo = this->getPropertyInfo(propType);

  if (propInfo.present) {
    FlyCapture2::Property prop;
    prop.type = propType;
    prop.onOff = on && propInfo.onOffSupported;
    prop.autoManualMode = auto_on && propInfo.autoSupported;
    prop.absControl = propInfo.absValSupported;
    value = std::max<double>(std::min<double>(value, propInfo.absMax),
                             propInfo.absMin);
    prop.absValue = value;
    FlyCapture2::Error error;
    error = m_camera.SetProperty(&prop);
    if (error != FlyCapture2::PGRERROR_OK) {
      error.PrintErrorTrace();
      throw (vpException(vpException::fatalError,
                         "Cannot set property %d.",
                         (int)propType) );
    }
  }
}

/*!
  Set camera frame rate.
  \param frame_rate [in/out] : Camera frame rate (fps).

  The following example shows how to use this function.
  \code
#include <iomanip>
#include <visp3/flycapture/vpFlyCaptureGrabber.h>

int main()
{
#if defined(VISP_HAVE_FLYCAPTURE)
  vpImage<unsigned char> I;

  vpFlyCaptureGrabber g;
  g.setCameraIndex(0);
  float framerate = 20;
  g.setFrameRate(framerate); // Set framerate to 20 fps
  std::cout << "Frame rate: " << std::fixed << std::setprecision(3) << framerate << " fps" << std::endl;
  std::cout << "Frame rate: " << std::fixed << std::setprecision(3) << g.getFrameRate() << " fps" << std::endl;

  g.open(I);
  while (1)
    g.acquire(I);
#endif
}
  \endcode

  \sa getFramerate()
 */
void vpFlyCaptureGrabber::setFrameRate(float &frame_rate)
{
  this->connect();

  this->setProperty(FlyCapture2::FRAME_RATE, true, false, frame_rate);
  FlyCapture2::Property prop = this->getProperty(FlyCapture2::FRAME_RATE);
  frame_rate = prop.absValue;
}

/*!
  Set camera shutter mode and parameter.
  \param auto_shutter [in/out]: It true set auto shutter, if false set manual shutter applying \e shutter_ms parameter.
  \param shutter_ms [in/out]: This is the speed at which the camera shutter opens and closes.

  To turn camera auto shutter on, use the following:
  \code
#include <visp3/flycapture/vpFlyCaptureGrabber.h>

int main()
{
#if defined(VISP_HAVE_FLYCAPTURE)
  vpImage<unsigned char> I;

  vpFlyCaptureGrabber g;
  g.setCameraIndex(0);

  bool shutter_mode = true;
  float shutter_ms = 10;
  g.setShutter(shutter_mode, shutter_ms);
  std::cout << "Shutter " << ((shutter_mode == true) ? "auto" : "manual") << " value: " << shutter_ms << std::endl;

  g.open(I);
  ...
#endif
}
  \endcode

  To set camera in manual shutter mode with 8 ms as shutter speed, modify the previous code like:
  \code
  g.setShutter(false, 8);
  \endcode

 */
void vpFlyCaptureGrabber::setShutter(bool &auto_shutter, float &shutter_ms)
{
  this->connect();

  this->setProperty(FlyCapture2::SHUTTER, true, auto_shutter, shutter_ms);
  FlyCapture2::Property prop = this->getProperty(FlyCapture2::SHUTTER);
  auto_shutter = prop.autoManualMode;
  shutter_ms = prop.absValue;
}
/*!
  Set camera gain mode and value.
  \param auto_gain [in/out]: It true set auto gain, if false set manual gain applying \e gain_db parameter.
  \param gain_db [in/out]: The amount of amplification that is applied to a pixel. An increase in gain
  can result in an increase in noise.

  To turn camera auto gain on, use the following:
  \code
#include <visp3/flycapture/vpFlyCaptureGrabber.h>

int main()
{
#if defined(VISP_HAVE_FLYCAPTURE)
  vpImage<unsigned char> I;

  vpFlyCaptureGrabber g;
  g.setCameraIndex(0);

  bool gain_mode = true;
  float gain_db = 5;
  g.setGain(gain_mode, gain_db);
  std::cout << "Gain " << ((gain_mode == true) ? "auto" : "manual") << " value: " << gain_db << std::endl;

  g.open(I);
  ...
#endif
}
  \endcode

  To set camera in manual gain mode with 8 db gain, modify the previous code like:
  \code
  g.setGain(false, 8);
  \endcode

 */
void vpFlyCaptureGrabber::setGain(bool &auto_gain, float &gain_db)
{
  this->connect();

  this->setProperty(FlyCapture2::SHUTTER, true, auto_gain, gain_db);
  FlyCapture2::Property prop = this->getProperty(FlyCapture2::GAIN);
  auto_gain = prop.autoManualMode;
  gain_db = prop.absValue;
}

/*!
  Return property values.
  \param prop_type : Property type.
 */
FlyCapture2::Property vpFlyCaptureGrabber::getProperty(const FlyCapture2::PropertyType &prop_type)
{
  this->connect();

  FlyCapture2::Property prop;
  prop.type = prop_type;
  FlyCapture2::Error error;
  error = m_camera.GetProperty( &prop );
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
    throw (vpException(vpException::fatalError,
                       "Cannot get property %d value.", (int)prop_type));
  }
  return prop;
}

/*!
  Return information concerning a given property type.
  \param prop_type : Property type.
  \exception vpException::fatalError : If property type doesn't exist.
 */
FlyCapture2::PropertyInfo
vpFlyCaptureGrabber::getPropertyInfo(const FlyCapture2::PropertyType &prop_type)
{
  this->connect();

  FlyCapture2::PropertyInfo propInfo;
  propInfo.type = prop_type;

  FlyCapture2::Error error;
  error = m_camera.GetPropertyInfo(&propInfo);
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
    throw (vpException(vpException::fatalError, "Cannot get property %d info.",
                       (int)prop_type) );
  }
  return propInfo;
}

/*!
  Set video mode and framerate of the active camera.
  \param video_mode : Camera video mode.
  \param frame_rate : Camera frame rate.

  The following example shows how to use this function to set the camera image resolution to
  1280 x 960, pixel format to Y8 and capture framerate to 60 fps.
  \code
#include <visp3/core/vpImage.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/flycapture/vpFlyCaptureGrabber.h>

int main()
{
#if defined(VISP_HAVE_FLYCAPTURE)
  int nframes = 100;
  vpImage<unsigned char> I;
  vpFlyCaptureGrabber g;

  g.setCameraIndex(0); // Default camera is the first on the bus
  g.setVideoModeAndFrameRate(FlyCapture2::VIDEOMODE_1280x960Y8, FlyCapture2::FRAMERATE_60);
  g.open(I);
  g.getCameraInfo(std::cout);

  for(int i=0; i< nframes; i++) {
    g.acquire(I);
  }
#endif
}
  \endcode
 */
void vpFlyCaptureGrabber::setVideoModeAndFrameRate(const FlyCapture2::VideoMode &video_mode,
                                                   const FlyCapture2::FrameRate &frame_rate)
{
  this->connect();

  FlyCapture2::Error error;
  error = m_camera.SetVideoModeAndFrameRate(video_mode, frame_rate);
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
    throw (vpException(vpException::fatalError, "Cannot set video mode and framerate.") );
  }
}

/*!
   Start active camera capturing images.

   \sa stopCapture()
 */
void vpFlyCaptureGrabber::startCapture()
{
  this->connect();

  if (m_capture == false) {

    FlyCapture2::Error error;
    error = m_camera.StartCapture();
    if (error != FlyCapture2::PGRERROR_OK) {
      error.PrintErrorTrace();
      throw (vpException(vpException::fatalError,
                         "Cannot start capture for camera with guid 0x%lx", m_guid));
    }
    m_capture = true;
  }
  if (m_connected && m_capture)
    init = true;
  else
    init = false;
}

/*!
   Stop active camera capturing images.

   \sa startCapture()
 */
void vpFlyCaptureGrabber::stopCapture()
{
  if (m_capture == true) {

    FlyCapture2::Error error;
    error = m_camera.StopCapture();
    if (error != FlyCapture2::PGRERROR_OK) {
      error.PrintErrorTrace();
      throw (vpException(vpException::fatalError, "Cannot stop capture.") );
    }
    m_capture = false;
  }
  if (m_connected && m_capture)
    init = true;
  else
    init = false;
}

/*!
   Connect the active camera.

   \sa disconnect()
 */
void vpFlyCaptureGrabber::connect()
{
  if (m_connected == false) {
    FlyCapture2::Error error;
    m_numCameras = this->getNumCameras();
    if (m_numCameras == 0) {
      throw (vpException(vpException::fatalError, "No camera found on the bus"));
    }

    FlyCapture2::BusManager busMgr;

    error = busMgr.GetCameraFromIndex(m_index, &m_guid);
    if (error != FlyCapture2::PGRERROR_OK) {
      error.PrintErrorTrace();
      throw (vpException(vpException::fatalError,
                         "Cannot retrieve guid of camera with index %u.",
                         m_index) );
    }
    // Connect to a camera
    error = m_camera.Connect(&m_guid);
    if (error != FlyCapture2::PGRERROR_OK) {
      error.PrintErrorTrace();
      throw (vpException(vpException::fatalError,
                         "Cannot connect to camera with guid 0x%lx", m_guid));
    }
    m_connected = true;
  }
  if (m_connected && m_capture)
    init = true;
  else
    init = false;
}

/*!
   Disconnect the active camera.

   \sa connect()
 */
void vpFlyCaptureGrabber::disconnect()
{
  if (m_connected == true) {

    FlyCapture2::Error error;
    error = m_camera.Disconnect();
    if (error != FlyCapture2::PGRERROR_OK) {
      error.PrintErrorTrace();
      throw (vpException(vpException::fatalError, "Cannot stop capture.") );
    }
    m_connected = false;
  }
  if (m_connected && m_capture)
    init = true;
  else
    init = false;
}

/*!
   Stop active camera capturing images and disconnect the active camera.
   If you want to use again this camera, you may call setCamera(const unsigned int &)
   and open(vpImage<unsigned char> &) or open(vpImage<vpRGBa> &) to connect again the camera.

   Similar then calling stopCapture() and disconnect():
   \code
   vpFlyCaptureGrabber g;
   ...
   g.stopCapture();
   g.disconnect();
   \endcode
 */
void vpFlyCaptureGrabber::close()
{
  this->stopCapture();
  this->disconnect();
}

/*!
  Acquire a gray level image from the active camera.

  \param I : Image data structure (8 bits image).
*/
void vpFlyCaptureGrabber::acquire(vpImage<unsigned char> &I)
{
  FlyCapture2::TimeStamp timestamp;
  this->acquire(I, timestamp);
}

/*!
  Acquire a gray level image from the active camera.

  \param I : Image data structure (8 bits image).

  \param timestamp : The acquisition timestamp.
*/
void vpFlyCaptureGrabber::acquire(vpImage<unsigned char> &I, FlyCapture2::TimeStamp &timestamp)
{
  this->open();

  FlyCapture2::Error error;
  // Retrieve an image
  error = m_camera.RetrieveBuffer( &m_rawImage );
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
    throw (vpException(vpException::fatalError,
                       "Cannot retrieve image for camera with guid 0x%lx",
                       m_guid) );
  }
  timestamp = m_rawImage.GetTimeStamp();

  // Create a converted image
  FlyCapture2::Image convertedImage;

  // Convert the raw image
  error = m_rawImage.Convert( FlyCapture2::PIXEL_FORMAT_MONO8, &convertedImage );
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
    throw (vpException(vpException::fatalError,
                       "Cannot convert image for camera with guid 0x%lx",
                       m_guid) );
  }
  height = convertedImage.GetRows();
  width = convertedImage.GetCols();
  unsigned char *data = convertedImage.GetData();
  I.resize(height, width);
  memcpy(I.bitmap, data, height*width);
}

/*!
  Acquire a color image from the active camera.

  \param I : Image data structure (RGBa image).
*/
void vpFlyCaptureGrabber::acquire(vpImage<vpRGBa> &I)
{
  FlyCapture2::TimeStamp timestamp;
  this->acquire(I, timestamp);
}

/*!
  Acquire a color image from the active camera.

  \param I : Image data structure (RGBa image).

  \param timestamp : The acquisition timestamp.
*/
void vpFlyCaptureGrabber::acquire(vpImage<vpRGBa> &I, FlyCapture2::TimeStamp &timestamp)
{
  this->open();

  FlyCapture2::Error error;
  // Retrieve an image
  error = m_camera.RetrieveBuffer( &m_rawImage );
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
    throw (vpException(vpException::fatalError,
                       "Cannot retrieve image for camera with guid 0x%lx",
                       m_guid) );
  }
  timestamp = m_rawImage.GetTimeStamp();

  // Create a converted image
  FlyCapture2::Image convertedImage;

  // Convert the raw image
  error = m_rawImage.Convert( FlyCapture2::PIXEL_FORMAT_RGBU, &convertedImage );
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
    throw (vpException(vpException::fatalError,
                       "Cannot convert image for camera with guid 0x%lx",
                       m_guid) );
  }
  height = convertedImage.GetRows();
  width = convertedImage.GetCols();
  unsigned char *data = convertedImage.GetData();
  I.resize(height, width);
  unsigned int bps = convertedImage.GetBitsPerPixel();
  memcpy(I.bitmap, data, width*height*bps/8);
}


/*!
   Connect to the active camera, start capture and retrieve an image.
   \param I : Captured image.
 */
void vpFlyCaptureGrabber::open(vpImage<unsigned char> &I)
{
  this->open();
  this->acquire(I);
}

/*!
   Connect to the active camera, start capture and retrieve an image.
   \param I : Captured image.
 */
void vpFlyCaptureGrabber::open(vpImage<vpRGBa> &I)
{
  this->open();
  this->acquire(I);
}

/*!
   Connect to the active camera and start capture.

   Similar then calling:
   \code
   vpFlyCaptureGrabber g;
   ...
   g.connect();
   g.startCapture();
   \endcode
 */
void vpFlyCaptureGrabber::open()
{
  this->connect();
  this->startCapture();
}

/*!
   Return true if camera power is available, false otherwise.

   \sa getCameraPower(), setCameraPowerOn(), setCameraPowerOff()
 */
bool vpFlyCaptureGrabber::isCameraPowerAvailable()
{
  this->connect();

  const unsigned int powerReg = 0x400;
  unsigned int powerRegVal = 0;

  FlyCapture2::Error error;
  error = m_camera.ReadRegister( powerReg, &powerRegVal );
  if ( error != FlyCapture2::PGRERROR_OK ) {
    return false;
  }

  return ( (powerRegVal & 0x00008000 ) != 0 );
}

/*!
  Return true if the camera is powered on, false otherwise

  \sa setCameraPower()
 */
bool vpFlyCaptureGrabber::getCameraPower()
{
  if ( ! isCameraPowerAvailable() )
    return false;
  const unsigned int powerReg = 0x610;
  unsigned int powerRegVal = 0 ;

  FlyCapture2::Error error;
  error = m_camera.ReadRegister( powerReg, &powerRegVal );
  if ( error != FlyCapture2::PGRERROR_OK ) {
    return false;
  }

  return ( (powerRegVal & (0x1 << 31)) != 0 );
}

/*!
  Power on/off the camera.

  \param on : true to power on the camera, false to power off the camera.

  The following example shows how to turn off a camera.
  \code
#include <visp3/flycapture/vpFlyCaptureGrabber.h>

int main()
{
#if defined(VISP_HAVE_FLYCAPTURE)
  vpFlyCaptureGrabber g;

  g.setCameraIndex(0);
  g.connect();

  bool power = g.getCameraPower();
  std::cout << "Camera is powered: " << ((power == true) ? "on" : "off") << std::endl;

  if (power)
    g.setCameraPower(false); // Power off the camera
#endif
}
  \endcode

  \sa getCameraPower()
 */
void vpFlyCaptureGrabber::setCameraPower(const bool &on)
{
  this->connect();

  if ( ! isCameraPowerAvailable() ) {
    throw (vpException(vpException::badValue,
                       "Cannot power on camera. Feature not available") );
  }

  // Power on the camera
  const unsigned int powerReg = 0x610;
  unsigned int powerRegVal = 0;

  powerRegVal = (on == true) ? 0x80000000 : 0x0;

  FlyCapture2::Error error;
  error  = m_camera.WriteRegister( powerReg, powerRegVal );
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
    throw (vpException(vpException::fatalError, "Cannot power on the camera.") );
  }

  const unsigned int millisecondsToSleep = 100;
  unsigned int regVal = 0;
  unsigned int retries = 10;

  // Wait for camera to complete power-up
  do
  {
    vpTime::wait(millisecondsToSleep);
    error = m_camera.ReadRegister(powerReg, &regVal);
    if (error == FlyCapture2::PGRERROR_TIMEOUT) {
      // ignore timeout errors, camera may not be responding to
      // register reads during power-up
    }
    else if (error != FlyCapture2::PGRERROR_OK) {
      error.PrintErrorTrace();
      throw (vpException(vpException::fatalError, "Cannot power on the camera.") );
    }

    retries--;
  } while ((regVal & powerRegVal) == 0 && retries > 0);

  // Check for timeout errors after retrying
  if (error == FlyCapture2::PGRERROR_TIMEOUT) {
    error.PrintErrorTrace();
    throw (vpException(vpException::fatalError, "Cannot power on the camera. Timeout occur") );
  }
}

#else
// Work arround to avoid warning: libvisp_flycapture.a(vpFlyCaptureGrabber.cpp.o) has no symbols
void dummy_vpFlyCaptureGrabber() {};
#endif


