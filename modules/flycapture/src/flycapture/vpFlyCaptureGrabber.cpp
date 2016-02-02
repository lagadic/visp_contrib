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
  std::cout << "Frame rate is " << std::fixed << std::setprecision(3) << g.getFrameRate() << " fps" << std::endl;
  \endcode
 */
float vpFlyCaptureGrabber::getFrameRate()
{
  // Check if the camera supports the FRAME_RATE property
  FlyCapture2::Error error;
  FlyCapture2::PropertyInfo propInfo;
  propInfo.type = FlyCapture2::FRAME_RATE;
  error = m_camera.GetPropertyInfo( &propInfo );
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
    throw (vpException(vpException::fatalError,
                       "Cannot get camera framerate. Do you connect the camera using connect() or open() ?") );
  }

  if (propInfo.present == true) {
    FlyCapture2::Property frmRate;
    frmRate.type = FlyCapture2::FRAME_RATE;
    error = m_camera.GetProperty( &frmRate );
    return frmRate.absValue;
  }
  return -1.f;
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
  Set video mode and framerate of the active camera.
  \param videoMode : Camera video mode.
  \param frameRate : Camera frame rate.

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
void vpFlyCaptureGrabber::setVideoModeAndFrameRate(const FlyCapture2::VideoMode videoMode,
                                                   const FlyCapture2::FrameRate &frameRate)
{
  connect();

  FlyCapture2::Error error;
  error = m_camera.SetVideoModeAndFrameRate(videoMode, frameRate);
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

   Similar then calling:
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
  open();

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
  open();

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

#else
// Work arround to avoid warning: libvisp_flycapture.a(vpFlyCaptureGrabber.cpp.o) has no symbols
void dummy_vpFlyCaptureGrabber() {};
#endif


