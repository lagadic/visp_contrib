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
  : m_camera(), m_guid(), m_index(0), m_numCameras(0), m_rawImage()
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
  open();

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
  \param index : Current camera index, a value comprised between 0 (the first
  camera found on the bus) and the number of cameras found on the bus and returned by
  getNumCameras() minus 1. If two cameras are connected on the bus,
  setting \e index to one allows to communicate with the second
  one. This identifier is not unique.

  \return Camera guid.
 */
FlyCapture2::PGRGuid vpFlyCaptureGrabber::getGuid(const unsigned int &index) const
{
  if(index >= m_numCameras) {
    throw (vpException(vpException::badValue,
                       "The camera with index %u is not present. Only %d cameras connected.",
                       index, m_numCameras) );
  }
  FlyCapture2::PGRGuid  guid;
  FlyCapture2::BusManager busMgr;

  FlyCapture2::Error error = busMgr.GetCameraFromIndex(m_index, &guid);
  return guid;
}
/*!

  If multiples cameras are connected on the bus, select the camero to dial
  with.

  \param index : Current camera index, a value comprised between 0 (the first
  camera found on the bus) and the number of cameras found on the bus and returned by
  getNumCameras() minus 1. If two cameras are connected on the bus,
  setting \e index to one allows to communicate with the second
  one. This identifier is not unique. That is why, it is also possible
  to select a camera by its GUID, which is unique using
  setCamera(const FlyCapture2::PGRGuid &).

  \exception vpException::badValue : If the index is greater or equal to the number of
  cameras connected to the bus.
  */
void vpFlyCaptureGrabber::setCamera(const unsigned int &index)
{
  if(index >= m_numCameras) {
    throw (vpException(vpException::badValue,
                       "The camera with index %u is not present. Only %d cameras connected.",
                       index, m_numCameras) );
  }

  m_index = index;
}

/*!
   Set the current camera from its unique guid identifier.
   \param guid : Camera guid.
 */
void vpFlyCaptureGrabber::setCamera(const FlyCapture2::PGRGuid &guid)
{
  m_guid = guid;
}

/*!
   Stop active camera capturing images and disconnect the active camera.
   If you want to use again this camera, you may call setCamera(const unsigned int &)
   and open(vpImage<unsigned char> &) or open(vpImage<vpRGBa> &) to connect again the camera.
 */
void vpFlyCaptureGrabber::close()
{
  if (init == true) {

    FlyCapture2::Error error;
    error = m_camera.StopCapture();
    if (error != FlyCapture2::PGRERROR_OK) {
      error.PrintErrorTrace();
      throw (vpException(vpException::fatalError, "Cannot stop capture.") );
    }

    // Disconnect the camera
    error = m_camera.Disconnect();
    if (error != FlyCapture2::PGRERROR_OK) {
      error.PrintErrorTrace();
      throw (vpException(vpException::fatalError, "Cannot disconnect camera.") );
    }
  }

  init = false;
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
  memcpy(I.bitmap, data, height, width);
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
 */
void vpFlyCaptureGrabber::open()
{
  if (init == false) {
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

    // Start capturing images
    error = m_camera.StartCapture();
    if (error != FlyCapture2::PGRERROR_OK) {
      error.PrintErrorTrace();
      throw (vpException(vpException::fatalError,
                         "Cannot start capture for camera with guid 0x%lx", m_guid));
    }

    init = true;
  }
}

#else
// Work arround to avoid warning: libvisp_flycapture.a(vpFlyCaptureGrabber.cpp.o) has no symbols
void dummy_vpFlyCaptureGrabber() {};
#endif


