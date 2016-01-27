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

#ifndef __vpFlyCaptureGrabber_h_
#define __vpFlyCaptureGrabber_h_

#include <ostream>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpFrameGrabber.h>
#include <visp3/flycapture/vpConfigFlycapture.h>

#ifdef VISP_HAVE_FLYCAPTURE

#include <FlyCapture2.h>

/*!
  \file vpFlyCaptureGrabber.h
  \brief Wrapper over PointGrey FlyCapture SDK to capture images from PointGrey cameras.
*/
/*!
  \class vpFlyCaptureGrabber
  \ingroup group_sensor_camera

  Allows to grab images from a PointGrey camera using FlyCapture SDK.

  To use this class install first FlyCapture that could be downloaded
  from https://www.ptgrey.com/support/downloads, and build ViSP with FlyCapture
  support.

  The following example shows how to use this class to capture images
  from the first camera that is found.
  \code
#include <visp3/core/vpImage.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/flycapture/vpFlyCaptureGrabber.h>

int main()
{
#if defined(VISP_HAVE_FLYCAPTURE)
  try {
    int nframes = 100;
    vpImage<unsigned char> I;
    char filename[255];
    std::cout << "Number of cameras detected: " << vpFlyCaptureGrabber::getNumCameras() << std::endl;

    vpFlyCaptureGrabber g;
    g.setCamera(0); // Default camera is the first on the bus
    g.open(I);
    g.getCameraInfo(std::cout);

    for(int i=0; i< nframes; i++) {
      g.acquire(I);
      sprintf(filename, "image%04d.pgm", i);
      vpImageIo::write(I, filename);
    }
  }
  catch(vpException &e) {
    std::cout << "Catch an exception: " << e.getStringMessage() << std::endl;
  }
#else
  std::cout << "You should install PointGrey FlyCapture SDK to use this binary..." << std::endl;
#endif
}
  \endcode

  If more than one camera is detected, you can use setCamera(const unsigned int &) to
  select the camera of interest.

  It is also possible to capture images from multiple cameras. The following example
  shows how to capture simultaneously images from multiple cameras.

  \code
#include <visp3/core/vpImage.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/flycapture/vpFlyCaptureGrabber.h>

int main()
{
#if defined(VISP_HAVE_FLYCAPTURE)
  try {
    int nframes = 100;
    char filename[255];
    unsigned int numCameras = vpFlyCaptureGrabber::getNumCameras();

    std::cout << "Number of cameras detected: " << numCameras << std::endl;

    vpFlyCaptureGrabber *g = new vpFlyCaptureGrabber [numCameras];
    std::vector< vpImage<unsigned char> > I(numCameras);

    for(unsigned int cam=0; cam < numCameras; cam++) {
      g[cam].setCamera(cam); // Default camera is the first on the bus
      g[cam].open(I[cam]);
      g[cam].getCameraInfo(std::cout);
    }

    for(int i=0; i< nframes; i++) {
      for(unsigned int cam=0; cam < numCameras; cam++) {
        g[cam].acquire(I[cam]);
        sprintf(filename, "image-camera%d-%04d.pgm", cam, i);
        vpImageIo::write(I[cam], filename);
      }
    }
    delete [] g;
  }
  catch(vpException &e) {
    std::cout << "Catch an exception: " << e.getStringMessage() << std::endl;
  }
#else
  std::cout << "You should install PointGrey FlyCapture SDK to use this binary..." << std::endl;
#endif
}
  \endcode
 */
class VISP_EXPORT vpFlyCaptureGrabber : public vpFrameGrabber
{
public:
  vpFlyCaptureGrabber();
  virtual ~vpFlyCaptureGrabber();

  void acquire(vpImage<unsigned char> &I);
  void acquire(vpImage<unsigned char> &I, FlyCapture2::TimeStamp &timestamp);
  void acquire(vpImage<vpRGBa> &I);
  void acquire(vpImage<vpRGBa> &I, FlyCapture2::TimeStamp &timestamp);

  void close();

  std::ostream &getCameraInfo(std::ostream & os); // Cannot be const since FlyCapture2::Camera::GetCameraInfo() isn't
  FlyCapture2::PGRGuid getGuid(const unsigned int &index) const;
  static unsigned int getNumCameras();

  void open(vpImage<unsigned char> &I);
  void open(vpImage<vpRGBa> &I);

  void setCamera(const unsigned int &index);
  void setCamera(const FlyCapture2::PGRGuid &guid);

protected:
  void open();

protected:
  FlyCapture2::Camera m_camera; //!< Pointer to each camera
  FlyCapture2::PGRGuid m_guid; //!< Active camera guid
  unsigned int m_index; //!< Active camera index
  unsigned int m_numCameras; //!< Number of connected cameras
  FlyCapture2::Image m_rawImage; //!< Image buffer
};

#endif
#endif
