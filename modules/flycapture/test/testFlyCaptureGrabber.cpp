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
 * Description:
 * Test for PointGrey FlyCapture SDK wrapper.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example testFlyCaptureGrabber.cpp

  Test PointGrey FlyCapture SDK wrapper to capture and display images.
*/

#include <iomanip>

#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/flycapture/vpFlyCaptureGrabber.h>

int main()
{
#if defined(VISP_HAVE_FLYCAPTURE)
  try {
    unsigned int numCameras = vpFlyCaptureGrabber::getNumCameras();
    std::cout << "Number of cameras detected: " << numCameras << std::endl;
    if (numCameras == 0)
      return 0;

    vpFlyCaptureGrabber g;
    vpImage<unsigned char> I;

    g.setCameraIndex(0);

    g.open(I);
    g.getCameraInfo(std::cout);

    std::cout << "Frame rate is " << std::fixed << std::setprecision(3) << g.getFrameRate() << " fps" << std::endl;

#if defined(VISP_HAVE_X11)
    vpDisplayX d(I);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I);
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV d(I);
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif

    bool ret = false;
    while(!ret) {
      g.acquire(I);
      vpDisplay::display(I);
      vpDisplay::displayText(I, 10, 10, "Click to quit...", vpColor::red);
      vpDisplay::flush(I);
      ret = vpDisplay::getClick(I, false);
    }
  }
  catch(vpException &e) {
    std::cout << "Catch an exception: " << e.getStringMessage() << std::endl;
  }
#else
  std::cout << "You should install PointGrey FlyCapture SDK to use this binary..." << std::endl;
#endif
}
