/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Static functions for basic image processing functions.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/



/*!
  \file vpImgproc.h
  \brief Static functions for basic image processing functions.

*/

#ifndef __vpImgproc_h__
#define __vpImgproc_h__

#include <visp3/core/vpImage.h>


namespace vp
{
  VISP_EXPORT void adjust(const vpImage<unsigned char> &I1, vpImage<unsigned char> &I2, const double alpha, const double beta);
  VISP_EXPORT void adjust(const vpImage<vpRGBa> &I1, vpImage<vpRGBa> &I2, const double alpha, const double beta);

  VISP_EXPORT void equalizeHistogram(const vpImage<unsigned char> &I1, vpImage<unsigned char> &I2);
  VISP_EXPORT void equalizeHistogram(const vpImage<vpRGBa> &I1, vpImage<vpRGBa> &I2, const bool useHSV=false);
}

#endif
