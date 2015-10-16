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
 * Convert image types.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/

/*!
  \file vpImgproc.cpp
  \brief Static functions for basic image processing functions.
*/

#include <visp3/imgproc/vpImgproc.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpHistogram.h>
#include <visp3/core/vpImageConvert.h>


/*!
  Adjust the brightness of a grayscale image such as the new intensity is alpha x old_intensity + beta.

  \param I1 : The original grayscale image.
  \param I2 : The grayscale image after adjusting pixel intensities.
  \param alpha : Multiplication coefficient.
  \param beta : Constant value added to the old intensity.
*/
void vp::adjust(const vpImage<unsigned char> &I1, vpImage<unsigned char> &I2, const double alpha, const double beta) {
  //Copy I1 to I2
  I2 = I1;

  //Construct the look-up table
  unsigned char lut[256];
  for(unsigned int i = 0; i < 256; i++) {
    lut[i] = vpMath::saturate<unsigned char>(alpha * i + beta);
  }

  //Apply the transformation using a LUT
  I2.performLut(lut);
}

/*!
  Adjust the brightness of a color image such as the new intensity is alpha x old_intensity + beta.

  \param I1 : The original color image.
  \param I2 : The color image after adjusting pixel intensities.
  \param alpha : Multiplication coefficient.
  \param beta : Constant value added to the old intensity.
*/
void vp::adjust(const vpImage<vpRGBa> &I1, vpImage<vpRGBa> &I2, const double alpha, const double beta) {
  //Copy I1 to I2
  I2 = I1;

  //Construct the look-up table
  vpRGBa lut[256];
  for(unsigned int i = 0; i < 256; i++) {
    lut[i].R = vpMath::saturate<unsigned char>(alpha * i + beta);
    lut[i].G = vpMath::saturate<unsigned char>(alpha * i + beta);
    lut[i].B = vpMath::saturate<unsigned char>(alpha * i + beta);
    lut[i].A = vpMath::saturate<unsigned char>(alpha * i + beta);
  }

  //Apply the transformation using a LUT
  I2.performLut(lut);
}

/*!
  Adjust the contrast of a grayscale image by performing an histogram equalization.
  The intensity distribution is redistributed over the full [0 - 255] range such as the cumulative histogram
  distribution becomes linear.

  \param I1 : The first grayscale image.
  \param I2 : The second grayscale image after histogram equalization.
*/
void vp::equalizeHistogram(const vpImage<unsigned char> &I1, vpImage<unsigned char> &I2) {
  if(I1.getWidth()*I1.getHeight() == 0) {
    return;
  }

  //Calculate the histogram
  vpHistogram hist;
  hist.calculate(I1);

  //Calculate the cumulative distribution function
  unsigned int cdf[256];
  unsigned int cdfMin = std::numeric_limits<unsigned int>::max(), cdfMax = 0;
  unsigned int minValue = std::numeric_limits<unsigned int>::max(), maxValue = 0;
  cdf[0] = hist[0];
  for(unsigned int i = 1; i < 256; i++) {
    cdf[i] = cdf[i-1] + hist[i];

    if(cdf[i] < cdfMin && cdf[i] > 0) {
      cdfMin = cdf[i];
      minValue = i;
    }

    if(cdf[i] > cdfMax) {
      cdfMax = cdf[i];
      maxValue = i;
    }
  }

  //Construct the look-up table
  unsigned char lut[256];
  unsigned int nbPixels = I1.getWidth()*I1.getHeight();
  for(unsigned int x = minValue; x <= maxValue; x++) {
    lut[x] = vpMath::round( (cdf[x]-cdfMin) / (double) (nbPixels-cdfMin) * 255.0 );
  }

  I2 = I1;
  I2.performLut(lut);
}

/*!
  Adjust the contrast of a color image by performing an histogram equalization.
  The intensity distribution is redistributed over the full [0 - 255] range such as the cumulative histogram
  distribution becomes linear.

  \param I1 : The first color image.
  \param I2 : The second color image after histogram equalization.
  \param useHSV : If true, the histogram equalization is performed on the value channel (in HSV space), otherwise
  the histogram equalization is performed independently on the RGB channels.
*/
void vp::equalizeHistogram(const vpImage<vpRGBa> &I1, vpImage<vpRGBa> &I2, const bool useHSV) {
  if(I1.getWidth()*I1.getHeight() == 0) {
    return;
  }

  if(!useHSV) {
    //Split the RGBa image into 4 images
    vpImage<unsigned char> pR(I1.getHeight(), I1.getWidth());
    vpImage<unsigned char> pG(I1.getHeight(), I1.getWidth());
    vpImage<unsigned char> pB(I1.getHeight(), I1.getWidth());
    vpImage<unsigned char> pa(I1.getHeight(), I1.getWidth());

    vpImageConvert::split(I1, &pR, &pG, &pB, &pa);

    //Apply histogram equalization for each channel
    vp::equalizeHistogram(pR, pR);
    vp::equalizeHistogram(pG, pG);
    vp::equalizeHistogram(pB, pB);

    //Merge the result in I2
    I2 = vpImage<vpRGBa>(I1.getHeight(), I1.getWidth());
    unsigned int size = I2.getWidth()*I2.getHeight();
    unsigned char *ptrStart = (unsigned char*) I2.bitmap;
    unsigned char *ptrEnd = ptrStart + size*4;
    unsigned char *ptrCurrent = ptrStart;

    unsigned int cpt = 0;
    while(ptrCurrent != ptrEnd) {
      *ptrCurrent = pR.bitmap[cpt];
      ++ptrCurrent;

      *ptrCurrent = pG.bitmap[cpt];
      ++ptrCurrent;

      *ptrCurrent = pB.bitmap[cpt];
      ++ptrCurrent;

      *ptrCurrent = pa.bitmap[cpt];
      ++ptrCurrent;

      cpt++;
    }
  } else {
    //TODO: add conversion RGBa and HSV
  }
//    vpImage<double> hue(I1.getHeight(), I1.getWidth());
//    vpImage<double> saturation(I1.getHeight(), I1.getWidth());
//    vpImage<double> value(I1.getHeight(), I1.getWidth());
//
//    unsigned int size = I1.getWidth()*I1.getHeight();
//    //Convert from RGBa to HSV
//    vpImageConvert::RGBaToHSV((unsigned char*) I1.bitmap, (double*) hue.bitmap, (double*) saturation.bitmap,
//        (double*) value.bitmap, size);
//
//    //Convert from double image to unsigned char image
//    vpImage<unsigned char> valueImg(I1.getHeight(), I1.getWidth());
//    for(unsigned int cpt = 0; cpt < size; cpt++) {
//      valueImg.bitmap[cpt] = (unsigned char) (value.bitmap[cpt] * 255.0);
//    }
//
//    //Histogram equalization on the value plane
//    vpImageTools::equalizeHistogram(valueImg, valueImg);
//
//    //Convert from unsigned char image to double image
//    for(unsigned int cpt = 0; cpt < size; cpt++) {
//      value.bitmap[cpt] = valueImg.bitmap[cpt] / 255.0;
//    }
//
//    //Convert from HSV to RGBa
//    vpImageConvert::HSVToRGBa((double*) hue.bitmap, (double*) saturation.bitmap,
//        (double*) value.bitmap, (unsigned char*) I2.bitmap, size);
//  }
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
