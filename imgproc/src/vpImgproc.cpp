/****************************************************************************
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

  \param I : The grayscale image to adjust the brightness.
  \param alpha : Multiplication coefficient.
  \param beta : Constant value added to the old intensity.
*/
void vp::adjust(vpImage<unsigned char> &I, const double alpha, const double beta) {
  //Construct the look-up table
  unsigned char lut[256];
  for(unsigned int i = 0; i < 256; i++) {
    lut[i] = vpMath::saturate<unsigned char>(alpha * i + beta);
  }

  //Apply the transformation using a LUT
  I.performLut(lut);
}

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

  vp::adjust(I2, alpha, beta);
}

/*!
  Adjust the brightness of a color image such as the new intensity is alpha x old_intensity + beta.

  \param I : The color image to adjust the brightness.
  \param alpha : Multiplication coefficient.
  \param beta : Constant value added to the old intensity.
*/
void vp::adjust(vpImage<vpRGBa> &I, const double alpha, const double beta) {
  //Construct the look-up table
  vpRGBa lut[256];
  for(unsigned int i = 0; i < 256; i++) {
    lut[i].R = vpMath::saturate<unsigned char>(alpha * i + beta);
    lut[i].G = vpMath::saturate<unsigned char>(alpha * i + beta);
    lut[i].B = vpMath::saturate<unsigned char>(alpha * i + beta);
    lut[i].A = vpMath::saturate<unsigned char>(alpha * i + beta);
  }

  //Apply the transformation using a LUT
  I.performLut(lut);
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

  vp::adjust(I2, alpha, beta);
}

/*!
  Adjust the contrast of a grayscale image by performing an histogram equalization.
  The intensity distribution is redistributed over the full [0 - 255] range such as the cumulative histogram
  distribution becomes linear.

  \param I : The grayscale image to apply histogram equalization.
*/
void vp::equalizeHistogram(vpImage<unsigned char> &I) {
  if(I.getWidth()*I.getHeight() == 0) {
    return;
  }

  //Calculate the histogram
  vpHistogram hist;
  hist.calculate(I);

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
  unsigned int nbPixels = I.getWidth()*I.getHeight();
  for(unsigned int x = minValue; x <= maxValue; x++) {
    lut[x] = vpMath::round( (cdf[x]-cdfMin) / (double) (nbPixels-cdfMin) * 255.0 );
  }

  I.performLut(lut);
}

/*!
  Adjust the contrast of a grayscale image by performing an histogram equalization.
  The intensity distribution is redistributed over the full [0 - 255] range such as the cumulative histogram
  distribution becomes linear.

  \param I1 : The first grayscale image.
  \param I2 : The second grayscale image after histogram equalization.
*/
void vp::equalizeHistogram(const vpImage<unsigned char> &I1, vpImage<unsigned char> &I2) {
  I2 = I1;
  vp::equalizeHistogram(I2);
}

/*!
  Adjust the contrast of a color image by performing an histogram equalization.
  The intensity distribution is redistributed over the full [0 - 255] range such as the cumulative histogram
  distribution becomes linear.

  \param I : The color image to apply histogram equalization.
  \param useHSV : If true, the histogram equalization is performed on the value channel (in HSV space), otherwise
  the histogram equalization is performed independently on the RGB channels.
*/
void vp::equalizeHistogram(vpImage<vpRGBa> &I, const bool useHSV) {
  if(I.getWidth()*I.getHeight() == 0) {
    return;
  }

  if(!useHSV) {
    //Split the RGBa image into 4 images
    vpImage<unsigned char> pR(I.getHeight(), I.getWidth());
    vpImage<unsigned char> pG(I.getHeight(), I.getWidth());
    vpImage<unsigned char> pB(I.getHeight(), I.getWidth());
    vpImage<unsigned char> pa(I.getHeight(), I.getWidth());

    vpImageConvert::split(I, &pR, &pG, &pB, &pa);

    //Apply histogram equalization for each channel
    vp::equalizeHistogram(pR, pR);
    vp::equalizeHistogram(pG, pG);
    vp::equalizeHistogram(pB, pB);

    //Merge the result in I
    unsigned int size = I.getWidth()*I.getHeight();
    unsigned char *ptrStart = (unsigned char*) I.bitmap;
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
  I2 = I1;
  vp::equalizeHistogram(I2, useHSV);
}

/*!
  Perform a gamma correction on a grayscale image.

  \param I : The grayscale image to apply gamma correction.
  \param gamma : Gamma value.
*/
void vp::gammaCorrection(vpImage<unsigned char> &I, const double gamma) {
  double inverse_gamma = 1.0;
  if(gamma > 0) {
    inverse_gamma = 1.0 / gamma;
  } else {
    throw vpException(vpException::badValue, "The gamma value must be positive !");
  }

  //Construct the look-up table
  unsigned char lut[256];
  for(unsigned int i = 0; i < 256; i++) {
    lut[i] = vpMath::saturate<unsigned char>( pow( (double) i / 255.0, inverse_gamma ) * 255.0 );
  }

  I.performLut(lut);
}

/*!
  Perform a gamma correction on a grayscale image.

  \param I1 : The first grayscale image.
  \param I2 : The second grayscale image after gamma correction.
  \param gamma : Gamma value.
*/
void vp::gammaCorrection(const vpImage<unsigned char> &I1, vpImage<unsigned char> &I2, const double gamma) {
  I2 = I1;
  vp::gammaCorrection(I2, gamma);
}

/*!
  Perform a gamma correction on a color image.

  \param I : The color image to apply gamma correction.
  \param gamma : Gamma value.
*/
void vp::gammaCorrection(vpImage<vpRGBa> &I, const double gamma) {
  double inverse_gamma = 1.0;
  if(gamma > 0) {
    inverse_gamma = 1.0 / gamma;
  } else {
    throw vpException(vpException::badValue, "The gamma value must be positive !");
  }

  //Construct the look-up table
  vpRGBa lut[256];
  for(unsigned int i = 0; i < 256; i++) {
    lut[i].R = vpMath::saturate<unsigned char>( pow( (double) i / 255.0, inverse_gamma ) * 255.0 );
    lut[i].G = vpMath::saturate<unsigned char>( pow( (double) i / 255.0, inverse_gamma ) * 255.0 );
    lut[i].B = vpMath::saturate<unsigned char>( pow( (double) i / 255.0, inverse_gamma ) * 255.0 );
    lut[i].A = vpMath::saturate<unsigned char>( pow( (double) i / 255.0, inverse_gamma ) * 255.0 );
  }

  I.performLut(lut);
}

/*!
  Perform a gamma correction on a color image.

  \param I1 : The first color image.
  \param I2 : The second color image after gamma correction.
  \param gamma : Gamma value.
*/
void vp::gammaCorrection(const vpImage<vpRGBa> &I1, vpImage<vpRGBa> &I2, const double gamma) {
  I2 = I1;
  vp::gammaCorrection(I2, gamma);
}

/*!
  Stretch the contrast of a grayscale image.

  \param I : The grayscale image to stretch the contrast.
*/
void vp::stretchContrast(vpImage<unsigned char> &I) {
  //Find min and max intensity values
  unsigned char min = 255, max = 0;
  I.getMinMaxValue(min, max);

  unsigned char range = max - min;

  //Construct the look-up table
  unsigned char lut[256];
  if(range > 0) {
    for(unsigned int x = min; x <= max; x++) {
      lut[x] = 255 * (x - min) / range;
    }
  } else {
    lut[min] = min;
  }

  I.performLut(lut);
}

/*!
  Stretch the contrast of a grayscale image.

  \param I1 : The first input grayscale image.
  \param I2 : The second output grayscale image.
*/
void vp::stretchContrast(const vpImage<unsigned char> &I1, vpImage<unsigned char> &I2) {
  //Copy I1 to I2
  I2 = I1;
  vp::stretchContrast(I2);
}

/*!
  Stretch the contrast of a color image.

  \param I1 : The color image to stretch the contrast.
*/
void vp::stretchContrast(vpImage<vpRGBa> &I) {
  //Find min and max intensity values
  vpRGBa min = 255, max = 0;

  //Split the RGBa image into 4 images
  vpImage<unsigned char> pR(I.getHeight(), I.getWidth());
  vpImage<unsigned char> pG(I.getHeight(), I.getWidth());
  vpImage<unsigned char> pB(I.getHeight(), I.getWidth());
  vpImage<unsigned char> pa(I.getHeight(), I.getWidth());

  vpImageConvert::split(I, &pR, &pG, &pB, &pa);
  //Min max values calculated for each channel
  unsigned char minChannel, maxChannel;
  pR.getMinMaxValue(minChannel, maxChannel);
  min.R = minChannel;
  max.R = maxChannel;

  pG.getMinMaxValue(minChannel, maxChannel);
  min.G = minChannel;
  max.G = maxChannel;

  pB.getMinMaxValue(minChannel, maxChannel);
  min.B = minChannel;
  max.B = maxChannel;

  pa.getMinMaxValue(minChannel, maxChannel);
  min.A = minChannel;
  max.A = maxChannel;


  //Construct the look-up table
  vpRGBa lut[256];
  unsigned char rangeR = max.R - min.R;
  if(rangeR > 0) {
    for(unsigned int x = min.R; x <= max.R; x++) {
      lut[x].R = 255 * (x - min.R) / rangeR;
    }
  } else {
    lut[min.R].R = min.R;
  }

  unsigned char rangeG = max.G - min.G;
  if(rangeG > 0) {
    for(unsigned int x = min.G; x <= max.G; x++) {
      lut[x].G = 255 * (x - min.G) / rangeG;
    }
  } else {
    lut[min.G].G = min.G;
  }

  unsigned char rangeB = max.B - min.B;
  if(rangeB > 0) {
    for(unsigned int x = min.B; x <= max.B; x++) {
      lut[x].B = 255 * (x - min.B) / rangeB;
    }
  } else {
    lut[min.B].B = min.B;
  }

  unsigned char rangeA = max.A - min.A;
  if(rangeA > 0) {
    for(unsigned int x = min.A; x <= max.A; x++) {
      lut[x].A = 255 * (x - min.A) / rangeA;
    }
  } else {
    lut[min.A].A = min.A;
  }

  I.performLut(lut);
}

/*!
  Stretch the contrast of a color image.

  \param I1 : The first input color image.
  \param I2 : The second output color image.
*/
void vp::stretchContrast(const vpImage<vpRGBa> &I1, vpImage<vpRGBa> &I2) {
  //Copy I1 to I2
  I2 = I1;
  vp::stretchContrast(I2);
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
