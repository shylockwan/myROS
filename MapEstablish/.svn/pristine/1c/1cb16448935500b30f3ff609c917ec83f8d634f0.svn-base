#ifndef __ZXINGDECODER_H__
#define __ZXINGDECODER_H__

#include <iostream>
#include <fstream>
#include <string>
#include "ImageReaderSource.h"
#include <zxing/common/Counted.h>
#include <zxing/Binarizer.h>
#include <zxing/MultiFormatReader.h>
#include <zxing/Result.h>
#include <zxing/ReaderException.h>
#include <zxing/common/GlobalHistogramBinarizer.h>
#include <zxing/common/HybridBinarizer.h>
#include <exception>
#include <zxing/Exception.h>
#include <zxing/common/IllegalArgumentException.h>
#include <zxing/BinaryBitmap.h>
#include <zxing/DecodeHints.h>

#include <zxing/qrcode/QRCodeReader.h>
#include <zxing/multi/qrcode/QRCodeMultiReader.h>
#include <zxing/multi/ByQuadrantReader.h>
#include <zxing/multi/MultipleBarcodeReader.h>
#include <zxing/multi/GenericMultipleBarcodeReader.h>

#include "opencvbitmapsource.h"

using namespace std;
using namespace zxing;
using namespace zxing::multi;
using namespace zxing::qrcode;

class ZxingDecoder
{
public:
	ZxingDecoder();
	~ZxingDecoder();
public:
	//float calRotationAngle(ArrayRef< Ref<ResultPoint> >& patterns);
	string decode(const cv::Mat& image, float& angle);

private:
	string decodeImage(Ref<OpenCVBitmapSource>& source, float& angle);
};

#endif
