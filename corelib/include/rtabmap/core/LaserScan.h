/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef CORELIB_INCLUDE_RTABMAP_CORE_LASERSCAN_H_
#define CORELIB_INCLUDE_RTABMAP_CORE_LASERSCAN_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <rtabmap/core/Transform.h>

namespace rtabmap {

class RTABMAP_EXP LaserScan
{
public:
	enum Format {
		kUnknown			=	0,
		kXY						=	1,
		kXYI					=	2,
		kXYNormal			=	3,
		kXYINormal		=	4,
		kXYZ					=	5,
		kXYZI					=	6,
		kXYZRGB				=	7,
		kXYZNormal		=	8,
		kXYZINormal		=	9,
		kXYZRGBNormal	=	10
	};

	static std::string formatName(const Format format);
	static int channels(const Format format) noexcept;
	static bool isScan2d(const Format format) noexcept;
	static bool isScanHasNormals(const Format format) noexcept;
	static bool isScanHasRGB(const Format format) noexcept;
	static bool isScanHasIntensity(const Format format) noexcept;
	static LaserScan backwardCompatibility(
			const cv::Mat & oldScanFormat,
			int maxPoints = 0,
			int maxRange = 0,
			const Transform & localTransform = Transform::getIdentity());
	static LaserScan backwardCompatibility(
			const cv::Mat & oldScanFormat,
			float minRange,
			float maxRange,
			float angleMin,
			float angleMax,
			float angleInc,
			const Transform & localTransform = Transform::getIdentity());

public:
	LaserScan();
	LaserScan(const cv::Mat & data,
			int maxPoints,
			float maxRange,
			Format format,
			const Transform & localTransform = Transform::getIdentity());
	LaserScan(const cv::Mat & data,
			Format format,
			float minRange,
			float maxRange,
			float angleMin,
			float angleMax,
			float angleIncrement,
			const Transform & localTransform = Transform::getIdentity());

	const cv::Mat & data() const noexcept {return data_;}
	Format format() const noexcept {return format_;}
	std::string formatName() const {return formatName(format_);}
	int channels() const noexcept {return data_.channels();}
	int maxPoints() const noexcept {return maxPoints_;}
	float rangeMin() const noexcept {return rangeMin_;}
	float rangeMax() const noexcept {return rangeMax_;}
	float angleMin() const noexcept {return angleMin_;}
	float angleMax() const noexcept {return angleMax_;}
	float angleIncrement() const noexcept {return angleIncrement_;}
	Transform localTransform() const noexcept {return localTransform_;}

	bool isEmpty() const noexcept {return data_.empty();}
	int size() const noexcept {return data_.cols;}
	int dataType() const noexcept {return data_.type();}
	bool is2d() const noexcept {return isScan2d(format_);}
	bool hasNormals() const noexcept {return isScanHasNormals(format_);}
	bool hasRGB() const noexcept {return isScanHasRGB(format_);}
	bool hasIntensity() const noexcept {return isScanHasIntensity(format_);}
	bool isCompressed() const noexcept {return !data_.empty() && data_.type()==CV_8UC1;}
	LaserScan clone() const;

	int getIntensityOffset() const noexcept {return hasIntensity()?(is2d()?2:3):-1;}
	int getRGBOffset() const noexcept {return hasRGB()?(is2d()?2:3):-1;}
	int getNormalsOffset() const noexcept {return hasNormals()?(2 + (is2d()?0:1) + ((hasRGB() || hasIntensity())?1:0)):-1;}

	void clear() {data_ = cv::Mat();}

private:
	cv::Mat data_;
	Format format_;
	int maxPoints_;
	float rangeMin_;
	float rangeMax_;
	float angleMin_;
	float angleMax_;
	float angleIncrement_;
	Transform localTransform_;
};

}

#endif /* CORELIB_INCLUDE_RTABMAP_CORE_LASERSCAN_H_ */
