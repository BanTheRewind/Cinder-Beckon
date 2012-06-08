/*
* 
* Copyright (c) 2012, Ban the Rewind
* All rights reserved.
* 
* Redistribution and use in source and binary forms, with or 
* without modification, are permitted provided that the following 
* conditions are met:
* 
* Redistributions of source code must retain the above copyright 
* notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright 
* notice, this list of conditions and the following disclaimer in 
* the documentation and/or other materials provided with the 
* distribution.
* 
* Neither the name of the Ban the Rewind nor the names of its 
* contributors may be used to endorse or promote products 
* derived from this software without specific prior written 
* permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
* ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* 
*/

#include "CinderBeckon.h"

#include <Windows.h>

namespace beckon
{

	using namespace ci;
	using namespace Omek;
	using namespace std;

	//////////////////////////////////////////////////////////////////////////////////////////////

	Joint::Joint( JointName jointName, const Vec3f &position, const Quatf &rotation, const Quatf &absoluteRotation )
		: mAbsRotation( absoluteRotation ), mJointName( jointName ), mPosition( position ), mRotation( rotation )
	{
	}

	const Quatf& Joint::getAbsoluteRotation() const
	{
		return mAbsRotation;
	}

	const Vec3f& Joint::getPosition() const
	{
		return mPosition;
	}

	const Quatf& Joint::getRotation() const
	{
		return mRotation;
	}

	JointName Joint::getJointName() const
	{
		return mJointName;
	}

	//////////////////////////////////////////////////////////////////////////////////////////////

	DeviceOptions::DeviceOptions()
	{
		mAngle						= 0.0f;
		mHeight						= 73.66f;
		mEnabledRgb					= false;
		mEnabledSkeletonTracking	= true;
		mEnabledManualSetup			= false;
		mResolution					= ImageResolution::IMAGE_RESOLUTION_DEFAULT;
		mTrackingType				= TrackingType::FULL_BODY;
	}

	float DeviceOptions::getAngle() const
	{
		return mAngle;
	}

	float DeviceOptions::getHeight() const
	{
		return mHeight;
	}

	DeviceOptions::TrackingType DeviceOptions::getTrackingType() const
	{
		return mTrackingType;
	}

	bool DeviceOptions::isManualSetupEnabled() const
	{
		return mEnabledManualSetup;
	}

	bool DeviceOptions::isRgbEnabled() const
	{
		return mEnabledRgb;
	}

	bool DeviceOptions::isSkeletonTrackingEnabled() const
	{
		return mEnabledSkeletonTracking;
	}

	DeviceOptions& DeviceOptions::enableManualSetup( bool enable )
	{
		mEnabledManualSetup = enable;
		return *this;
	}

	DeviceOptions& DeviceOptions::enableRgb( bool enable )
	{
		mEnabledRgb = enable;
		return *this;
	}

	DeviceOptions& DeviceOptions::enableSkeletonTracking( bool enable, TrackingType trackingType )
	{
		mEnabledSkeletonTracking = enable;
		mTrackingType = trackingType;
		return *this;
	}

	ImageResolution DeviceOptions::getResolution() const
	{
		return mResolution;
	}

	DeviceOptions& DeviceOptions::setCameraPosition( float heightInCm, float angleToFloorInDegrees )
	{
		mAngle = angleToFloorInDegrees;
		mHeight = heightInCm;
		return *this;
	}

	DeviceOptions& DeviceOptions::setResolution( ImageResolution resolution )
	{
		mResolution = resolution;
		return *this;
	}

	//////////////////////////////////////////////////////////////////////////////////////////////

	DeviceRef Device::create()
	{
		return DeviceRef( new Device() );
	}

	Device::Device()
	{
		init();
	}

	Device::~Device()
	{
		if ( mCapture ) {
			stop();
		}
	}

	bool Device::checkNewFrame()
	{
		bool newFrame = mNewFrame;
		mNewFrame = false;
		return newFrame;
	}

	bool Device::checkNewSkeletons()
	{
		bool newSkeletons = mNewSkeletons;
		mNewSkeletons = false;
		return newSkeletons;
	}

	void Device::enableRgbMatching( bool enable ) 
	{
		mMatchRgb = enable;
		mSensor->setCameraParameter( "matchRGB", mMatchRgb ? 1 : 0 );
	}

	float Device::getDepthAt( const ci::Vec2i &position ) const
	{
		if ( mFrame ) {
			uint16_t value = 0;// = mData + ();
			return (float)value;// TO DO convert to 0 -1
		}
		return 1.0f;
	}

	const DeviceOptions& Device::getDeviceOptions() const
	{
		return mDeviceOptions;
	}

	Surface8u Device::getFrame()
	{
		mNewFrame = false;
		return mFrame;
	}

	int32_t Device::getFrequency() const
	{
		return mFrequency;
	}

	const Vec2i& Device::getSize() const
	{
		return mSize;
	}

	vector<Skeleton> Device::getSkeletons()
	{
		mNewSkeletons = false;
		return mSkeletons;
	}

	void Device::init()
	{
		mBinary				= false;
		mCapture			= false;
		mData				= 0;
		mFrequency			= 1;
		mGreyScale			= false;
		mInverted			= false;
		mMotionSensor		= 0;
		mNewFrame			= false;
		mNewSkeletons		= false;
		mPaused				= false;
		mRemoveBackground	= false;
		mRunning			= false;
		mSensor				= 0;
		mSize				= Vec2i::zero();
		mSkeleton			= 0;
		mSkeletons.clear();
	}

	bool Device::isCapturing() const
	{
		return mCapture;
	}

	bool Device::isFlipped() const
	{
		return mFlipped;
	}

	bool Device::isPaused() const
	{
		return mPaused;
	}

	bool Device::isRgbMatchingEnabled() const
	{
		return mMatchRgb;
	}
	
	void Device::pause()
	{
		if ( mMotionSensor && mMotionSensor->pause() == OMK_SUCCESS ) {
			mPaused = true;
		}
	}
	
	void Device::resume()
	{
		if ( mPaused && mMotionSensor && mMotionSensor->resume() == OMK_SUCCESS ) {
			mPaused = false;
		}
	}

	void Device::run()
	{
		while ( mRunning ) {
			if ( mCapture && mSensor->isAlive() ) {
				bool newFrame = false;
				uint32_t status = mMotionSensor->processNextImage( false, newFrame );
				if ( status == OMK_SUCCESS && newFrame ) { 
					int32_t widthStep = 0;
					status = mMotionSensor->copyRawImage( mData, mBufferSize, widthStep, mDeviceOptions.isRgbEnabled() );
					if ( status == OMK_SUCCESS ) {

						int32_t bitsPerPixel = widthStep / ( mNumChannels * mSize.x );
						if ( bitsPerPixel >= 1 && bitsPerPixel <= 2 && ( mNumChannels == 1 || mNumChannels == 3 ) ) {

							int16_t maxValue	= 0;
							int16_t minValue	= 32767;
							int32_t offset1		= 0;
							int32_t offset2		= 0;
							int32_t offset3		= 0;
							int16_t range		= 0;

							if ( bitsPerPixel == 1 ) {
								offset1 = 2;
								offset2 = 1;
								offset3 = 0;
							} else if ( bitsPerPixel == 2 ) {
								uint16_t* data = (uint16_t*)mData;
								for ( int32_t y = 0; y < mSize.y; ++y ) {
									for ( int32_t x = 0; x < mSize.x; ++x ) {
										int16_t value = data[ y * mSize.x + x ];
										if ( value < minValue && value > 0 ) {
											minValue = value;
										}
										if ( value > maxValue ) {
											maxValue = value;
										}
									}
								}
								range = maxValue == minValue ? 1 : maxValue - minValue;
							}

							uint32_t position = 0;
							uint16_t* data = (uint16_t*)mData;
							uint32_t stride = mBufferSize / mSize.y;
							for ( int32_t y = 0; y < mSize.y; ++y ) {
								uint8_t* row = mFrame.getData() + ( y * stride );
								for ( int32_t x = 0; x < mSize.x; ++x ) {
									position = y * widthStep + x * bitsPerPixel * mNumChannels;

									if ( bitsPerPixel == 1 ) {
										row[ x * 3 + 0 ]	= mData[ position + offset1 ];
										row[ x * 3 + 1 ]	= mData[ position + offset2 ];
										row[ x * 3 + 2 ]	= mData[ position + offset3 ];
									} else {
										uint16_t valueShort	= data[ y * mSize.x + x ];
										uint_fast8_t value	= (uint_fast8_t)( ( valueShort - minValue ) * 255 / range );
										row[ x * 3 + 0 ]	= value;
										row[ x * 3 + 1 ]	= value;
										row[ x * 3 + 2 ]	= value;
									}
								}
							}
							mNewFrame = true;
						}
					}

				}
			}
			Sleep( 7 );
		}
	}

	void Device::setFlipped( bool flipped ) 
	{
		mFlipped = flipped;
		mSensor->setCameraParameter( "flipped", mFlipped ? 1 : 0 );
	}

	void Device::setFrequency( int32_t frequency ) 
	{
		mFrequency = frequency;
		mSensor->setCameraParameter( "frequency", mFrequency );
	}

	void Device::start( const DeviceOptions &deviceOptions )
	{
		if ( mCapture ) {
			stop();
		}
		mDeviceOptions = deviceOptions;

		mMotionSensor = IMotionSensor::createCameraSensor();
		if ( !mMotionSensor ) {
			return;
		}
		mSensor = mMotionSensor->getSensor();
		if ( !mSensor->isAlive() ){
			return;
		}

		enableRgbMatching( mMatchRgb );
		setFlipped( mFlipped );
		mSensor->setCameraParameter( "enableRGB", mDeviceOptions.isRgbEnabled() ? 1 : 0 );
		if ( mDeviceOptions.isRgbEnabled() ) {
			mSensor->setCameraParameter( "outImageRes", mDeviceOptions.getResolution() );
		} else {
			mSensor->setCameraParameter( "outDepthRes", mDeviceOptions.getResolution() );
		}
		setFrequency( mFrequency );

		if ( mDeviceOptions.isSkeletonTrackingEnabled() ) {
			mSkeleton = IMotionSensor::createSkeleton();
			if ( mMotionSensor->setTrackingOptions( mDeviceOptions.getTrackingType() ) != OMK_SUCCESS ) {
			}
		}

		if ( mDeviceOptions.isManualSetupEnabled() ) {
			mMotionSensor->setCameraSetup( mDeviceOptions.getHeight(), mDeviceOptions.getAngle() );
		}

		ImageType imageType	= mDeviceOptions.isRgbEnabled() ? IMAGE_TYPE_COLOR : IMAGE_TYPE_DEPTH;
		int32_t bitsPerPixel	= mSensor->getImageBpp( imageType ) / 8;
		mSize.y					= mSensor->getImageHeight( imageType );
		mSize.x					= mSensor->getImageWidth( imageType );
		int32_t numChannels		= mSensor->getImageChannels( imageType );
		
		if ( mSize.x <= 0 || mSize.y <= 0 || numChannels <= 0 || bitsPerPixel <= 0 ) {
			return;
		}

		mNumChannels	= numChannels;
		mBufferSize		= mSize.x * mSize.y * mNumChannels * bitsPerPixel;
		mData			= new int_fast8_t[ mBufferSize * sizeof( int_fast8_t ) ];
		mFrame			= Surface8u( mSize.x, mSize.y, false, SurfaceChannelOrder::RGB );

		mCapture		= true;
		mRunning		= true;
		mThread			= ThreadRef( new boost::thread( bind( &Device::run, this ) ) );
	}

	void Device::stop()
	{
		mRunning = false;
		if ( mThread ) {
			mThread->join();
		}

		if ( mMotionSensor != 0 ) {
			mMotionSensor->stop();
			IMotionSensor::releaseMotionSensor( mMotionSensor );
		}
		if ( mSkeleton != 0 ) {
			IMotionSensor::releaseSkeleton( mSkeleton );
		}

		if ( mData != 0 ) {
			delete [] mData;
		}
		
		mCapture = false;
		mSkeletons.clear();
	}

}
