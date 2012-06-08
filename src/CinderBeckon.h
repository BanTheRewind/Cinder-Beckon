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

#pragma once

#include "Shadow/ISensor.h"
#include "Shadow/ISkeleton.h"
#include "Shadow/IMotionSensor.h"
#include "Shadow/ShadowDefines.h"

#include "cinder/Quaternion.h"
#include "cinder/Surface.h"
#include "cinder/Thread.h"
#include "cinder/Vector.h"
#include <map>

namespace beckon
{

	typedef std::shared_ptr<class Device>		DeviceRef;
	typedef Omek::ImageResolution				ImageResolution;
	typedef Omek::JointID						JointName;
	typedef std::map<JointName, class Joint>	Skeleton;

	class Joint
	{
	public:
		const ci::Quatf&		getAbsoluteRotation() const;
		const ci::Vec3f&		getPosition() const;
		const ci::Quatf&		getRotation() const;
		JointName				getJointName() const;
	private:
		Joint( JointName jointName = JointName::JointID_unknown, const ci::Vec3f &position = ci::Vec3f::zero(), 
			const ci::Quatf &rotation = ci::Quatf(), const ci::Quatf &absoluteRotation = ci::Quatf() );
		ci::Quatf		mAbsRotation;
		JointName		mJointName;
		ci::Vec3f		mPosition;
		ci::Quatf		mRotation;

		friend class	Device;
	};
	
	//////////////////////////////////////////////////////////////////////////////////////////////

	class DeviceOptions
	{
	public:
		enum : uint32_t
		{
			FULL_BODY		= TRACK_ALL, 
			HANDS_IN_FRONT	= TRACK_BASIC
		} typedef TrackingType;

		DeviceOptions();

		DeviceOptions&		enableManualSetup( bool enable = true );
		DeviceOptions&		enableRgb( bool enable = true );
		DeviceOptions&		enableSkeletonTracking( bool enable = true, TrackingType trackingType = TrackingType::FULL_BODY );
		DeviceOptions&		setCameraPosition( float heightInCm = 73.66f, float angleToFloorInDegrees = 0.0f );
		DeviceOptions&		setResolution( ImageResolution resolution );

		float				getAngle() const;
		float				getHeight() const;
		ImageResolution		getResolution() const;
		TrackingType		getTrackingType() const;

		bool				isManualSetupEnabled() const;
		bool				isRgbEnabled() const;
		bool				isSkeletonTrackingEnabled() const;
	private:
		float				mAngle;
		float				mHeight;
		bool				mEnabledManualSetup;
		bool				mEnabledRgb;
		bool				mEnabledSkeletonTracking;
		ImageResolution		mResolution;
		TrackingType		mTrackingType;
	};

	//////////////////////////////////////////////////////////////////////////////////////////////

	class Device
	{
	public:
		static DeviceRef		create();
		~Device();

		bool					isCapturing() const;
		bool					isPaused() const;
		void					pause();
		void					resume();
		void					start( const DeviceOptions &deviceOptions = DeviceOptions() );
		void					stop();

		bool					checkNewFrame();
		float					getDepthAt( const ci::Vec2i &position ) const;
		ci::Surface8u			getFrame();

		bool					checkNewSkeletons();
		std::vector<Skeleton>	getSkeletons();

		const ci::Vec2i&		getSize() const;

		bool					isFlipped() const;
		void					setFlipped( bool flipped = true );
		bool					isRgbMatchingEnabled() const;
		void					enableRgbMatching( bool enable = true );

		void					setFrequency( int32_t frequency );
		int32_t					getFrequency() const;

		const DeviceOptions&	getDeviceOptions() const;
	private:
		typedef std::shared_ptr<boost::thread>	ThreadRef;

		Device();
			
		Omek::IMotionSensor		*mMotionSensor;
		Omek::ISensor			*mSensor;
		Omek::ISkeleton			*mSkeleton;

		uint32_t				mBufferSize;
		int_fast8_t				*mData;
		uint32_t				mNumChannels;

		void					init();

		DeviceOptions			mDeviceOptions;

		bool					mBinary;
		bool					mCapture;
		bool					mFlipped;
		bool					mGreyScale;
		bool					mInverted;
		bool					mMatchRgb;
		bool					mPaused;
		bool					mRemoveBackground;

		boost::mutex			mMutexDepth;
		boost::mutex			mMutexSkeletons;
		volatile bool			mRunning;
		ThreadRef				mThread;
		void					run();

		std::vector<Skeleton>	mSkeletons;
		bool					mNewSkeletons;

		ci::Surface8u			mFrame;
		bool					mNewFrame;

		int32_t					mFrequency;

		ci::Vec2i				mSize;
	};

}
