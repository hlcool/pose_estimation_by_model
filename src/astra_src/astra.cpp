#include "astra.hpp"
#include <unistd.h>

mutex* astra::mtx=new mutex;
astra::astra(int deviceID)
{
	continue_sample=true;
	bDevOpen=false;
	is_waiting=false;

	Status rc = openni::STATUS_OK;
    Status rc_device = openni::STATUS_OK;
    Status rc_depth = openni::STATUS_OK;
    Status rc_color = openni::STATUS_OK;

	if(deviceID%2==0)
	{
		std::thread astra_thread_two(&astra::astra_ctrl_two,this,deviceID);
		astra_thread_two.detach();
	}
	else
	{
		std::thread astra_thread_one(&astra::astra_ctrl_one,this,deviceID);
		astra_thread_one.detach();
	}
}

bool astra::astra_ctrl_one(int deviceID)
{
	string str_dev_id=to_string(deviceID);
	while(continue_sample)
	{
		mtx->lock();
		while(is_waiting)
		{
			sleep(1);
		}
		if(!bDevOpen)
		{
			rc_device = openni::OpenNI::initialize();
	//		printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());
			openni::Array<DeviceInfo> deviceInfoList;
			openni::OpenNI::enumerateDevices(&deviceInfoList);
			int device_num=deviceInfoList.getSize();
			if(device_num!=0)
			{
				if(device_num==1)
					rc_device = device.open(deviceInfoList[0].getUri());
				else if(device_num>1)
				{
					if(deviceID<device_num)
						rc_device = device.open(deviceInfoList[deviceID].getUri());
					else
						return false;
				}
				else
					return false;
			}
			else
				return false;
	//		const char* deviceURI = openni::ANY_DEVICE;
	//		rc_device = device.open(deviceURI);
			if (rc_device != openni::STATUS_OK)
			{
				
				printf("Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
				openni::OpenNI::shutdown();
				return false;
			}
			
			device.setDepthColorSyncEnabled(true);
		
			rc_depth = m_depthStream.create(device, openni::SENSOR_DEPTH);
			depth_mode.setResolution(640,480);
			depth_mode.setFps(30);
			depth_mode.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);
			m_depthStream.setVideoMode(depth_mode);
		
			rc_color = m_colorStream.create(device, openni::SENSOR_COLOR);
			color_mode.setResolution(640,480);
			color_mode.setFps(30);
			color_mode.setPixelFormat(PIXEL_FORMAT_RGB888);

			m_colorStream.setVideoMode(color_mode);
			
			CameraSettings *camera_set=m_colorStream.getCameraSettings();
			if(camera_set)
			{
				camera_set->setAutoWhiteBalanceEnabled(true);
				camera_set->setAutoExposureEnabled(true);
			}

			if (device.isImageRegistrationModeSupported(IMAGE_REGISTRATION_DEPTH_TO_COLOR))
			{
				device.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
			}
		
			if (rc_depth == openni::STATUS_OK)
			{
				rc_depth = m_depthStream.start();
				if (rc_depth != openni::STATUS_OK)
				{
					printf("Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
					m_depthStream.destroy();
					return false;
				}
			}
			else
			{
				printf("Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
				return false;
			}
			
			if (rc_color == openni::STATUS_OK)
			{
				rc_color = m_colorStream.start();
				if (rc_color != openni::STATUS_OK)
				{
					printf("Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
					m_colorStream.destroy();
					return false;
				}
			}
			else
			{
				printf("Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
				return false;
			}
			
			if (!m_depthStream.isValid() || !m_colorStream.isValid())
			{
				printf("No valid streams. Exiting\n");
				openni::OpenNI::shutdown();
				return false;
			}

			bDevOpen=true;
		}
		
		int changedIndex;
		openni::VideoStream** m_streams=new openni::VideoStream*[2];
		m_streams[0] = &m_depthStream;
		m_streams[1] = &m_colorStream;

		try
		{
			rc = OpenNI::waitForAnyStream(m_streams, 1, &changedIndex, SAMPLE_READ_WAIT_TIMEOUT);
		}
		catch(...)
		{
			continue;
		}
		
		if (rc != openni::STATUS_OK)
		{
			printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
			continue;
		}
		rc = m_depthStream.readFrame(&m_depthFrame);
		if (rc != openni::STATUS_OK)
		{
			printf("Read depth frame failed!\n%s\n", OpenNI::getExtendedError());
			continue;
		}
		rc = m_colorStream.readFrame(&m_colorFrame);
		if (rc != openni::STATUS_OK)
		{
			printf("Read color frame failed!\n%s\n", OpenNI::getExtendedError());
			continue;
		}

		int width_d=m_depthFrame.getWidth();
		int height_d=m_depthFrame.getHeight();
		int width_c=m_colorFrame.getWidth();
		int height_c=m_colorFrame.getHeight();
		
		Mat coordinate_3d(height_d,width_d,CV_32FC3);
		float w_x,w_y,w_z;

		Mat mat_color(height_c,width_c,CV_8UC3,(RGB888Pixel*)m_colorFrame.getData());
		Mat mat_depth_1mm(height_d,width_d,CV_16UC1,(DepthPixel*)m_depthFrame.getData());
		
		cvtColor(mat_color,data_color,CV_RGB2BGR);
		mat_depth_1mm.copyTo(data_depth);
		/*******************display****************************/
		// int rows=data_color.rows;
		// int cols=data_color.cols;
		// Mat mat_bgr,mat_depth,mat_cvt_depth,mat_fusion;
		// double max,min;

		// data_depth.copyTo(mat_depth);
		// minMaxIdx(mat_depth,&min,&max);

		// mat_depth.convertTo(mat_depth,CV_8UC1,255.0/(max+1));
		// cvtColor(mat_depth, mat_cvt_depth, CV_GRAY2BGR);
		// mat_bgr=data_color;
		// addWeighted(mat_bgr,0.5,mat_cvt_depth,0.5,0,mat_fusion);
		
		// Mat mat_merge=Mat::zeros(2*rows,2*cols,CV_8UC3);
		// mat_bgr.copyTo(mat_merge(Range(0,rows),Range(0,cols)));
		// mat_cvt_depth.copyTo(mat_merge(Range(0,rows),Range(cols,2*cols)));
		// mat_fusion.copyTo(mat_merge(Range(rows,2*rows),Range(0,cols)));

		// imshow("view_port_"+str_dev_id,mat_merge);
		// waitKey(10);

		/**********************3d*****************************/		
		for(int i=0;i<height_d;i++)
		{
			for(int j=0;j<width_d;j++)
			{
				if(mat_depth_1mm.at<ushort>(i,j)>300&&mat_depth_1mm.at<ushort>(i,j)<=3000)
				{
					
					openni::CoordinateConverter::convertDepthToWorld(m_depthStream,j,i,mat_depth_1mm.at<ushort>(i,j),&w_x,&w_y,&w_z);
					coordinate_3d.at<Vec3f>(i,j)[0]=w_x;
					coordinate_3d.at<Vec3f>(i,j)[1]=w_y;
					coordinate_3d.at<Vec3f>(i,j)[2]=w_z;
				}
				else
				{
					coordinate_3d.at<Vec3f>(i,j)[0]=0;
					coordinate_3d.at<Vec3f>(i,j)[1]=0;
					coordinate_3d.at<Vec3f>(i,j)[2]=0;
				}
			}
		}
		coordinate_3d.copyTo(data_3d_pts);
		delete [] m_streams;
		mtx->unlock();
	}
    // m_depthStream.stop();
    // m_colorStream.stop();
    return true;
}

bool astra::astra_ctrl_two(int deviceID)
{
	string str_dev_id=to_string(deviceID);
	while(continue_sample)
	{
		mtx->lock();
		while(is_waiting)
		{
			sleep(1);
		}
		if(!bDevOpen)
		{
			rc_device = openni::OpenNI::initialize();
	//		printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());
			openni::Array<DeviceInfo> deviceInfoList;
			openni::OpenNI::enumerateDevices(&deviceInfoList);
			int device_num=deviceInfoList.getSize();
			if(device_num!=0)
			{
				if(device_num==1)
					rc_device = device.open(deviceInfoList[0].getUri());
				else if(device_num>1)
				{
					if(deviceID<device_num)
						rc_device = device.open(deviceInfoList[deviceID].getUri());
					else
						return false;
				}
				else
					return false;
			}
			else
				return false;
	//		const char* deviceURI = openni::ANY_DEVICE;
	//		rc_device = device.open(deviceURI);
			if (rc_device != openni::STATUS_OK)
			{
				
				printf("Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
				openni::OpenNI::shutdown();
				return false;
			}
			
			device.setDepthColorSyncEnabled(true);
		
			rc_depth = m_depthStream.create(device, openni::SENSOR_DEPTH);
			depth_mode.setResolution(640,480);
			depth_mode.setFps(30);
			depth_mode.setPixelFormat( PIXEL_FORMAT_DEPTH_1_MM );
			m_depthStream.setVideoMode(depth_mode);
		
			rc_color = m_colorStream.create(device, openni::SENSOR_COLOR);
			color_mode.setResolution(640,480);
			color_mode.setFps(30);
			color_mode.setPixelFormat(PIXEL_FORMAT_RGB888);
			m_colorStream.setVideoMode(color_mode);
			
			CameraSettings *camera_set=m_colorStream.getCameraSettings();
			if(camera_set)
			{
				camera_set->setAutoWhiteBalanceEnabled(true);
				camera_set->setAutoExposureEnabled(true);
			}

			if (device.isImageRegistrationModeSupported(IMAGE_REGISTRATION_DEPTH_TO_COLOR))
			{
				device.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
			}
		
			if (rc_depth == openni::STATUS_OK)
			{
				rc_depth = m_depthStream.start();
				if (rc_depth != openni::STATUS_OK)
				{
					printf("Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
					m_depthStream.destroy();
					return false;
				}
			}
			else
			{
				printf("Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
				return false;
			}
			
			if (rc_color == openni::STATUS_OK)
			{
				rc_color = m_colorStream.start();
				if (rc_color != openni::STATUS_OK)
				{
					printf("Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
					m_colorStream.destroy();
					return false;
				}
			}
			else
			{
				printf("Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
				return false;
			}
			
			if (!m_depthStream.isValid() || !m_colorStream.isValid())
			{
				printf("No valid streams. Exiting\n");
				openni::OpenNI::shutdown();
				return false;
			}

			bDevOpen=true;
		}
		
		int changedIndex;
		openni::VideoStream** p_streams=new openni::VideoStream*[2];
		p_streams[0] = &m_depthStream;
		p_streams[1] = &m_colorStream;

		try
		{
			rc = OpenNI::waitForAnyStream(p_streams, 1, &changedIndex, SAMPLE_READ_WAIT_TIMEOUT);
		}
		catch(...)
		{
			continue;
		}
		
		if (rc != openni::STATUS_OK)
		{
			printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
			continue;
		}
		rc = m_depthStream.readFrame(&m_depthFrame);
		if (rc != openni::STATUS_OK)
		{
			printf("Read depth frame failed!\n%s\n", OpenNI::getExtendedError());
			continue;
		}
		rc = m_colorStream.readFrame(&m_colorFrame);
		if (rc != openni::STATUS_OK)
		{
			printf("Read color frame failed!\n%s\n", OpenNI::getExtendedError());
			continue;
		}

		int width_d=m_depthFrame.getWidth();
		int height_d=m_depthFrame.getHeight();
		int width_c=m_colorFrame.getWidth();
		int height_c=m_colorFrame.getHeight();
		
		Mat coordinate_3d(height_d,width_d,CV_32FC3);
		float w_x,w_y,w_z;

		Mat mat_color(height_c,width_c,CV_8UC3,(RGB888Pixel*)m_colorFrame.getData());
		Mat mat_depth_1mm(height_d,width_d,CV_16UC1,(DepthPixel*)m_depthFrame.getData());
		
		cvtColor(mat_color,data_color,CV_RGB2BGR);
		mat_depth_1mm.copyTo(data_depth);
		/*******************display****************************/
		int rows=data_color.rows;
		int cols=data_color.cols;
		Mat mat_bgr,mat_depth,mat_cvt_depth,mat_fusion;
		double max,min;

		data_depth.copyTo(mat_depth);
		minMaxIdx(mat_depth,&min,&max);

		mat_depth.convertTo(mat_depth,CV_8UC1,255.0/(max+1));
		cvtColor(mat_depth, mat_cvt_depth, CV_GRAY2BGR);
		mat_bgr=data_color;
		addWeighted(mat_bgr,0.5,mat_cvt_depth,0.5,0,mat_fusion);
		
		Mat mat_merge=Mat::zeros(2*rows,2*cols,CV_8UC3);
		mat_bgr.copyTo(mat_merge(Range(0,rows),Range(0,cols)));
		mat_cvt_depth.copyTo(mat_merge(Range(0,rows),Range(cols,2*cols)));
		mat_fusion.copyTo(mat_merge(Range(rows,2*rows),Range(0,cols)));

		imshow("view_port_"+str_dev_id,mat_merge);
		waitKey(10);

		/**********************3d*****************************/		
		for(int i=0;i<height_d;i++)
		{
			for(int j=0;j<width_d;j++)
			{
				if(mat_depth_1mm.at<ushort>(i,j)>300&&mat_depth_1mm.at<ushort>(i,j)<=3000)
				{
					
					openni::CoordinateConverter::convertDepthToWorld(m_depthStream,j,i,mat_depth_1mm.at<ushort>(i,j),&w_x,&w_y,&w_z);
					coordinate_3d.at<Vec3f>(i,j)[0]=w_x;
					coordinate_3d.at<Vec3f>(i,j)[1]=w_y;
					coordinate_3d.at<Vec3f>(i,j)[2]=w_z;
				}
				else
				{
					coordinate_3d.at<Vec3f>(i,j)[0]=0;
					coordinate_3d.at<Vec3f>(i,j)[1]=0;
					coordinate_3d.at<Vec3f>(i,j)[2]=0;
				}
			}
		}
		coordinate_3d.copyTo(data_3d_pts);
		delete [] p_streams;
		mtx->unlock();
	}
    // m_depthStream.stop();
    // m_colorStream.stop();
    return true;
}

void astra::hMirrorTrans(const Mat &src, Mat &dst)
{
    dst.create(src.rows, src.cols, src.type());  
    
      int rows = src.rows;  
      int cols = src.cols;  
    
      switch (src.channels())  
      {  
      case 1:
          if(src.type()==0)
          {
              const uchar *origal;  
              uchar *p;  
              for (int i = 0; i < rows; i++){  
                  origal = src.ptr<uchar>(i);  
                  p = dst.ptr<uchar>(i);  
                  for (int j = 0; j < cols; j++){  
                      p[j] = origal[cols - 1 - j];  
                  }  
              }  
          }
          else if(src.type()==2)
          {
              const ushort *origal;  
              ushort *p;  
              for (int i = 0; i < rows; i++){
                  origal = src.ptr<ushort>(i);  
                  p = dst.ptr<ushort>(i);  
                  for (int j = 0; j < cols; j++){  
                      p[j] = origal[cols - 1 - j];  
                  }  
              }  
          }
          else;
          break;  
      case 3:
          if(src.type()==16)
          {
              const Vec3b *origal3;
              Vec3b *p3;  
              for (int i = 0; i < rows; i++) {  
                  origal3 = src.ptr<Vec3b>(i);  
                  p3 = dst.ptr<Vec3b>(i);  
                  for (int j = 0; j < cols; j++){  
                      p3[j] = origal3[cols - 1 - j];  
                  }  
              }  
          }
          else if(src.type()==21)
          {
              const Vec3f *origal3;  
              Vec3f *p3;  
              for (int i = 0; i < rows; i++) {  
                  origal3 = src.ptr<Vec3f>(i);  
                  p3 = dst.ptr<Vec3f>(i);  
                  for (int j = 0; j < cols; j++){  
                      p3[j] = origal3[cols - 1 - j];  
                  }  
              }  
          }
          else;
          break;  
      default:  
          break;  
      }  
}

bool astra::open_astra()
{
//	thread *pthread=astra_thread(mem_fn(&astra::astra_ctrl),this,1);
	is_waiting=false;
	bDevOpen=false;
	return true;
}

bool astra::close_astra()
{
	is_waiting=true;
	try
	{
		m_depthStream.stop();
		m_colorStream.stop();
		m_depthStream.destroy();
		m_colorStream.destroy();
		device.close();
	}
	catch(...)
	{
		return false;
	}
	return true;
}

bool astra::is_opened()
{
	if(is_waiting)
	{
		return false;
	}
	else
	{
		return true;
	}
}

int astra::get_dev_id()
{
	return DevID;
}

astra::~astra()
{
	bDevOpen=false;
	continue_sample=false;
	m_depthStream.stop();
	m_colorStream.stop();
    m_depthStream.destroy();
	m_colorStream.destroy();
	device.close();
	if(mtx)
		delete mtx;
}