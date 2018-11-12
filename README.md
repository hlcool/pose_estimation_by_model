
一、需要依赖的第三方库
   -> OpenCV 
   	2.4以上版本，可以通过源码编译安装
    
   -> OpenGL
   	sudo apt-get install libgl1-mesa-dev libglu1-mesa-dev freeglut3-dev libdevil-dev
   
   去glew的网址：http://glew.sourceforge.net/下载那个1.13.0版本，然后安装glew
   make 
   sudo make install
   默认把编译好的库文件libglew.so.1.13放到了/usr/lib64下
   sudo ldconfig /usr/lib64/
   
   
二、编译walker2_pose

	cmake ..
	make
	
	
三、运行
	./demo_astra ../models/ ../models/camera_astra.xml 
	 
   
   
   
