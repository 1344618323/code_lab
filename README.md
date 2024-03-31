# CODE LAB

Some exercises of C++ and python

## setup

1. git clone --recurse-submodules git@github.com:1344618323/code_lab.git
2. use docker

    ```sh
    # build a ubuntu20 image
    cd /home/cxn/leofile/code_lab/docker
    docker build -t ubuntu20-image .

    # build container
    bash docker_run.sh --run --image-name ubuntu20-image --container-name ubuntu20-container
    ```
3. compile thirdparty
   
   ```sh
   cd /home/cxn/leofile/code_lab/Thirdparty
   sh ./build_thirdparty.sh 
   ```
4. complie source
   ```sh
   cd /home/cxn/leofile/code_lab
   make
   ```

## pitfalls

不知道为啥，如果没有把Pangolin安装到系统默认路径下，而是自定义的路径下，编译出的可执行文件运行时，有：
```
cxn@cxn-XPS-15-7590:~/leofile/code_lab/build$ ldd main
libpango_display.so => /home/cxn/leofile/code_lab/Thirdparty/Thirdparty_install/lib/libpango_display.so (0x00007a3f8da1b000)
libpango_opengl.so => /home/cxn/leofile/code_lab/Thirdparty/Thirdparty_install/lib/libpango_opengl.so (0x00007a3f8d9d0000)
libpango_windowing.so => not found
libpango_vars.so => not found
libpango_image.so => not found
libpango_core.so => not found
```
一部分pango lib找到了，一部分找不到。个人感觉是pangolin代码的锅。解决办法：
1. 直接把pangolin转到/usr/local下算球
2. 运行可执行文件前，执行`export LD_LIBRARY_PATH=/home/cxn/leofile/code_lab/Thirdparty/Thirdparty_install/lib:$LD_LIBRARY_PATH`