# LiteS

**A Rendering Engine using xml file to control rendering passes**

# Summary

This is a very very tiny rendering engine just including few features. It is mainly for the research purpose.

It also includes some algorithms as a testcase that I have implemented through some papers.

**The engine works in following steps:**

- Import models or pre-defined shape using [Assimp]()
- Set up the environment(glew,glfw......)
- Reading the xml config file and define all the rendering pass and the textures that will be used though each pass 
- Rendering as the config file defined
- At the end of each frame, some status will be updated. Such as the position of camera, frame debug message......

**Now the engine surpports these functions:**

|Function|Support
| -------| -----: |
|Import Models|√
|Read Image As Texture|√
|Render to Texture|√
|Deferred Rendering|√
|Multiple Target|√

# Hello World

## **Step 0** Set up the environment and the dependencies

First you need to download the source code of LiteS and compile it. We use CMake to generate the project file which is convenient for user in all platform.

You can download the source code using Git Bash:

`git clone https://www.github.com/whatseven/LiteS.git`

And compile it using CMake:

`cd LiteS/`

`mkdir debug`

`cmake ..`

## **Step 1** Compile the Source code

Now you can see the project file through your `debug/`. If you are using Visual Studio, it mush be a `.sln` file. Then open the file with Visual Studio.

Before you compile the project, you mush activate the `Install` project by click `Build -> Configuration Manager` and make the `Install` option available.

Then you can click `Build Solutions` to compile the source code. And when the process finished, it must have some errors with the testcase. You need to build again for the testcase.

> **Remenber: Cmake will find the dependencies like glew,glfw3,assimp,glm.... in the system environment path. So if you don't set the environment variebles, you mush specify the path of the dependencies later in the project **

> The pre defined environment variebles:

>|Dependency|Name of the Environment Varieble
>| -------| -----: |
>|GLM|GLM
>|GLEW AND GLFW3|OPENGL
>|ASSIMP|ASSIMP

## **Step 2** Try to Launch the Testcase

You can right click any of the testcase and set it as a entrance project. Then you can test if it is work. 

**Now the testcase of engine mainly includes:**

|Testcase|Papper or Features
| -------| -----: |
|T-Texture-Loaded|Loading Models
|T-BaseSSAO|Base SSAO Demo
|T-RobustSSAO|[Scalable Adaptive SSAO in GPU Zen](https://www.amazon.com/gp/product/B0711SD1DW?tag=realtimerenderin&pldnSite=1)
|T-ASSAO|[Robust Screen Space Ambient Occlusion in 1 ms in 1080p on PS4 in GPU Zen](https://www.amazon.com/gp/product/B0711SD1DW?tag=realtimerenderin&pldnSite=1)
|T-CHC|[Coherent Hierarchical Culling](https://cg.tuwien.ac.at/research/vr/chcull/bittner-eg04-chcull.pdf)

## **Step 3** Start Your Own Project

