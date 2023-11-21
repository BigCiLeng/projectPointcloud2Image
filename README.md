# Project a point cloud as images

## Input:   

- 点云文件, 点云格式为(x, y, z, R, G, B)   
- 相机pose   

## Output:   

- 每个相机pose下的image与depth


![image](https://raw.githubusercontent.com/BigCiLeng/picgo_photos/main/img/image_0.jpg)

![depth](https://raw.githubusercontent.com/BigCiLeng/picgo_photos/main/img/depth_0.jpg)
      
## 流程:   

1. 读取点云文件   

        read_pc()   

2. 将点云中心位置移动到世界坐标系中心   

        reg_pc() 

3. 生成相机位姿   

        create_spheric_pose()   

4. 根据相机位姿投影点云   

        project_point_cloud()   

5. 渲染视图与深度图   

        pc2image()   
