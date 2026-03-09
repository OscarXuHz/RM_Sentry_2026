import open3d as o3d
import numpy as np
def read_pcd_to_list(pcd_path):
    # 读取PCD文件
    pcd = o3d.io.read_point_cloud(pcd_path)
    # 将点云数据转换为numpy数组
    points = np.asarray(pcd.points)
    # 将numpy数组转换为list
    return points
def write_list_to_pcd(points_list, pcd_path):
    # 将点列表转换为Open3D的点云格式
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_list)
    # 写入PCD文件
    o3d.io.write_point_cloud(pcd_path, pcd)

# 使用示例
pcd_path_base = "Jul30_19_40/scans_1"
pcd_num = 13
points_list = []
for i in range(1, pcd_num + 1):
    pcd_path = pcd_path_base + str(pcd_num) + '.pcd'
    point_list = read_pcd_to_list(pcd_path)
    points_list.extend(point_list)
new_points_list = []
max_x = -1000
min_x = 1000
max_y = -1000
min_y = 1000
max_z = -1000
min_z = 1000
for i in points_list:
    if i[0] > max_x:
        max_x = i[0]
    if i[0] < min_x:
        min_x = i[0]
    if i[1] > max_y:
        max_y = i[1]
    if i[1] < min_y:
        min_y = i[1]
    if i[2] > max_z:
        max_z = i[2]
    if i[2] < min_z:
        min_z = i[2]
resolution = 0.05
print(max_x, min_x, max_y, min_y, max_z, min_z)
num_x = int((max_x - min_x)/resolution+0.5)+1
num_y = int((max_y - min_y)/resolution+0.5)+1
num_z = int((max_z - min_z)/resolution+0.5)+1
print(num_x, num_y, num_z)
points = np.zeros((num_x, num_y, num_z))
for i in points_list:
    x = int((i[0] - min_x)/resolution+0.5)
    y = int((i[1] - min_y)/resolution+0.5)
    z = int((i[2] - min_z)/resolution+0.5)
    points[x, y, z] = 1
del points_list
for i in range(num_x):
    for j in range(num_y):
        for k in range(num_z):
            if points[i, j, k] == 1:
                flags=[0,0,0,0,0,0]
                for d in range(1,4):
                    if i+d < num_x and points[i+d, j, k] == 1:
                        flags[0] = 1
                    if i-d >= 0 and points[i-d, j, k] == 1:
                        flags[1] = 1
                    if j+d < num_y and points[i, j+d, k] == 1:
                        flags[2] = 1
                    if j-d >= 0 and points[i, j-d, k] == 1:
                        flags[3] = 1
                    if k+d < num_z and points[i, j, k+d] == 1:
                        flags[4] = 1
                    if k-d >= 0 and points[i, j, k-d] == 1:
                        flags[5] = 1
                num = 0
                for flag in flags:
                    num += flag
                if k!=0 and num <= 5:
                    new_points_list.append([i*resolution+min_x, j*resolution+min_y, k*resolution+min_z])
# point_cloud = 
pcd_path = 'Jul30_19_40/scans_all.pcd'  # 输出文件路径
write_list_to_pcd(new_points_list, pcd_path)
print(len(new_points_list))