# Go2 Navigation 编译流程说明

本文档说明如何编译 `go2_navigation` 包。

## 环境要求

- **操作系统**: Ubuntu 20.04
- **ROS 版本**: ROS 2 Foxy
- **工作空间**: `go2_ws1`

## 编译步骤

### 1. 进入工作空间目录

```bash
cd ~/go2_ws1
```

### 2. 检查源码目录

确认源码位于 `src` 目录下：

```bash
cd src
ls
```

应该看到以下包：
- `go2_bringup`
- `go2_description`
- `go2_driver`
- `go2_interfaces`
- `go2_navigation`
- `go2_robot`
- `go2_rviz`
- `livox_ros_driver2`
- `unitree_api`
- `unitree_go`

### 3. 更新源码（可选）

如果使用 Git 管理源码，可以更新到最新版本：

```bash
git pull
```

### 4. 返回工作空间根目录

```bash
cd ..
```

### 5. 编译 go2_navigation 包

使用 `colcon` 编译工具，使用 `--symlink-install` 选项创建符号链接（便于开发时修改配置）：

```bash
colcon build --packages-select go2_navigation --symlink-install
```

**编译选项说明：**
- `--packages-select go2_navigation`: 只编译指定的包
- `--symlink-install`: 使用符号链接安装，修改配置文件后无需重新编译

### 6. 编译输出

编译成功后，会看到类似输出：

```
Starting >>> go2_navigation
Finished <<< go2_navigation [1.01s]

Summary: 1 package finished [4.38s]
```

### 7. Source 工作空间（如果尚未 source）

编译完成后，需要 source 工作空间以使更改生效：

```bash
source install/setup.bash
```

或者将其添加到 `~/.bashrc` 中以便每次打开终端自动 source：

```bash
echo "source ~/go2_ws1/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 完整编译流程（一键执行）

```bash
# 进入工作空间
cd ~/go2_ws1

# 更新源码（可选）
cd src
git pull
cd ..

# 编译 go2_navigation 包
colcon build --packages-select go2_navigation --symlink-install

# Source 工作空间
source install/setup.bash
```

## 编译所有包

如果需要编译工作空间中的所有包：

```bash
cd ~/go2_ws1
colcon build --symlink-install
```

## 清理编译文件

如果需要清理编译生成的文件：

```bash
cd ~/go2_ws1
rm -rf build install log
```

## 常见问题

### 1. 编译错误：找不到依赖包

**解决方法：**
```bash
# 安装缺失的依赖
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 2. 配置文件修改后不生效

**解决方法：**
- 确保使用了 `--symlink-install` 选项
- 重新 source 工作空间：`source install/setup.bash`
- 如果仍不生效，重新编译：`colcon build --packages-select go2_navigation --symlink-install`

### 3. 符号链接问题

如果遇到符号链接相关错误，可以尝试不使用 `--symlink-install`：

```bash
colcon build --packages-select go2_navigation
```

## 验证编译结果

编译完成后，可以验证包是否正确安装：

```bash
# 检查包是否存在
ros2 pkg list | grep go2_navigation

# 查看包的信息
ros2 pkg prefix go2_navigation
```

## 相关文件位置

编译完成后，相关文件位置：

- **可执行文件**: `~/go2_ws1/install/go2_navigation/lib/go2_navigation/`
- **配置文件**: `~/go2_ws1/install/go2_navigation/share/go2_navigation/config/`
- **Launch 文件**: `~/go2_ws1/install/go2_navigation/share/go2_navigation/launch/`

## 注意事项

1. **使用 `--symlink-install` 的优势**：
   - 修改配置文件后无需重新编译
   - 节省编译时间
   - 便于开发和调试

2. **首次编译**：
   - 首次编译可能需要较长时间
   - 建议先编译所有依赖包

3. **增量编译**：
   - 只编译修改的包可以节省时间
   - 使用 `--packages-select` 指定要编译的包

## 参考

- [ROS 2 Colcon 文档](https://colcon.readthedocs.io/)
- [Nav2 官方文档](https://navigation.ros.org/)

